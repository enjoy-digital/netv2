#!/usr/bin/env python3

import sys
import os
import math
import argparse

from migen import *

from litex.build import tools

from litex.boards.platforms import netv2

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.integration.export import *

from litex.soc.cores.clock import S7PLL, S7IDELAYCTRL
from litex.soc.cores.dna import DNA
from litex.soc.cores.xadc import XADC
from litex.soc.cores.icap import ICAP
from litex.soc.cores.freqmeter import FreqMeter
from litex.soc.cores.spi_flash import S7SPIFlash

from litedram.modules import K4B2G1646F
from litedram.phy import s7ddrphy

from liteeth.phy.rmii import LiteEthPHYRMII
from liteeth.core.mac import LiteEthMAC
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
from litepcie.frontend.dma import LitePCIeDMA
from litepcie.frontend.wishbone import LitePCIeWishboneBridge

from litevideo.input import HDMIIn
from litevideo.output import VideoOut

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module, AutoCSR):
    def __init__(self, platform, sys_clk_freq):
        self.reset = CSR() # FIXME: not used for now

        self.clock_domains.cd_sys       = ClockDomain()
        self.clock_domains.cd_sys4x     = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_clk200    = ClockDomain()
        self.clock_domains.cd_clk100    = ClockDomain()
        self.clock_domains.cd_eth       = ClockDomain()

        # # #

        clk50 = platform.request("clk50")
        platform.add_period_constraint(clk50, 1e9/50e6)

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys,       sys_clk_freq)
        pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_clk200,    200e6)
        pll.create_clkout(self.cd_clk100,    100e6)
        pll.create_clkout(self.cd_eth,       50e6)

        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_clk200)

# NeTV2 --------------------------------------------------------------------------------------------

class NeTV2(SoCSDRAM):
    def __init__(self, platform,
        with_cpu       = True,
        with_sdram     = True,
        with_etherbone = True,
        with_pcie      = True,
        with_hdmi_in0  = True,
        with_hdmi_out0 = True):
        sys_clk_freq = int(100e6)

        # SoCSDRAM ---------------------------------------------------------------------------------
        SoCSDRAM.__init__(self, platform, sys_clk_freq,
            cpu_type                 = "vexriscv" if with_cpu else None,
            cpu_variant              = "lite",
            l2_size                  = 128,
            csr_data_width           = 32,
            with_uart                = with_cpu,
            uart_name                = "crossover",
            integrated_rom_size      = 0x8000 if with_cpu else 0x0000,
            integrated_main_ram_size = 0x1000 if not with_sdram else 0x0000,
            ident                    = "NeTV2 LiteX SoC",
            ident_version            = True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)
        self.add_csr("crg")

        # DNA --------------------------------------------------------------------------------------
        self.submodules.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)
        self.add_csr("dna")

        # XADC -------------------------------------------------------------------------------------
        self.submodules.xadc = XADC()
        self.add_csr("xadc")

        # ICAP -------------------------------------------------------------------------------------
        self.submodules.icap = ICAP(platform)
        self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)
        self.add_csr("icap")

        # Flash ------------------------------------------------------------------------------------
        self.submodules.flash = S7SPIFlash(platform.request("flash"), sys_clk_freq, 25e6)
        self.add_csr("flash")

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.submodules.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
                memtype      = "DDR3",
                nphases      = 4,
                sys_clk_freq = sys_clk_freq)
            self.add_csr("ddrphy")
            sdram_module = K4B2G1646F(sys_clk_freq, "1:4")
            self.register_sdram(self.ddrphy,
                geom_settings   = sdram_module.geom_settings,
                timing_settings = sdram_module.timing_settings)

        # Etherbone --------------------------------------------------------------------------------
        if with_etherbone:
            # ethphy
            self.submodules.ethphy = LiteEthPHYRMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            self.add_csr("ethphy")
            # ethcore
            self.submodules.ethcore = LiteEthUDPIPCore(
                phy         = self.ethphy,
                mac_address = 0x10e2d5000000,
                ip_address  = "192.168.1.50",
                clk_freq    = self.clk_freq)
            # etherbone
            self.submodules.etherbone = LiteEthEtherbone(self.ethcore.udp, 1234)
            self.add_wb_master(self.etherbone.wishbone.bus)
            # timing constraints
            self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, 1e9/50e6)
            self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, 1e9/50e6)
            self.platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.ethphy.crg.cd_eth_rx.clk,
                self.ethphy.crg.cd_eth_tx.clk)

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            # PHY ----------------------------------------------------------------------------------
            self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                data_width = 64,
                bar0_size  = 0x20000)
            platform.add_false_path_constraint(self.crg.cd_sys.clk, self.pcie_phy.cd_pcie.clk)
            self.add_csr("pcie_phy")

            # Endpoint -----------------------------------------------------------------------------
            self.submodules.pcie_endpoint = LitePCIeEndpoint(self.pcie_phy)

            # Wishbone bridge ----------------------------------------------------------------------
            self.submodules.pcie_bridge = LitePCIeWishboneBridge(self.pcie_endpoint,
                base_address = self.mem_map["csr"])
            self.add_wb_master(self.pcie_bridge.wishbone)

            # DMA0 ---------------------------------------------------------------------------------
            self.submodules.pcie_dma0 = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint,
                with_buffering = True, buffering_depth=1024,
                with_loopback  = True)
            self.add_csr("pcie_dma0")

            # DMA1 ---------------------------------------------------------------------------------
            self.submodules.pcie_dma1 = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint,
                with_buffering = True, buffering_depth=1024,
                with_loopback  = True)
            self.add_csr("pcie_dma1")

            self.add_constant("DMA_CHANNELS", 2)

            # MSI ----------------------------------------------------------------------------------
            self.submodules.pcie_msi = LitePCIeMSI()
            self.add_csr("pcie_msi")
            self.comb += self.pcie_msi.source.connect(self.pcie_phy.msi)
            self.interrupts = {
                "PCIE_DMA0_WRITER":    self.pcie_dma0.writer.irq,
                "PCIE_DMA0_READER":    self.pcie_dma0.reader.irq,
                "PCIE_DMA1_WRITER":    self.pcie_dma1.writer.irq,
                "PCIE_DMA1_READER":    self.pcie_dma1.reader.irq,
            }
            for i, (k, v) in enumerate(sorted(self.interrupts.items())):
                self.comb += self.pcie_msi.irqs[i].eq(v)
                self.add_constant(k + "_INTERRUPT", i)

            # FIXME : Dummy counter capture, connect to HDMI In ------------------------------------
            pcie_dma0_counter = Signal(32)
            self.sync += [
                self.pcie_dma0.sink.valid.eq(1),
                If(self.pcie_dma0.sink.ready,
                    pcie_dma0_counter.eq(pcie_dma0_counter + 1)
                ),
                self.pcie_dma0.sink.data.eq(pcie_dma0_counter)
            ]

            pcie_dma1_counter = Signal(32)
            self.sync += [
                self.pcie_dma1.sink.valid.eq(1),
                If(self.pcie_dma1.sink.ready,
                    pcie_dma1_counter.eq(pcie_dma1_counter + 2)
                ),
                self.pcie_dma1.sink.data.eq(pcie_dma1_counter)
            ]

        # HDMI In 0 --------------------------------------------------------------------------------
        if with_hdmi_in0:
            hdmi_in0_pads = platform.request("hdmi_in", 0)
            self.submodules.hdmi_in0_freq = FreqMeter(period=sys_clk_freq)
            self.add_csr("hdmi_in0_freq")
            self.submodules.hdmi_in0 = HDMIIn(
                pads       = hdmi_in0_pads,
                dram_port  = self.sdram.crossbar.get_port(mode="write"),
                fifo_depth = 512,
                device     = "xc7",
                split_mmcm = True)
            self.add_csr("hdmi_in0")
            self.add_csr("hdmi_in0_edid_mem")
            self.comb += self.hdmi_in0_freq.clk.eq(self.hdmi_in0.clocking.cd_pix.clk),
            platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.hdmi_in0.clocking.cd_pix.clk,
                self.hdmi_in0.clocking.cd_pix1p25x.clk,
                self.hdmi_in0.clocking.cd_pix5x.clk)
            self.platform.add_period_constraint(platform.lookup_request("hdmi_in", 0).clk_p, 1e9/74.25e6)

        # HDMI Out 0 -------------------------------------------------------------------------------
        if with_hdmi_out0:
            self.submodules.hdmi_out0 = VideoOut(
                device     = platform.device,
                pads       = platform.request("hdmi_out", 0),
                dram_port  = self.sdram.crossbar.get_port(
                    mode         = "read",
                    data_width   = 16,
                    clock_domain = "hdmi_out0_pix",
                    reverse      = True),
                mode       = "ycbcr422",
                fifo_depth = 512)
            self.add_csr("hdmi_out0")
            platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.hdmi_out0.driver.clocking.cd_pix.clk,
                self.hdmi_out0.driver.clocking.cd_pix5x.clk)

    def generate_software_headers(self):
        csr_header = get_csr_header(self.csr_regions, self.constants, with_access_functions=False)
        tools.write_to_file(os.path.join("software", "kernel", "csr.h"), csr_header)
        soc_header = get_soc_header(self.constants, with_access_functions=False)
        tools.write_to_file(os.path.join("software", "kernel", "soc.h"), soc_header)
        mem_header = get_mem_header(self.mem_regions)
        tools.write_to_file(os.path.join("software", "kernel", "mem.h"), mem_header)

# Build --------------------------------------------------------------------------------------------

def main():
    with open("README") as f:
        description = [str(f.readline()) for i in range(7)]
    parser = argparse.ArgumentParser(description="".join(description[0:]), formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load bitstream")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream")
    args = parser.parse_args()

    if args.load:
        from litex.build.openocd import OpenOCD
        prog = OpenOCD("openocd/openocd.cfg")
        prog.load_bitstream("build/gateware/top.bit")
        exit()

    if args.flash:
        from litex.build.openocd import OpenOCD
        prog = OpenOCD("openocd/openocd.cfg", flash_proxy_basename="openocd/bscan_spi_xc7a35t.bit")
        prog.set_flash_proxy_dir(".")
        prog.flash(0, "build/gateware/top.bin")
        exit()

    platform = netv2.Platform()
    soc      = NeTV2(platform)
    builder  = Builder(soc, output_dir="build", csr_csv="test/csr.csv")
    builder.build(run=args.build)
    soc.generate_software_headers()


if __name__ == "__main__":
    main()
