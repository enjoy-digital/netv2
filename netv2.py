#!/usr/bin/env python3

import sys
import os
import math
import argparse

from migen import *
from migen.genlib.misc import WaitTimer

from litex.build import tools

from litex_boards.platforms import netv2

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
from litex.soc.cores.led import LedChaser

from litedram.modules import K4B2G1646F
from litedram.phy import s7ddrphy
from litedram.frontend.dma import LiteDRAMDMAReader
from litedram.frontend.dma import LiteDRAMDMAWriter

from liteeth.phy.rmii import LiteEthPHYRMII

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

from litevideo.input import HDMIIn
from litevideo.output import VideoOut

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys       = ClockDomain()
        self.clock_domains.cd_sys4x     = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_idelay    = ClockDomain()
        self.clock_domains.cd_clk100    = ClockDomain()
        self.clock_domains.cd_eth       = ClockDomain()

        # Clk/Rst
        clk50 = platform.request("clk50")

        # PLL
        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys,       sys_clk_freq)
        pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_idelay,    200e6)
        pll.create_clkout(self.cd_clk100,    100e6)
        pll.create_clkout(self.cd_eth,       50e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# NeTV2 --------------------------------------------------------------------------------------------

class NeTV2(SoCSDRAM):
    def __init__(self, platform,
        with_cpu        = True,
        with_sdram      = True,
        with_etherbone  = False,
        with_pcie       = True,
        with_sdram_dmas = False,
        with_hdmi_in0   = False,
        with_hdmi_out0  = False):
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
        self.submodules.spiflash = S7SPIFlash(platform.request("spiflash"), sys_clk_freq, 25e6)
        self.add_csr("spiflash")

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.submodules.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
                memtype      = "DDR3",
                nphases      = 4,
                sys_clk_freq = sys_clk_freq)
            self.add_csr("ddrphy")
            self.add_sdram("sdram",
                phy    = self.ddrphy,
                module = K4B2G1646F(sys_clk_freq, "1:4"),
                origin = self.mem_map["main_ram"],
            )

        # Etherbone --------------------------------------------------------------------------------
        if with_etherbone:
            self.submodules.ethphy = LiteEthPHYRMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            self.add_csr("ethphy")
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address="192.168.1.50")

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)
        self.add_csr("leds")


        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                data_width = 64,
                bar0_size  = 0x20000)
            self.add_csr("pcie_phy")
            self.add_pcie(phy=self.pcie_phy, ndmas=2, max_pending_requests=2)

        # SDRAM DMAs -------------------------------------------------------------------------------
        if with_sdram_dmas:
            self.submodules.sdram_reader = LiteDRAMDMAReader(self.sdram.crossbar.get_port())
            self.sdram_reader.add_csr()
            self.add_csr("sdram_reader")

            self.submodules.sdram_writer = LiteDRAMDMAWriter(self.sdram.crossbar.get_port())
            self.sdram_writer.add_csr()
            self.add_csr("sdram_writer")

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

# Build --------------------------------------------------------------------------------------------

def main():
    with open("README") as f:
        description = [str(f.readline()) for i in range(7)]
    parser = argparse.ArgumentParser(description="".join(description[0:]), formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load bitstream")
    parser.add_argument("--flash",  action="store_true", help="Flash bitstream")
    parser.add_argument("--driver", action="store_true", help="Generate PCIe driver")
    args = parser.parse_args()

    platform = netv2.Platform()
    soc      = NeTV2(platform)
    builder  = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bin"))


if __name__ == "__main__":
    main()
