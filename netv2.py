#!/usr/bin/env python3

import sys
import os
import math

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.misc import timeline

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.integration.cpu_interface import get_csr_header
from litex.soc.cores.clock import *
from litex.soc.cores import dna, xadc, timer, uart
from litex.soc.cores.frequency_meter import FrequencyMeter

from litedram.modules import MT41J128M16
from litedram.phy import a7ddrphy
from litedram.core import ControllerSettings

from liteeth.common import convert_ip
from liteeth.phy.rmii import LiteEthPHYRMII
from liteeth.core.mac import LiteEthMAC
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litesdcard.phy import SDPHY
from litesdcard.clocker import SDClockerS7
from litesdcard.core import SDCore
from litesdcard.bist import BISTBlockGenerator, BISTBlockChecker

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
from litepcie.frontend.dma import LitePCIeDMA
from litepcie.frontend.wishbone import LitePCIeWishboneBridge

from liteiclink.transceiver.gtp_7series import GTPQuadPLL, GTP

from litevideo.input import HDMIIn
from litevideo.output import VideoOut

from gateware.icap import ICAP
from gateware.flash import Flash

_io = [
    # clock
    ("clk50", 0, Pins("J19"), IOStandard("LVCMOS33")),

    # leds
    ("user_led", 0, Pins("M21"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("N20"), IOStandard("LVCMOS33")),
    ("user_led", 2, Pins("L21"), IOStandard("LVCMOS33")),
    ("user_led", 3, Pins("AA21"), IOStandard("LVCMOS33")),
    ("user_led", 4, Pins("R19"), IOStandard("LVCMOS33")),
    ("user_led", 5, Pins("M16"), IOStandard("LVCMOS33")),

    # flash
    ("flash", 0,
        Subsignal("cs_n", Pins("T19")),
        Subsignal("mosi", Pins("P22")),
        Subsignal("miso", Pins("R22")),
        Subsignal("vpp", Pins("P21")),
        Subsignal("hold", Pins("R21")),
        IOStandard("LVCMOS33")
    ),

    # serial
    ("serial", 0,
        Subsignal("tx", Pins("E14")),
        Subsignal("rx", Pins("E13")),
        IOStandard("LVCMOS33"),
    ),

    # dram
    ("ddram", 0,
        Subsignal("a", Pins(
            "U6 V4 W5 V5 AA1 Y2 AB1 AB3",
            "AB2 Y3 W6 Y1 V2 AA3"
            ),
            IOStandard("SSTL15")),
        Subsignal("ba", Pins("U5 W4 V7"), IOStandard("SSTL15")),
        Subsignal("ras_n", Pins("Y9"), IOStandard("SSTL15")),
        Subsignal("cas_n", Pins("Y7"), IOStandard("SSTL15")),
        Subsignal("we_n", Pins("V8"), IOStandard("SSTL15")),
        Subsignal("dm", Pins("M5 L3"), IOStandard("SSTL15")),
        Subsignal("dq", Pins(
            "N2 M6 P1 N5 P2 N4 R1 P6 "
            "K3 M2 K4 M3 J6 L5 J4 K6 "
            ),
            IOStandard("SSTL15"),
            Misc("IN_TERM=UNTUNED_SPLIT_50")),
        Subsignal("dqs_p", Pins("P5 M1"), IOStandard("DIFF_SSTL15")),
        Subsignal("dqs_n", Pins("P4 L1"), IOStandard("DIFF_SSTL15")),
        Subsignal("clk_p", Pins("R3"), IOStandard("DIFF_SSTL15")),
        Subsignal("clk_n", Pins("R2"), IOStandard("DIFF_SSTL15")),
        Subsignal("cke", Pins("Y8"), IOStandard("SSTL15")),
        Subsignal("odt", Pins("W9"), IOStandard("SSTL15")),
        Subsignal("reset_n", Pins("AB5"), IOStandard("LVCMOS15")),
        Subsignal("cs_n", Pins("V9"), IOStandard("SSTL15")),
        Misc("SLEW=FAST"),
    ),

    # pcie
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("E18"), IOStandard("LVCMOS33")),
        Subsignal("clk_p", Pins("F10")),
        Subsignal("clk_n", Pins("E10")),
        Subsignal("rx_p", Pins("D11")),
        Subsignal("rx_n", Pins("C11")),
        Subsignal("tx_p", Pins("D5")),
        Subsignal("tx_n", Pins("C5"))
    ),

    ("pcie_x2", 0,
        Subsignal("rst_n", Pins("E18"), IOStandard("LVCMOS33")),
        Subsignal("clk_p", Pins("F10")),
        Subsignal("clk_n", Pins("E10")),
        Subsignal("rx_p", Pins("D11 B10")),
        Subsignal("rx_n", Pins("C11 A10")),
        Subsignal("tx_p", Pins("D5 B6")),
        Subsignal("tx_n", Pins("C5 A6"))
    ),

    ("pcie_x4", 0,
        Subsignal("rst_n", Pins("E18"), IOStandard("LVCMOS33")),
        Subsignal("clk_p", Pins("F10")),
        Subsignal("clk_n", Pins("E10")),
        Subsignal("rx_p", Pins("D11 B10 D9 B8")),
        Subsignal("rx_n", Pins("C11 A10 C9 A8")),
        Subsignal("tx_p", Pins("D5 B6 D7 B4")),
        Subsignal("tx_n", Pins("C5 A6 C7 A4"))
    ),

    # interboard communication
    ("interboard_comm_tx", 0,
        Subsignal("p", Pins("D5")),
        Subsignal("n", Pins("C5"))
    ),
    ("interboard_comm_rx", 0,
        Subsignal("p", Pins("D11")),
        Subsignal("n", Pins("C11"))
    ),

    # ethernet
    ("eth_clocks", 0,
        Subsignal("ref_clk", Pins("D17")),
        IOStandard("LVCMOS33"),
    ),

    ("eth", 0,
        Subsignal("rst_n", Pins("F16")),
        Subsignal("rx_data", Pins("A20 B18")),
        Subsignal("crs_dv", Pins("C20")),
        Subsignal("tx_en", Pins("A19")),
        Subsignal("tx_data", Pins("C18 C19")),
        Subsignal("mdc", Pins("F14")),
        Subsignal("mdio", Pins("F13")),
        Subsignal("rx_er", Pins("B20")),
        Subsignal("int_n", Pins("D21")),
        IOStandard("LVCMOS33")
     ),

     # sdcard
     ("sdcard", 0,
        Subsignal("data", Pins("L15 L16 K14 M13"), Misc("PULLUP True")),
        Subsignal("cmd", Pins("L13"), Misc("PULLUP True")),
        Subsignal("clk", Pins("K18")),
        IOStandard("LVCMOS33"), Misc("SLEW=FAST")
    ),

    # hdmi in
    ("hdmi_in", 0,
        Subsignal("clk_p", Pins("L19"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("clk_n", Pins("L20"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data0_p", Pins("K21"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data0_n", Pins("K22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data1_p", Pins("J20"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data1_n", Pins("J21"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data2_p", Pins("J22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data2_n", Pins("H22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("scl", Pins("T18"), IOStandard("LVCMOS33")),
        Subsignal("sda", Pins("V18"), IOStandard("LVCMOS33")),
    ),

    ("hdmi_in", 1,
        Subsignal("clk_p", Pins("Y18"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("clk_n", Pins("Y19"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data0_p", Pins("AA18"), IOStandard("TMDS_33")),
        Subsignal("data0_n", Pins("AB18"), IOStandard("TMDS_33")),
        Subsignal("data1_p", Pins("AA19"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data1_n", Pins("AB20"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data2_p", Pins("AB21"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data2_n", Pins("AB22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("scl", Pins("W17"), IOStandard("LVCMOS33"), Inverted()),
        Subsignal("sda", Pins("R17"), IOStandard("LVCMOS33")),
    ),

    # hdmi out
    ("hdmi_out", 0,
        Subsignal("clk_p", Pins("W19"), Inverted(), IOStandard("TMDS_33")),
        Subsignal("clk_n", Pins("W20"), Inverted(), IOStandard("TMDS_33")),
        Subsignal("data0_p", Pins("W21"), IOStandard("TMDS_33")),
        Subsignal("data0_n", Pins("W22"), IOStandard("TMDS_33")),
        Subsignal("data1_p", Pins("U20"), IOStandard("TMDS_33")),
        Subsignal("data1_n", Pins("V20"), IOStandard("TMDS_33")),
        Subsignal("data2_p", Pins("T21"), IOStandard("TMDS_33")),
        Subsignal("data2_n", Pins("U21"), IOStandard("TMDS_33"))
    ),

    ("hdmi_out", 1,
        Subsignal("clk_p", Pins("G21"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("clk_n", Pins("G22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data0_p", Pins("E22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data0_n", Pins("D22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data1_p", Pins("C22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data1_n", Pins("B22"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data2_p", Pins("B21"), IOStandard("TMDS_33"), Inverted()),
        Subsignal("data2_n", Pins("A21"), IOStandard("TMDS_33"), Inverted()),
    ),
]


class Platform(XilinxPlatform):
    def __init__(self):
        XilinxPlatform.__init__(self, "xc7a35t-fgg484-2", _io, toolchain="vivado")
        self.toolchain.bitstream_commands = [
            "set_property CONFIG_VOLTAGE 3.3 [current_design]",
            "set_property CFGBVS VCCO [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 40 [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
        ]
        self.toolchain.additional_commands = \
            ["write_cfgmem -verbose -force -format bin -interface spix4 -size 16 "
             "-loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin"]


    def do_finalize(self, fragment):
        XilinxPlatform.do_finalize(self, fragment)
        self.add_platform_command("""create_clock -name s7pciephy_pcie_clk -period 10 [get_nets pcie_x2_clk_p]""")
        from gateware import constraints
        constraints.apply_xilinx_pcie_constraints(self)


def csr_map_update(csr_map, csr_peripherals):
    csr_map.update(dict((n, v)
        for v, n in enumerate(csr_peripherals, start=max(csr_map.values()) + 1)))


def period_ns(freq):
    return 1e9/freq


class CRG(Module, AutoCSR):
    def __init__(self, platform, sys_clk_freq):
        self.reset = CSR()

        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sys4x = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_clk200 = ClockDomain()
        self.clock_domains.cd_clk100 = ClockDomain()
        self.clock_domains.cd_eth = ClockDomain()

        clk50 = platform.request("clk50")
        platform.add_period_constraint(clk50, period_ns(50e6))

        soft_reset = Signal()
        self.sync += timeline(self.reset.re, [(1024, [soft_reset.eq(1)])])

        self.submodules.pll = pll = S7PLL()
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_sys4x, 4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_clk200, 200e6)
        pll.create_clkout(self.cd_clk100, 100e6)
        pll.create_clkout(self.cd_eth, 50e6)

        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_clk200)


class NeTV2SoC(SoCSDRAM):
    csr_peripherals = [
        "crg",
        "dna",
        "xadc",
        "flash",
        "icap",

        "ddrphy",

        "ethphy",
        "ethmac",

        "pcie_phy",
        "pcie_dma0",
        "pcie_dma1",
        "pcie_msi",

        "hdmi_out0",
        "hdmi_in0",
        "hdmi_in0_freq",
        "hdmi_in0_edid_mem",
    ]
    csr_map_update(SoCSDRAM.csr_map, csr_peripherals)

    interrupt_map = {
        "ethmac": 3,
    }
    interrupt_map.update(SoCSDRAM.interrupt_map)

    mem_map = {
        "ethmac": 0x30000000,
    }
    mem_map.update(SoCSDRAM.mem_map)

    mem_map["csr"] = 0x00000000
    mem_map["rom"] = 0x20000000

    def __init__(self, platform,
        with_sdram=True,
        with_ethernet=True,
        with_pcie=True,
        with_hdmi_in0=True, with_hdmi_out0=True,
        with_hdmi_in1=False, with_hdmi_out1=False,
        with_interboard_communication=False):
        assert not (with_pcie and with_interboard_communication)
        sys_clk_freq = int(100e6)
        SoCSDRAM.__init__(self, platform, sys_clk_freq,
            cpu_type="vexriscv", l2_size=32,
            csr_data_width=8, csr_address_width=15,
            integrated_rom_size=0x8000,
            integrated_sram_size=0x4000,
            integrated_main_ram_size=0x8000 if not with_sdram else 0,
            ident="NeTV2 LiteX Test SoC", ident_version=True,
            reserve_nmi_interrupt=False)

        # crg
        self.submodules.crg = CRG(platform, sys_clk_freq)

        # dnax
        self.submodules.dna = dna.DNA()

        # xadc
        self.submodules.xadc = xadc.XADC()

        # icap
        self.submodules.icap = ICAP(platform)

        # flash
        self.submodules.flash = Flash(platform.request("flash"), div=math.ceil(sys_clk_freq/25e6))

        # sdram
        if with_sdram:
            self.submodules.ddrphy = a7ddrphy.A7DDRPHY(platform.request("ddram"),
                sys_clk_freq=sys_clk_freq, iodelay_clk_freq=200e6)
            sdram_module = MT41J128M16(sys_clk_freq, "1:4")
            self.register_sdram(self.ddrphy,
                                sdram_module.geom_settings,
                                sdram_module.timing_settings,
                                controller_settings=ControllerSettings(with_bandwidth=True,
                                                                       cmd_buffer_depth=8,
                                                                       with_refresh=True))
        # ethernet
        if with_ethernet:
            self.submodules.ethphy = LiteEthPHYRMII(self.platform.request("eth_clocks"),
                                                    self.platform.request("eth"))
            self.submodules.ethmac = LiteEthMAC(phy=self.ethphy, dw=32, interface="wishbone")
            self.add_wb_slave(mem_decoder(self.mem_map["ethmac"]), self.ethmac.bus)
            self.add_memory_region("ethmac", self.mem_map["ethmac"] | self.shadow_base, 0x2000)

            self.crg.cd_eth.clk.attr.add("keep")
            self.platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.crg.cd_eth.clk)

        # pcie
        if with_pcie:
            # pcie phy
            self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x2"))
            platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.pcie_phy.cd_pcie.clk)

            # pcie endpoint
            self.submodules.pcie_endpoint = LitePCIeEndpoint(self.pcie_phy, with_reordering=True)

            # pcie wishbone bridge
            self.submodules.pcie_bridge = LitePCIeWishboneBridge(self.pcie_endpoint, lambda a: 1, shadow_base=0x80000000)
            self.add_wb_master(self.pcie_bridge.wishbone)

            # pcie dma
            self.submodules.pcie_dma0 = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint, with_loopback=True)

            # pcie msi
            self.submodules.pcie_msi = LitePCIeMSI()
            self.comb += self.pcie_msi.source.connect(self.pcie_phy.msi)
            self.interrupts = {
                "PCIE_DMA0_WRITER":    self.pcie_dma0.writer.irq,
                "PCIE_DMA0_READER":    self.pcie_dma0.reader.irq
            }
            for i, (k, v) in enumerate(sorted(self.interrupts.items())):
                self.comb += self.pcie_msi.irqs[i].eq(v)
                self.add_constant(k + "_INTERRUPT", i)

        # interboard communication
        if with_interboard_communication:
            self.clock_domains.cd_refclk = ClockDomain()
            self.submodules.refclk_pll = refclk_pll = S7PLL()
            refclk_pll.register_clkin(platform.lookup_request("clk50"), 50e6)
            refclk_pll.create_clkout(self.cd_refclk, 125e6)

            platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

            # qpll
            qpll = GTPQuadPLL(ClockSignal("refclk"), 125e6, 1.25e9)
            print(qpll)
            self.submodules += qpll

            # gtp
            gtp = GTP(qpll,
                platform.request("interboard_comm_tx"),
                platform.request("interboard_comm_rx"),
                sys_clk_freq,
                clock_aligner=True, internal_loopback=False)
            self.submodules += gtp

            counter = Signal(32)
            self.sync.tx += counter.eq(counter + 1)

            # send counter to other-board
            self.comb += [
                gtp.encoder.k[0].eq(1),
                gtp.encoder.d[0].eq((5 << 5) | 28),
                gtp.encoder.k[1].eq(0),
                gtp.encoder.d[1].eq(counter[26:])
            ]

            # receive counter and display it on leds
            self.comb += [
                platform.request("user_led", 3).eq(gtp.rx_ready),
                platform.request("user_led", 4).eq(gtp.decoders[1].d[0]),
                platform.request("user_led", 5).eq(gtp.decoders[1].d[1])
            ]

            gtp.cd_tx.clk.attr.add("keep")
            gtp.cd_rx.clk.attr.add("keep")
            platform.add_period_constraint(gtp.cd_tx.clk, 1e9/gtp.tx_clk_freq)
            platform.add_period_constraint(gtp.cd_rx.clk, 1e9/gtp.tx_clk_freq)
            self.platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                gtp.cd_tx.clk,
                gtp.cd_rx.clk)

        # hdmi in 0
        if with_hdmi_in0:
            hdmi_in0_pads = platform.request("hdmi_in", 0)
            self.submodules.hdmi_in0_freq = FrequencyMeter(period=sys_clk_freq)
            self.submodules.hdmi_in0 = HDMIIn(
                hdmi_in0_pads,
                self.sdram.crossbar.get_port(mode="write"),
                fifo_depth=512,
                device="xc7",
                split_mmcm=True)
            self.comb += self.hdmi_in0_freq.clk.eq(self.hdmi_in0.clocking.cd_pix.clk),
            for clk in [self.hdmi_in0.clocking.cd_pix.clk,
                        self.hdmi_in0.clocking.cd_pix1p25x.clk,
                        self.hdmi_in0.clocking.cd_pix5x.clk]:
                self.platform.add_false_path_constraints(self.crg.cd_sys.clk, clk)
            self.platform.add_period_constraint(platform.lookup_request("hdmi_in", 0).clk_p, period_ns(148.5e6))

        # hdmi out 0
        if with_hdmi_out0:
            hdmi_out0_dram_port = self.sdram.crossbar.get_port(mode="read", dw=16, cd="hdmi_out0_pix", reverse=True)
            self.submodules.hdmi_out0 = VideoOut(
                platform.device,
                platform.request("hdmi_out", 0),
                hdmi_out0_dram_port,
                "ycbcr422",
                fifo_depth=4096)
            for clk in [self.hdmi_out0.driver.clocking.cd_pix.clk,
                        self.hdmi_out0.driver.clocking.cd_pix5x.clk]:
                self.platform.add_false_path_constraints(self.crg.cd_sys.clk, clk)

        # hdmi in 1
        if with_hdmi_in1:
            hdmi_in1_pads = platform.request("hdmi_in", 1)
            self.submodules.hdmi_in1_freq = FrequencyMeter(period=sys_clk_freq)
            self.submodules.hdmi_in1 = HDMIIn(
                hdmi_in1_pads,
                self.sdram.crossbar.get_port(mode="write"),
                fifo_depth=512,
                device="xc7",
                split_mmcm=True)
            self.comb += self.hdmi_in1_freq.clk.eq(self.hdmi_in1.clocking.cd_pix.clk),
            for clk in [self.hdmi_in1.clocking.cd_pix.clk,
                        self.hdmi_in1.clocking.cd_pix1p25x.clk,
                        self.hdmi_in1.clocking.cd_pix5x.clk]:
                self.platform.add_false_path_constraints(self.crg.cd_sys.clk, clk)
            self.platform.add_period_constraint(platform.lookup_request("hdmi_in", 1).clk_p, period_ns(148.5e6))

        # hdmi out 1
        if with_hdmi_out1:
            hdmi_out1_dram_port = self.sdram.crossbar.get_port(mode="read", dw=16, cd="hdmi_out1_pix", reverse=True)
            self.submodules.hdmi_out1 = VideoOut(
                platform.device,
                platform.request("hdmi_out", 1),
                hdmi_out1_dram_port,
                "ycbcr422",
                fifo_depth=4096)
            for clk in [self.hdmi_out1.driver.clocking.cd_pix.clk,
                        self.hdmi_out1.driver.clocking.cd_pix5x.clk]:
                self.platform.add_false_path_constraints(self.crg.cd_sys.clk, clk)

        # led blinking (sys)
        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        self.comb += platform.request("user_led", 0).eq(sys_counter[26])


        # led blinking (pcie)
        if with_pcie:
            pcie_counter = Signal(32)
            self.sync.pcie += pcie_counter.eq(pcie_counter + 1)
            self.comb += platform.request("user_led", 1).eq(pcie_counter[26])


    def generate_software_header(self):
        csr_header = get_csr_header(self.get_csr_regions(),
                                    self.get_constants(),
                                    with_access_functions=False,
                                    with_shadow_base=False)
        tools.write_to_file(os.path.join("software", "pcie", "kernel", "csr.h"), csr_header)


class NeTV2MVPSoC(NeTV2SoC):
    def __init__(self, platform):
        NeTV2SoC.__init__(self, platform, with_ethernet=False)


def main():
    platform = Platform()
    if "mvp" in sys.argv[1:]:
        soc = NeTV2MVPSoC(platform)
    else:
        soc = NeTV2SoC(platform)
    if "no-compile" in sys.argv[1:]:
        compile_gateware = False
    else:
        compile_gateware = True
    builder = Builder(soc, output_dir="build", csr_csv="test/csr.csv", compile_gateware=compile_gateware)
    vns = builder.build(build_name="netv2")
    soc.generate_software_header()

if __name__ == "__main__":
    main()
