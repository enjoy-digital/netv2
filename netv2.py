#!/usr/bin/env python3

import sys
import os
import math

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.integration.cpu_interface import get_csr_header
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
        Subsignal("dm", Pins("G1 H4 M5 L3"), IOStandard("SSTL15")),
        Subsignal("dq", Pins(
            "C2 F1 B1 F3 A1 D2 B2 E2 "
            "J5 H3 K1 H2 J1 G2 H5 G3 "
            "N2 M6 P1 N5 P2 N4 R1 P6 "
            "K3 M2 K4 M3 J6 L5 J4 K6 "
            ),
            IOStandard("SSTL15"),
            Misc("IN_TERM=UNTUNED_SPLIT_50")),
        Subsignal("dqs_p", Pins("E1 K2 P5 M1"), IOStandard("DIFF_SSTL15")),
        Subsignal("dqs_n", Pins("D1 J2 P4 L1"), IOStandard("DIFF_SSTL15")),
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


class CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sys4x = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_clk200 = ClockDomain()
        self.clock_domains.cd_clk100 = ClockDomain()
        self.clock_domains.cd_eth = ClockDomain()

        clk50 = platform.request("clk50")
        platform.add_period_constraint(clk50, period_ns(50e6))

        pll_locked = Signal()
        pll_fb = Signal()
        pll_sys = Signal()
        pll_sys4x = Signal()
        pll_sys4x_dqs = Signal()
        pll_eth = Signal()
        pll_clk200 = Signal()
        self.specials += [
            Instance("PLLE2_BASE",
                     p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

                     # VCO @ 1600 MHz
                     p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=20.0,
                     p_CLKFBOUT_MULT=32, p_DIVCLK_DIVIDE=1,
                     i_CLKIN1=clk50, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

                     # 100 MHz
                     p_CLKOUT0_DIVIDE=16, p_CLKOUT0_PHASE=0.0,
                     o_CLKOUT0=pll_sys,

                     # 400 MHz
                     p_CLKOUT1_DIVIDE=4, p_CLKOUT1_PHASE=0.0,
                     o_CLKOUT1=pll_sys4x,

                     # 400 MHz dqs
                     p_CLKOUT2_DIVIDE=4, p_CLKOUT2_PHASE=90.0,
                     o_CLKOUT2=pll_sys4x_dqs,

                     # 50 MHz
                     p_CLKOUT3_DIVIDE=32, p_CLKOUT3_PHASE=0.0,
                     o_CLKOUT3=pll_eth,

                     # 200 MHz
                     p_CLKOUT4_DIVIDE=8, p_CLKOUT4_PHASE=0.0,
                     o_CLKOUT4=pll_clk200

            ),
            Instance("BUFG", i_I=pll_sys, o_O=self.cd_sys.clk),
            Instance("BUFG", i_I=pll_sys, o_O=self.cd_clk100.clk),
            Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk),
            Instance("BUFG", i_I=pll_sys4x, o_O=self.cd_sys4x.clk),
            Instance("BUFG", i_I=pll_sys4x_dqs, o_O=self.cd_sys4x_dqs.clk),
            Instance("BUFG", i_I=pll_eth, o_O=self.cd_eth.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll_locked),
            AsyncResetSynchronizer(self.cd_clk100, ~pll_locked),
            AsyncResetSynchronizer(self.cd_eth, ~pll_locked),
            AsyncResetSynchronizer(self.cd_clk200, ~pll_locked)
        ]

        reset_counter = Signal(4, reset=15)
        ic_reset = Signal(reset=1)
        self.sync.clk200 += \
            If(reset_counter != 0,
                reset_counter.eq(reset_counter - 1)
            ).Else(
                ic_reset.eq(0)
            )
        self.specials += Instance("IDELAYCTRL", i_REFCLK=ClockSignal("clk200"), i_RST=ic_reset)


class NeTV2SoC(SoCSDRAM):
    csr_peripherals = [
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

    SoCSDRAM.mem_map["csr"] = 0x00000000
    SoCSDRAM.mem_map["rom"] = 0x20000000

    def __init__(self, platform,
        with_sdram=True,
        with_ethernet=True,
        with_pcie=True,
        with_hdmi_in0=True, with_hdmi_out0=True):
        clk_freq = int(100e6)
        SoCSDRAM.__init__(self, platform, clk_freq,
            cpu_type="lm32",
            csr_data_width=32, csr_address_width=15,
            l2_size=64,
            integrated_rom_size=0x8000,
            integrated_sram_size=0x4000,
            integrated_main_ram_size=0x8000 if not with_sdram else 0,
            ident="NeTV2 LiteX Test SoC", ident_version="True",
            reserve_nmi_interrupt=False)

        # crg
        self.submodules.crg = CRG(platform)

        # dnax
        self.submodules.dna = dna.DNA()

        # xadc
        self.submodules.xadc = xadc.XADC()

        # icap
        self.submodules.icap = ICAP(platform)

        # flash
        self.submodules.flash = Flash(platform.request("flash"), div=math.ceil(clk_freq/25e6))

        # sdram
        if with_sdram:
            self.submodules.ddrphy = a7ddrphy.A7DDRPHY(platform.request("ddram"))
            sdram_module = MT41J128M16(self.clk_freq, "1:4")
            self.add_constant("READ_LEVELING_BITSLIP", 3)
            self.add_constant("READ_LEVELING_DELAY", 14)
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

                pix_freq = 148.50e6

        # hdmi in 0
        if with_hdmi_in0:
            hdmi_in0_pads = platform.request("hdmi_in")
            self.submodules.hdmi_in0_freq = FrequencyMeter(period=self.clk_freq)
            self.submodules.hdmi_in0 = HDMIIn(hdmi_in0_pads,
                                             self.sdram.crossbar.get_port(mode="write"),
                                             fifo_depth=512,
                                             device="xc7")
            self.comb += self.hdmi_in0_freq.clk.eq(self.hdmi_in0.clocking.cd_pix.clk),
            for clk in [self.hdmi_in0.clocking.cd_pix.clk,
                        self.hdmi_in0.clocking.cd_pix1p25x.clk,
                        self.hdmi_in0.clocking.cd_pix5x.clk]:
                self.platform.add_false_path_constraints(self.crg.cd_sys.clk, clk)

        # hdmi out 0
        if with_hdmi_out0:
            hdmi_out0_dram_port = self.sdram.crossbar.get_port(mode="read", dw=16, cd="hdmi_out0_pix", reverse=True)
            self.submodules.hdmi_out0 = VideoOut(platform.device,
                                                 platform.request("hdmi_out"),
                                                 hdmi_out0_dram_port,
                                                 "ycbcr422",
                                                 fifo_depth=4096)
            for clk in [self.hdmi_out0.driver.clocking.cd_pix.clk,
                        self.hdmi_out0.driver.clocking.cd_pix5x.clk]:
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
