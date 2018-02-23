#!/usr/bin/env python3
import sys
import os

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.integration.soc_core import mem_decoder
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.cores import dna, xadc
from litex.soc.cores.frequency_meter import FrequencyMeter

from litedram.modules import MT41J128M16
from litedram.phy import a7ddrphy
from litedram.core import ControllerSettings

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
from litepcie.frontend.dma import LitePCIeDMA
from litepcie.frontend.wishbone import LitePCIeWishboneBridge

from litevideo.input import HDMIIn
from litevideo.output import VideoOut

import cpu_interface


_io = [
    ("clk50", 0, Pins("R2"), IOStandard("LVCMOS33")),

    ("user_led", 0, Pins("U2"), IOStandard("LVCMOS33")),

    ("serial", 0,
        Subsignal("tx", Pins("M5")),
        Subsignal("rx", Pins("N6")),
        IOStandard("LVCMOS33"),
    ),

    ("ddram", 0,
        Subsignal("a", Pins(
            "U15 M17 N18 U16 R18 P18 T18 T17",
            "U17 N16 R16 N17 V17 R17"),
            IOStandard("SSTL15")),
        Subsignal("ba", Pins("T15 M16 P15"), IOStandard("SSTL15")),
        Subsignal("ras_n", Pins("L18"), IOStandard("SSTL15")),
        Subsignal("cas_n", Pins("K17"), IOStandard("SSTL15")),
        Subsignal("we_n", Pins("P16"), IOStandard("SSTL15")),
        Subsignal("dm", Pins("D9 B14 F14 C18"), IOStandard("SSTL15")),
        Subsignal("dq", Pins(
            "D11 B11 D8  C11 C8  B10 C9  A10 "
            "A15 A14 E13 B12 C13 A12 D13 A13 "
            "H18 G17 G16 F17 G14 E18 H16 H17 "
            "C17 D16 B17 E16 C16 E17 D15 D18 "
            ),
            IOStandard("SSTL15"),
            Misc("IN_TERM=UNTUNED_SPLIT_50")),
        Subsignal("dqs_p", Pins("B9 C14 G15 B16"), IOStandard("DIFF_SSTL15")),
        Subsignal("dqs_n", Pins("A9 B15 F15 A17"), IOStandard("DIFF_SSTL15")),
        Subsignal("clk_p", Pins("P14"), IOStandard("DIFF_SSTL15")),
        Subsignal("clk_n", Pins("R15"), IOStandard("DIFF_SSTL15")),
        Subsignal("cke", Pins("K15"), IOStandard("SSTL15")),
        Subsignal("odt", Pins("K18"), IOStandard("SSTL15")),
        Subsignal("reset_n", Pins("V16"), IOStandard("LVCMOS15")),
        Subsignal("cs_n", Pins("J16"), IOStandard("SSTL15")),
        Misc("SLEW=FAST"),
    ),

    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("N1"), IOStandard("LVCMOS33")),
        Subsignal("clk_p", Pins("D6")),
        Subsignal("clk_n", Pins("D5")),
        Subsignal("rx_p", Pins("E4")),
        Subsignal("rx_n", Pins("E3")),
        Subsignal("tx_p", Pins("H2")),
        Subsignal("tx_n", Pins("H1"))
    ),

    ("hdmi_in", 0,
        Subsignal("clk_p", Pins("P4"), IOStandard("TMDS_33")),
        Subsignal("clk_n", Pins("P3"), IOStandard("TMDS_33")),
        Subsignal("data0_p", Pins("U4"), IOStandard("TMDS_33")),
        Subsignal("data0_n", Pins("V4"), IOStandard("TMDS_33")),
        Subsignal("data1_p", Pins("P6"), IOStandard("TMDS_33")),
        Subsignal("data1_n", Pins("P5"), IOStandard("TMDS_33")),
        Subsignal("data2_p", Pins("R7"), IOStandard("TMDS_33")),
        Subsignal("data2_n", Pins("T7"), IOStandard("TMDS_33")),
        Subsignal("scl", Pins("K5"), IOStandard("LVCMOS33")), # FPIO5 (not connected)
        Subsignal("sda", Pins("J5"), IOStandard("LVCMOS33")), # FPIO4 (not connected)
    ),

    ("hdmi_out", 0,
        Subsignal("clk_p", Pins("R3"), IOStandard("TMDS_33")),
        Subsignal("clk_n", Pins("T2"), IOStandard("TMDS_33")),
        Subsignal("data0_p", Pins("T4"), IOStandard("TMDS_33")),
        Subsignal("data0_n", Pins("T3"), IOStandard("TMDS_33")),
        Subsignal("data1_p", Pins("U6"), IOStandard("TMDS_33")),
        Subsignal("data1_n", Pins("U5"), IOStandard("TMDS_33")),
        Subsignal("data2_p", Pins("V7"), IOStandard("TMDS_33")),
        Subsignal("data2_n", Pins("V6"), IOStandard("TMDS_33")),
    ),

    ("hdmi_sda_over_up", 0, Pins("V7"), IOStandard("LVCMOS33")),
    ("hdmi_sda_over_dn", 0, Pins("R6"), IOStandard("LVCMOS33")),
    ("hdmi_hdp_over", 0, Pins("V8"), IOStandard("LVCMOS33")),

]


class Platform(XilinxPlatform):
    def __init__(self, toolchain="vivado", programmer="vivado"):
        XilinxPlatform.__init__(self, "xc7a50t-csg325-2", _io,
                                toolchain=toolchain)

        self.add_platform_command(
            "set_property CONFIG_VOLTAGE 1.5 [current_design]")
        self.add_platform_command(
            "set_property CFGBVS GND [current_design]")
        self.add_platform_command(
            "set_property BITSTREAM.CONFIG.CONFIGRATE 22 [current_design]")
        self.add_platform_command(
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 1 [current_design]")
        self.toolchain.bitstream_commands = [
            "set_property CONFIG_VOLTAGE 1.5 [current_design]",
            "set_property CFGBVS GND [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 22 [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 1 [current_design]",
        ]
        self.toolchain.additional_commands = \
            ["write_cfgmem -verbose -force -format bin -interface spix1 -size 64 "
             "-loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin"]
        self.programmer = programmer

        self.add_platform_command("""
create_clock -name pcie_phy_clk -period 10.0 [get_pins {{pcie_phy/pcie_support_i/pcie_i/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gtp_channel.gtpe2_channel_i/TXOUTCLK}}]
""")

    def create_programmer(self):
        if self.programmer == "vivado":
            return VivadoProgrammer(flash_part="n25q128-3.3v-spi-x1_x2_x4")
        else:
            raise ValueError("{} programmer is not supported"
                             .format(self.programmer))

    def do_finalize(self, fragment):
        XilinxPlatform.do_finalize(self, fragment)


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

        clk50 = platform.request("clk50")
        rst = Signal()

        pll_locked = Signal()
        pll_fb = Signal()
        self.pll_sys = Signal()
        pll_sys4x = Signal()
        pll_sys4x_dqs = Signal()
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
                     o_CLKOUT0=self.pll_sys,

                     # 400 MHz
                     p_CLKOUT1_DIVIDE=4, p_CLKOUT1_PHASE=0.0,
                     o_CLKOUT1=pll_sys4x,

                     # 400 MHz dqs
                     p_CLKOUT2_DIVIDE=4, p_CLKOUT2_PHASE=90.0,
                     o_CLKOUT2=pll_sys4x_dqs,

                     # 200 MHz
                     p_CLKOUT3_DIVIDE=8, p_CLKOUT3_PHASE=0.0,
                     o_CLKOUT3=pll_clk200
            ),
            Instance("BUFG", i_I=self.pll_sys, o_O=self.cd_sys.clk),
            Instance("BUFG", i_I=self.pll_sys, o_O=self.cd_clk100.clk),
            Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk),
            Instance("BUFG", i_I=pll_sys4x, o_O=self.cd_sys4x.clk),
            Instance("BUFG", i_I=pll_sys4x_dqs, o_O=self.cd_sys4x_dqs.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll_locked | rst),
            AsyncResetSynchronizer(self.cd_clk200, ~pll_locked | rst),
            AsyncResetSynchronizer(self.cd_clk100, ~pll_locked | rst)
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


class BaseSoC(SoCSDRAM):
    csr_peripherals = {
        "ddrphy",
        "dna",
        "xadc",
    }
    csr_map_update(SoCSDRAM.csr_map, csr_peripherals)

    def __init__(self, platform, **kwargs):
        clk_freq = int(100e6)
        SoCSDRAM.__init__(self, platform, clk_freq,
            integrated_rom_size=0x8000,
            integrated_sram_size=0x8000,
            ident="NeTV2 LiteX Base SoC",
            reserve_nmi_interrupt=False,
            **kwargs)

        self.submodules.crg = CRG(platform)
        self.submodules.dna = dna.DNA()
        self.submodules.xadc = xadc.XADC()

        self.crg.cd_sys.clk.attr.add("keep")
        self.platform.add_period_constraint(self.crg.cd_sys.clk, period_ns(100e6))

        # sdram
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

        # common led
        self.sys_led = Signal()
        self.pcie_led = Signal()
        self.comb += platform.request("user_led", 0).eq(self.sys_led ^ self.pcie_led)

        # sys led
        sys_counter = Signal(32)
        self.sync += sys_counter.eq(sys_counter + 1)
        self.comb += self.sys_led.eq(sys_counter[26])


class PCIeSoC(BaseSoC):
    csr_map = {
        "pcie_phy": 20,
        "dma":      21,
        "msi":      22,
    }
    csr_map.update(BaseSoC.csr_map)

    BaseSoC.mem_map["csr"] = 0x00000000
    BaseSoC.mem_map["rom"] = 0x20000000

    def __init__(self, platform, **kwargs):
        BaseSoC.__init__(self, platform, csr_data_width=32, **kwargs)

        # pcie phy
        self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"))
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.pcie_phy.cd_pcie.clk)

        # pcie endpoint
        self.submodules.pcie_endpoint = LitePCIeEndpoint(self.pcie_phy, with_reordering=True)

        # pcie wishbone bridge
        self.submodules.pcie_wishbone = LitePCIeWishboneBridge(self.pcie_endpoint, lambda a: 1)
        self.add_wb_master(self.pcie_wishbone.wishbone)

        # pcie dma
        self.submodules.dma = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint, with_loopback=True)
        self.dma.source.connect(self.dma.sink)

        # pcie msi
        self.submodules.msi = LitePCIeMSI()
        self.comb += self.msi.source.connect(self.pcie_phy.msi)
        self.interrupts = {
            "DMA_WRITER":    self.dma.writer.irq,
            "DMA_READER":    self.dma.reader.irq
        }
        for i, (k, v) in enumerate(sorted(self.interrupts.items())):
            self.comb += self.msi.irqs[i].eq(v)
            self.add_constant(k + "_INTERRUPT", i)

        # pcie led
        pcie_counter = Signal(32)
        self.sync.pcie += pcie_counter.eq(pcie_counter + 1)
        self.comb += self.pcie_led.eq(pcie_counter[26])

    def generate_software_header(self):
        csr_header = get_csr_header(self.get_csr_regions(),
                                    self.get_constants(),
                                    with_access_functions=False)
        tools.write_to_file(os.path.join("software", "pcie", "kernel", "csr.h"), csr_header)

class VideoSoC(BaseSoC):
    csr_peripherals = {
        "hdmi_out0",
        "hdmi_in0",
        "hdmi_in0_freq",
        "hdmi_in0_edid_mem"
    }
    csr_map_update(BaseSoC.csr_map, csr_peripherals)

    interrupt_map = {
        "hdmi_in0": 3,
    }
    interrupt_map.update(BaseSoC.interrupt_map)

    def __init__(self, platform, *args, **kwargs):
        BaseSoC.__init__(self, platform, *args, **kwargs)

        # # #

        pix_freq = 148.50e6

        # hdmi in
        hdmi_in0_pads = platform.request("hdmi_in")
        self.submodules.hdmi_in0_freq = FrequencyMeter(period=self.clk_freq)
        self.submodules.hdmi_in0 = HDMIIn(hdmi_in0_pads,
                                         self.sdram.crossbar.get_port(mode="write"),
                                         fifo_depth=512,
                                         device="xc7")
        self.comb += self.hdmi_in0_freq.clk.eq(self.hdmi_in0.clocking.cd_pix.clk)
        self.platform.add_period_constraint(self.hdmi_in0.clocking.cd_pix.clk, period_ns(1*pix_freq))
        self.platform.add_period_constraint(self.hdmi_in0.clocking.cd_pix1p25x.clk, period_ns(1.25*pix_freq))
        self.platform.add_period_constraint(self.hdmi_in0.clocking.cd_pix5x.clk, period_ns(5*pix_freq))

        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.hdmi_in0.clocking.cd_pix.clk,
            self.hdmi_in0.clocking.cd_pix1p25x.clk,
            self.hdmi_in0.clocking.cd_pix5x.clk)

        # hdmi out
        hdmi_out0_dram_port = self.sdram.crossbar.get_port(mode="read", dw=16, cd="hdmi_out0_pix", reverse=True)
        self.submodules.hdmi_out0 = VideoOut(platform.device,
                                            platform.request("hdmi_out"),
                                            hdmi_out0_dram_port,
                                            "ycbcr422",
                                            fifo_depth=4096)

        self.platform.add_period_constraint(self.hdmi_out0.driver.clocking.cd_pix.clk, period_ns(1*pix_freq))
        self.platform.add_period_constraint(self.hdmi_out0.driver.clocking.cd_pix5x.clk, period_ns(5*pix_freq))

        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.hdmi_out0.driver.clocking.cd_pix.clk,
            self.hdmi_out0.driver.clocking.cd_pix5x.clk)

        # hdmi over
        self.comb += [
            platform.request("hdmi_sda_over_up").eq(0),
            platform.request("hdmi_sda_over_dn").eq(0),
            platform.request("hdmi_hdp_over").eq(0),
        ]


def main():
    platform = Platform()
    if len(sys.argv) < 2:
        print("missing target (base or pcie or video)")
        exit()
    if sys.argv[1] == "base":
        soc = BaseSoC(platform)
    elif sys.argv[1] == "pcie":
        soc = PCIeSoC(platform)
    elif sys.argv[1] == "video":
        soc = VideoSoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="test/csr.csv")
    vns = builder.build()

    if sys.argv[1] == "pcie":
        soc.generate_software_header()

if __name__ == "__main__":
    main()
