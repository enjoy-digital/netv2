#!/usr/bin/env python3
import sys
import os

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.boards.platforms import arty

from litex.soc.integration.soc_core import mem_decoder
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.cores import dna, xadc

from litedram.modules import MT41J128M16
from litedram.phy import a7ddrphy
from litedram.core import ControllerSettings

from gateware.dma import DMAWriter, DMAReader


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
        self.clock_domains.cd_clk50 = ClockDomain()

        clk100 = platform.request("clk100")
        rst = ~platform.request("cpu_reset")

        pll_locked = Signal()
        pll_fb = Signal()
        self.pll_sys = Signal()
        pll_sys4x = Signal()
        pll_sys4x_dqs = Signal()
        pll_clk200 = Signal()
        pll_clk50 = Signal()
        self.specials += [
            Instance("PLLE2_BASE",
                     p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

                     # VCO @ 1600 MHz
                     p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=10.0,
                     p_CLKFBOUT_MULT=16, p_DIVCLK_DIVIDE=1,
                     i_CLKIN1=clk100, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

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
                     o_CLKOUT3=pll_clk200,

                     # 50MHz
                     p_CLKOUT4_DIVIDE=32, p_CLKOUT4_PHASE=0.0,
                     o_CLKOUT4=pll_clk50
            ),
            Instance("BUFG", i_I=self.pll_sys, o_O=self.cd_sys.clk),
            Instance("BUFG", i_I=pll_sys4x, o_O=self.cd_sys4x.clk),
            Instance("BUFG", i_I=pll_sys4x_dqs, o_O=self.cd_sys4x_dqs.clk),
            Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk),
            Instance("BUFG", i_I=pll_clk50, o_O=self.cd_clk50.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll_locked | rst),
            AsyncResetSynchronizer(self.cd_clk200, ~pll_locked | rst),
            AsyncResetSynchronizer(self.cd_clk50, ~pll_locked | rst),
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
            l2_size=32,
            integrated_rom_size=0x8000,
            integrated_sram_size=0x8000,
            ident="Arty DMA Test SoC",
            ident_version=True,
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


class DMATestSoC(BaseSoC):
    def __init__(self, platform, *args, **kwargs):
        BaseSoC.__init__(self, platform, *args, **kwargs)

        # # #

        # parameters
        slot_length = 1280*720*32
        slot_offset = 0x00000000
        slot0_base = slot_offset + 0*slot_length
        slot1_base = slot_offset + 1*slot_length

        # create fake pixel clock
        self.clock_domains.cd_pix = ClockDomain() # Remove once hdmi in integrated
        self.comb += [
            self.cd_pix.clk.eq(ClockSignal()),
            self.cd_pix.rst.eq(ResetSignal())
        ]

        # dram dmas
        dma_writer = DMAWriter(self.sdram.crossbar.get_port(mode="write", dw=32, cd="pix"))
        dma_writer = ClockDomainsRenamer("pix")(dma_writer)
        dma_reader = DMAReader(self.sdram.crossbar.get_port(mode="read", dw=32, cd="pix"))
        dma_reader = ClockDomainsRenamer("pix")(dma_reader)
        self.submodules += dma_writer, dma_reader

        # quick "user manual" :)
        # user_sw0 : dma writer enable
        # user_sw1 : dma writer valid
        # user sw2 : dma reader enable
        # user sw3 : dma reader ready

        # user_btn0: dma writer start
        # user_btn1: dma_reader start
        # user_btn2: error injection

        # user_led0: dma_writer idle
        # user_led1: dma_writer ready
        # user_led2: dma_reader idle
        # user_led3: dma_reader valid

        # test
        idata0 = Signal(10)
        idata1 = Signal(10)
        idata2 = Signal(10)

        self.sync.pix += [
            If(~platform.request("user_btn", 2),
                idata0.eq(idata0 + 1),
                idata1.eq(idata1 + 2),
                idata2.eq(idata2 + 4)
            ).Else(
                idata0.eq(0),
                idata1.eq(0),
                idata2.eq(0)
            )
        ]

        # dma
        self.comb += [
            # control
            dma_writer.enable.eq(platform.request("user_sw", 0)),
            dma_writer.slot0_base.eq(slot0_base),
            dma_writer.slot1_base.eq(slot1_base),
            dma_writer.length.eq(slot_length),

            # stream
            dma_writer.start.eq(platform.request("user_btn", 0)),
            platform.request("user_led", 0).eq(dma_writer.idle),
            dma_writer.sink.valid.eq(platform.request("user_sw", 1)),
            platform.request("user_led", 1).eq(dma_writer.sink.ready),
            dma_writer.sink.data[0:10].eq(idata0),
            dma_writer.sink.data[10:20].eq(idata1),
            dma_writer.sink.data[20:30].eq(idata2),
        ]

         # test
        odata0 = Signal(10)
        odata1 = Signal(10)
        odata2 = Signal(10)

        # hdmi out dma
        self.comb += [
            # control
            dma_reader.enable.eq(platform.request("user_sw", 2)),
            dma_reader.slot0_base.eq(slot0_base),
            dma_reader.slot1_base.eq(slot1_base),
            dma_reader.length.eq(slot_length),

            # stream
            dma_reader.start.eq(platform.request("user_btn", 1)),
            platform.request("user_led", 2).eq(dma_reader.idle),
            platform.request("user_led", 3).eq(dma_reader.source.valid),
            dma_reader.source.ready.eq(platform.request("user_sw", 3)),
            odata0.eq(dma_reader.source.data[0:10]),
            odata1.eq(dma_reader.source.data[10:20]),
            odata2.eq(dma_reader.source.data[20:30]),
        ]

        # check
        odata0_d = Signal(10)
        odata1_d = Signal(10)
        odata2_d = Signal(10)

        errors = platform.request("rgb_leds")

        self.sync.pix += [
            errors.r.eq(0b000),
            errors.g.eq(0b111),
            odata0_d.eq(odata0),
            odata1_d.eq(odata1),
            odata2_d.eq(odata2),
            If(odata0 != (odata0_d + 1),
                errors.r[0].eq(1),
                errors.g[0].eq(0)),
            If(odata1 != (odata1_d + 2),
                errors.r[1].eq(1),
                errors.g[1].eq(0)),
            If(odata2 != (odata2_d + 4),
                errors.r[2].eq(1),
                errors.g[2].eq(0)),
        ]

    def do_exit(self, vns):
        pass

def main():
    platform = arty.Platform()
    soc = DMATestSoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="test/csr.csv")
    vns = builder.build()
    soc.do_exit(vns)

if __name__ == "__main__":
    main()
