from litex.gen import *

from litex.build.tools import write_to_file
from litex.build.generic_platform import *
from litex.build.xilinx.platform import XilinxPlatform

from litex.soc.integration.soc_core import SoCCore
from litex.soc.cores.uart import UARTWishboneBridge
from litex.soc.integration import cpu_interface

from litescope import LiteScopeAnalyzer, LiteScopeIO


_io = [
    ("clock", 0, Pins(1)),
    ("reset", 1, Pins(1)),
    ("serial", 0,
        Subsignal("tx", Pins(1)),
        Subsignal("rx", Pins(1)),
    ),
    ("bus", 0, Pins(128)),
    ("i", 0, Pins(16)),
    ("o", 0, Pins(16))
]

class CorePlatform(XilinxPlatform):
    def __init__(self):
        XilinxPlatform.__init__(self, "", _io)


class Core(SoCCore):
    platform = CorePlatform()
    csr_map = {
        "analyzer": 16,
        "io":       17
    }
    csr_map.update(SoCCore.csr_map)

    def __init__(self, platform, clk_freq=int(100e6)):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.comb += [
            self.cd_sys.clk.eq(platform.request("clock")),
            self.cd_sys.rst.eq(platform.request("reset"))
        ]
        SoCCore.__init__(self, platform, clk_freq,
            cpu_type=None,
            csr_data_width=32,
            with_uart=False,
            with_timer=False
        )
        self.add_cpu_or_bridge(UARTWishboneBridge(platform.request("serial"), clk_freq))
        self.add_wb_master(self.cpu_or_bridge.wishbone)
        self.submodules.analyzer = LiteScopeAnalyzer(platform.request("bus"), 512)
        self.submodules.io = LiteScopeIO(16)
        self.comb += [
            self.io.input.eq(platform.request("i")),
            platform.request("o").eq(self.io.output),
        ]


# define platform/core
platform = CorePlatform()
core = Core(platform)

# generate verilog
v_output = platform.get_verilog(core, name="litescope")
v_output.write("litescope.v")

# generate csr.csv
memory_regions = core.get_memory_regions()
csr_regions = core.get_csr_regions()
constants = core.get_constants()
write_to_file("csr.csv", cpu_interface.get_csr_csv(csr_regions, constants, memory_regions))

# generate analyzer.csv
core.analyzer.export_csv(v_output.ns, "analyzer.csv")
