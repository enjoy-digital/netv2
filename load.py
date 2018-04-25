#!/usr/bin/env python3
import sys
if "vivado" in sys.argv[1:]:
    from litex.build.xilinx.programmer import VivadoProgrammer
    prog = VivadoProgrammer()
    prog.load_bitstream("build/gateware/top.bit")
else:
    from litex.build.openocd import OpenOCD
    prog = OpenOCD("openocd.cfg")
    prog.load("build/gateware/top.bit")
