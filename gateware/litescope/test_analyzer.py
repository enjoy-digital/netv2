#!/usr/bin/env python3

from litex import RemoteClient

from litescope import LiteScopeAnalyzerDriver

wb = RemoteClient()
wb.open()

# # #

analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
analyzer.configure_trigger(cond={})
analyzer.configure_subsampler(1)
analyzer.run(offset=128, length=256)
analyzer.wait_done()
analyzer.upload()
analyzer.save("dump.vcd")

# # #

wb.close()
