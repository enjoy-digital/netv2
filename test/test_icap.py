#!/usr/bin/env python3

from litex.soc.tools.remote import RemoteClient

wb = RemoteClient()
wb.open()

# # #

def icap_send(addr, data):
	wb.regs.icap_addr.write(addr)
	wb.regs.icap_data.write(data)
	wb.regs.icap_send.write(1)
	while (wb.regs.icap_done.read() == 0):
		pass

# iprog
icap_send(0x04, 0x0000000f)

# # #

wb.close()
