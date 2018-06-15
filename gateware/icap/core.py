import os

from migen import *

from litex.soc.interconnect.csr import *


class ICAP(Module, AutoCSR):
    def __init__(self, platform):
        self.addr = CSRStorage(5)
        self.data = CSRStorage(32)
        self.send = CSR()
        self.done = CSRStatus()

        # # #

        stb = Signal()
        ack = Signal()

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.status.eq(1),
            If(self.send.re,
                NextState("SEND")
            )
        )
        fsm.act("SEND",
            stb.eq(1),
            If(ack,
                NextState("IDLE")
            )
        )
        self.specials += [
            Instance("wbicapetwo",
                i_i_clk=ClockSignal(),
                i_i_wb_cyc=stb,
                i_i_wb_stb=stb,
                i_i_wb_we=1,
                i_i_wb_addr=self.addr.storage,
                i_i_wb_data=self.data.storage,
                o_o_wb_ack=ack
            )
        ]

        file_path = os.path.abspath(os.path.dirname(__file__))
        platform.add_source(os.path.join(file_path, "wbicapetwo.v"))
