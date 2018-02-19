from litex.gen import *
from litex.gen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litedram.frontend.dma import LiteDRAMDMAWriter, LiteDRAMDMAReader


class DMA(Module):
    def __init__(self, mode, dram_port, fifo_depth):
        assert mode == dram_port.mode
        ashift = log2_int(dram_port.dw//8)
        awidth = dram_port.aw + ashift
        self.cd = dram_port.cd

        # control
        self.enable = Signal(reset=1) # reset to 1 if not used
        self.start = Signal(reset=1)  # i / reset to 1 if not used
        self.idle = Signal()          # o
        self.slot = Signal()          # o

        # parameters
        self.slot0_base = Signal(awidth) # in bytes
        self.slot1_base = Signal(awidth) # in bytes
        self.length = Signal(awidth)     # in bytes

        # stream
        self.endpoint = endpoint = stream.Endpoint([("data", dram_port.dw)])

        # # #

        base = Signal(dram_port.aw)
        length = Signal(dram_port.aw)
        offset = Signal(dram_port.aw)

        # slot selection
        self.comb += \
            If(self.slot,
                base.eq(self.slot1_base[ashift:])
            ).Else(
                base.eq(self.slot0_base[ashift:]))

        # length
        self.comb += length.eq(self.length[ashift:])

        # dma
        if mode == "write":
            # dma
            self.submodules.dma = dma = ResetInserter()(LiteDRAMDMAWriter(dram_port, fifo_depth, True))
            # data
            self.comb += dma.sink.data.eq(endpoint.data)
        elif mode == "read":
            # dma
            self.submodules.dma = dma = ResetInserter()(LiteDRAMDMAReader(dram_port, fifo_depth, True))
            # data
            self.comb += [
                endpoint.valid.eq(dma.source.valid),
                dma.source.ready.eq(endpoint.ready),
                endpoint.data.eq(dma.source.data)
            ]

        # control
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.idle.eq(1),
            If(self.enable & self.start,
                NextValue(offset, 0),
                NextState("RUN")
            )
        )
        fsm.act("RUN",
            If(mode == "write",
                dma.sink.valid.eq(endpoint.valid),
                endpoint.ready.eq(dma.sink.ready),
            ).Elif(mode == "read",
                dma.sink.valid.eq(1),
            ),
            If(~self.enable,
                dma.reset.eq(1),
                dram_port.flush.eq(1),
                NextState("IDLE")
            ).Elif(dma.sink.valid & dma.sink.ready,
                NextValue(offset, offset + 1),
                If(offset == (length - 1),
                    NextValue(offset, 0),
                    NextValue(self.slot, ~self.slot)
                )
            )
        )
        self.comb += dma.sink.address.eq(base + offset)


class DMAWriter(DMA):
    def __init__(self, dram_port, dw=32, fifo_depth=512):
        self.sink = stream.Endpoint([("data", dw)])

        # # #

        DMA.__init__(self, "write", dram_port, fifo_depth)
        converter = stream.Converter(dw, dram_port.dw, reverse=False)
        self.submodules += converter
        self.comb += [
            self.sink.connect(converter.sink),
            converter.source.connect(self.endpoint)
        ]


class DMAReader(DMA):
    def __init__(self, dram_port, dw=32, fifo_depth=512):
        self.source = stream.Endpoint([("data", dw)])

        # # #

        DMA.__init__(self, "read", dram_port, fifo_depth)
        converter = stream.Converter(dram_port.dw, dw, reverse=False)
        self.submodules += converter
        self.comb += [
            self.endpoint.connect(converter.sink),
            converter.source.connect(self.source)
        ]


class DMAControl(DMA, AutoCSR):
    def __init__(self, dma):
        self.enable = CSRStorage()
        self.slot0_base = CSRStorage(32)
        self.slot1_base = CSRStorage(32)
        self.length = CSRStorage(32)

        self.start = CSR()
        self.idle = CSRStatus()
        self.slot = CSRStatus()

        # # #

        self.specials += [
            MultiReg(self.enable.storage, dma.enable, dma.cd),
            MultiReg(self.slot0_base.storage, dma.slot0_base, dma.cd),
            MultiReg(self.slot1_base.storage, dma.slot1_base, dma.cd),
            MultiReg(self.length.storage, dma.length, dma.cd),

            MultiReg(dma.idle, self.idle.status),
            MultiReg(dma.slot, self.slot.status),
        ]

        start_sync = PulseSynchronizer("sys", dma.cd)
        self.submodules += start_sync
        self.comb += [
            start_sync.i.eq(self.start.re),
            dma.start.eq(start_sync.o)
        ]

        if hasattr(dma, "source"):
            self.underflows = CSRStatus(32)
            self.sync.pix += [
                If(~dma.source.valid, self.underflows.status.eq(self.underflows.status + 1))
            ]
        if hasattr(dma, "sink"):
            self.overflows = CSRStatus(32)
            self.sync.pix += [
                If(~dma.sink.ready, self.overflows.status.eq(self.overflows.status + 1))
            ]
