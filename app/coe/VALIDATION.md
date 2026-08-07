# CAN over Ethernet: Design Rationale and Validation

Companion to [README.md](README.md). The README describes how the
bridge works; this document records why each design choice was made
over its alternatives, and how the bridge was validated on hardware
against independent tooling.

## Design decisions

### Transport: IEEE 1722 NTSCF with ACF-CAN

Options evaluated for carrying CAN over the T1 link:

| Option | Verdict |
|---|---|
| cannelloni over UDP | Worked (an earlier revision shipped it), but a private framing on IP with no path to stream identification, VLAN priority, or time-aware shaping |
| TCP tunnel | Retransmission causes head-of-line blocking: one lost segment delays every newer frame, the opposite of CAN semantics |
| IEEE 1722 TSCF | Adds one presentation timestamp per AVTPDU; unnecessary here because the ACF-CAN header already carries a 64-bit per-message timestamp, which is finer-grained than a per-PDU stamp under batching |
| IEEE 1722 NTSCF + ACF-CAN (chosen) | The transport the platform network specification assigns to CAN streams; raw Ethernet with 64-bit stream IDs, standard tooling on the Linux side (Open1722, Wireshark dissection), and a direct upgrade path to timestamped TSCF and 802.1Q shaping |

Loss semantics stay CAN-like: a lost Ethernet frame discards old data
instead of delaying new data, and the per-stream sequence number makes
loss observable to any listener.

### Two packet sockets, not one

Transmission and reception use separate AF_PACKET sockets. The socket
layer holds a per-descriptor lock for the whole duration of each call,
so a receive blocked waiting for a packet holds off every send issued
on the same descriptor until inbound traffic happens to release it:
measured on hardware, a single-socket bridge starved its transmit
direction almost completely on a quiet link (8 of 210 sends escaped,
each coinciding with an inbound frame). The receive socket carries the
AVTP ethertype; the transmit socket takes protocol 0, which inbound
delivery never matches, so its receive path stays empty. The
transmitted ethertype is taken from the destination address of each
send, so the protocol-0 socket transmits normally.

### Batching bound: 15 messages per AVTPDU

The byte budget alone would allow ~90 minimal ACF-CAN messages per
AVTPDU. Widely deployed listeners decode an AVTPDU into a fixed
15-entry frame array without checking the write index, so the encoder
caps each AVTPDU at 15 messages in addition to the byte bound. The cap
costs nothing at the configured rates and keeps any standard listener
safe.

### Per-bus queues and writer threads

The Ethernet receive thread only decodes, demuxes by stream ID, and
enqueues; each bus has its own 32-frame queue and writer thread that
performs the blocking CAN transmit. A bus whose frames go
unacknowledged (unplugged, error recovery) therefore backs up only its
own queue, and traffic for the other bus keeps flowing. The same
isolation exists in the opposite direction through the shared
transport queue's drop-on-full behavior with per-bus counters.

### CAN clock: crystal-referenced PLL1 at 48 MHz

Standard CAN FD rates must divide the controller clock into an integer
number of time quanta:

| Clock | 4 Mbit/s | Verdict |
|---|---|---|
| FRO 144 MHz internal RC | divides, but +/-1% tolerance | outside ISO 11898-1 oscillator limits; bit timing drifts with temperature |
| PLL0 150 MHz (main PLL) | 12.5 tq | not an integer, rate unrealizable |
| PLL1 96 MHz / 2 = 48 MHz (chosen) | 12 tq exact | crystal-referenced (+/-10 ppm), integer quanta at 1, 2, 4, 8 Mbit/s |

### Per-frame timestamps: ISR sampling, discipline-gated validity

The PHC is sampled in the CAN receive interrupt, immediately before
the frame is queued, so `message_timestamp` reflects arrival rather
than queueing or batching delay (frames drained together still carry
their own arrival times). The driver's clock read is a lock-free
register sequence guarded against nanosecond rollover, safe in
interrupt context; the mutating clock operations (step, rate) stay in
the servo's thread. Two honest bounds: frames decoded from one
controller interrupt are stamped serially in mailbox order
(microsecond-scale skew), and a servo re-step could in principle land
between the seconds and nanoseconds reads (bounded to the rare
post-lock error above 100 ms).

MTV is asserted only when the servo has ever locked, and stays
asserted through holdover, exactly mirroring the announced clockClass
lifecycle (248 never-locked, 6 locked, 7 holdover). The alternative,
stamping whenever the clock reads successfully, would emit epoch-era
timestamps marked valid from a node that never saw GNSS; validity on
the wire must mean traceability, so it is owned by the discipline
state.

### Remote frames

`CONFIG_CAN_ACCEPT_RTR=y` delivers remote frames to the bridge (the
default masks them out of the receive mailboxes entirely). With the
controllers in FD mode the FlexCAN SDK's FD mailbox accessors neither
set nor read the RTR bit, so a remote frame crosses the bridge as a
data frame with identifier and DLC preserved. This is a documented
transform rather than a supported feature; CAN FD itself has no remote
frames and the platform's protocols do not use them.

## Validation

All tests ran against the stock COVESA Open1722 `acf-can-bridge`
daemon (v0.9.1, unmodified) on a Linux host reached through a
transparent 100BASE-T1 media converter pair, with an external CAN FD
adapter wired to one connector of each bus and termination at both
ends. The adapter runs on its own oscillator, so sustained 4 Mbit/s
BRS traffic exercises real cross-oscillator bit-timing margin.
Interoperating with an independent implementation, rather than a
loopback of the bridge's own codec, is the point: every field of the
wire format is checked by code the bridge does not share.

### Integrity

Randomized identifiers and payloads in both directions on both buses,
compared field by field at the far end:

- FD variant: 408 frames (classic, extended, FD, BRS, ESI cases at
  every FD length class), all byte-exact in identifier, length, and
  payload, zero loss.
- Classic variant: 192 frames including remote frames, all exact,
  zero loss.

Every flag-level difference observed was attributed to a documented
transform of the test path, none of them the bridge's: the stock
daemon force-sets FDF on everything it encodes in FD mode, always
writes FD framing to the CAN interface, and carries a sticky-flags
defect between AVTPDUs; ESI cannot be commanded onto a real wire; and
remote frames shadow to data frames as described above.

### Latency against the 500 us budget

One-way latency, 1000 frames per path, 64-byte FD frames with BRS at
1 kHz, timestamped on a single host clock (microseconds):

| Path | min | median | p99 | max |
|---|---|---|---|---|
| vcan0 to can0 | 341.9 | 355.7 | 389.3 | 409.6 |
| can0 to vcan0 | 320.4 | 345.0 | 381.7 | 548.1 |
| vcan1 to can1 | 337.4 | 352.1 | 386.0 | 448.5 |
| can1 to vcan1 | 324.5 | 350.5 | 385.0 | 393.4 |

Every path meets the 500 us end-to-end budget at p99. The measured
path is strictly worse than a deployment path: it includes the
user-space daemon (Linux scheduling produced the single 548 us
outlier), the USB transfer latency of the CAN adapter, and ~170 us of
CAN wire time for a 64-byte FD frame, which alone accounts for
roughly half the median. The bridge's own contribution fits inside
the remaining margin.

### Full-duplex flood

Ten seconds of simultaneous transmission on all four endpoints (both
buses, both directions, 32-byte FD frames, ~7100 frames/s aggregate
through the node): 35634 bridged frames, and every receive count
reconciled exactly against the corresponding send count. Zero loss,
zero daemon errors, no queue shed. This load pattern is the one a
serialized transport cannot survive, and it is the regression test
for the two-socket design.

### CAN physical layer

The CAN side of this bridge (bit timing, transceivers, termination,
clocking) was validated separately under sustained bidirectional
4 Mbit/s BRS flood, including a thermal sweep of the board from 32.7
to 73.2 degC under load with zero errors on every counter at both
ends. Those results are transport-independent and carry over
unchanged.

### Timestamp traceability, end to end

Method: a Linux host disciplines its clock to the node over gPTP
(software timestamping, this node as grandmaster), while a raw capture
on the AVTP ethertype records each ACF-CAN message's arrival time and
carried `message_timestamp`; the delta subtracts the two on the shared
timescale. Traffic: 200 frames/s per bus; the announced clock class
was verified as 6 (GNSS locked) both before and after the capture.

Results over 9162 messages (both buses): MTV coverage 100%; delta
minimum 63 us, median 81 us, 99th percentile 153 us, maximum 324 us;
zero samples at or before their own stamp; zero stamp inversions; zero
sequence gaps, losses, or duplicates. The 16 to 20 us spread matches
the software-timestamped gPTP sync quality, and the median matches the
physical path (encode, 100BASE-T1 wire, an 18 us media converter,
host receive path). A hardware-timestamping peer NIC would tighten the
observer; the node-side nanosecond story is already covered by the
closed-loop EXTINT audit in the timing library.

### Observability

The bridge keeps per-bus counters (frames bridged each way, AVTPDUs
sent and received, queue drops, transmit errors with last errno),
exposed by the `coe stats` shell command over the console or mcumgr;
they were used to verify every stage of the pipeline on target during
validation.

## Results summary

| Quantity | Value |
|---|---|
| Interoperability | stock Open1722 acf-can-bridge, both variants, both directions |
| Integrity | 600 randomized frames byte-exact, zero loss, zero unexplained differences |
| Latency, 64-byte FD BRS at 1 kHz | p99 385 to 390 us vs 500 us budget, all four paths |
| Full-duplex flood | 10 s, ~7100 frames/s aggregate, all counts reconciled exactly |
| CAN physical layer (carried over) | zero errors through 32.7 to 73.2 degC under 4 Mbit/s flood |
| Timestamp coverage | 9162 of 9162 bridged frames stamped (MTV=1), GNSS locked throughout |
| Stamp-to-arrival delta at a gPTP-synced observer | median 81 us, p99 153 us, none negative, none inverted |

### Planned

- VLAN priority tagging and validation through an 802.1Q time-aware
  bridge once switch firmware supports it.
- A hardware-timestamping observer NIC, turning the traceability check
  into a nanosecond-class one-way latency instrument.
- Latency characterization on a switched path without the USB adapter
  and user-space daemon in the loop.
