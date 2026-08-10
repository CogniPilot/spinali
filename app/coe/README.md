# coe: CAN over Ethernet Bridge (IEEE 1722 ACF-CAN)

Transparent bidirectional bridge between the MR-MCXN-T1 hub's two
CAN FD buses and IEEE 1722 NTSCF streams on the 100BASE-T1 Ethernet
link. Each bus is carried as ACF-CAN messages inside NTSCF AVTPDUs on
ethertype 0x22F0, the transport the platform network specification
assigns to wheel and body CAN traffic. A Linux host runs the stock
COVESA [Open1722](https://github.com/COVESA/Open1722) `acf-can-bridge`
daemon against a virtual CAN interface and gets native SocketCAN
access to the hub's physical buses: candump, cansend, python-can, and
higher stacks work unmodified, including CAN FD with bit-rate
switching. Wireshark dissects the stream natively.

Validated against the stock daemon: 600 randomized frames across both
frame variants crossed byte-exact with zero loss, one-way latency
measured p99 385 to 390 us against the 500 us end-to-end budget (on a
path that includes a USB CAN adapter and the user-space daemon), and a
10 s simultaneous full-duplex flood on both buses reconciled every
frame. Design rationale and the full validation record live in
[VALIDATION.md](VALIDATION.md).

## Data path

```
flexcan0 (J3/J4) <-> queue -> batch/encode -> NTSCF stream mac|0 <-> peer
flexcan1 (J6/J7) <-> queue -> batch/encode -> NTSCF stream mac|1 <-> peer
```

Each bus owns one 64-bit IEEE 1722 stream ID: the interface MAC in the
upper 48 bits and the stream index `SPINALI_COE_STREAM_UID_BASE + bus`
in the lower 16, so a node's streams are unique on the network without
coordination. The hub stamps it on everything it transmits for that bus,
and inbound AVTPDUs are matched to a bus by their low 16 bit stream
index. The buses are bridged
independently and never forward to each other; inbound frames pass
through a per-bus queue and writer thread, so a bus with no peer to
acknowledge its frames cannot stall the other bus.

## Wire format

NTSCF AVTPDU header (3 quadlets, big-endian):

| Field | Width | Value |
|---|---|---|
| subtype | 8 | 0x82 (NTSCF) |
| sv | 1 | 1 (stream ID valid) |
| version | 3 | 0 |
| ntscf_data_length | 11 | octets of ACF payload after this header |
| sequence_num | 8 | per-stream, advances only on a successful send |
| stream_id | 64 | base + bus index |

Each AVTPDU carries one or more ACF-CAN messages (4-quadlet header):

| Field | Width | Value |
|---|---|---|
| acf_msg_type | 7 | 0x01 (CAN) |
| acf_msg_length | 9 | quadlets, header plus padded payload |
| pad | 2 | payload padding octets to the quadlet boundary |
| MTV / RTR / EFF / BRS / FDF / ESI | 1 each | frame attribute flags |
| can_bus_id | 5 | bus index |
| message_timestamp | 64 | PHC nanoseconds at CAN frame arrival, PTP timescale; MTV set once the time discipline has locked |
| can_identifier | 29 | CAN ID |

Under backlog, up to 15 ACF-CAN messages are batched into one AVTPDU
(an interoperability bound: widely deployed listeners decode into a
fixed 15-entry array), within a 1450-octet cap so frames traverse
standard Ethernet untagged. An idle bus sends one frame per AVTPDU
with no added latency.

## Addressing and peer model

Outbound AVTPDUs go to `SPINALI_COE_DST_MAC`, by default a multicast
address inside the MAAP dynamic pool (91:E0:F0:00:0C:0E). The source
MAC of the first valid inbound AVTPDU replaces it, so a unicast peer
is followed automatically. The hub's MAC filter passes all multicast
on this silicon, so the default works with no join on the hub side;
the Linux daemon joins the multicast group itself. Static IP
configuration on the same link is unchanged and carries mcumgr.

## CAN configuration

Both controllers run CAN FD at 1 Mbit/s nominal and 4 Mbit/s data
rate (`CONFIG_CAN_DEFAULT_BITRATE` / `_DATA`, 75% sample points),
falling back to classic with a logged warning if FD is unavailable.
Catch-all filters accept all standard and extended IDs, and
`CONFIG_CAN_ACCEPT_RTR=y` delivers remote frames to the bridge. The
FlexCAN bit clock is 48 MHz from the crystal-referenced PLL1 (96 MHz
/ 2): standard FD rates divide it exactly, and the internal FRO's
+/-1% tolerance is outside ISO 11898-1 requirements. See the board
clock setup in `zephyr_boards` (`boards/nxp/mr_mcxn_t1/board.c`).

## Hardware

| Bus | Connectors | Transceiver | Termination |
|---|---|---|---|
| can0 | J3 / J4 | TJA1462 CAN SIC | on-board split, 2 x 60.4 ohm + 56 pF |
| can1 | J6 / J7 | TJA1462 CAN SIC | on-board split, 2 x 60.4 ohm + 56 pF |

Connectors are JST-GH; pin 2 is CANH, pin 3 is CANL. The two
connectors per bus are wired in parallel for daisy-chaining. The
fitted split termination makes the hub one terminated end of each
bus; the far end of the cable run needs its own termination.

## Host setup

Build Open1722 (the examples target is separate from the default
build):

```
cmake -S Open1722 -B Open1722/build
make -C Open1722/build -j
make -C Open1722/build examples -j
```

Create one vcan per bus and start one bridge daemon per bus. The
daemon needs CAP_NET_RAW (run as root or setcap the binary). One
daemon instance serves one frame variant: `--fd` for CAN FD, without
it for classic (only the classic variant can carry remote frames).

```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 mtu 72       # required for CAN FD frames
sudo ip link set vcan0 up
sudo acf-can-bridge -i <ethernet interface> -d 91:E0:F0:00:0C:0E \
    --canif vcan0 --fd \
    --talker-stream-id 00cafe0000000000 \
    --listener-stream-id 00cafe0000000000
```

Repeat with `vcan1` and stream ID `...0001` for the second bus.
Anything that speaks SocketCAN then works against `vcan0`/`vcan1` as
if attached to the hub's physical buses.

For bench work with a physical CAN FD adapter on a hub connector,
matching bit timing on the Linux side:

```
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 dbitrate 4000000 \
    fd on sample-point 0.75 dsample-point 0.75 restart-ms 100
```

A CAN frame is only completed when some node acknowledges it, so
transmitting onto a bus with no other node attached fails by design
(logged as a warning).

## Build and flash

```
west build -b mr_mcxn_t1/mcxn947/cpu0 spinali/app/coe --sysbuild
west flash
```

## Configuration

| Option | Default | Meaning |
|---|---|---|
| `SPINALI_COE_STREAM_UID_BASE` | 0x0000 | 16-bit stream index for bus 0; the full stream ID is the interface MAC in the upper 48 bits and the index in the lower 16 |
| `SPINALI_COE_DST_MAC` | 91:E0:F0:00:0C:0E | destination until a peer is learned from traffic |
| `CAN_DEFAULT_BITRATE` / `_DATA` | 1 M / 4 M | bus bit timing |

## Overload behavior

The CAN-to-Ethernet direction runs through a 32-frame queue; if
egress falls behind, new frames are dropped with a logged warning and
delivered frames are never reordered. Batching engages exactly when
backlog exists. In the Ethernet-to-CAN direction each bus has its own
32-frame queue and writer thread; a frame that cannot be transmitted
within 100 ms is dropped with a warning, and a full queue sheds with
a per-bus drop counter. Sequence numbers advance only on successful
sends, so a listener's loss accounting stays truthful.

## Time discipline and per-frame timestamps

The node runs the shared GNSS time discipline (`lib/timing`, the same
stack the rtk_gnss application uses): the F9P shield's hardware
timepulse is captured by CTIMER4 and a PI servo steers the Ethernet
MAC's PTP hardware clock onto GNSS time. The same clock does three
jobs: it serves the network as a gPTP grandmaster, it timestamps gPTP
Sync egress, and it stamps every CAN frame this bridge forwards.

Each frame's `message_timestamp` is sampled in the CAN receive
interrupt, so the stamp reflects frame arrival, not queueing. MTV is
set only once the servo has locked (and stays set through holdover,
mirroring the announced clock quality): a node that has never seen
GNSS sends MTV clear rather than a stamp that looks traceable but is
not. The announced gPTP quality follows the servo: clockClass 248 at
boot, 6 (primary reference, GPS, 100 ns) on lock, 7 (holdover) on
pulse loss.

Without the GNSS shield fitted the bridge still forwards frames
normally; timestamps are withheld and the node announces the honest
boot defaults.

## Also on board

- Console and shell on the FC1 UART (J5 debug connector), with the
  Zephyr CAN and network shells plus a `coe stats` command (per-bus
  frame and error counters, peer state, discipline state) for bench
  diagnosis; all reachable over mcumgr as well.
- mcumgr over UDP on the same link: firmware update and remote shell
  with no extra wiring.

## Known limitations

- Remote frames cross the bridge as their data-frame shadow
  (identifier and DLC preserved, RTR type lost) while the controllers
  run FD mode: the FlexCAN SDK's FD mailbox accessors neither set nor
  read the RTR bit. Classic-mode operation is unaffected, and CAN FD
  itself has no remote frames.
- ESI is carried toward Ethernet but not replayed onto the local bus;
  the controller drives that bit from its own error state.
- Frames bridged before the first GNSS lock carry MTV clear and a
  zero timestamp; discipline state is visible in `coe stats`.
- Some host NICs deliver the AVTP multicast only after a specific
  group membership (the Open1722 daemon performs one itself);
  promiscuous or allmulti mode is not a reliable substitute on such
  hardware.
- A lost Ethernet frame is a lost batch of CAN frames; the sequence
  number lets a peer account for loss, but there is no retransmission.
- Any station on the segment that sends a valid AVTPDU under a known
  stream ID becomes the peer; there is no authentication. Intended
  for closed machine networks.
