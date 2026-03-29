# Optical Flow Zenoh TODO

## Zenoh Transport (Blocking)

The zenoh-pico TCP client connection to zenohd fails with `_Z_ERR_TRANSPORT_OPEN_FAILED` (-102).
The TCP socket creates successfully but the zenoh protocol handshake fails.

**Root causes found and fixed so far:**
- `CONFIG_HEAP_MEM_POOL_SIZE=0` caused `_Z_ERR_SYSTEM_OUT_OF_MEMORY` (-78) — fixed with 32KB heap
- `ZVFS_OPEN_MAX` too low caused `ENFILE` (errno=23) — fixed with 16
- zenoh-pico `config.h` hardcodes all `Z_FEATURE_*` defines, overriding Kconfig — fixed with `#ifndef` guards
- `CONFIG_NET_TCP` was not enabled — fixed

**Still broken:**
- `z_open()` returns -102 after TCP link opens but protocol handshake fails
- Need to instrument zenoh-pico `_z_unicast_open_client()` in `src/transport/unicast/transport.c` to find exact failure point
- May be a zenoh protocol version negotiation issue between zenoh-pico 1.8.0 and zenohd 1.8.0
- Alternative: try peer-to-peer mode (board listens, desktop connects) — requires `Z_FEATURE_UNICAST_PEER`
- The zenoh branch used client mode with all features enabled including scouting/multicast — unclear if it ever worked

## Ethernet Boot Crash

The NXP ENET QOS MAC driver crashes when ethernet is plugged in during boot.
`enet_qos_dma_rx_resume()` gets called with `dev=NULL` because the network interface
isn't registered when the first RX interrupt fires.

**Mitigated:** Lowered ethernet MAC IRQ priority to 0x4 in DTS overlay.
**Proper fix:** Need to defer enabling the PHY/MAC RX interrupts until after
`net_if` registration completes. This is a Zephyr driver issue.

## PAA3905 LED Control

The PAA3905 optical flow sensor LED stays on in auto mode (`0x6F = 0x2C`).
Tested values for register `0x6F` in bank `0x14`:
- `0x2C` = auto (sensor controls LED) — LED turns on in low light
- `0x0C` = enable LED driver
- `0x30` = force bright mode
- `0x00` = tested, LED still on

Need to find the correct register value to fully disable the LED, or determine
if the LED is hardware-controlled and cannot be disabled via register writes alone.
The `led-force-on` DTS property was added to the zephyr `spinali-optical-flow-v2`
branch but only works on boards with inverted LED logic.

## MCUboot

Disabled for now (`CONFIG_BOOTLOADER_MCUBOOT` commented out) because the
`cerebri2-base` zephyr_boards branch doesn't have the MCUboot board config,
and the MCUboot build fails with missing `fsl_lpflexcomm.h` on the current
zephyr base. The `spinali-optical-flow-v2` zephyr branch is based on the
original `main-with-patches` which does have the MCUboot board config but
we'd need to re-add it.

## Dependencies

- `zenoh-pico`: pinned to `main` (v1.8.0), `config.h` locally patched with `#ifndef` guards
- `zephyr`: `spinali-optical-flow-v2` branch on CogniPilot/zephyr (cherry-picks: ethernet null-check, ICM45686 unaligned fix, LPSPI null deref fix, PAA3905 led-force-on)
- `synapse_msgs_fbs`: `cerebri2` branch with `synapse_optical_flow.fbs` added
- `zros_drivers`: stock `main`, no changes

## Desktop Side

- `zenohd` must be running on `tcp/192.0.2.2:7447` for client mode
- Python subscriber script at `scripts/zenoh_optical_flow_sub.py` (untested, blocked on zenoh connection)
- ROS2 bridge node not yet created (depends on working zenoh)
- `eclipse-zenoh` and `flatbuffers` Python packages required
