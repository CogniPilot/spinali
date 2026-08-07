# rtk_gnss: GNSS-Disciplined gPTP Grandmaster

RTK GNSS receiver node and IEEE 802.1AS (gPTP) grandmaster for the
MR-MCXN-T1 (MCXN947) with the F9P GNSS shield. The application
disciplines the Ethernet MAC's PTP hardware clock (PHC) to GNSS time
using the receiver's hardware timepulse, then serves that time to the
network as a gPTP grandmaster with traceable clock quality.

Measured steady-state performance: phase error sigma 23 ns against
GNSS (quantized at the PHC's 20 ns resolution), 99% of samples within
+/-60 ns, lock from power-on in under 30 s, continuous lock through
overnight thermal drift.

Design rationale (the alternatives considered at each decision point
and why the shipped choice won) and the full validation methodology
with measured results live in [TIMING.md](TIMING.md).

The timing stack (PPS capture, discipline servo, closed-loop audit)
is the shared `lib/timing` library, also used by the coe CAN bridge
to run a grandmaster and stamp bridged frames on the same clock.

## Clock architecture

All timekeeping descends from the board's 24 MHz +/-10 ppm crystal:

```
24 MHz XTAL -> PLL0 (150 MHz) -> CPU / SysTick
                              -> ENET QoS PTP reference -> PHC (20 ns steps)
                              -> CTIMER4 (PPS capture timebase)
              PLL1 (96 MHz)   -> /2 -> FlexCAN bit clock (48 MHz)
```

The PHC and the capture timer share one PLL, so captured timer ticks
convert to PHC time by exact arithmetic with no cross-domain drift.
The internal FRO is deliberately not used as a PLL source: its +/-1%
tolerance makes the PHC unusable for PTP and violates ISO 11898-1
oscillator tolerance for CAN FD. See the board clock setup in
`zephyr_boards` (`boards/nxp/mr_mcxn_t1/board.c`) for the tree.

## PPS capture (lib/timing/src/pps_capture.c)

The F9P TIMEPULSE pin is muxed as a CTIMER capture input (CT_INP4,
routed through INPUTMUX to CTIMER4 channel 0). Each pulse edge latches
the free-running 150 MHz timer in hardware, giving ~6.7 ns capture
resolution with zero interrupt-latency error. The capture ISR
translates the latched tick to PHC time with a bracketed read (timer,
PHC, timer) and hands the result to the servo.

The MCXN947's ENET QoS is a reduced Synopsys EQOS configuration with
no auxiliary timestamp block and no external 1588 pins, so timer
capture on the pulse input is the highest-fidelity path available on
this silicon.

## Time labels (UBX TIM-TP / NAV-TIMELS)

Each captured edge is paired with the receiver's UBX-TIM-TP message,
which states the GNSS time of exactly that pulse (time of week, week
number, sub-millisecond term, and picosecond quantization error).
UBX-NAV-TIMELS supplies the current leap-second count. Labels are
converted to the PTP timescale (TAI-based):

- GPS timebase: PTP seconds = GPS epoch (Unix 315964800) + week * 604800 + tow + 19
- UTC timebase: as above, with 19 + leap seconds (37 s total as of 2026)

Both messages are parsed by the u-blox GNSS driver behind
`CONFIG_GNSS_U_BLOX_F9P_TIMEPULSE` and exposed through
`u_blox_f9p_timepulse_get()` / `u_blox_f9p_leap_get()`.

## Discipline servo (lib/timing/src/pps_servo.c)

A 1 Hz PI controller steers the PHC onto the labeled edges:

1. Boot: the PHC is seeded from the RTC when it holds plausible time,
   so the node serves approximately correct time before a fix.
2. First valid label: one phase step (`ptp_clock_set`) onto GNSS time.
3. Thereafter: rate-only corrections (`ptp_clock_rate_adjust`, ~0.7
   ppb granularity) from the PI loop (Kp 0.4, Ki 0.08 ppb per ns of
   phase error). Time stays monotonic after the initial step.
4. Lock is declared after 5 consecutive samples within +/-1 us and
   dropped past +/-10 us. On lock the RTC is set from GNSS and
   refreshed hourly.

The integrator continuously absorbs crystal temperature drift
(measured ~1 ppm across an overnight room cycle) without the phase
error leaving the quantization floor. Loop bandwidth (seconds) exceeds
thermal time constants (minutes to hours) by orders of magnitude, so
ambient swings do not disturb lock.

## Holdover

A 2.5 s watchdog detects pulse loss. In holdover the last learned rate
is frozen and the PHC coasts on the crystal. Drift budget is the
uncorrected temperature coefficient times elapsed time: 1 ppm of
residual error accumulates 3.6 us per hour. Reacquisition steps only
if the accumulated error exceeds 100 ms; otherwise the loop slews back.

## RTC

The battery-domain RTC provides wall-clock persistence across resets
(and across power loss once a backup source feeds the VBAT diode-OR).
It runs from the internal 16.384 kHz oscillator: the 32.768 kHz
crystal is fitted but intentionally unused because enabling its VBAT
oscillator causes the MCXN947 boot ROM to hang on every warm reset
until power removal (see the note in the board clock setup). Accuracy
is percent-class, which is sufficient for its only role: a boot-time
seed that GNSS replaces within seconds of a fix. RTC power-on defaults
report an implausible year; the seed path validates the range before
use.

## gPTP grandmaster

With `CONFIG_NET_GPTP_GM_CAPABLE=y` the node participates in BMCA. The
announced clock quality follows the servo state at runtime:

| State | clockClass | clockAccuracy | timeSource |
|---|---|---|---|
| boot, never locked | 248 (default) | unknown | internal oscillator |
| servo locked | 6 (primary reference) | 0x21 (100 ns) | 0x20 (GPS) |
| holdover / lock lost | 7 (holdover) | 0x27 (100 us) | 0x20 (GPS) |

UTC offset, leap flags, and the time/frequency-traceable flags are
kept current from NAV-TIMELS on the same transitions.

Because gPTP Sync egress timestamps are taken directly by the
disciplined PHC, downstream nodes inherit GNSS time with no further
application involvement. The port becomes asCapable only after a
successful peer-delay exchange, which requires a live gPTP peer on the
link. `CONFIG_NET_GPTP_NEIGHBOR_PROP_DELAY_THR` accommodates
100BASE-T1 media converters with fixed ~18 us path delay.

Boot defaults come from `CONFIG_NET_GPTP_CLOCK_CLASS` /
`CONFIG_NET_GPTP_TIME_SOURCE`; the servo owns the values at runtime
through `gptp_update_gm_quality()` / `gptp_update_time_properties()`,
so a clock that has never locked cannot claim primary-reference
quality.

## Verification

The crystal frequency error is observable three independent ways and
the measurements agree: the PPS capture period, the servo integrator's
learned rate, and the frequency estimate of a linuxptp slave receiving
the Sync stream. A built-in closed-loop audit (lib/timing/src/tm_audit.c) drives
a timed edge into the receiver's EXTINT input every 10 s and reads
back the receiver's own GNSS timestamp of it (UBX-TIM-TM2), measuring
absolute grandmaster error with no external equipment: sub-microsecond
bounded, with ~35 ns sample scatter at the receiver's noise floor.

## Known limitations

- TIM-TP reports its quantization error as invalid in the current
  receiver configuration; the term is treated as zero.
- Holdover currently freezes the learned rate without temperature
  compensation.
