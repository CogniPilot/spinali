# Time Architecture: Design Rationale and Validation

Companion to [README.md](README.md). The README describes how the
timing system works; this document records why each design choice was
made over its alternatives, and how the system was validated.

## Design decisions

### Clock source: crystal-referenced PLL0

The PHC, CPU, and capture timer all derive from PLL0 referenced to the
24 MHz +/-10 ppm crystal. The alternative, the internal FRO RC
oscillator (+/-1%), was measured at +4550 ppm with tens of
microseconds of second-to-second wander: unusable as a PTP reference
and outside ISO 11898-1 oscillator tolerance for CAN FD. With the
crystal, the same measurement reads about +2 ppm with nanosecond-class
wander. FlexCAN takes a separate crystal-referenced PLL1 at 96 MHz
because standard CAN FD data rates divide 48 MHz exactly but not
150 MHz (4 Mbit/s needs 12 tq; 150/3 = 50 MHz yields 12.5).

### Pulse timestamping: CTIMER hardware capture

Options evaluated for timestamping the TIMEPULSE edge:

| Option | Verdict |
|---|---|
| ENET QoS auxiliary timestamp snapshot | Not synthesized in the MCXN947's reduced EQOS; the register space is reserved and no 1588 pins are bonded out |
| GPIO interrupt + immediate PHC read | Works, but interrupt entry adds microsecond-class latency and load-dependent jitter |
| CTIMER capture input (chosen) | The pulse pin muxes to CT_INP4; the edge latches a 150 MHz timer in hardware (~6.7 ns), and interrupt latency affects only when the value is read, not what was captured |

Because CTIMER4 and the PHC reference share PLL0, a captured tick
converts to PHC time by exact ratio arithmetic; there is no
cross-domain drift term.

### Edge labeling: UBX-TIM-TP

NAV-PVT carries time of fix, delivered over a serial link with
millisecond-class, variable latency: unusable for sub-microsecond
pairing. TIM-TP instead states the GNSS time of a specific pulse edge
(week, time of week, sub-millisecond term, quantization error in
picoseconds), sent ahead of the pulse it describes. Each captured edge
pairs with exactly one label; stale or missing labels cause the sample
to be skipped rather than mis-paired.

### Timescale handling

The PHC runs on the PTP timescale (TAI-based). Labels arrive on the
receiver's GPS or UTC timebase per its configuration; conversion uses
the fixed GPS-to-TAI offset (19 s) plus, for UTC, the live leap-second
count from NAV-TIMELS. The leap count is never hardcoded: a hardcoded
value is correct until the next leap second and silently wrong after.

### Servo: step once, then slew only

The first valid sample may step the PHC (`ptp_clock_set`), since boot
time is arbitrary. Until the first-ever lock the step threshold is
1 ms (an RTC seed can land tens of milliseconds off, and slewing that
out would take minutes); after a lock has been achieved the threshold
rises to 100 ms so holdover recovery stays slew-only. After stepping,
the servo only adjusts rate. gPTP
consumers assume monotonic grandmaster time; steps propagate as
offset spikes to every downstream node. The PI gains (Kp 0.4, Ki 0.08
ppb per ns at 1 Hz) were chosen for lock within tens of seconds with
overshoot below the lock threshold; the rate actuator has ~0.7 ppb
resolution, far below the noise floor. Reacquisition after an outage
deliberately reuses the same path: accumulated holdover error is
orders of magnitude below the 100 ms step threshold, so recovery slews
and time stays monotonic across the entire outage cycle.

### Holdover: freeze the learned rate

On pulse loss (2.5 s watchdog) the integrator freezes and the PHC
coasts at the last learned rate. Alternatives rejected: continuing to
integrate (integrates noise or nothing), or reverting to nominal rate
(discards the learned ~2 ppm crystal correction and multiplies drift).
The receiver stops its timepulse when it loses timing validity, which
is the desired behavior: the servo is never fed an undisciplined
pulse. Temperature-compensated holdover (mapping the integrator
history against temperature) is a possible future refinement.

### RTC as boot seed only

The RTC runs from the internal 16.384 kHz oscillator despite a
32.768 kHz crystal being fitted: enabling the crystal's VBAT
oscillator hangs the boot ROM on every warm reset until power removal
(see the board clock notes). Percent-class RTC accuracy is acceptable
because its only job is giving the grandmaster plausible time at boot;
GNSS replaces it within seconds of a fix. The seed path bounds the
year because the RTC's power-on default reports an implausible date.

### Announce quality

Announced quality follows servo state rather than static
configuration: the node boots at clockClass 248 (unspecified), is
promoted to class 6 / GPS / 100 ns on lock, and demotes to class 7
(holdover, accuracy 100 us) on pulse loss. A clock that never locked
announcing class 6 would be a lie that BMCA acts on; runtime ownership
makes the announce truthful in every state. UTC offset, leap flags,
and traceability flags are refreshed from NAV-TIMELS on lock and
hourly, so a pending leap second propagates to the domain.

## Validation methodology

The central technique: measure the same physical quantity through
independent paths and require agreement.

### Frequency error, three independent instruments

The crystal's frequency error is observable by:

1. Raw PPS capture: the PHC-measured period between GNSS pulses.
2. The servo integrator: the learned rate correction at lock.
3. A linuxptp slave on the T1 link: its frequency estimate for the
   grandmaster, derived purely from received Sync timestamps.

All three agree at about 2.1 ppm (sign per measurement direction).
Path 3 is the strongest: it exercises the PHC, the MAC's hardware
egress timestamping, the network path, and a third-party PTP
implementation, end to end.

### Steady-state soak

Method: continuous 1 Hz servo telemetry over 10.5 h spanning an
overnight ambient cycle. Results: 26221 samples, 100% in lock, phase
error mean +0.3 ns, sigma 23 ns, 99% within +/-60 ns (error values
quantize at the PHC's 20 ns resolution, which is the measurement
floor). The integrator moved 1.1 ppm through the night, tracking
crystal temperature drift with no effect on phase error: loop
bandwidth (seconds) exceeds thermal time constants (minutes to hours)
by orders of magnitude.

### Announce verification, in protocol

A linuxptp slave's management interface (`pmc 'GET PARENT_DATA_SET'
'GET TIME_PROPERTIES_DATA_SET'`; note `-t 1` is required against the
gPTP profile) reports the received grandmaster attributes:
clockClass 6, clockAccuracy 0x21, timeSource 0x20, currentUtcOffset
37, ptpTimescale 1, timeTraceable 1. This verifies the advertised
quality as received over the wire, not as configured. The full
lifecycle was captured this way through an antenna pull and recovery:
locked (class 6, accuracy 0x21, UTC-offset-valid and
frequency-traceable both set), holdover within seconds of pulse loss
(class 7, accuracy 0x27, frequency-traceable cleared, UTC offset
retained as valid), and full restoration on re-lock. The slave
remained synchronized to the honestly-demoted grandmaster throughout
the outage.

### Outage and recovery

Method: detach the antenna under way, observe both sides, reattach.
Observed: receiver stops the timepulse on fix loss; servo enters
holdover; the slave's view of the grandmaster remains stable through
the outage; a ~15 minute outage accumulated ~35 us of phase error
(~39 ppb effective frozen-rate error); recovery was a warm start
followed by slew-only convergence back to the noise floor, with no
step at any point.

### Path-delay sanity

Through a 100BASE-T1 media converter pair the slave measures a
constant ~18.2 us path delay with ~200 ns spread, matching the
converter's fixed store-free latency; the board's own peer-delay
measurement from the opposite direction agrees. A single converter
contributes a fixed directional asymmetry, so mean offset through this
path carries a constant bias; spread and frequency, not mean, are the
meaningful readouts.

### Closed-loop absolute audit (lib/timing/src/tm_audit.c)

Every 10 s the application drives a GPIO edge into the receiver's
EXTINT input, bracketing the pin write between two PHC reads with
interrupts locked (~0.7 us window; the midpoint is the edge estimate
and half the window its uncertainty). The receiver timestamps the same
edge in GNSS time and reports it as UBX-TIM-TM2; the difference is the
absolute grandmaster error, measured end to end by the receiver
itself. Results: sample scatter sigma ~35 ns (matching the receiver's
own ~36 ns accuracy estimate, so the measurement is receiver-limited)
around a stable systematic of roughly +0.5 us attributable to the
bracket midpoint assumption plus fixed pad, trace, and input-threshold
delays. Absolute error is therefore bounded well under 1 us, and the
35 ns scatter shows the clock itself is far steadier than the
instrument's absolute calibration. Reset transients on the EXTINT line
can record one spurious mark at boot; stale marks are rejected at
pairing time by a sequence check rather than gating emission, which
would deadlock.

### Planned

- Characterize and subtract the audit's fixed systematic (one-time
  oscilloscope comparison, or a hardware-timed edge source) to turn
  the sub-microsecond bound into a direct sub-100 ns measurement.
- The same slave comparison run through an 802.1AS time-aware bridge
  once available, quantifying residence-time correction against the
  transparent-converter baseline.

## Results summary

| Quantity | Value |
|---|---|
| Lock time from power-on | < 30 s (including leap-data wait) |
| Phase error, steady state | mean +0.3 ns, sigma 23 ns |
| 99th percentile |error| | 60 ns |
| Crystal error, learned and cross-verified | ~2.1 ppm |
| Overnight tempco absorbed | 1.1 ppm, no lock impact |
| Holdover drift (~15 min outage) | ~35 us (~39 ppb) |
| Recovery | slew-only, monotonic, no step |
| Announce (received in protocol) | class 6, 100 ns, GPS, UTC offset 37 |
