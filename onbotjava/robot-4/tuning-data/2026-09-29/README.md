# Robot 4 extension tuning evidence (2026-09-29)

This directory contains the selected raw CSV evidence used during the Robot 4 extension tuning session. `manifest.csv` records the original log name, trial purpose, outcome, tuning usability, row count, byte size, and SHA-256 digest.

These files are intentionally limited to the useful and representative trials. The CSVs contain mechanism telemetry, test configuration, and electrical samples. They do not contain the Control Hub serial, Driver Hub serial, or local PC paths.

## Mechanical and control context

- Robot: FGC2026 Robot 4 only.
- Motor: REV Core Hex Motor with a user-reported 10.4:1 reduction.
- Robot 3 has a different extension gear ratio. Do not copy Robot 4 gains or identified friction directly to Robot 3.
- Configured mechanical limits were 21 to 75 degrees. The automatic trial endpoints were 25 and 71 degrees to retain margin.
- Desired motion is approximately 2 seconds across the 54-degree automatic range, using a gradually slewed target. This corresponds to a 27 deg/s target slew.
- The photographed robot held about 20 balls, but only balls whose load path moves with the extension influence the extension torque. `balls=20` is therefore inventory metadata, not an effective extension load value. Record ball placement or moving-load configuration with future trials.
- Gear friction is high. Uncommanded motion under ball weight is not assumed, and the operator manually moved the mechanism during an earlier observation.

## Static-friction evidence

The holding baseline was generally about 1.26 to 1.40 V. The old v4/v5 direction checks were invalidated by observed backlash, compliance, or a nontrivial relationship between the motor direction and measured angle, so files 01 through 05 are reference evidence rather than final breakaway measurements.

File 06 is the accepted direct motor-negative trial from Tune v6:

- Start angle: about 64.88 degrees.
- Holding baseline: +0.839 V.
- Ramp delta at detection: 2.468 V.
- Command at detection: -1.629 V.
- Detected displacement: about -0.777 degrees.
- Sampled current near detection: about 0.32 A; maximum sampled file current: about 0.54 A.
- Valid Hub voltage near the event: about 12.42 V.
- The mechanism coasted roughly another 2.3 degrees after output was set to zero. The event voltage is usable, but the stop distance must be considered when selecting test clearance.

The positive-direction reference trials suggest roughly +1.7 V above holding, or about +0.14 normalized output at 12.4 V. The accepted negative-direction trial suggests roughly -2.47 V relative to holding, or about -0.20 normalized output. These values support the existing symmetric friction value near 0.15 as a safe initial controller value, but also show that a direction-dependent friction model may be useful. More repeated measurements are required before treating either directional estimate as precise.

## Automatic trajectory evidence

| File | Configuration | Result | Interpretation |
| --- | --- | --- | --- |
| 07 | max power 0.35, upward, kV 0 | 25 to 71 degree move completed in about 3.46 s; max sampled current 1.09 A | Safe but too slow. |
| 08 | max power 0.50, downward, kV 0 | Overspeed near -45.7 deg/s; max sampled current 0.52 A | Downward motion needs separate treatment for gravity and stick-slip. |
| 09 | max power 0.50, upward, kV 0 | Active move completed in about 2.21 s; approximately 24.67 to 69.22 degrees; max speed 44.6 deg/s; max sampled current 1.11 A | Closest successful upward baseline. A 54-degree projection is about 2.67 s. |
| 10 | max power 0.50, upward, kV_up 0.010 | Saturated throughout and stopped on overspeed after about 1.14 s; max speed 53.2 deg/s; max sampled current 1.18 A | `kV_up=0.010` is too aggressive. |

The last configuration written to the Robot 4 Hub used `auto_slew_dps=27`, `auto_max_power=0.50`, and `auto_kv_up=0.003`. The 0.003 value was not tested in this session and must not be reported as validated.

## Electrical-data limitations

- Motor current, Hub current, and Hub voltage are multiplexed rather than sampled simultaneously. Use the sequence and age columns and deduplicate by sequence before analysis.
- Zero current or voltage values during shutdown can be logger artifacts. Check validity, age, event, and state columns before accepting minima or maxima.
- Sampling can miss brief current peaks. No Control Hub reboot occurred during the included trials, so these logs do not establish the actual reboot threshold.
- The onsite Driver Hub battery was very low during part of the session. This can affect field operations even though it is separate from Robot 4 Control Hub input voltage.

## Next trial

1. Place the extension at approximately 24 to 25 degrees with clear travel to 71 degrees.
2. Run the Robot 4 Auto v4 trial with `auto_slew_dps=27`, `auto_max_power=0.50`, and `auto_kv_up=0.003`.
3. Keep the electrical guards enabled: extension motor 2 A, each Hub 8 A, and minimum Hub voltage 11 V.
4. Accept the trial only if it completes without overspeed, overcurrent, undervoltage, or Hub restart. Compare active move time, tracking error, saturation ratio, speed, current, and voltage with files 09 and 10.
5. Tune descending motion separately. Do not raise the shared maximum output solely to improve upward time because file 08 already demonstrates downward overspeed at the same 0.50 limit.

## Integration status

The measurements and experimental auto-tune settings have not been applied to the normal Robot 4 OpModes. The normal extension controller still needs an explicit reviewed change after the `kV_up=0.003` trial and a separate descending trial. In particular, its target slew and final direction-dependent feedforward values remain production decisions.
