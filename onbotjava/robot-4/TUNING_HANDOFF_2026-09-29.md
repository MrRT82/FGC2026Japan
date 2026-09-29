# Robot 4 extension tuning handoff — 2026-09-29

The complete selected evidence, per-file disposition, hashes, and next-test instructions are in [`tuning-data/2026-09-29/README.md`](tuning-data/2026-09-29/README.md) and [`manifest.csv`](tuning-data/2026-09-29/manifest.csv).

## Current decision

- Use the successful Auto v3 upward run at max power 0.50 and `kV=0` as the baseline.
- Treat upward `kV=0.010` as rejected because it saturated and triggered overspeed.
- Test the Hub's current upward candidate `kV_up=0.003` next. It is installed but unvalidated.
- Keep the 27 deg/s target slew, which represents the requested 54-degree travel in about 2 seconds.
- Identify and tune descending behavior independently because the downward trial oversped even with `kV=0`.
- Retain the current electrical safety stops throughout tuning.

## Boundaries for other tasks

- These results apply to Robot 4. Robot 3 uses a different extension gear ratio and requires its own system identification and tuning.
- Ball count alone does not define extension load. Only the balls and structures carried by the moving extension contribute to the relevant torque; record their placement in future logs.
- The normal Robot 4 controller has not yet received these experimental gains or the 27 deg/s slew. Do not describe the production OpModes as tuned until those changes are separately reviewed, deployed, and tested.
- CSV electrical samples are multiplexed and may miss brief peaks. They are useful for trend and safety analysis but do not prove peak current or Control Hub reboot margin.

## Update after the session: limits removed, gains re-selected

This section supersedes "Current decision" and the electrical-guard point above.

- The tuning OpModes (`Extend_Tune_R4`, `Extend_TuneAuto_R4`, `Extend_TuneSweep_R4`, `Extend_SysId_R4`) no longer enforce angle, current, voltage, speed, stall, time or power limits. Build strings end in `_nolimits`. Motion still requires `motion_enabled=true`, RB held and an A press. Releasing RB or pressing B sets the output to zero.
- `tools/r4_pidf_tune.py` follows the FGC_PIDF_Tuner flow: identify, closed-loop simulation of the Java law, then a grid search with robustness and loop-period checks. It adds velocity feedforward. The plant is identified from robot 4 logs only because the gear ratio differs. Results are in `tuning-data/2026-09-29/r4_pidf/`.
- Identified gravity is `0.202*cos(q+17)+0.045`, about three times the old cosine term. The model kV is 0.0057 duty per deg/s. Static friction is 0.126 up and 0.206 down.
- Selected candidate: kp 0.14, ki 0.005, kd 0.001, friction 0.126, `auto_kv_up=0.0028`, `auto_kv_down=0`, with the new gravity values and a 27 deg/s slew. The simulated move from 25 to 71 degrees takes 1.64 s. It passes all robustness cases and 30/40 ms loops. It has not been run on the robot.
- The model is 20-30 % slower than the recorded runs and shows lower peak speeds. Expect the robot to move faster than simulated. The downward model rests on one short run (08).
- `auto_kv_down` was added to `Extend_TuneAuto_R4`. The default is 0.

## Next steps

1. Build the OnBot Java project (done by the team).
2. Copy `tools/robot4_extend_tune.properties.r4pidf` to `/sdcard/FIRST/robot4_extend_tune.properties` on Control Hub `dddad6c23c4eeb3b` only.
   `adb -s dddad6c23c4eeb3b push tools/robot4_extend_tune.properties.r4pidf /sdcard/FIRST/robot4_extend_tune.properties`
3. With the file still at `motion_enabled=false`, INIT `Extend_TuneAuto_R4`. Confirm that the build ends in `_nolimits` and that kp 0.14, `Auto kV up / down 0.0028 / 0`, angle and Hub V/A are shown.
4. Set `motion_enabled=true` and re-INIT. With no automatic stops, someone must support the mechanism and be ready to release RB. Start near 25 degrees with clear travel to 71 degrees.
5. Run 25 to 71 degrees first. Release RB immediately if the move overshoots, oscillates, runs away or makes abnormal noise.
6. Collect `/sdcard/FIRST/logs/robot4_extend_tune_auto_*.csv` and its `.properties`. Record the ball placement on the moving extension.
7. Add the new CSV to `tuning-data/`, then run `py -3.13 tools/r4_pidf_tune.py identify <folder> --out r4_plant.json`, `replay`, and `search` again. The fit must include the upward and downward moves from the new run.
8. Test the downward move only after the upward result has been reviewed. The downward model is the least supported.
9. Do not copy these gains to the normal Robot 4 OpModes or to Robot 3 until they have been validated on the robot.

## Reproduction and verification

Use `manifest.csv` to verify each raw CSV with SHA-256 before analysis. The accepted negative breakaway evidence is file 06. Files 07 through 10 are the automatic trajectory sequence used for the current control decision. File 04 is excluded because the deadman control was released; files 01 through 05 otherwise remain useful only as reference evidence for direction mapping and friction variability.
