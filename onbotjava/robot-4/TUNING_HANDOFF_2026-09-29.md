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

## Reproduction and verification

Use `manifest.csv` to verify each raw CSV with SHA-256 before analysis. The accepted negative breakaway evidence is file 06. Files 07 through 10 are the automatic trajectory sequence used for the current control decision. File 04 is excluded because the deadman control was released; files 01 through 05 otherwise remain useful only as reference evidence for direction mapping and friction variability.
