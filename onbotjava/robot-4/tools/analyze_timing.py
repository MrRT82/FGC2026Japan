"""Summarize robot3 timing_v1 CSV using only the Python standard library."""
import argparse
import csv
from pathlib import Path


def summarize(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        names = [key[3:-2] for key in (reader.fieldnames or [])
                 if key.startswith("tm_") and key.endswith("_n")]
        if not names:
            raise ValueError("No timing_v1 columns found. Check the installed build.")
        totals = {name: [0, 0, 0, 0] for name in names}
        rows = 0
        skipped = 0
        for row in reader:
            try:
                values = {name: [int(row[f"tm_{name}_{suffix}"])
                                 for suffix in ("n", "total_us", "max_us", "errors")]
                          for name in names}
            except (TypeError, ValueError, KeyError):
                skipped += 1
                continue
            rows += 1
            for name, value in values.items():
                total = totals[name]
                total[0] += value[0]
                total[1] += value[1]
                total[2] = max(total[2], value[2])
                total[3] += value[3]
        return totals, rows, skipped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    args = parser.parse_args()
    totals, rows, skipped = summarize(args.csv_path)
    print(f"Valid rows: {rows}; incomplete/invalid rows skipped: {skipped}")
    print(f"{'metric':24} {'calls':>9} {'mean_ms':>10} {'max_ms':>10} {'total_ms':>12} {'errors':>8}")
    for name, (count, total_us, max_us, errors) in totals.items():
        if count:
            print(f"{name:24} {count:9d} {total_us/count/1000:10.3f} "
                  f"{max_us/1000:10.3f} {total_us/1000:12.3f} {errors:8d}")
    print("Times overlap: do not add distance, loop_work, log_build and other categories.")
    print("Mode-labelled work times include logging; sensor windows can straddle mode changes.")


if __name__ == "__main__":
    main()
