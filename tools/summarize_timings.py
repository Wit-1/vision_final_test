#!/usr/bin/env python3
"""Summarize per-frame benchmark CSV with nearest-rank P95."""

import argparse
import csv
import math
from pathlib import Path
import sys

STAGES = ("decode", "detect", "pose", "track", "selector", "draw", "total")


def summarize(paths):
    groups = {}
    for path in paths:
        count = 0
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            required = {"dataset", "mode", *(stage + "_ms" for stage in STAGES)}
            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError(f"{path}: missing benchmark CSV columns")
            for line, row in enumerate(reader, start=2):
                try:
                    dataset, mode = row["dataset"], row["mode"]
                    if not dataset or mode not in ("tradition", "nn") or None in row:
                        raise ValueError("invalid dataset, mode or column count")
                    values = [float(row[stage + "_ms"]) for stage in STAGES]
                    if any(not math.isfinite(value) or value < 0 for value in values):
                        raise ValueError("timings must be finite and nonnegative")
                except (TypeError, ValueError) as error:
                    raise ValueError(f"{path}:{line}: {error}") from error
                samples = groups.setdefault((dataset, mode), [[] for _ in STAGES])
                for stage_samples, value in zip(samples, values):
                    stage_samples.append(value)
                count += 1
        if not count:
            raise ValueError(f"{path}: no timing samples")
    rows = []
    for (dataset, mode), stages in sorted(groups.items()):
        for stage, samples in zip(STAGES, stages):
            ordered = sorted(samples)
            p95 = ordered[math.ceil(0.95 * len(ordered)) - 1]
            rows.append((dataset, mode, stage, len(samples),
                         f"{math.fsum(samples) / len(samples):.6f}", f"{p95:.6f}"))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, help="Write a new summary CSV; default is stdout")
    args = parser.parse_args()
    try:
        if len({path.resolve() for path in args.inputs}) != len(args.inputs):
            raise ValueError("The same input CSV was provided more than once")
        rows = summarize(args.inputs)
        stream = args.output.open("x", newline="") if args.output else sys.stdout
        try:
            writer = csv.writer(stream)
            writer.writerow(("dataset", "mode", "stage", "samples", "mean_ms", "p95_ms"))
            writer.writerows(rows)
        finally:
            if args.output:
                stream.close()
    except (OSError, ValueError, csv.Error) as error:
        parser.exit(1, f"Timing summary failed: {error}\n")


if __name__ == "__main__":
    main()
