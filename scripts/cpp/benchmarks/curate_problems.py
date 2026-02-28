#!/usr/bin/env python3
"""
curate_problems.py

Read a JSON file containing a large set of problems (each problem is a dict).
Sample 100 problems for each obstacle-count bin:
  bin0  = 0-10 obstacles (inclusive)
  bin1  = 11-20 obstacles
  bin2  = 21-30 obstacles
  ...
(there are 10 bins by default, producing 1000 problems total).

Output a JSON file with the sampled problems as a flat list. Each sampled problem
will be augmented with a unique `problem_number` (0..N-1) in the output file.

Usage:
    curate_problems.py input.json output.json [--seed 42] [--bins 10] [--bin-size 10]
                     [--require-full] [--verbose]

Options:
  --seed: reproducible sampling seed (optional)
  --bins: number of bins (default 10)
  --bin-size: size of each bin in obstacle counts (default 10)
  --require-full: if set, fail if any bin cannot provide at least `per_bin` distinct problems
  --per-bin: how many problems per bin (default 100)
  --verbose: print progress / warnings

Notes:
- If a bin has fewer than `per_bin` available problems and --require-full is NOT set,
  sampling will proceed with replacement (i.e., problems may repeat) to reach `per_bin`.
  If a bin has zero available problems, the script will fail unless --require-full is false,
  in which case it will raise an error because sampling with replacement from an empty set
  is not possible.
- The script expects the input JSON to be either:
    * a list of problem dicts, or
    * an object with a top-level key 'problems' whose value is a list of dicts.
  Each problem dict is expected to contain a list key named 'cuboid_obstacles' (a list
  of obstacle descriptions) whose length is used to determine the obstacle count.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from typing import List, Dict, Tuple, Any


def make_bins(num_bins: int, bin_size: int) -> List[Tuple[int, int]]:
    """
    Construct inclusive bins:
      bin 0: 0 .. bin_size
      bin 1: (bin_size+1) .. (2*bin_size)
      ...
    Example for bin_size=10, num_bins=10:
      (0,10), (11,20), (21,30), ..., (91,100)
    """
    bins = []
    for i in range(num_bins):
        if i == 0:
            low = 0
        else:
            low = i * bin_size + 1
        high = (i + 1) * bin_size
        bins.append((low, high))
    return bins


def count_obstacles(problem: Dict[str, Any]) -> int:
    """
    Return number of cuboid obstacles for the given problem dict.
    If the key is missing or not a list, return 0.
    """
    obs = problem.get("cuboid_obstacles", None)
    if isinstance(obs, list):
        return len(obs)
    # Some problem variants might use a different key; if not present treat as 0
    return 0


def problems_in_bin(problems: List[Dict[str, Any]], low: int, high: int) -> List[Dict[str, Any]]:
    """Return sub-list of problems whose obstacle count is in [low, high] (inclusive)."""
    return [p for p in problems if low <= count_obstacles(p) <= high]


def sample_from_list(source: List[Dict[str, Any]], k: int, rng: random.Random, require_full: bool, verbose: bool=False) -> List[Dict[str, Any]]:
    """
    Sample k items from source.
    - If len(source) >= k: sample without replacement.
    - If len(source) < k:
        * if require_full: raise RuntimeError
        * else: sample with replacement (allow duplicates) using random.choices
    """
    n = len(source)
    if n == 0:
        raise RuntimeError("Cannot sample from an empty source list for a bin.")
    if n >= k:
        return rng.sample(source, k)
    else:
        if require_full:
            raise RuntimeError(f"Not enough items in bin ({n} available, {k} requested) and --require-full was set.")
        if verbose:
            print(f"Warning: bin has only {n} items, sampling {k} with replacement to fill the quota.")
        # use choices to pick with replacement
        return [rng.choice(source) for _ in range(k)]


def curate(
    input_path: str,
    output_path: str,
    num_bins: int = 10,
    bin_size: int = 10,
    per_bin: int = 100,
    seed: int | None = None,
    require_full: bool = False,
    verbose: bool = False,
) -> None:
    # Load input
    with open(input_path, 'r') as f:
        data = json.load(f)

    if isinstance(data, dict) and 'problems' in data and isinstance(data['problems'], list):
        problems: List[Dict[str, Any]] = data['problems']
    elif isinstance(data, list):
        problems = data
    else:
        raise RuntimeError("Input JSON must be either a list of problems or an object with a 'problems' list.")

    if verbose:
        print(f"Loaded {len(problems)} problems from {input_path}")

    bins = make_bins(num_bins, bin_size)
    if verbose:
        print("Using bins (inclusive ranges):")
        for i, (low, high) in enumerate(bins):
            print(f"  bin {i}: {low} - {high}")

    rng = random.Random(seed)

    sampled: List[Dict[str, Any]] = []
    bin_counts = []

    for i, (low, high) in enumerate(bins):
        bucket = problems_in_bin(problems, low, high)
        bucket_size = len(bucket)
        bin_counts.append(bucket_size)
        if verbose:
            print(f"Bin {i} [{low}-{high}]: {bucket_size} available")

        if bucket_size == 0:
            # no problems in this bin; cannot proceed if we want per_bin results
            raise RuntimeError(f"No problems available in bin {i} range [{low}-{high}]. Aborting.")

        picks = sample_from_list(bucket, per_bin, rng, require_full=require_full, verbose=verbose)
        sampled.extend(picks)

    # Assign new problem_number indices and write output list
    out_list: List[Dict[str, Any]] = []
    for idx, prob in enumerate(sampled):
        # Shallow copy to avoid mutating original input dict if it's reused elsewhere
        pcopy = dict(prob)
        pcopy['problem_number'] = idx
        out_list.append(pcopy)

    # Save to output_path
    with open(output_path, 'w') as f:
        json.dump(out_list, f, indent=2)

    if verbose:
        print(f"Wrote {len(out_list)} curated problems to {output_path}")
        total_expected = num_bins * per_bin
        if len(out_list) != total_expected:
            print(f"Note: expected {total_expected} problems but produced {len(out_list)}")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Curate problems from a large JSON dataset into obstacle-count bins.")
    ap.add_argument("input", help="Input JSON file containing problems (list or {'problems': [...]})")
    ap.add_argument("output", help="Output JSON file for curated problems")
    ap.add_argument("--seed", type=int, default=None, help="Random seed for reproducible sampling")
    ap.add_argument("--bins", type=int, default=8, help="Number of bins (default 10)")
    ap.add_argument("--bin-size", type=int, default=10, help="Bin size in obstacle counts (default 10)")
    ap.add_argument("--per-bin", type=int, default=100, help="Number of problems to sample per bin (default 100)")
    ap.add_argument("--require-full", action="store_true", help="Require each bin to have at least per-bin distinct problems (fail otherwise)")
    ap.add_argument("--verbose", action="store_true", help="Verbose output")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    try:
        curate(
            input_path=args.input,
            output_path=args.output,
            num_bins=args.bins,
            bin_size=args.bin_size,
            per_bin=args.per_bin,
            seed=args.seed,
            require_full=args.require_full,
            verbose=args.verbose,
        )
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
