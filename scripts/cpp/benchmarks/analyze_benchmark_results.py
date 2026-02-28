#!/usr/bin/env python3
"""
Analyze benchmark results from N JSON files and create comparison plots.
Plots:
  1. num_obstacles vs success rate
  2. num_obstacles vs mean planning time
  3. num_obstacles vs median planning time
    4. num_obstacles vs mean path cost
"""

import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict


def load_results(json_file):
    """Load results from a JSON file."""
    with open(json_file, 'r') as f:
        data = json.load(f)
    return data


def compute_statistics(results):
    """
    Compute statistics grouped by number of obstacles.

    Returns:
        dict: {num_obstacles: {'success_count': int,
                               'total_count': int,
                               'success_rate': float,
                               'times_ms': [float],
                               'mean_time_ms': float,
                               'median_time_ms': float}}
    """
    stats_by_obstacles = defaultdict(lambda: {
        'success_count': 0,
        'total_count': 0,
        'times_ms': [],
        'path_costs': []
    })

    for result in results:
        num_obs = result['num_cuboid_obstacles']
        stats_by_obstacles[num_obs]['total_count'] += 1

        if result['success']:
            stats_by_obstacles[num_obs]['success_count'] += 1
            stats_by_obstacles[num_obs]['times_ms'].append(result['total_solve_time'])
            stats_by_obstacles[num_obs]['path_costs'].append(result.get('path_cost', 0.0))

    # Compute derived statistics
    for num_obs in stats_by_obstacles:
        stats = stats_by_obstacles[num_obs]
        total = stats['total_count']
        success = stats['success_count']

        stats['success_rate'] = (success / total * 100.0) if total > 0 else 0.0

        if stats['times_ms']:
            stats['mean_time_ms'] = np.mean(stats['times_ms'])
            stats['median_time_ms'] = np.median(stats['times_ms'])
        else:
            stats['mean_time_ms'] = 0.0
            stats['median_time_ms'] = 0.0

        if stats['path_costs']:
            stats['mean_path_cost'] = np.mean(stats['path_costs'])
        else:
            stats['mean_path_cost'] = 0.0

    return dict(stats_by_obstacles)


def plot_comparison(stats_list, labels, output_prefix="benchmark_comparison", stats_time_list=None):
    """Create four comparison plots for N methods.

    If stats_time_list is provided (list of stats dicts corresponding to stats_list),
    the planning time and path cost plots will be generated using those filtered stats
    (e.g., containing only problems where all planners succeeded). The success-rate
    plot always uses stats_list (the full data).
    """

    # Extract sorted obstacle counts from all methods (union over the rate stats)
    all_obstacles = sorted(set().union(*(stats.keys() for stats in stats_list)))

    # Create figure with 4 subplots
    fig, axes = plt.subplots(1, 4, figsize=(24, 5))

    # Plot 1: Success Rate vs Num Obstacles (always use full stats)
    ax = axes[0]
    for stats, label in zip(stats_list, labels):
        success_rate = [stats.get(obs, {}).get('success_rate', 0) for obs in all_obstacles]
        ax.plot(all_obstacles, success_rate, '-', label=label, linewidth=2)
    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Success Rate (%)', fontsize=12)
    ax.set_title('Success Rate vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_ylim([70, 105])

    # Determine which stats to use for time/cost plots:
    # If a filtered stats_time_list is provided, use it; otherwise fall back to stats_list.
    use_stats_for_time = stats_time_list if stats_time_list is not None else stats_list

    # Ensure the obstacle set for time/cost plots covers the union of keys in the chosen stats
    time_obstacles = sorted(set().union(*(s.keys() for s in use_stats_for_time)))

    # Plot 2: Mean Planning Time vs Num Obstacles
    ax = axes[1]
    for i, (stats, label) in enumerate(zip(stats_list, labels)):
        stats_for_time = use_stats_for_time[i]
        mean_time = [stats_for_time.get(obs, {}).get('mean_time_ms', 0) for obs in time_obstacles]
        ax.plot(time_obstacles, mean_time, '-', label=label, linewidth=2)
    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Mean Planning Time (ms)', fontsize=12)
    ax.set_title('Mean Planning Time vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)

    # Plot 3: Median Planning Time vs Num Obstacles (with quartile shading)
    ax = axes[2]

    # First pass: compute quartiles for each method and draw fills.
    # We draw all fills first so that median lines can be drawn on top to avoid being overwritten.
    median_data = []
    for i, (stats, label) in enumerate(zip(stats_list, labels)):
        stats_for_time = use_stats_for_time[i]

        q1_vals = []
        median_vals = []
        q3_vals = []
        for obs in time_obstacles:
            times = np.array(stats_for_time.get(obs, {}).get('times_ms', []))
            if times.size > 0:
                q1_vals.append(np.percentile(times, 10))
                median_vals.append(np.percentile(times, 50))
                q3_vals.append(np.percentile(times, 90))
            else:
                q1_vals.append(np.nan)
                median_vals.append(np.nan)
                q3_vals.append(np.nan)

        q1_arr = np.array(q1_vals)
        med_arr = np.array(median_vals)
        q3_arr = np.array(q3_vals)

        # Determine color for this method (use a plotted line color cycle if possible)
        # We'll draw the fill now (no label) and plot the median line later so it stays on top.
        color = None
        # peek at default cycle color for consistency; fall back to None to let matplotlib choose
        try:
            color = plt.rcParams['axes.prop_cycle'].by_key()['color'][i]
        except Exception:
            color = None

        ax.fill_between(time_obstacles, q1_arr, q3_arr,
                        where=~np.isnan(q1_arr) & ~np.isnan(q3_arr),
                        color=color, alpha=0.20, interpolate=True, zorder=1)

        median_data.append((med_arr, label, color))

    # Second pass: draw median lines on top of all fills so they are never overwritten.
    for i, (med_arr, label, color) in enumerate(median_data):
        ax.plot(time_obstacles, med_arr, '-', label=label, color=color, linewidth=3, zorder=10 + i)

    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Median Planning Time (ms)', fontsize=12)
    ax.set_title('Median Planning Time vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)

    # Plot 4: Path Cost vs Num Obstacles (plot median with quartile shading similar to timing)
    ax = axes[3]

    # First pass: compute quartiles/medians for path costs and draw fills.
    path_median_data = []
    for i, (stats, label) in enumerate(zip(stats_list, labels)):
        stats_for_time = use_stats_for_time[i]

        p1_vals = []
        pmed_vals = []
        p3_vals = []
        for obs in time_obstacles:
            costs = np.array(stats_for_time.get(obs, {}).get('path_costs', []))
            if costs.size > 0:
                p1_vals.append(np.percentile(costs, 25))
                pmed_vals.append(np.percentile(costs, 50))
                p3_vals.append(np.percentile(costs, 75))
            else:
                p1_vals.append(np.nan)
                pmed_vals.append(np.nan)
                p3_vals.append(np.nan)

        p1_arr = np.array(p1_vals)
        pmed_arr = np.array(pmed_vals)
        p3_arr = np.array(p3_vals)

        try:
            color = plt.rcParams['axes.prop_cycle'].by_key()['color'][i]
        except Exception:
            color = None

        ax.fill_between(time_obstacles, p1_arr, p3_arr,
                        where=~np.isnan(p1_arr) & ~np.isnan(p3_arr),
                        color=color, alpha=0.20, interpolate=True, zorder=1)

        path_median_data.append((pmed_arr, label, color))

    # Second pass: draw median path cost lines on top of fills
    for i, (pmed_arr, label, color) in enumerate(path_median_data):
        ax.plot(time_obstacles, pmed_arr, '-', label=label, color=color, linewidth=2, zorder=10 + i)

    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Median Path Cost', fontsize=12)
    ax.set_title('Median Path Cost vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    # Save figure
    output_file = f"{output_prefix}.png"
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Saved plot to: {output_file}")

    plt.show()


def print_statistics(stats_list, labels):
    """Print statistics summary for N methods."""
    all_obstacles = sorted(set().union(*(stats.keys() for stats in stats_list)))

    columns = ["Num Obs"]
    for label in labels:
        columns.extend([f"{label} Success", f"{label} Mean (ms)", f"{label} Median (ms)"])

    col_width = 24
    header = f"{columns[0]:<10}" + "".join(f"{col:<{col_width}}" for col in columns[1:])
    line_len = max(100, len(header))

    print("\n" + "=" * line_len)
    print(header)
    print("=" * line_len)

    for obs in all_obstacles:
        row = f"{obs:<10}"
        for stats in stats_list:
            s = stats.get(obs, {})
            success_str = f"{s.get('success_rate', 0):>6.1f}% ({s.get('success_count', 0):>2}/{s.get('total_count', 0):<2})"
            mean_str = f"{s.get('mean_time_ms', 0):>10.2f}"
            med_str = f"{s.get('median_time_ms', 0):>10.2f}"
            row += f"{success_str:<{col_width}}{mean_str:<{col_width}}{med_str:<{col_width}}"
        print(row)

    print("=" * line_len + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze and compare benchmark results from N JSON files"
    )
    parser.add_argument('json_files', nargs='+', help='JSON results files to compare (2 or more)')
    parser.add_argument('--labels', nargs='*', default=None, help='Optional labels matching json_files order')
    parser.add_argument('--output', default='benchmark_comparison', help='Output file prefix for plots')

    args = parser.parse_args()

    if len(args.json_files) < 2:
        parser.error("Provide at least two JSON files for comparison.")

    if args.labels is not None and len(args.labels) != len(args.json_files):
        parser.error("If provided, --labels must have exactly one label per JSON file.")

    labels = args.labels if args.labels is not None else [f"Method {i + 1}" for i in range(len(args.json_files))]

    # Load results
    all_results = []
    for json_file in args.json_files:
        print(f"Loading results from {json_file}...")
        results = load_results(json_file)
        print(f"  Loaded {len(results)} results")
        all_results.append(results)

    # Compute statistics
    print("\nComputing statistics...")
    stats_list = [compute_statistics(results) for results in all_results]

    # Determine problems where each planner succeeded, then find the intersection
    success_sets = []
    for results in all_results:
        s = set()
        for r in results:
            # Some result entries may not have 'problem_number' key; guard against that
            if r.get('success') and ('problem_number' in r):
                s.add(r['problem_number'])
        success_sets.append(s)

    if success_sets:
        common_success_problems = set.intersection(*success_sets)
    else:
        common_success_problems = set()

    if common_success_problems:
        print(f"Found {len(common_success_problems)} problems where all planners succeeded. "
              "Timing and path length stats will be computed only for these problems.")
        # Filter each method's results to only those problems that are in the common set
        filtered_results = []
        for results in all_results:
            fr = [r for r in results if ('problem_number' in r) and (r['problem_number'] in common_success_problems)]
            filtered_results.append(fr)
        stats_time_list = [compute_statistics(fr) for fr in filtered_results]
    else:
        print("No problems found where all planners succeeded. Timing/path plots will use empty data.")
        stats_time_list = None

    # Print summary
    print_statistics(stats_list, labels)

    # Create plots (pass filtered stats for timing/cost plots)
    print("Creating plots...")
    plot_comparison(stats_list, labels, args.output, None)


if __name__ == '__main__':
    main()
