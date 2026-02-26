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


def plot_comparison(stats_list, labels, output_prefix="benchmark_comparison"):
    """Create four comparison plots for N methods."""

    # Extract sorted obstacle counts from all methods
    all_obstacles = sorted(set().union(*(stats.keys() for stats in stats_list)))
    
    # Create figure with 4 subplots
    fig, axes = plt.subplots(1, 4, figsize=(24, 5))
    
    # Plot 1: Success Rate vs Num Obstacles
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
    
    # Plot 2: Mean Planning Time vs Num Obstacles
    ax = axes[1]
    for stats, label in zip(stats_list, labels):
        mean_time = [stats.get(obs, {}).get('mean_time_ms', 0) for obs in all_obstacles]
        ax.plot(all_obstacles, mean_time, '-', label=label, linewidth=2)
    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Mean Planning Time (ms)', fontsize=12)
    ax.set_title('Mean Planning Time vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Median Planning Time vs Num Obstacles
    ax = axes[2]
    for stats, label in zip(stats_list, labels):
        median_time = [stats.get(obs, {}).get('median_time_ms', 0) for obs in all_obstacles]
        ax.plot(all_obstacles, median_time, '-', label=label, linewidth=2)
    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Median Planning Time (ms)', fontsize=12)
    ax.set_title('Median Planning Time vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)

    # Plot 4: Mean Path Cost vs Num Obstacles
    ax = axes[3]
    for stats, label in zip(stats_list, labels):
        mean_path_cost = [stats.get(obs, {}).get('mean_path_cost', 0) for obs in all_obstacles]
        ax.plot(all_obstacles, mean_path_cost, '-', label=label, linewidth=2)
    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Mean Path Cost', fontsize=12)
    ax.set_title('Mean Path Cost vs Number of Obstacles', fontsize=13, fontweight='bold')
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
    
    # Print summary
    print_statistics(stats_list, labels)
    
    # Create plots
    print("Creating plots...")
    plot_comparison(stats_list, labels, args.output)


if __name__ == '__main__':
    main()
