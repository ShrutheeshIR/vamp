#!/usr/bin/env python3
"""
Analyze benchmark results from two JSON files and create comparison plots.
Plots: 
  1. num_obstacles vs success rate
  2. num_obstacles vs mean planning time
  3. num_obstacles vs median planning time
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
        'times_ms': []
    })
    
    for result in results:
        num_obs = result['num_cuboid_obstacles']
        stats_by_obstacles[num_obs]['total_count'] += 1
        
        if result['success']:
            stats_by_obstacles[num_obs]['success_count'] += 1
            stats_by_obstacles[num_obs]['times_ms'].append(result['total_solve_time'])
    
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
    
    return dict(stats_by_obstacles)


def plot_comparison(stats1, stats2, label1, label2, output_prefix="benchmark_comparison"):
    """Create three comparison plots."""
    
    # Extract sorted obstacle counts
    all_obstacles = sorted(set(list(stats1.keys()) + list(stats2.keys())))
    
    # Prepare data
    success_rate_1 = [stats1.get(obs, {}).get('success_rate', 0) for obs in all_obstacles]
    success_rate_2 = [stats2.get(obs, {}).get('success_rate', 0) for obs in all_obstacles]
    
    mean_time_1 = [stats1.get(obs, {}).get('mean_time_ms', 0) for obs in all_obstacles]
    mean_time_2 = [stats2.get(obs, {}).get('mean_time_ms', 0) for obs in all_obstacles]
    
    median_time_1 = [stats1.get(obs, {}).get('median_time_ms', 0) for obs in all_obstacles]
    median_time_2 = [stats2.get(obs, {}).get('median_time_ms', 0) for obs in all_obstacles]
    
    # Create figure with 3 subplots
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # Plot 1: Success Rate vs Num Obstacles
    ax = axes[0]
    ax.plot(all_obstacles, success_rate_1, '-', label=label1, linewidth=2)
    ax.plot(all_obstacles, success_rate_2, '-', label=label2, linewidth=2)
    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Success Rate (%)', fontsize=12)
    ax.set_title('Success Rate vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_ylim([0, 105])
    
    # Plot 2: Mean Planning Time vs Num Obstacles
    ax = axes[1]
    ax.plot(all_obstacles, mean_time_1, '-', label=label1, linewidth=2)
    ax.plot(all_obstacles, mean_time_2, '-', label=label2, linewidth=2)
    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Mean Planning Time (ms)', fontsize=12)
    ax.set_title('Mean Planning Time vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Median Planning Time vs Num Obstacles
    ax = axes[2]
    ax.plot(all_obstacles, median_time_1, '-', label=label1, linewidth=2)
    ax.plot(all_obstacles, median_time_2, '-', label=label2, linewidth=2)
    ax.set_xlabel('Number of Obstacles', fontsize=12)
    ax.set_ylabel('Median Planning Time (ms)', fontsize=12)
    ax.set_title('Median Planning Time vs Number of Obstacles', fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save figure
    output_file = f"{output_prefix}.png"
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Saved plot to: {output_file}")
    
    plt.show()


def print_statistics(stats1, stats2, label1, label2):
    """Print statistics summary."""
    all_obstacles = sorted(set(list(stats1.keys()) + list(stats2.keys())))
    
    print("\n" + "="*100)
    print(f"{'Num Obs':<10} {label1 + ' Success':<20} {label2 + ' Success':<20} {label1 + ' Mean (ms)':<20} {label2 + ' Mean (ms)':<20}")
    print("="*100)
    
    for obs in all_obstacles:
        s1 = stats1.get(obs, {})
        s2 = stats2.get(obs, {})
        
        sr1 = s1.get('success_rate', 0)
        sr2 = s2.get('success_rate', 0)
        mean1 = s1.get('mean_time_ms', 0)
        mean2 = s2.get('mean_time_ms', 0)
        
        print(f"{obs:<10} {sr1:>6.1f}% ({s1.get('success_count', 0):>2}/{s1.get('total_count', 0):<2})  "
              f"{sr2:>6.1f}% ({s2.get('success_count', 0):>2}/{s2.get('total_count', 0):<2})  "
              f"{mean1:>10.2f}              {mean2:>10.2f}")
    
    print("="*100 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze and compare benchmark results from two JSON files"
    )
    parser.add_argument('json_file_1', help='First JSON results file')
    parser.add_argument('json_file_2', help='Second JSON results file')
    parser.add_argument('--label1', default='Method 1', help='Label for first method')
    parser.add_argument('--label2', default='Method 2', help='Label for second method')
    parser.add_argument('--output', default='benchmark_comparison', help='Output file prefix for plots')
    
    args = parser.parse_args()
    
    # Load results
    print(f"Loading results from {args.json_file_1}...")
    results1 = load_results(args.json_file_1)
    print(f"  Loaded {len(results1)} results")
    
    print(f"Loading results from {args.json_file_2}...")
    results2 = load_results(args.json_file_2)
    print(f"  Loaded {len(results2)} results")
    
    # Compute statistics
    print("\nComputing statistics...")
    stats1 = compute_statistics(results1)
    stats2 = compute_statistics(results2)
    
    # Print summary
    print_statistics(stats1, stats2, args.label1, args.label2)
    
    # Create plots
    print("Creating plots...")
    plot_comparison(stats1, stats2, args.label1, args.label2, args.output)


if __name__ == '__main__':
    main()
