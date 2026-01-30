#!/usr/bin/env python3

import os
import sys
import random
import subprocess

def create_obstacles(filepath: str, n: int):
    # Bounds for x, y, z
    x_bounds = (-1.0, 1.0)
    y_bounds = (-1.0, 1.0)
    z_bounds = (-0.2, 1.2)

    # Ensure directory exists
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    # Generate obstacles and write to file
    with open(filepath, "w") as f:
        for _ in range(n):
            x = random.uniform(*x_bounds)
            y = random.uniform(*y_bounds)
            z = random.uniform(*z_bounds)
            f.write(f"{x} {y} {z}\n")

    print(f"{n} obstacles written to {filepath}")

def run_executables(filepath):
    exit_codes = {}

    # First executable
    result = subprocess.run(
        ["./build/vamp_crrtc_spheres", "--obstacles", filepath],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    exit_codes["vamp_crrtc_spheres"] = result.returncode

    # Environment for the next two
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/home/liu3447/ompl_install/lib"

    # Second executable
    result = subprocess.run(
        ["./build/vamp_ompl_integration", "--obstacles", filepath],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    exit_codes["vamp_ompl_integration"] = result.returncode

    # Third executable
    result = subprocess.run(
        ["./build/vamp_ompl_integration_worse", "--obstacles", filepath],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    exit_codes["vamp_ompl_integration_worse"] = result.returncode

    # Print results
    print("Exit codes:")
    result = 1
    for exe, code in exit_codes.items():
        print(f"{exe}: {code}")
        result = result and code


    return result

    


def main():
    if len(sys.argv) != 3:
        print("Usage: python create_obstacles.py <relative_filepath> <n>")
        sys.exit(1)

    user_path = sys.argv[1]  # e.g., "extra_folder/obstacle1.txt"
    n = int(sys.argv[2])

    base_path = "scripts/cpp/problem_setup/obstacle_files"
    filepath = os.path.join(base_path, user_path)

    create_obstacles(filepath, n)

    print("\nRunning executables...\n")
    result = run_executables(filepath)
    print("\nAll executables finished.")

    if (result == 0):
        print("At least one succeeded! File saved")
    else:
        if os.path.exists(filepath):
            os.remove(filepath)
        print("Problem is infeasible, file deleted")
        


if __name__ == "__main__":
    main()
