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
    mask = 0

    # 0th bit
    proc = subprocess.run(
        ["./build/vamp_crrtc_spheres", "--obstacles", filepath],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    print(f"vamp_crrtc_spheres exited with {proc.returncode}")
    if proc.returncode != 0:
        mask |= 1 << 0

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/home/liu3447/ompl_install/lib"

    # 1st bit
    proc = subprocess.run(
        ["./build/vamp_ompl_integration", "--obstacles", filepath],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    print(f"vamp_ompl_integration exited with {proc.returncode}")
    if proc.returncode != 0:
        mask |= 1 << 1

    # 2nd bit
    proc = subprocess.run(
        ["./build/vamp_ompl_integration_worse", "--obstacles", filepath],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    print(f"vamp_ompl_integration_worse exited with {proc.returncode}")
    if proc.returncode != 0:
        mask |= 1 << 2
    return mask


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

    if (result < 7):
        print("At least one succeeded! File saved")
    else:
        if os.path.exists(filepath):
            os.remove(filepath)
        print("Problem is infeasible, file deleted")

    return result
            


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
