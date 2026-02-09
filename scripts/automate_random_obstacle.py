import subprocess
import sys
import os
import pprint
# Path to the other Python script
script_to_run = "scripts/random_obstacle.py"

MIN_OBSTACLES = 45
MAX_OBSTACLES = 105
STEPS = 15
ITERATIONS = 100
PLANNERS = {
    0: "vamp_crrtc_spheres",
    1: "vamp_ompl_integration",
    2: "vamp_ompl_integration_worse",
}

stats = {
    planner: {
        num_obstacle: {"success": 0, "total": 0}
        for num_obstacle in range(MIN_OBSTACLES, MAX_OBSTACLES, STEPS)
    }
    for planner in PLANNERS.values()
}

for num_obstacle in range(MIN_OBSTACLES, MAX_OBSTACLES, STEPS):
    for iteration in range(0, ITERATIONS):
        relative_filepath = os.path.join(str(num_obstacle), f"{iteration}.txt")

        proc = subprocess.run(
            [sys.executable, script_to_run, relative_filepath, str(num_obstacle)]
        )

        mask = proc.returncode
        print(
            f"obstacles={num_obstacle}, iteration={iteration}, mask={mask:03b}"
        )

        # Ignore cases where all planners failed
        if mask == 7:
            continue

        for bit, planner in PLANNERS.items():
            stats[planner][num_obstacle]["total"] += 1

            # success if bit == 0
            if not (mask & (1 << bit)):
                stats[planner][num_obstacle]["success"] += 1

percentages = {}

for planner, obstacle_data in stats.items():
    percentages[planner] = {}

    for num_obstacle, counts in obstacle_data.items():
        total = counts["total"]
        print("total is " + str(total) + " and num_obstacle " + str(num_obstacle))
        if total == 0:
            continue

        percentages[planner][num_obstacle] = (
            counts["success"] / total
        )
pprint.pprint(percentages)