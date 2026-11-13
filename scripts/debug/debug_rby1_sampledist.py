import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
s = d.module.halton(); Q = np.array([s.next() for _ in range(2000)], float)
dist = np.linalg.norm(Q - setup.q_start, axis=1)
print(f"\ndistance from q_start to a Halton sample (23 joints, rad/m): median {np.median(dist):.2f}, 10th-90th percentile {np.percentile(dist,10):.2f}-{np.percentile(dist,90):.2f}")
print("sample range per joint group (min..max):", {n: (round(float(Q[:,i].min()),1), round(float(Q[:,i].max()),1)) for i, n in enumerate(model.joint_names) if i in (0, 1, 2, 3, 9, 16)})
