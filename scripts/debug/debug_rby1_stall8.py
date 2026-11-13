"""At the stall of the exact walk: which VAMP collision pairs (or joint limits) reject the candidates, for the starts whose walk stalls."""
import collections, contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

for sid in [int(a) for a in sys.argv[1:]]:
    with contextlib.redirect_stdout(io.StringIO()):
        problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
    path, rej = d.walk_reference(setup, "BoundedLS")
    if not rej:
        print(f"sid {sid}: walk reaches the end"); continue
    reasons = collections.Counter(w for w, _ in rej); pairs = collections.Counter(); oob = collections.Counter()
    for w, o in rej:
        if o is None: continue
        if w == "self-collision":
            for p in d.pair_names(setup, o): pairs[p] += 1
        elif w == "joint limit":
            for j in np.flatnonzero((o < model.lo) | (o > model.hi)): oob[model.joint_names[j].split("/")[-1]] += 1
    print(f"sid {sid}: walk stalls at t={path[-1][0]:.2f}; rejected: {dict(reasons)}")
    if pairs: print("     pairs:", dict(pairs.most_common(3)))
    if oob: print("     joints out of bounds:", dict(oob.most_common(3)))
