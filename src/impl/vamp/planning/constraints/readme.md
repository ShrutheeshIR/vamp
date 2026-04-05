# Planning with Constraints

## Defining Constraints
To enable planning with constraints, the constraints must be defined. `task_space_constraint.hh` provides an example of defining a constraint. Each constraint requires 2 interfaces:

1. `distanceToConstraint(q)` - takes in a `Robot::ConfigurationBlock` set of configurations, and computes the distance to the constraint. Internally, it could also optionally compute the jacobian of the computation for future re-use.
2. `projectStep(q, q_new, projection_method, learning_rate)` - performs one step of the projection operation. 

To speed-up computations, the distance and descent direction operations are typically trace compiled using the [Cricket](https://github.com/ShrutheeshIR/cricket/tree/tsr_constraints) package.


An example of creating a TSR constraint:

```
vamp::planning::constraint::TaskSpaceConstraint<Robot, rake> tsr_constraint(
    eef_transforms_ref_frame_w_world,
    eef_transforms,
    tsr_lower_bound,
    tsr_upper_bound);
```

## Using Constraints in Planning

To use constraints in the current planning framework, the interface `ComposableConstraint` exposes a generic class. To create an instance with 2 constraints:

```
vamp::planning::constraint::ComposableConstraints<
    Robot,
    rake,
    decltype(tsr_constraint),
    decltype(com_constraint),
    task_constraint(
        tsr_constraint, com_constraint);
```

### Implementation details
The file [composable_constraint](composable_constraint.hh) provides the interface. It takes in an arbitrary number of constraints, and exposes 4 operations:
1. `distanceToConstraint(q)` - It computes the individual distance to each constraint and sums them up.
2. `projectStep(q, q_new, projection_method, learning_rate)` - Performs one step of projection to each constraint in a sequential fashion.
3. `projectConfiguration(q, q_new, projection_method, max_q_dist, descent_rate, num_iters)` - performs upto `num_iters` number of `projectStep` operations. It early exits if (i) all configurations are projected successfully, or (ii) if none of them make any meaningful progress or (iii) if any of them project farther than `max_q_dist` distance away. To disable (iii), set `max_q_dist` to infinity or a large number
4. `projectAnyConfiguration` - is similar to `projectConfiguration`, but it exits if any single configuration in the simd vector succeeds instead of all projections.

# Settings
[ConstraintSettings](constraint_settings.hh) provides some hyperparameters for supporting constraints with planning.


# Use in planning
The [validate_constraint_motion](validate_constraint_motion.hh) and [simplify_constraints](simplify_constraints.hh) use the above operations for planning. They are copies of [validate](../validate.hh) and [simplify](../simplify.hh) operations, but employ constraint based motion validation operations.. In the future, steps to integrate the constraint-based motion validations into the standard setup. The interesting hyperparameter for `validate_constraint_motion` is `std_dev_scaling_factor`, which specifies how far to sample around `q_steer` to project. 
