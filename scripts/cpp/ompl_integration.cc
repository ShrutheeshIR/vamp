#include <vector>
#include <array>
#include <utility>
#include <csignal>
#include <iostream>
#include <fstream>
#include <vamp/vector.hh>
#include <vamp/vector/interface.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>

#include <vamp/robots/panda.hh>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Exception.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>
#include "problem_setup/plane_constraint_problem_setup.hh"
namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using ConfigurationBlock = typename Robot::template ConfigurationBlock<rake>;


// Maximum planning time
static constexpr float planning_time = 30.0;

// Maximum simplification time
static constexpr float simplification_time = 1.0;

// Helper function to convert projected state into a double veector
std::vector<double> extractStateReals(
    const ob::State *state,
    const ob::ProjectedStateSpace *space)
{
    // Cast the top-level state space to ProjectedStateSpace

    std::size_t dim = space->getDimension();

    std::vector<double> values(dim);
    space->copyToReals(values, state);

    return values;
}

//outputs into state. takes a double vector and turns it into state for projectedstatespace
void fillStateFromReals(
    const std::vector<double> &values,
    ob::State *state,
    const ob::ProjectedStateSpace *space)
{
    std::size_t dim = space->getDimension();

    if (values.size() != dim)
        throw std::runtime_error("Input size does not match ProjectedStateSpace dimension");

    space->copyFromReals(state, values);
}

//Helper function to turn double vector into vamp vector
inline static auto double_vector_to_vamp(const std::vector<double> &values) -> Configuration
{
    if (values.size() != dimension)
        throw std::runtime_error("Input vector size does not match robot dimension");

    // Create an aligned buffer for the VAMP configuration
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    // Copy the vector values into the aligned buffer
    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(values[i]);
    }

    // Construct a Configuration from the aligned buffer
    return Configuration(aligned_buffer.data());
}

inline static std::vector<double> vamp_to_double_vector(const Configuration &c)
{
    std::vector<double> values(dimension);

    for (size_t i = 0; i < dimension; ++i)
    {

        values[i] = static_cast<double>(c[{0, i}]);
    }

    return values;
}

// Convert an OMPL state into a VAMP vector
inline static auto ompl_to_vamp(const ob::State *state) -> Configuration
{
    // Create an aligned memory buffer to load VAMP vector from
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    // Copy OMPL data into aligned buffer
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(as->values[i]);
    }

    // Create configuration from aligned buffer data
    return Configuration(aligned_buffer.data());
}

// Convert a VAMP vector to an OMPL state
inline static auto vamp_to_ompl(const Configuration &c, ob::State *state)
{
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        as->values[i] = static_cast<double>(c[{i, 0}]);
    }
}

class CustomConstraint : public ob::Constraint
{
public:
    vamp::planning::TaskSpaceConstraint<Robot, rake>& original_taskspace_constraint;
private:
    std::weak_ptr<ob::ProjectedStateSpace> ps_space_;
public:
    CustomConstraint(vamp::planning::TaskSpaceConstraint<Robot, rake>& x)
        : ob::Constraint(6, 0), 
        original_taskspace_constraint(x)
    {
    }
 

    // Member fields
    void setProjectedStateSpace(const std::shared_ptr<ob::ProjectedStateSpace>& space)
    {
        ps_space_ = space;
    }
    std::shared_ptr<ob::ProjectedStateSpace> getProjectedStateSpace() const
    {
        return ps_space_.lock();   // Returns nullptr if expired
    }
    
    ConfigurationBlock turn_configuration_into_configuration_block(const Robot::Configuration &c) const{
        typename Robot::template ConfigurationBlock<rake> block;

        for (auto i = 0U; i < Robot::dimension; ++i) {
            block[i] = c.broadcast(i) + 0.0;
        }
        return block;
    }
       
    // Here we define the actual constraint function, which takes in some state "x"
    // (from the ambient space) and sets the values of "out" to the result of the
    // constraint function. Note that we are implementing `function` which has this
    // function signature, not the one that takes in ompl::base::State.
    // for now, I left out the function, since it might not be necessary.
    bool project(ob::State *state) const override
    {
        //std::cout << "Our project state version is called!\n";
        std::shared_ptr<ob::ProjectedStateSpace> pss_ptr = getProjectedStateSpace();
        auto pss = pss_ptr.get();
        if (!pss) {
            throw std::runtime_error("ProjectedStateSpace member field weak pointer no longer exists!");
        }
        //std::cout << "At the very begnning of project, the state is: \n";
        //pss->printState(state, std::cout);
        Configuration c = double_vector_to_vamp(extractStateReals(state, pss));
        //raise(SIGTRAP);
        
        bool result = project(c);
        //TODO: after c has been modified, convert the Configuration c into a double vector.
        std::vector<double> newReals = vamp_to_double_vector(c);
        //std::cout << "After converting vamp to double vector, we get: ";
        //for (double d : newReals) {
        //    std::cout << d << ", ";
        //}
        //std::cout << "\n";
        fillStateFromReals(newReals, state, pss);
        //std::cout << "At the end, we get: \n";
        //pss->printState(state, std::cout);
        return result;

    }
    double distance(const ob::State *state) const override
    {
        auto configuration = ompl_to_vamp(state);
        return distance(configuration);
    }

    bool isSatisfied(const ob::State *state) const override
    {
        //std::cout << "our custom implementation of isSatisfied is called!\n";
        return distance(state) < 0.00001;
    }

    bool project(Configuration &configuration) const
    {
        //std::cout << "Project: COnfiguration is " << configuration << "\n";
        ConfigurationBlock block = turn_configuration_into_configuration_block(configuration);
        //std::cout << "After converted to configuration block, it is " << block << "\n";
        bool result = original_taskspace_constraint.projectConfiguration(block, block);
        //std::cout << "After pojection, the configuration block is " << block << "\n";
        //std::cout << "Result of projection is " << result << "\n";
        typename Robot::ConfigurationArray last_projected;
        for (auto i = 0; i < rake; i++)
        {
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected[j] = block[{j, i}];
            }
        }
        configuration = Robot::Configuration(last_projected);
        //std::cout << "After Project: COnfiguration is " << configuration << "\n";
        return result;

    } 
    double distance(const Configuration &configuration) const
    {
        ConfigurationBlock block =  turn_configuration_into_configuration_block(configuration);
        //std::cout << "block is" << block << "\n";
        auto simd_float_vector = original_taskspace_constraint.distanceToConstraint(block);
        //std::cout << "distanceToConstraint returned " << simd_float_vector << "\n";
        //std::cout << "is the start position on the manifold?? Answer: " << simd_float_vector.test_all_less_equal(0.0001F) << "\n";
        double dist = simd_float_vector[{0, 0}];
        //std::cout << "dist is" << dist << "\n";
        return dist;
    }

    bool isSatisfied(const Configuration &configuration) const
    {
        //std::cout << "our custom implementation of isSatisfied is called! CONFIGURATION version\n";
        return distance(configuration) < 0.0001;
    }
    // Fully dont know what is going on, need to ask shru
    void function(const Eigen::Ref<const Eigen::VectorXd> &x,
              Eigen::Ref<Eigen::VectorXd> out) const override
    {
        std::cout << "THIS IS REALLY BAD!!! FUNCTION GOT CALLED!";
        return;
    }

};

// State validator using VAMP
struct VAMPStateValidator : public ob::StateValidityChecker
{
    const EnvironmentVector &env_v;
    VAMPStateValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        // Convert OMPL to VAMP vector and validate
        //std::cout << "our implemtnation of is valid is called!\n";
       auto *pss = dynamic_cast<ob::ProjectedStateSpace*>(si_->getStateSpace().get());
        if (!pss)
            throw ompl::Exception("Expected ProjectedStateSpace");

        // Step 2. Get the OMPL Constraint object
        auto constraint_base = pss->getConstraint();

        // Step 3. Downcast it to CustomConstraint
        auto *custom_constraint = dynamic_cast<CustomConstraint*>(constraint_base.get());
        if (!custom_constraint)
            throw ompl::Exception("Expected CustomConstraint");

        Configuration c = double_vector_to_vamp(extractStateReals(state, pss));
        //auto *projState = state->as<ob::ProjectedStateSpace::StateType>();
        //const ob::State *ambient = projState->getAmbientState();
        //si_->getStateSpace()->printState(ambient, std::cout);
        auto result = custom_constraint->isSatisfied(c);
        //std::cout << "At the end of isValid for stateValdiator, is valid returns " << result << "\n";
        return result;
    }
    
};

struct VAMPMotionValidator : public ob::MotionValidator
{
    const EnvironmentVector &env_v;
    VAMPMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    VAMPMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        // Convert OMPL states to VAMP vectors and check motion between states
        auto *pss = dynamic_cast<ob::ProjectedStateSpace*>(si_->getStateSpace().get());
        if (!pss)
            throw ompl::Exception("Expected ProjectedStateSpace");

        // Step 2. Get the OMPL Constraint object
        auto constraint_base = pss->getConstraint();

        // Step 3. Downcast it to your CustomConstraint
        auto *custom_constraint = dynamic_cast<CustomConstraint*>(constraint_base.get());
        if (!custom_constraint)
            throw ompl::Exception("Expected CustomConstraint");

        // Step 4. Convert state into VAMP configuration
        //std::cout << "Check motion called with these following states:\n";
        //State(s1, std::cout);
        //pss->printState(s2, std::cout);
        Configuration configuration1 = double_vector_to_vamp(extractStateReals(s1, pss));;
        Configuration configuration2 = double_vector_to_vamp(extractStateReals(s2, pss));;
        //std::cout << "configuration 1 for checkMotion is " << configuration1 << "\n";
        //std::cout << "configuration 2 for checkMotion is " << configuration2 << "\n";
        std::vector<Configuration> projected_vector_dummy;

        // Step 5. Call VAMP’s constrained validator
        auto result = vamp::planning::project_constraint_motion<Robot, rake, 1>(
            configuration1,
            configuration2,
            projected_vector_dummy,
            custom_constraint->original_taskspace_constraint,
            env_v
        );
        //std::cout << "The result of project_constraint_motion is " << result << "\n";
        return result;
        //return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
            //ompl_to_vamp(s1), ompl_to_vamp(s2), env_v);
    }

    auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const
        -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }

    
};

auto main(int argc, char **) -> int
{

    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(in_hand_pose, target_pose, std::make_pair(lower_bound, upper_bound));

    bool optimize = false;  // Flag - if true, will spend entire planning budget optimizing, otherwise exit on
                            // first solution

    // Set optimize flag if another argument is provided
    if (argc == 2)
    {
        optimize = true;
    }

    // Build sphere cage environment
    EnvironmentInput environment;
    std::ofstream outfile_sph("spheres.txt");
    for (const auto &sphere : problem)
    {
        outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }
    outfile_sph.close();
    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Create OMPL state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);

    // Get bounds from VAMP Robot information, scale 0/1 config to min/max
    static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
    static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

    auto zero_v = Configuration(zeros);
    auto one_v = Configuration(ones);
    Robot::scale_configuration(zero_v);
    Robot::scale_configuration(one_v);


    ob::RealVectorBounds bounds(dimension);
    for (auto i = 0U; i < dimension; ++i)
    {
        bounds.setLow(i, zero_v[{0, i}]);
        bounds.setHigh(i, one_v[{0, i}]);
    }
    
    space->setBounds(bounds);
    
    // Set up the constraint manifold
    std::shared_ptr<ob::Constraint> constraint = std::make_shared<CustomConstraint>(task_constraint);


    // Combine the ambient space and the constraint into a constrained state space.
    auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
    auto cc = std::dynamic_pointer_cast<CustomConstraint>(constraint);
    if (!cc)
        throw std::runtime_error("Constraint is not a CustomConstraint");

    cc->setProjectedStateSpace(css);
    // Define the constrained space information for this constrained state space.
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    // Create space information and set state validator and custom VAMP motion validator
    //auto si = std::make_shared<ob::SpaceInformation>(space);
    

    csi->setStateValidityChecker(std::make_shared<VAMPStateValidator>(csi, env_v));
    csi->setMotionValidator(std::make_shared<VAMPMotionValidator>(csi, env_v));
    csi->setup();

    // Set start and goal
    ob::ScopedState<> start_ompl(css), goal_ompl(css);
    for (auto i = 0U; i < dimension; ++i)
    {
        std::cout << start[i] << "original elemnt at index " << i << "\n";
        start_ompl[i] = start[i];
        goal_ompl[i] = goal[i];
        
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(csi);
    pdef->setStartAndGoalStates(start_ompl, goal_ompl);
    std::cout << "Start valid? " << csi->isValid(start_ompl.get()) << "\n";
    std::cout << "Goal valid? " << csi->isValid(goal_ompl.get()) << "\n";

    // Set optimization objective
    auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
    pdef->setOptimizationObjective(obj);

    if (not optimize)
    {
        // Set planner to terminate as soon as a solution is found.
        obj->setCostThreshold(obj->infiniteCost());
    }

    // Create planner - BITstar by default, but you can change this to use other geometric planners instead
    auto planner = std::make_shared<og::RRTConnect>(csi);

    planner->setProblemDefinition(pdef);
    std::cout << "default range is "<< planner->getRange() <<"\n";
    //planner->setRange(1.0);
    planner->setup();

    // Solve the problem
    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerStatus solved = planner->ob::Planner::solve(planning_time);
    auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    if (solved)
    {   
        og::PathGeometric *path =
        pdef->getSolutionPath()->as<og::PathGeometric>();
        std::ofstream outfile("trajectory.txt");
        std::cout << "Raw path length: " << path->length() << std::endl;
        std::cout << "Raw path states: " << path->getStateCount() << std::endl;

        path->print(std::cout);
        for (std::size_t i = 0; i < path->getStateCount(); ++i)
        {
            const auto *state =
                path->getState(i)->as<ob::RealVectorStateSpace::StateType>();

            for (std::size_t j = 0; j < dimension; ++j)
            {
                if (j > 0)
                    outfile << ",";
                outfile << state->values[j];
            }
            outfile << "\n";
        }
        outfile.close();
    }
    // Only accept exact solutions
    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "Found solution in " << nanoseconds / 1e6 << "ms! Simplfying..." << std::endl;

        // Simplify the path using OMPL's path simplification
        const ob::PathPtr &path = pdef->getSolutionPath();
        og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);

        auto initial_cost = path_geometric.cost(obj);

        og::PathSimplifier simplifier(csi, pdef->getGoal(), obj);
        if (not simplifier.simplify(path_geometric, simplification_time))
        {
            std::cout << "Path not valid!" << std::endl;
        }

        auto simplified_cost = path_geometric.cost(obj);

        // Output statistics
        std::cout << "Found initial solution with cost " << initial_cost.value() << std::endl;
        std::cout << "Simplified solution to cost " << simplified_cost.value() << std::endl;
        std::cout << "Simplified solution:" << std::endl;

        path_geometric.print(std::cout);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    return 0;
}
