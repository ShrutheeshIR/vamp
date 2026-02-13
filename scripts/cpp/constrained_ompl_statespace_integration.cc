#include <vector>
#include <array>
#include <utility>
#include <csignal>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vamp/vector.hh>
#include <vamp/vector/interface.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>

#include <sstream>
#include <string>
#include <nlohmann/json.hpp>

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
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <csignal>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using ConfigurationBlock = typename Robot::template ConfigurationBlock<rake>;
using AttachmentInput = vamp::collision::Attachment<float>;


// Maximum planning time
static constexpr float planning_time = 100.0;
static constexpr int maxIterations = 100000;

// Maximum simplification time
static constexpr float simplification_time = 1.0;


static bool load_cuboids_from_json(EnvironmentInput &environment, const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return false;
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception &e) {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return false;
    }

    if (!j.is_array()) {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return false;
    }

    for (const auto &obj : j) {
        if (!obj.is_object()) {
            std::cerr << "Skipping non-object element in array" << std::endl;
            continue;
        }

        // required fields
        if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z") ||
            !obj.contains("dx") || !obj.contains("dy") || !obj.contains("dz")) {
            std::cerr << "Skipping object missing required fields (x,y,z,dx,dy,dz)" << std::endl;
            continue;
        }

        try {
            float x = obj.at("x").get<float>();
            float y = obj.at("y").get<float>();
            float z = obj.at("z").get<float>();
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>();

            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (obj.contains("roll")) roll = obj.at("roll").get<float>();
            if (obj.contains("pitch")) pitch = obj.at("pitch").get<float>();
            if (obj.contains("yaw")) yaw = obj.at("yaw").get<float>();

            std::array<float,3> posf = {x, y, z};
            std::array<float,3> rotf = {roll, pitch, yaw};
            std::array<float,3> sizef = {dx / 2, dy / 2, dz / 2};
            // std::cout << "Creating cuboid at position: " << posf[0] << ", " << posf[1] << ", " << posf[2] << " with size: " << sizef[0] << ", " << sizef[1] << ", " << sizef[2] << std::endl;
            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(posf, rotf, sizef));
        } catch (const std::exception &e) {
            std::cerr << "Error reading object fields: " << e.what() << " -- skipping object" << std::endl;
            continue;
        }
    }

    return true;
}


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

    std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> temp_array;
    c.to_array(temp_array.data());

    for (size_t i = 0; i < dimension; ++i)
    {

        values[i] = static_cast<double>(temp_array[i]);
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
    std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> temp_array;
    c.to_array(temp_array.data());

    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        as->values[i] = static_cast<double>(temp_array[i]);
    }
}



class CustomConstraint : public ob::Constraint
{
public:
    mutable vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> constraints;
    mutable size_t num_failed_projections = 0;

public:
    CustomConstraint(vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>& x)
        : ob::Constraint(Robot::dimension, Robot::dimension - 6, 0.0001),
        constraints(x)
    {
    }


    ConfigurationBlock turn_configuration_into_configuration_block(const Robot::Configuration &c) const{
        typename Robot::template ConfigurationBlock<rake> block;

        for (auto i = 0U; i < Robot::dimension; ++i) {
            block[i] = c.broadcast(i) + 0.0;
        }
        return block;
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        // need to convert x to a configuration block
        std::array<float, Robot::dimension> float_config_from_x;
        for (auto i = 0U; i < Robot::dimension; ++i) {
            // cast x[i] to float
            float_config_from_x[i] = static_cast<float>(x[i]);
        }
        auto config_block = turn_configuration_into_configuration_block(Configuration(float_config_from_x));
        vamp::FloatVector<rake, 1> distance = constraints.distanceToConstraint(config_block);
        double dist = 0.0;
        // get the first element of distance
        out[0] = distance[{0, 0}];
    }

    bool project (Eigen::Ref< Eigen::VectorXd > x) const override
    {
        std::array<float, Robot::dimension> float_config_from_x;
        for (auto i = 0U; i < Robot::dimension; ++i) {
            // cast x[i] to float
            float_config_from_x[i] = static_cast<float>(x[i]);
        }
        auto config_block = turn_configuration_into_configuration_block(Configuration(float_config_from_x));
        ConfigurationBlock last_projected_block;
        // bool result = true;
        // std::cout << "Projecting configuration block...";
        // return true;

        bool result = constraints.projectConfiguration(config_block, last_projected_block, vamp::planning::ProjMethod::InnerLM, 5.0, 1.0, 25, false);
        if (result){
            for (auto i = 0U; i < Robot::dimension; ++i) {
                x[i] = static_cast<double>(last_projected_block[{i, rake - 1}]);
                // std::cout << x[i] << " ";
            }
        }
        // std::cout << "Projection result " << result << std::endl;
        return result;

    }

    double distance(const Eigen::Ref<const Eigen::VectorXd> &x) const override {
        // need to convert x to a configuration block
        std::array<float, Robot::dimension> float_config_from_x;
        for (auto i = 0U; i < Robot::dimension; ++i) {
            // cast x[i] to float
            float_config_from_x[i] = static_cast<float>(x[i]);
        }
        auto config_block = turn_configuration_into_configuration_block(Configuration(float_config_from_x));
        vamp::FloatVector<rake, 1> distance = constraints.distanceToConstraint(config_block);
        // std::cout << "distance: " << distance[{0, 0}] << std::endl;
        // cast distance[{0,0}]
        return static_cast<double>(distance[{0, 0}]);
    }


    bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
    {
        bool result = distance(x) < 0.0001;
        // std::cout << "our custom implementation of isSatisfied is called " << result << std::endl;
        return result;
    }

};

bool obstacle(const ob::State *state)
{
    // As ob::ConstrainedStateSpace::StateType inherits from
    // Eigen::Map<Eigen::VectorXd>, we can grab a reference to it for some easier
    // state access.
    const Eigen::Map<Eigen::VectorXd> &x = *state->as<ob::ConstrainedStateSpace::StateType>();

    // Alternatively, we could access the underlying real vector state with the
    // following incantation:
    //   auto x = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<ob::RealVectorStateSpace::StateType>();
    // Note the use of "getState()" on the constrained state. This accesss the
    // underlying state that was allocated by the ambient state space.

    // Define a narrow band obstacle with a small hole on one side.
    // if (-0.1 < x[2] && x[2] < 0.1)
    // {
    //     if (-0.05 < x[0] && x[0] < 0.05)
    //         return x[1] < 0;

    //     return false;
    // }
    std::cout << "Checking obstacle" << std::endl;
    return true;
}


struct VAMPStateValidator : public ob::StateValidityChecker
{
    VAMPStateValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v, vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint)
      : ob::StateValidityChecker(si), env_v(env_v), task_constraint(task_constraint)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v, vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint)
      : ob::StateValidityChecker(si), env_v(env_v), task_constraint(task_constraint)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        // Convert OMPL to VAMP vector and validate
        // auto configuration = ompl_to_vamp(state);

        const Eigen::Map<Eigen::VectorXd> &x = *state->as<ob::ConstrainedStateSpace::StateType>();

        std::array<float, Robot::dimension> float_config_from_x;
        for (auto i = 0U; i < Robot::dimension; ++i) {
            // cast x[i] to float
            float_config_from_x[i] = static_cast<float>(x[i]);
        }
        Configuration robot_config(float_config_from_x);

        std::vector <typename Robot::Configuration> projected_vector;
        bool projection_result = vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(robot_config, robot_config, projected_vector, task_constraint, env_v, vamp::planning::ProjMethod::InnerLM, 0.5, 25, true);
        // std::cout << "Projection result: " << projection_result << std::endl;
        return projection_result;

        // std::vector<double> newReals = vamp_to_double_vector(projected_vector.back());
        // space->copyFromReals(state, values);


        // return vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v);
    }



    const EnvironmentVector &env_v;
    vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint;
};

struct VAMPMotionValidator : public ob::MotionValidator
{
    VAMPMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v, vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint)
      : ob::MotionValidator(si), env_v(env_v), task_constraint(task_constraint)
    {
    }

    VAMPMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v, vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint)
      : ob::MotionValidator(si), env_v(env_v), task_constraint(task_constraint)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {

        std::array<float, Robot::dimension> float_config_from_x1, float_config_from_x2;
        const Eigen::Map<Eigen::VectorXd> &x1 = *s1->as<ob::ConstrainedStateSpace::StateType>();
        const Eigen::Map<Eigen::VectorXd> &x2 = *s2->as<ob::ConstrainedStateSpace::StateType>();


        for (auto i = 0U; i < Robot::dimension; ++i) {
            // cast x[i] to float
            float_config_from_x1[i] = static_cast<float>(x1[i]);
            float_config_from_x2[i] = static_cast<float>(x2[i]);
        }
        Configuration robot_config_1(float_config_from_x1);
        Configuration robot_config_2(float_config_from_x2);

        std::vector <typename Robot::Configuration> projected_vector;
        bool projection_result = vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(robot_config_1, robot_config_2, projected_vector, task_constraint, env_v, vamp::planning::ProjMethod::InnerLM, 0.5, 25, true);
        return projection_result;
    }

    auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const
        -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }

    const EnvironmentVector &env_v;
    vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint;
};



// Constraints must inherit from the constraint base class. By default, a
// numerical approximation to the Jacobian of the constraint function is computed
// using a central finite difference.
class SphereConstraint : public ob::Constraint
{
public:
    // ob::Constraint's constructor takes in two parameters, the dimension of
    // the ambient state space, and the dimension of the real vector space the
    // constraint maps into. For our sphere example, as we are planning in R^3, the
    // dimension of the ambient space is 3, and as our constraint outputs one real
    // value the second parameter is one (this is also the co-dimension of the
    // constraint manifold).
    SphereConstraint() : ob::Constraint(3, 1, 1e-3)
    {
    }

    // Here we define the actual constraint function, which takes in some state "x"
    // (from the ambient space) and sets the values of "out" to the result of the
    // constraint function. Note that we are implementing `function` which has this
    // function signature, not the one that takes in ompl::base::State.
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        // The function that defines a sphere is f(q) = || q || - 1, as discussed
        // above. Eigen makes this easy to express:
        // std::cout << "Calculating  sphere constraint ..." << std::endl;
        out[0] = x.norm() - 1;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out = x.transpose().normalized();
    }
};

int main()
{
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(dimension);

    // ob::RealVectorBounds bounds(dimension);
    // bounds.setLow(-2);
    // bounds.setHigh(2);
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

    rvss->setBounds(bounds);


    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.01, -10.01, -0.01, -0.01, -0.01, -10.01
    };

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        10.01, 10.01, 0.01, 0.01, 0.01, 10.01
    };

    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783}};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

    vamp::planning::TaskSpaceConstraint<Robot, rake> tsr_constraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        tsr_lower_bound,
        tsr_upper_bound
    );
    vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> task_constraint(
        tsr_constraint
    );



    // Create a shared pointer to our constraint.
    // auto constraint = std::make_shared<SphereConstraint>();
    auto constraint = std::make_shared<CustomConstraint>(task_constraint);

    // Combine the ambient space and the constraint into a constrained state space.
    auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);

    // Define the constrained space information for this constrained state space.
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);

    // auto ss = std::make_shared<og::SimpleSetup>(csi);

    // Set the state validity checker in simple setup.
    // csi->setStateValidityChecker(obstacle);
    //
    //
    EnvironmentInput environment;

    static const std::vector<std::array<float, 3>> problem = {
        // {0.55, 0, 0.25},
        // {0.55, 0, 0.50},
        // {0.55, 0, 0.60},
        // {0.56, 0, 0.450},
        // {0.1, 0, 0.7},
        // {0.35, 0.35, 0.25},
        {0, 0.55, 0.25},
        {-0.55, 0, 0.25},
        {-0.35, -0.35, 0.25},
        {0, -0.55, 0.25},
        // {0.35, -0.35, 0.25},
        {0.35, 0.35, 0.8},
        {0, 0.55, 0.8},
        {-0.35, 0.35, 0.8},
        {-0.55, 0, 0.8},
        {-0.35, -0.35, 0.8},
        {0, -0.55, 0.8},
        {0.35, -0.35, 0.8},
    };
    // Radius for obstacle spheres
    // static constexpr float radius = 0.15;
    // for (const auto &sphere : problem)
    // {
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }

    // Try a few likely paths for the JSON file
    const std::vector<std::string> candidate_paths = {
        "/src/myfork/vamp/resources/environments/cuboids/real_maze.json",
    };

    bool loaded = false;
    for (const auto &p : candidate_paths) {
        if (load_cuboids_from_json(environment, p)) {
            std::cout << "Loaded cuboids from: " << p << std::endl;
            loaded = true;
            break;
        }
    }
    if (!loaded) {
        std::cerr << "Failed to load cuboids JSON from any candidate path. Exiting." << std::endl;
        return 1;
    }
    environment.sort();

    std::vector<vamp::collision::Sphere<float>> spheres;
    for(auto i=0U; i < 8; i++){
        spheres.push_back(vamp::collision::Sphere<float>(0.0, 0.0, i * 0.02, 0.01));
    }
    auto attach_transform = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    attach_transform.translation().z() = 0.0;
    AttachmentInput attachment(attach_transform);

    attachment.spheres.insert(attachment.spheres.end(), spheres.cbegin(), spheres.cend());
    environment.attach(attachment, 0);

    auto env_v = EnvironmentVector(environment);

    csi->setStateValidityChecker(std::make_shared<VAMPStateValidator>(csi, env_v, task_constraint));
    // csi->setMotionValidator(std::make_shared<VAMPMotionValidator>(csi, env_v, task_constraint));

    csi->setup();

    // si->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si, env_v));
    // si->setMotionValidator(std::make_shared<VAMPMotionValidator>(si, env_v));
    // si->setup();

    // Start and goal vectors.
    Eigen::VectorXd sv(dimension), gv(dimension);
    sv << -0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294;
    gv << 1.40490, 0.35201, -0.22762, -1.90963, 0.10796, 2.26183, 0.22238;

    // Scoped states that we will add to simple setup.
    ob::ScopedState<> start(css);
    ob::ScopedState<> goal(css);

    // Copy the values from the vectors into the start and goal states.
    start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
    goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);

    // If we were using an Atlas or TangentBundleStateSpace, we would also have to anchor these states to charts:
    //   css->anchorChart(start.get());
    //   css->anchorChart(goal.get());
    // Which gives a starting point for the atlas to grow.


    auto pdef = std::make_shared<ob::ProblemDefinition>(csi);
    pdef->setStartAndGoalStates(start, goal);

    // ss->setStartAndGoalStates(start, goal);

    auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
    pdef->setOptimizationObjective(obj);
    obj->setCostThreshold(obj->infiniteCost());


    auto planner = std::make_shared<og::RRTConnect>(csi);

    planner->setProblemDefinition(pdef);
    planner->setRange(0.5);
    planner->setup();


    // auto pp = std::make_shared<og::RRTConnect>(csi);
    // ss->setPlanner(pp);
    // ss->setup();
    // Solve a problem like normal, for 5 seconds.
    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerStatus stat = planner->ob::Planner::solve(planning_time);
    auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);


    if (stat)
    {
        std::cout << "Found solution in " << nanoseconds / 1e6 << "ms! Simplfying..." << std::endl;

        const ob::PathPtr &path = pdef->getSolutionPath();
        og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);

        auto initial_cost = path_geometric.cost(obj);

        og::PathSimplifier simplifier(csi, pdef->getGoal(), obj);


        std::ofstream outfile("/src/trajectory.txt");
        outfile << std::fixed << std::setprecision(10);
        // std::cout << "Raw path length: " << path->length() << std::endl;

        // path->print(std::cout);
        for (std::size_t i = 0; i < path_geometric.getStateCount(); ++i)
        {
            std::vector<double> result = extractStateReals(path_geometric.getState(i), css.get());

            for (std::size_t j = 0; j < dimension; ++j)
            {
                if (j > 0)
                    outfile << ",";
                outfile << result[j];
            }
            outfile << "\n";
        }
        outfile.close();

        // if (not simplifier.simplify(path_geometric, simplification_time))
        // {
        //     std::cout << "Path not valid!" << std::endl;
        // }

        // auto simplified_cost = path_geometric.cost(obj);

        // // Output statistics
        // std::cout << "Found initial solution with cost " << initial_cost.value() << std::endl;
        // std::cout << "Simplified solution to cost " << simplified_cost.value() << std::endl;
        // std::cout << "Simplified solution:" << std::endl;

        path_geometric.print(std::cout);


        // // Path simplification also works when using a constrained state space!
        // ss->simplifySolution(5.);

        // // Get solution path.
        // auto path = ss->getSolutionPath();

        // // Interpolation also works on constrained state spaces, and is generally required.
        // path.interpolate();

        // // Then do whatever you want with the path, like normal!
        // for (auto state : path.getStates())
        // {
        //     std::cout << state << std::endl;
        // }
    }
    else
        OMPL_WARN("No solution found!");


}
