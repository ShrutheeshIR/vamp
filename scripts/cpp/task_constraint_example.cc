#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/cbirrt.hh>
#include <vamp/planning/task_space_constraints.hh>

#include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};


// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
    {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, -0.35, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
};
// Radius for obstacle spheres
static constexpr float radius = 0.2;


auto main(int, char **) -> int
{


    // Build sphere cage environment
    EnvironmentInput environment;
    for (const auto &sphere : problem)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);




    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    auto fk = Robot::eefk(start);

    auto fkg = Robot::eefk(goal);

    std::cout << fk.matrix() <<std::endl;

    std::cout << fkg.matrix() <<std::endl;

    Eigen::Vector<float, 6> lower_bound = {
        -3.14, -3.14, -3.14, -10.25, -10.0, -0.09
    };
    Eigen::Vector<float, 6> upper_bound = {
        3.14, 3.14, 3.14, 10.25, 10.0, 0.09
    };

    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();

    fk(2, 3) -= 0.08;
    const auto eef_target_pose = fk;

    vamp::planning::TaskSpaceConstraint<Robot> task_constraint(eef_target_pose, in_hand_pose, std::make_pair(lower_bound, upper_bound));


    const Eigen::Vector<float, 6> distance_vec = task_constraint.distanceToConstraint(start);

    std::cout << distance_vec << std::endl;

    Robot::ConfigurationArray q_new;


    const auto jacobian = Robot::jacobian(start);
    std::cout << "-----Printing Jacobian------";
    std::cout << jacobian << std::endl;


    Eigen::Matrix<float, 6, Eigen::Dynamic> J;
    Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;

    Robot::jacobian_eefk(start, J, eef_pose);
    std::cout << "-----Printing Jacobian EEFK ------";
    std::cout << J << std::endl;
    std::cout << eef_pose.matrix() << std::endl;
    std::cout << "-----Printed Jacobian EEFK ------";


    std::cout << "projecting" << std::endl;

    double distance = task_constraint.projectStep(start, q_new, true);
    std::cout << distance << std::endl;

    // task_constraint.project(start, q_new);
    for (float element : q_new) {
        std::cout << element << " ";
    }
    std::cout << std::endl;


    distance = task_constraint.projectStepOld(start, q_new, true);
    std::cout << distance << std::endl;

    // task_constraint.project(start, q_new);
    for (float element : q_new) {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    distance = task_constraint.projectStepJT(start, q_new, true);
    std::cout << distance << std::endl;

    // task_constraint.project(start, q_new);
    for (float element : q_new) {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    bool success = task_constraint.project(start, q_new);
    std::cout << success << std::endl;

    // task_constraint.project(start, q_new);
    for (float element : q_new) {
        std::cout << element << " ";
    }
    std::cout << std::endl;


    // Setup RRTC and plan
    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 1.0;

    auto result =
        CRRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

    if (result.path.size() > 0)
    {

        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            Robot::ConfigurationArray soln;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                std::cout << array[i] << ", ";
                soln[i] = array[i];
            }

            auto fka = Robot::eefk(soln);
            std::cout <<std::endl << fka.matrix() <<std::endl;

            std::cout << std::endl;
        }
    }
    

    return 0;
}
