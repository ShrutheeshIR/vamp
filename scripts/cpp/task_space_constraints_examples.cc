#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
// #include <vamp/planning/cbirrt.hh>
#include <vamp/planning/task_space_constraint.hh>

#include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.35, 0.35, 0.25},
    // {0, 0.55, 0.25},
    // {-0.55, 0, 0.25},
    // {-0.35, -0.35, 0.25},
    // {0, -0.55, 0.25},
    // {0.35, -0.35, 0.25},
    // {0.35, 0.35, 0.8},
    // {0, 0.55, 0.8},
    // {-0.35, 0.35, 0.8},
    // {-0.55, 0, 0.8},
    // {-0.35, -0.35, 0.8},
    // {0, -0.55, 0.8},
    // {0.35, -0.35, 0.8},
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

    std::array<float, 6> lower_bound = {-0.01, -10.01, -10.03, -0.14, -0.14, -3.14};
    std::array<float, 6> upper_bound = {0.03, 10.01, 10.03, 0.14, 0.14, 3.14};

    Eigen::Matrix<float, 4, 4> T;

    T << 1, 0, 0, 0.543325, 0, -0.009, -0.999, 0.570738, 0, 0.999, -0.009, 0.121557, 0, 0, 0, 1;

    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    std::cout << "Target pose is : " << target_pose.translation().transpose() << std::endl;
    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(
        in_hand_pose, target_pose, std::make_pair(lower_bound, upper_bound));


    auto start_time = std::chrono::steady_clock::now();


    float delta[10] = {-0.2, -0.1, 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.5, 0.1};

    for(auto del_ind = 0U; del_ind < 10; del_ind++){

        typename Robot::template ConfigurationBlock<rake> block;
        for (auto i = 0U; i < Robot::dimension; ++i)
            block[i] = Robot::Configuration(goal).broadcast(i) + delta[del_ind];

        start_time = std::chrono::steady_clock::now();
        auto dist = task_constraint.distanceToConstraint(block);
        // std::cout << "Dist to constraint" << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << std::endl;
        // std::cout << "From block : " << dist << std::endl;
        // task_constraint.print_robot_tsr_error(block);

        typename Robot::template ConfigurationBlock<rake> projected_block;
        bool success;


        std::cout << " Output of project step inner ";
        start_time = std::chrono::steady_clock::now();
        task_constraint.projectStep(block, projected_block);
        // for(auto i=0U; i < Robot::dimension; i++)
        //     std::cout << projected_block[{i, 0}] << " ";
        std::cout << "->" << vamp::utils::get_elapsed_nanoseconds(start_time);

        std::cout << " Output of project step outer ";
        start_time = std::chrono::steady_clock::now();
        task_constraint.projectStep(block, projected_block, vamp::planning::ProjMethod::OuterLM);
        // for(auto i=0U; i < Robot::dimension; i++)
        //     std::cout << projected_block[{i, 0}] << " ";
        std::cout << "->" << vamp::utils::get_elapsed_nanoseconds(start_time);

        std::cout << " Output of project step Jac ";
        start_time = std::chrono::steady_clock::now();
        task_constraint.projectStep(block, projected_block, vamp::planning::ProjMethod::GradDesc);
        // for(auto i=0U; i < Robot::dimension; i++)
        //     std::cout << projected_block[{i, 0}] << " ";
        std::cout << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << std::endl;

        // start_time = std::chrono::steady_clock::now();
        // success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::OuterLM);
        // // for(auto i=0U; i < Robot::dimension; i++)
        // //     std::cout << projected_block[{i, 0}] << " ";
        // std::cout << "Outer " << success << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << " ";

        // start_time = std::chrono::steady_clock::now();
        // success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM);
        // // for(auto i=0U; i < Robot::dimension; i++)
        // //     std::cout << projected_block[{i, 0}] << " ";
        // std::cout << "Inner " << success << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << std::endl;

        // start_time = std::chrono::steady_clock::now();
        // success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::GradDesc);
        // // for(auto i=0U; i < Robot::dimension; i++)
        // //     std::cout << projected_block[{i, 0}] << " ";
        // std::cout << "Jac " << success << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << std::endl;
    }


    return 0;
}
