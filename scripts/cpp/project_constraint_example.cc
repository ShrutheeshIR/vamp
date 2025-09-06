#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/cbirrt.hh>
#include <vamp/planning/task_space_constraints.hh>
#include <vamp/planning/validate_constraint.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;

// Start and goal configurations
// static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
// static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};
static constexpr Robot::ConfigurationArray goal = {0.88,1.05,0.0,-0.66,0.0,1.73,0.0};
static constexpr Robot::ConfigurationArray start = {-0.92,1.05,0.0,-0.66,0.0,1.73,0.0};

// static constexpr Robot::ConfigurationArray start = {0.88,1.05,0.0,-0.66,0.0,1.73,0.0};
// static constexpr Robot::ConfigurationArray goal = {-0.92,1.05,0.0,-0.66,0.0,1.73,0.0};


// static constexpr Robot::ConfigurationArray goal = {-0.839708,  0.496555, -0.630832, -0.573204,  0.232247,  1.8259,   -0.467584};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
    {0.55, 0, 0.50},
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
static constexpr float radius = 0.15;


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


    std::array<float, 6> lower_bound = {
        -0.01, -10.01, -10.03, -3.14, -3.14, -3.14
    };
    std::array<float, 6> upper_bound = {
        0.03, 10.01, 10.03, 3.14, 3.14, 3.14
    };


    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    Eigen::Matrix<float, 4, 4> T;
    // T << 1,  0.000398119,  7.35017e-08,      0.30702,  0.000398119,           -1, -6.92765e-12, -5.94873e-12,  7.35017e-08,  3.61875e-11,           -1,      0.48527,            0,            0,            0,            1;
    // T <<  0.99086916, -0.13428134,  0.01211568,  0.48284483, -0.13408315, -0.99084246, -0.01591116, -0.6341026,  0.0141413,   0.01414137, -0.9998001,   0.34187168,  0.,          0.,          0.,          1.;

    T <<   1,0,0,   0.48284483,   0,1,0,     -0.6341026,   0,0,1,    0.34187168,          0,           0,           0,           1;

    // T <<   -0.537748,    0.711259,     -0.4527,   0.48284483,   0.543885,     0.70293,    0.458344,     -0.6341026,   0.644218, 0.000256485,   -0.764842,    0.34187168,          0,           0,           0,           1;
    // T << 1,  0.000398163,  4.62412e-17, 5.0781602e-01, 0.000398163, -1, -6.92765e-12, 6.1428678e-01, -2.7121e-15,  6.92765e-12, -1, 3.4187165e-01, 0.0, 0.0, 0.0, 1;
    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    std::cout << "Target pose is : " << target_pose.translation().transpose() << std::endl;
    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(in_hand_pose, target_pose, std::make_pair(lower_bound, upper_bound));


    typename Robot::template ConfigurationBlock<rake> block;
    typename Robot::template ConfigurationBlock<rake> projected_block;

    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i) + 0.05;


    // auto dist = task_constraint.distanceToConstraintAuto(block);
    // std::cout << "From block : " << dist << std::endl;


    Robot::ConfigurationArray goal2 = {-0.92,1.05,0.0,-0.66,0.0,1.73,0.0};
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal2).broadcast(i) + 0.05;
    auto dist = task_constraint.distanceToConstraint(block);
    // std::cout << "From block : " << dist << std::endl;



    // const Eigen::Vector<float, 6> distance_vec = task_constraint.distanceToConstraint(goal);
    // std::cout << "From single : "<< distance_vec << std::endl;


    bool success = task_constraint.project(block, projected_block);
    std::cout << success << std::endl;
    // std::cout << block << std::endl;
    // std::cout << projected_block << std::endl;
    // std::cout << " printed config " << std::endl;

    // typename Robot::template ConfigurationArray last_projected;
    // for (auto i = 0U; i < Robot::dimension; ++i) {  
    //     last_projected[i] = projected_block[{i, rake-1}];  
    // }

    // std::cout << Robot::Configuration(last_projected) << std::endl;


    // Robot::Configuration vector = Robot::Configuration(std::array<float, 7>{-0.109852, -0.302075, 0.31963, -0.108644, -0.0344217, -0.0659725, -0.164862});
    // std::vector<Robot::Configuration> projected_vectors;

    // auto goal_config = Robot::Configuration(start);
    // std::cout << "\n\n---> Going to project a vector " << std::endl;
    // auto valid = vamp::planning::project_constraint_vector<Robot, rake, Robot::resolution>(
    //     goal_config,
    //     vector,
    //     0.53F,
    //     projected_vectors,
    //     task_constraint,
    //     env_v
    // );
    // auto valid = vamp::planning::validate_vector<Robot, rake, Robot::resolution>(
    //     goal_config,
    //     vector,
    //     0.53F,
    //     env_v
    // );

    // std::cout << "Are projects valid : " << valid << std::endl;
    // std::cout << projected_vectors.back() << std::endl;




    // Setup RRTC and plan
    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.5;
    std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
    auto result =
        CRRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

    std::cout << result.path.size() << std::endl;

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

            // auto fka = Robot::eefk(soln);
            // std::cout <<std::endl << fka.matrix() <<std::endl;
            std::cout << std::endl;
        }
    }

    std::cout << "Planner took " << std::setprecision(5) << result.nanoseconds / 1e9 << "s and " << result.iterations << " steps" << std::endl;



    return 0;
}
