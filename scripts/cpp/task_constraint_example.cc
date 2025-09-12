#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
// #include <vamp/planning/cbirrt.hh>
#include <vamp/planning/task_space_constraints.hh>

#include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
// using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};
static constexpr Robot::ConfigurationArray goal2 = {0.88,1.05,0.0,-0.66,0.0,1.73,0.0};


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


    std::array<float, 6> lower_bound = {
        -0.01, -10.01, -10.03, -3.14, -3.14, -3.14
    };
    std::array<float, 6> upper_bound = {
        0.03, 10.01, 10.03, 3.14, 3.14, 3.14
    };


    Eigen::Matrix<float, 4, 4> T;

    T <<   1,0,0, 0.543325, 0,-0.009, -0.999, 0.570738, 0, 0.999, -0.009, 0.121557, 0, 0, 0, 1;


    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    std::cout << "Target pose is : " << target_pose.translation().transpose() << std::endl;
    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(in_hand_pose, target_pose, std::make_pair(lower_bound, upper_bound));


    Eigen::Quaternion<float> q1(in_hand_pose.linear());
    std::array<float, 7> transform1 = {q1.w(), q1.x(), q1.y(), q1.z(), in_hand_pose.translation().x(), in_hand_pose.translation().y(), in_hand_pose.translation().z()};

    Eigen::Quaternion<float> q2(target_pose.linear());
    std::array<float, 7> transform2 = {q2.w(), q2.x(), q2.y(), q2.z(), target_pose.translation().x(), target_pose.translation().y(), target_pose.translation().z()};


    // creating the 33 element arr
    std::array<float, 33>x;

    // testing blend operation
    // std::array<float, 8>
    // float a[8] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
    // float b[8] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
    // float mask[8] = {-0.1, -88.0, -0.002, 0.0, -0.0, 0.2, 0.5, 99};

    // auto blend1 = blend(Robot::Configuration(a), Robot::Configuration(b), Robot::Configuration(mask));
    // std::cout << blend1 << std::endl;

    // auto blend2 = blend(99.0, Robot::Configuration(b), Robot::Configuration(mask));
    // std::cout << blend2 << std::endl;

    // auto blend3 = blend(Robot::Configuration(a), 30.0, Robot::Configuration(mask));
    // std::cout << blend3 << std::endl;

    // auto blend4 = blend(99.0, 30.0, Robot::Configuration(mask));
    // std::cout << blend4 << std::endl;

    // for (auto i=0U; i < 8; i++)
    //     std::cout << blend(a[i], b[i], mask[i]) << " ";
    // std::cout << std::endl;

    // for (auto i=0U; i < 8; i++)
    //     std::cout << blend(99.0, b[i], mask[i]) << " ";
    // std::cout << std::endl;

    // for (auto i=0U; i < 8; i++)
    //     std::cout << blend(a[i], 30.0, mask[i]) << " ";
    // std::cout << std::endl;

    // for (auto i=0U; i < 8; i++)
    //     std::cout << blend(99.0, 30.0, mask[i]) << " ";
    // std::cout << std::endl;


    float delta[10] = {-0.2, -0.1, 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.5, 0.1};

    for(auto del_ind = 0U; del_ind < 1; del_ind++){
    // for (auto i=0U; i < 7; i++)
    //     x[i] = goal[i] + delta[del_ind];

    // for (auto i=0U; i < 6; i++) {
    //     x[3 * 7 + i] = lower_bound[i];
    //     x[3 * 7 + 6 + i] = upper_bound[i];
    // }
    // for (auto i=0U; i < 7; i++) {
    //     x[7 + i] = transform1[i];
    //     x[7 + 7 + i] = transform2[i];
    // }
    // std::array<float, 48> y;
    // Robot::tsr_function(x, y);
    // for (auto i = 0U; i < 48; i++)
    //     std::cout << y[i] << " ";
    // std::cout << std::endl;

    // Robot::tsr_function_ori(x, y);
    // for (auto i = 0U; i < 48; i++)
    //     std::cout << y[i] << " ";

    // std::cout << std::endl;
    // // std::cout << y << std::endl;


    // Robot::tsr_function_ori_blend(x, y);
    // for (auto i = 0U; i < 48; i++)
    //     std::cout << y[i] << " ";

    // std::cout << std::endl;

    typename Robot::template ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i) + delta[del_ind];


    task_constraint.print_robot_tsr_error(block);


    }

    vamp::FloatVector<8, 170> v;
    auto val = v[10].acos();
    std::cout << "Acos is " << val << std::endl;


    typename Robot::template ConfigurationBlock<rake> block;
    // typename Robot::template ConfigurationBlock<rake> projected_block;

    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i) + 0.05;
    
    std::cout << block[0] << std::endl;

    Robot::ConfigurationArray sines = {-0.999, 0.9, 0.8, 0.5, 0.6, 0.7, 0.999};
    std::cout << std::setprecision(10) << Robot::Configuration(sines).asin() << std::endl;


    // // auto dist = task_constraint.distanceToConstraintAuto(block);
    // // std::cout << "From block : " << dist << std::endl;


    Robot::ConfigurationArray goal2 = {0.81,1.57,0.0,0.0,0.0,1.57,1.64};
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal2).broadcast(i) - 0.25;
    auto dist = task_constraint.distanceToConstraint(block);
    std::cout << "From block : " << dist << std::endl;
    task_constraint.print_robot_tsr_error(block);




    return 0;
}
