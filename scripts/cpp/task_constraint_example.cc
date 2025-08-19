#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/task_space_constraints.hh>

#include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};


auto main(int, char **) -> int
{
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    auto fk = Robot::eefk(start);

    // std::cout << fk.matrix() <<std::endl;

    Eigen::Vector<float, 6> lower_bound = {
        -3.14, -3.14, -3.14, -0.25, -1.0, -1.0
    };
    Eigen::Vector<float, 6> upper_bound = {
        3.14, 3.14, 3.14, 0.25, 1.0, 1.0
    };


    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();

    fk(0, 3) += 0.5;
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
    task_constraint.project(start, q_new);
    for (float element : q_new) {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    return 0;
}
