#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/bimanual_panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::BimanualPanda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;
using AttachmentInput = vamp::collision::Attachment<float>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {-1.3238,  1.358 ,  1.0783, -2.4974,  0.5572,  2.5477, -1.4485, 1.2848,  1.2911, -1.0714, -2.4884, -0.6705,  2.5082,  0.7243 };
static constexpr Robot::ConfigurationArray goal = {-1.997 ,  0.385 ,  2.1832, -2.0013,  1.3083,  1.8498, -0.7243, 1.2835,  1.3097, -2.0683, -2.1051, -0.1333,  2.4786, -0.7243 };


struct Attempt {
    float range;
    bool dynamic_domain;
    vamp::planning::ProjMethod proj_method;
    float descend_rate;
    bool success;
    std::size_t planning_time;
    std::size_t planning_iterations;
    std::size_t path_length;
    
    bool operator<(const Attempt& other) const {
        return planning_time < other.planning_time;
    }
};


auto main(int, char **) -> int
{

    vamp::planning::RRTCSettings rrtc_settings;

    EnvironmentInput environment;
    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


    std::array<float, 6> lower_bound = {
        -0.00001, -0.00001, -0.00001, -0.00001, -0.00001, -0.00001
    };
    std::array<float, 6> upper_bound = {
        0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001
    };


    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    Eigen::Matrix<float, 4, 4> T;
    T << -0.719427, 0.694568, 6.59173e-05, -2.2769e-05, 0.694568, 0.719427, -2.26738e-05, -0.000370264, -6.31774e-05, 2.94462e-05, -1, 0.171814, 0, 0, 0, 1;
    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    vamp::planning::BimanualTaskSpaceConstraint<Robot, rake> task_constraint(target_pose, std::make_pair(lower_bound, upper_bound));


    typename Robot::template ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(start).broadcast(i) + 0.0;

    // task_constraint.print_robot_tsr_error(block);

    auto fks = Robot::eefk(goal);
    auto err = (fks[0].inverse() * fks[1]).matrix();

    // std::cout << fks[0].matrix() << std::endl;
    // std::cout << fks[1].matrix() << std::endl;
    // for (int i = 0; i < err.rows(); ++i) {
    //     for (int j = 0; j < err.cols(); ++j) {
    //     std::cout << err(i, j);
    //     std::cout << ", ";
    // }
    // }
    // std::cout << std::endl;


    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i) + 0.0;

    // task_constraint.print_robot_tsr_error(block);

    fks = Robot::eefk(goal);
    err = (fks[0].inverse() * fks[1]).matrix();

    Robot::ConfigurationArray test = {-1.45844, 1.1634, 1.29928, -2.39818, 0.70742, 2.40812, -1.30366, 1.28454, 1.29482, -1.27078, -2.41174, -0.56306, 2.50228, 0.43458};
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(test).broadcast(i);

    task_constraint.print_robot_tsr_error(block);
    std::cout << "----Projecting ----" << std::endl;
    typename Robot::template ConfigurationBlock<rake> projected_block;
    bool success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::GradDesc);


    // std::cout << fks[0].matrix() << std::endl;
    // std::cout << fks[1].matrix() << std::endl;
    // for (int i = 0; i < err.rows(); ++i) {
    //     for (int j = 0; j < err.cols(); ++j) {
    //     std::cout << err(i, j);
    //     std::cout << ", ";
    // }
    // }
    // std::cout << std::endl;

    auto startc = Robot::Configuration(start);
    auto goalc = Robot::Configuration(goal);

    auto extension_vector = goalc - startc;

    for(size_t ext = 0; ext < 11; ext++){
        std::cout << "Extension attempt " << ext << " ";
        auto cfg = startc + extension_vector * ext / 10;
        for (auto i = 0U; i < Robot::dimension; ++i)
            block[i] = cfg.broadcast(i) + 0.0;

        for(auto i=0U; i < Robot::dimension; i++)
            std::cout << block[{i, 0}] << " ";
        std::cout << " --> " ;

        task_constraint.print_robot_tsr_error(block);

        typename Robot::template ConfigurationBlock<rake> projected_block;

        bool success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::GradDesc);
        for(auto i=0U; i < Robot::dimension; i++)
            std::cout << projected_block[{i, 0}] << " ";
        std::cout << " --> " << success << " -- ";
        task_constraint.print_robot_tsr_error(projected_block);
    }

    // Robot::eefk(start)



}
