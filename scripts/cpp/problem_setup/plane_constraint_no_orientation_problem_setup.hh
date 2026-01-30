#pragma once
#include <vector>
#include <array>
#include <filesystem>
#include <Eigen/Dense>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;

inline const Robot::ConfigurationArray start = {-0.841244, 1.02584, 0.0819682, -0.894846, -0.025647, 1.8293, 0.0346282};

inline const Robot::ConfigurationArray goal = {0.824404, 1.28846, -0.0256292, -0.410736, -0.00798986, 1.61264, -0.0149243};

inline constexpr float radius = 0.1;

inline const std::string default_obstacle_filepath = "scripts/cpp/problem_setup/obstacle_files/plane_constraint_no_orientation_problem.txt";

inline const std::array<float, 6> tsr_lower_bound = {
    -0.01, -10.01, -10.01, -3.14, -3.14, -3.14
};

inline const std::array<float, 6> tsr_upper_bound = {
    0.01, 10.01, 10.01, 3.14, 3.14, 3.14
};

inline const Eigen::Matrix<float, 4, 4> T = []{
    Eigen::Matrix<float, 4, 4> m;
    m << 1,0,0,   0.543325,
         0,-1,0,  0.570738,
         0,0,-1,  0.121557,
         0,0,0,   1;
    return m;
}();