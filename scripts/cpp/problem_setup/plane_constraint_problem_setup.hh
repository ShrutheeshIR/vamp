#pragma once
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;

inline const Robot::ConfigurationArray start = {-0.830087, 0.976207, 0.0227157, -0.991399, -0.0196006, 1.95428, -0.0185915};

inline const Robot::ConfigurationArray goal = {0.917846, 0.987762, -0.206241, -0.975562, 0.198935, 1.94586, 1.52356};

inline const std::string default_obstacle_filepath = "scripts/cpp/problem_setup/obstacle_files/plane_constraint_problem.txt";

inline constexpr float radius = 0.1;

inline const std::array<float, 6> tsr_lower_bound = {
    -0.01, -10.01, -10.01, -0.01, -0.01, -0.01
};

inline const std::array<float, 6> tsr_upper_bound = {
    0.01, 10.01, 10.01, 0.01, 0.01, 0.01
};

inline const Eigen::Matrix<float, 4, 4> T = []{
    Eigen::Matrix<float, 4, 4> m;
    m << 1,0,0,   0.543325,
         0,-1,0,  0.570738,
         0,0,-1,  0.121557,
         0,0,0,   1;
    return m;
}();
