#pragma once
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;

inline const Robot::ConfigurationArray start = {1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891};

inline const Robot::ConfigurationArray goal = {-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24};

inline const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.55, 0, 0.50},
    // {0.55, 0, 0.60},
    {0.56, 0, 0.450},
    {0.1, 0, 0.7},
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

inline constexpr float radius = 0.15;

inline const std::array<float, 6> tsr_lower_bound = {
    -0.01, -10.01, -0.01, -0.01, -0.01, -0.01
};

inline const std::array<float, 6> tsr_upper_bound = {
    0.01, 10.01, 0.01, 0.01, 0.01, 0.01
};

inline const Eigen::Matrix<float, 4, 4> T = []{
    Eigen::Matrix<float, 4, 4> m;
    m << 1,0,0,   0.3486,
         0,-1,0,  0.647752,
         0,0,-1,  0.24,
         0,0,0,   1;
    return m;
}();