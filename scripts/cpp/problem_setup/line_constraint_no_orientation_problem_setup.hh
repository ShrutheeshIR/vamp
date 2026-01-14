#pragma once
#include <vector>
#include <array>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
extern const Robot::ConfigurationArray goal;
extern const Robot::ConfigurationArray start;
extern const std::vector<std::array<float, 3>> problem;
extern const float radius;

extern const std::array<float, 6 * Robot::n_eef> tsr_lower_bound;
extern const std::array<float, 6 * Robot::n_eef> tsr_upper_bound;

extern const Eigen::Matrix<float, 4, 4> T;