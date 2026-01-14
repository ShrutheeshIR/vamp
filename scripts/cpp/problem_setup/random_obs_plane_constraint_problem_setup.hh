#pragma once
#include <vector>
#include <array>
#include <random>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
extern const Robot::ConfigurationArray goal;
extern const Robot::ConfigurationArray start;
std::vector<std::array<float, 3>> make_problem(std::size_t n, std::uint32_t seed = std::random_device{}())
extern const float radius;

extern const std::array<float, 6 * Robot::n_eef> tsr_lower_bound;
extern const std::array<float, 6 * Robot::n_eef> tsr_upper_bound;

extern const Eigen::Matrix<float, 4, 4> T;