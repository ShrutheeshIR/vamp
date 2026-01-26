#include "plane_constraint_problem_setup.hh"

const Robot::ConfigurationArray start = {-0.830087, 0.976207, 0.0227157, -0.991399, -0.0196006, 1.95428, -0.0185915};
const Robot::ConfigurationArray goal = {0.917846, 0.987762, -0.206241, -0.975562, 0.198935, 1.94586, 1.52356};

const std::vector<std::array<float, 3>> problem = {
    {0.543325, 0, 0.3},
    {0.543325, 0.1, 0.4},
    {0.543325, 0.1, 0.2},
    {0.543325, 0.15, 0.47},
    {0.543325, 0.15, 0.13},
    {0.543325, 0.25, 0.40},
    {0.543325, 0.25, 0},
};

const float radius = 0.1;

//used for constraint
const std::array<float, 6> tsr_lower_bound = {
    -0.01, -10.01, -10.01, -0.01, -0.01, -0.01
};
const std::array<float, 6> tsr_upper_bound = {
    0.03, 10.01, 10.01, 0.01, 0.01, 0.01
};


const Eigen::Matrix<float, 4, 4> T = []{
    Eigen::Matrix<float, 4, 4> m;
    m << 1,0,0,   0.543325,
         0,-1,0,  0.570738,
         0,0,-1,  0.121557,
         0,0,0,   1;
    return m;
}();

