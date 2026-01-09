#include "plane_constraint_problem_setup.hh"
const Robot::ConfigurationArray goal = {0.88,1.05,0.0,-0.66,0.0,1.73,0.0};
// should be {0.543325, 0, 0.3},
const Robot::ConfigurationArray start = {-0.92,1.05,0.0,-0.66,0.0,1.73,0.0};
// should be {0.543325, 0, -0.3},

const std::vector<std::array<float, 3>> problem = {
    {0.543325, 0, 0.3},
    {0.543325, 0.1, 0.4},
    {0.543325, 0.1, 0.2},
    {0.543325, 0.15, 0.47},
    {0.543325, 0.15, 0.13},
    {0.543325, 0.25, 0.60},
    {0.543325, 0.25, 0},
};

const float radius = 0.1;

//used for constraint
const std::array<float, 6> lower_bound = {
    -0.01, -10.01, -10.01, -3.14, -3.14, -3.14
};
const std::array<float, 6> upper_bound = {
    0.03, 10.01, 10.01, 3.14, 3.14, 3.14
};


const Eigen::Matrix<float, 4, 4> T = []{
    Eigen::Matrix<float, 4, 4> m;
    m << 1,0,0,   0.543325,
         0,-1,0,  0.570738,
         0,0,-1,  0.121557,
         0,0,0,   1;
    return m;
}();