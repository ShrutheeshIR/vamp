#include "plane_constraint_problem_setup.hh"

const Robot::ConfigurationArray start = {-0.855292,1.12975,0.0380571,-0.69921,0.0141439,1.73613,0.0000393391};
const Robot::ConfigurationArray goal = {0.841046,1.13069,-0.0242863,-0.697919,-0.00982904,1.74158,0.000353813};

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
const std::array<float, Robot::n_eef> tsr_lower_bound = {
    -0.01, -10.01, -10.01, -3.14, -3.14, -3.14
};
const std::array<float, Robot::n_eef> tsr_upper_bound = {
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