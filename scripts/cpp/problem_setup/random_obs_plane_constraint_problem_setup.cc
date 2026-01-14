#include "random_obs_plane_constraint_problem_setup.hh"

const Robot::ConfigurationArray start = {-0.855292,1.12975,0.0380571,-0.69921,0.0141439,1.73613,0.0000393391};
const Robot::ConfigurationArray goal = {0.841046,1.13069,-0.0242863,-0.697919,-0.00982904,1.74158,0.000353813};

const std::vector<std::array<float, 3>> set1 = {
    {0.543325, 0, 0.3},
    {0.543325, 0.1, 0.4},
    {0.543325, 0.1, 0.2},
    {0.543325, 0.15, 0.47},
    {0.543325, 0.15, 0.13},
    {0.543325, 0.25, 0.60},
    {0.543325, 0.25, 0},
};

const std::vector<std::array<float, 3>> set2 = {
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

static std::vector<std::array<float, 3>>
sample_without_replacement(const std::vector<std::array<float, 3>>& src,
                            std::size_t n,
                            std::mt19937& rng)
{
    n = std::min(n, src.size());

    std::vector<std::size_t> indices(src.size());
    std::iota(indices.begin(), indices.end(), 0);

    std::shuffle(indices.begin(), indices.end(), rng);

    std::vector<std::array<float, 3>> result;
    result.reserve(n);

    for (std::size_t i = 0; i < n; ++i)
        result.push_back(src[indices[i]]);

    return result;
}

std::vector<std::array<float, 3>>
make_problem(std::size_t n, std::uint32_t seed)
{
    std::mt19937 rng(seed);

    auto part1 = sample_without_replacement(set1, n/2, rng);
    auto part2 = sample_without_replacement(set2, n - n/2, rng);

    std::vector<std::array<float, 3>> problem;
    problem.reserve(part1.size() + part2.size());

    problem.insert(problem.end(), part1.begin(), part1.end());
    problem.insert(problem.end(), part2.begin(), part2.end());

    return problem;
}

const float radius = 0.1;

//used for constraint
const std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
    -0.01, -10.01, -10.01, -3.14, -3.14, -3.14
};
const std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
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