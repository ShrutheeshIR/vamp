#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/task_space_constraints.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {-0.75,0.21,-0.05,-2.29,-0.32,2.44,1.64};
static constexpr Robot::ConfigurationArray goal = {1.31,0.65,-0.05,-1.58,-0.32,2.37,-1.03};
// static constexpr Robot::ConfigurationArray goal = {-0.92, 1.05, 0, -0.66, 0, 1.73, 0};
// static constexpr Robot::ConfigurationArray goal = {0.88,1.05,0.0,-0.66,0.0,1.73,0.0};
// static constexpr Robot::ConfigurationArray start = {-0.92,1.05,0.0,-0.66,0.0,1.73,0.0};


// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
    {0.55, 0, 0.50},
    {0.65, 0, 0.50},
    {0.55, 0, 0.60},
    {0.75, 0, 0.50},
    {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, -0.35, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
};

// Radius for obstacle spheres
static constexpr float radius = 0.1;

auto main(int, char **) -> int
{
    // Build sphere cage environment
    EnvironmentInput environment;

    std::ifstream infile("/src/myfork/vamp/environments/cuboids/maze_cuboids.txt");
    if (!infile.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        char delim;
        float x, y, z, dx, dy, dz;

        if (!(iss >> x >> delim >> y >> delim >> z >> delim >> dx >> delim >> dy >> delim >> dz)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        }
        std::cout << x << ", " << y << ", " << z << ", " << dx << ", " << dy << ", " << dz << std::endl;
        environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({x + 0.1, y + 1.0, z + 0.1}, {0.0, 0.0, 0.0}, {dx, dy, dz}));
    }        
    infile.close();




    // for (const auto &sphere : problem)
    // {
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }
    // for (const auto &sphere : problem)
    // environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({1.22, 0.22, 0.01}, {0.0, 0.0, 0.0}, {0.61, 0.14, 0.06}));

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


    auto fk = Robot::eefk(start);
    auto fkg = Robot::eefk(goal);
    std::cout << fk.matrix() <<std::endl;
    std::cout << fkg.matrix() <<std::endl;

    Robot::Configuration vector = Robot::Configuration(std::array<float, 7>{-0.219703, -0.604149, 0.63926, -0.217288, -0.0688434, -0.131945, -0.329724});
    auto valid = vamp::planning::validate_vector<Robot, rake, Robot::resolution>(
        Robot::Configuration(goal),
        vector,
        0.53F,
        env_v
    );
    std::cout << "\n\nvalid " << valid << std::endl;

    // Setup RRTC and plan
    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 1.0;

    auto result =
        RRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, rng);

    // If successful
    if (result.path.size() > 0)
    {
        std::ofstream outfile("trajectory.txt");
        // Simplify path with default settings
        vamp::planning::SimplifySettings simplify_settings;
        auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
            result.path, env_v, simplify_settings, rng);

        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            Robot::ConfigurationArray soln;
            bool first = true;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                std::cout << array[i] << ", ";
                soln[i] = array[i];

                if (!first) outfile << ",";
                outfile << array[i];
                first = false;

            }

            auto fka = Robot::eefk(soln);
            // std::cout <<std::endl << fka.matrix() <<std::endl;

            std::cout << std::endl;
            outfile << "\n";
        }
    }
    else
        std::cout << "Failed" << std::endl;
    std::cout << "Planner took " << std::setprecision(5) << result.nanoseconds / 1e6 << "ms and " << result.iterations << " steps" << std::endl;

    return 0;
}
