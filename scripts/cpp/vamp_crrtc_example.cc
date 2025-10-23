#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>

#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>


using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;


static constexpr Robot::ConfigurationArray start = {-0.75,0.21,-0.05,-2.29,-0.32,2.44,1.64};
static constexpr Robot::ConfigurationArray goal = {1.31,0.67,-0.05,-1.58,-0.32,2.3,-0.81};
// static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
// static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};


static const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.55, 0, 0.50},
    // {0.55, 0, 0.60},
    // {0.65, 0, 0.50},
    // {0.75, 0, 0.50},
    // // {0.35, 0.35, 0.25},
    // // {0, 0.55, 0.25},
    // {-0.55, 0, 0.25},
    // // {-0.35, -0.35, 0.25},
    // {0, -0.55, 0.25},
    // // {0.35, -0.35, 0.25},
    // {0.35, 0.35, 0.8},
    // {0, 0.55, 0.8},
    // {-0.35, 0.35, 0.8},
    // {-0.55, 0, 0.8},
    // {-0.35, -0.35, 0.8},
    // {0, -0.55, 0.8},
    // {0.35, -0.35, 0.8},
};
// Radius for obstacle spheres
// static constexpr float radius = 0.2;

auto main(int, char **) -> int
{
    // Build sphere cage environment
    EnvironmentInput environment;
    // std::ofstream outfile_sph("/src/spheres.txt");
    // for (const auto &sphere : problem)
    // {
    //     outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }
    // outfile_sph.close();
    std::ifstream infile("/src/myfork/vamp/resources/environments/cuboids/maze_cuboids.txt");
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
        environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({x + 0.1, y + 1.0, z + 0.1}, {0.0, 0.0, 0.0}, {dx, dy, dz}));
    }        
    infile.close();


    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


    // Create constraints
    std::array<float, 6> lower_bound = {
        -10.01, -10.01, -0.02, -10.1, -10.1, -3.14
    };
    std::array<float, 6> upper_bound = {
        10.03, 10.01, 0.02, 10.1, 10.1, 3.14
    };
    Eigen::Matrix<float, 4, 4> T;
    T << 
        1.000, 0.000, 0.000, 0.246,
        0.000, 1.000, 0.000, 0.670,
        0.000, 0.000, 1.000, 0.151,
        0.000, 0.000, 0.000, 1.000;
    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);

    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(in_hand_pose, target_pose, std::make_pair(lower_bound, upper_bound));
    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.5;
    rrtc_settings.max_samples = 1000000;
    rrtc_settings.max_iterations = 1000000;
    rrtc_settings.dynamic_domain = false;
    // rrtc_settings.radius = 0.1;
    // rrtc_settings.min_radius = 0.01;

    // rrtc_settings.range = 1.0;
    std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
    auto result =
        CRRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

    std::cout << std::endl << result.path.size() << std::endl;
    // If successful
    // If successful
    if (result.path.size() > 0)
    {
        // Simplify path with default settings
        // vamp::planning::SimplifySettings simplify_settings;
        // auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
        //     result.path, env_v, simplify_settings, rng);
        std::ofstream outfile("/src/trajectory.txt");

        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                // std::cout << array[i] << ", ";
                outfile << array[i] << ",";
            }
            outfile << "\n";
            // std::cout << std::endl;
        }
        outfile.close();
    }
    std::cout << "Planner took " << std::setprecision(5) << result.nanoseconds / 1e6 << "ms and " << result.iterations << " steps" << std::endl;

}