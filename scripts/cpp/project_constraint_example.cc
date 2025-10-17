#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/cbirrt.hh>
#include <vamp/planning/task_space_constraints.hh>
#include <vamp/planning/validate_constraint.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/bimanual_panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::BimanualPanda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;

static constexpr Robot::ConfigurationArray start = {0.930205, 0.966287, 0.194365, -1.51657, -0.6965, 3.8223, -0.959755, 1.14244, 0.93196, -0.00581666, -1.49359, -0.609867, 0.687591, -0.73099};
static constexpr Robot::ConfigurationArray goal = {-0.67113, 1.66257, 0.0235852, -1.51145, 1.2212, 1.35781, -0.16863, -0.375588, 1.3998, -0.174771, -1.43568, -1.51361, 1.9924, 1.39243};
static const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.55, 0, 0.50},
    // {0.55, 0, 0.60},
    // {0.65, 0, 0.50},
    // {0.75, 0, 0.50},
    // {0.35, 0.35, 0.25},
    // {0, 0.55, 0.25},
    // {-0.55, 0, 0.25},
    // {-0.35, -0.35, 0.25},
    // {0, -0.55, 0.25},
    // {0.35, -0.35, 0.25},
    // {0.35, 0.35, 0.8},
    // {0, 0.55, 0.8},
    // {-0.35, 0.35, 0.8},
    // {-0.55, 0, 0.8},
    // {-0.35, -0.35, 0.8},
    // {0, -0.55, 0.8},
    // {0.35, -0.35, 0.8},
};
// Radius for obstacle spheres
static constexpr float radius = 0.15;


auto main(int, char **) -> int
{


    // Build sphere cage environment
    EnvironmentInput environment;
    std::ofstream outfile_sph("spheres.txt");
    for (const auto &sphere : problem)
    {
        outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }
    outfile_sph.close();

    // std::ifstream infile("/src/myfork/vamp/environments/cuboids/maze_cuboids.txt");
    // if (!infile.is_open()) {
    //     std::cerr << "Failed to open file!" << std::endl;
    //     return 1;
    // }

    // std::string line;
    // while (std::getline(infile, line)) {
    //     std::istringstream iss(line);
    //     char delim;
    //     float x, y, z, dx, dy, dz;

    //     if (!(iss >> x >> delim >> y >> delim >> z >> delim >> dx >> delim >> dy >> delim >> dz)) {
    //         std::cerr << "Error reading line: " << line << std::endl;
    //         continue;
    //     }
    //     std::cout << x << ", " << y << ", " << z << ", " << dx << ", " << dy << ", " << dz << std::endl;
    //     environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({x + 0.1, y + 1.0, z + 0.1}, {0.0, 0.0, 0.0}, {dx, dy, dz}));
    // }        
    // infile.close();



    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


    std::array<float, 6> lower_bound = {
        -0.01, -0.01, -0.03, -0.014, -0.014, -0.014
    };
    std::array<float, 6> upper_bound = {
        0.03, 0.01, 0.03, 0.014, 0.014, 0.014
    };


    Eigen::Matrix<float, 4, 4> T;
    T << 1.0, 0.0, 0.0, 0.0,  0.0, -1.0, 0.0, 0.0, 0.0,  0.0, -1.0, 0.15, 0.0, 0.0, 0.0, 1;


    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(target_pose, std::make_pair(lower_bound, upper_bound));


    typename Robot::template ConfigurationBlock<rake> block;
    typename Robot::template ConfigurationBlock<rake> projected_block;

    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(start).broadcast(i) + 0.05;


    // auto dist = task_constraint.distanceToConstraintAuto(block);
    // std::cout << "From block : " << dist << std::endl;


    Robot::ConfigurationArray goal2 = {0.61,0.985695,0.061017,-1.46231,-0.641869,3.8223,-0.863943,1.09,0.927539,-0.0600018,-1.52787,-0.385771,0.668267,-0.642295};
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal2).broadcast(i) + 0.0;
    task_constraint.print_robot_tsr_error(block);
    // std::cout << "From block : " << dist << std::endl;

    // auto d = task_constraint.projectStepJt(block, projected_block);


    bool success = task_constraint.project(block, projected_block);
    std::cout << success << std::endl;

    typename Robot::template ConfigurationArray last_projected;
    for (auto i = 0U; i < Robot::dimension; ++i) {  
        last_projected[i] = projected_block[{i, rake-1}];  
    }

    std::cout << Robot::Configuration(last_projected) << std::endl;


    // Robot::Configuration vector = Robot::Configuration(std::array<float, 7>{-0.109852, -0.302075, 0.31963, -0.108644, -0.0344217, -0.0659725, -0.164862});
    // std::vector<Robot::Configuration> projected_vectors;

    // auto goal_config = Robot::Configuration(start);
    // std::cout << "\n\n---> Going to project a vector " << std::endl;
    // auto valid = vamp::planning::project_constraint_vector<Robot, rake, Robot::resolution>(
    //     goal_config,
    //     vector,
    //     0.53F,
    //     projected_vectors,
    //     task_constraint,
    //     env_v
    // );
    // auto valid = vamp::planning::validate_vector<Robot, rake, Robot::resolution>(
    //     goal_config,
    //     vector,
    //     0.53F,
    //     env_v
    // );

    // std::cout << "Are projects valid : " << valid << std::endl;
    // std::cout << projected_vectors.back() << std::endl;




    // Setup RRTC and plan
    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.5;
    rrtc_settings.max_iterations = 100000;
    std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
    auto result =
        CRRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

    std::cout << result.path.size() << std::endl;

    if (result.path.size() > 0)
    {

        std::ofstream outfile("trajectory.txt");


        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            Robot::ConfigurationArray soln;
            bool first = true;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                // std::cout << array[i] << ", ";
                soln[i] = array[i];

                if (!first) outfile << ",";
                outfile << array[i];
                first = false;
            }

            // auto fka = Robot::eefk(soln);
            // std::cout <<std::endl << fka.matrix() <<std::endl;
            // std::cout << std::endl;
            outfile << "\n";
        }
    }

    std::cout << "Planner took " << std::setprecision(5) << result.nanoseconds / 1e6 << "ms and " << result.iterations << " steps" << std::endl;



    return 0;
}
