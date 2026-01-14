#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>
#include "problem_setup/random_obs_plane_constraint_problem_setup.hh"

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;


struct Attempt {
    float range;
    bool dynamic_domain;
    vamp::planning::ProjMethod proj_method;
    float descend_rate;
    bool success;
    std::size_t planning_time;
    std::size_t planning_iterations;
    std::size_t path_length;

    bool operator<(const Attempt& other) const {
        return planning_time < other.planning_time;
    }
};

auto main(int argc, char** argv) -> int
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <num_obstacles> <iterations>\n";
        return 1;
    }

    int n_obstacles = std::stoi(argv[1]);
    int iterations  = std::stoi(argv[2]);

    std::cout << "Obstacles: " << n_obstacles << "\n";
    std::cout << "Iterations: " << iterations << "\n";



    // Setup RRTC and plan
    vamp::planning::RRTCSettings rrtc_settings;

    float ranges[] = {1.0};
    bool dd[] = {false};
    vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM};
    float descend_rates[] = {1.0};


    std::vector<Attempt> succ_attempts;
    for (int iteration = 0; iteration < iterations; iteration++) {
    for(const auto range: ranges){
        for(const auto dyndom: dd){
            for(const auto &pm: projection_method){
                for(const auto descent_rate: descend_rates){

                // if(pm < 2) continue;

    // Build sphere cage environment
    EnvironmentInput environment;
    std::vector<std::array<float, 3>> problem = make_problem(n_obstacles);
    std::ofstream outfile_sph("/src/spheres.txt");
    for (const auto &sphere : problem)
    {
        outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }
    outfile_sph.close();

    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms;

    eef_transforms[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms_ref_frame_w_world;
    Eigen::Matrix<float, 4, 4> T_identity;
    T_identity << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    
    eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T_identity);


    vamp::planning::TaskSpaceConstraint<Robot, rake> tsr_constraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        std::make_pair(tsr_lower_bound, tsr_upper_bound)
    );


    vamp::planning::ComposableConstraints<Robot, rake, decltype(tsr_constraint)> task_constraint(
        tsr_constraint
    );



    rrtc_settings.range = range;
    rrtc_settings.dynamic_domain = dyndom;
    rrtc_settings.projection_method = pm;
    rrtc_settings.descend_rate = descent_rate;
    // std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
    std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";
    vamp::planning::invalid_distance_counter_outside = 0;
    vamp::planning::invalid_distance_counter_inside = 0;
    vamp::planning::collision_counter = 0;
    vamp::planning::unable_to_project_counter = 0;


    auto result =
        vamp::planning::CRRTC<Robot, rake, Robot::resolution, decltype(tsr_constraint)>::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

    if(result.path.size() > 0)
    {
        Attempt a{
            range,
            dyndom,
            pm,
            descent_rate,
            true,
            result.nanoseconds,
            result.iterations,
            result.path.size()
        };


        if((succ_attempts.size() == 0) || (succ_attempts.size() > 0 && a < succ_attempts[0])){

        std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        std::ofstream outfile("/src/trajectory.txt");
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
        outfile.close();
    }
        // std::cin.ignore();
        succ_attempts.push_back(a);
        std::sort(succ_attempts.begin(), succ_attempts.end());



    }
            }
        }
    }
    }
    }

    double total_time = 0.0;
    std::size_t total_iterations = 0;
    std::size_t total_path_length = 0;
    std::cout << "------Final Result --------" << std::endl;
    std::sort(succ_attempts.rbegin(), succ_attempts.rend());
    if (!succ_attempts.empty())
    {
        double total_time = 0.0;
        std::size_t total_iterations = 0;
        std::size_t total_path_length = 0;
        for (const auto &a : succ_attempts) {
            total_time += a.planning_time/1e6;
            total_iterations += a.planning_iterations;
            total_path_length += a.path_length;
            std::cout << a.planning_time/1e6 << ", " << a.planning_iterations << ", " << a.path_length << std::endl;

        }
        const std::size_t n = succ_attempts.size();

        std::cout << "\n====== Averages over " << n << " successful attempts ======\n";
        std::cout << "Average planning time (ms): "
                << (total_time / n) << "\n";
        std::cout << "Average planning iterations: "
                << static_cast<double>(total_iterations) / n << "\n";
        std::cout << "Average path length: "
                << static_cast<double>(total_path_length) / n << "\n";

    }
    std::cout << "---------------------------" << std::endl;

    // auto fka = Robot::eefk(goal);
    std::cout << std::endl << Robot::eefk(goal)[0].matrix() << std::endl;
    std::cout << std::endl << Robot::eefk(start)[0].matrix() << std::endl;

    return 0;
}
