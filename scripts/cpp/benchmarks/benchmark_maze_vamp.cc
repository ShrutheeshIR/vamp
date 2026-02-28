#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <future>
#include <atomic>
#include <thread>
#include <nlohmann/json.hpp>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>
#include <vamp/planning/simplify_constraints.hh>

#include <Eigen/Geometry>
#include <Eigen/Dense>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;
using AttachmentInput = vamp::collision::Attachment<float>;

using Configuration = typename Robot::Configuration;
using ConfigurationArray = typename Robot::ConfigurationArray;
static constexpr auto dimension = Robot::dimension;
using RNG = typename vamp::rng::RNG<Robot>;


// Define your problem structure
struct Problem {
	std::array<float, 7> problem_start;
	std::array<float, 7> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
};

// This helper function allows nlohmann::json to "just work" with your struct
void to_json(json& j, const Problem& p) {
	j = json{
		{"problem_start", p.problem_start},
		{"problem_end", p.problem_end},
        {"start_eef_pos", p.start_eef_pos},
        {"goal_eef_pos", p.goal_eef_pos}
	};
}

void load_problems_from_json(std::vector<Problem> &problems, const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return;
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception &e) {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return;
    }

    if (!j.is_array()) {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return;
    }

    for (const auto &item : j) {
        try {
            Problem p;
            p.problem_start = item.at("problem_start").get<std::array<float, 7>>();
            p.problem_end = item.at("problem_end").get<std::array<float, 7>>();
            p.start_eef_pos = item.at("start_eef_pos").get<std::array<float, 3>>();
            p.goal_eef_pos = item.at("goal_eef_pos").get<std::array<float, 3>>();
            problems.push_back(p);
        } catch (const std::exception &e) {
            std::cerr << "Error reading problem fields: " << e.what() << " -- skipping problem" << std::endl;
            continue;
        }
    }
}


inline auto config_to_array(const Configuration &config){
	auto array = config.to_array();
	Robot::ConfigurationArray soln;
	bool first = true;
	for (auto i = 0U; i < Robot::dimension; ++i)
	{
		soln[i] = array[i];
	}
	return soln;
}


static auto load_cuboids_from_json(EnvironmentInput &environment, const std::string &path) {

    std::array<float, 2> min_bound = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    std::array<float, 2> max_bound = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return std::make_pair(min_bound, max_bound);
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception &e) {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return std::make_pair(min_bound, max_bound);
    }

    if (!j.is_array()) {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return std::make_pair(min_bound, max_bound);
    }


    for (const auto &obj : j) {
        if (!obj.is_object()) {
            std::cerr << "Skipping non-object element in array" << std::endl;
            continue;
        }

        // required fields
        if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z") ||
            !obj.contains("dx") || !obj.contains("dy") || !obj.contains("dz")) {
            std::cerr << "Skipping object missing required fields (x,y,z,dx,dy,dz)" << std::endl;
            continue;
        }

        try {
            float x = obj.at("x").get<float>();
            float y = obj.at("y").get<float>();
            float z = obj.at("z").get<float>();
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>();

            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (obj.contains("roll")) roll = obj.at("roll").get<float>();
            if (obj.contains("pitch")) pitch = obj.at("pitch").get<float>();
            if (obj.contains("yaw")) yaw = obj.at("yaw").get<float>();

            std::array<float,3> posf = {x, y, z};
            std::array<float,3> rotf = {roll, pitch, yaw};
            std::array<float,3> sizef = {dx / 2, dy / 2, dz / 2};
            // std::cout << "Creating cuboid at position: " << posf[0] << ", " << posf[1] << ", " << posf[2] << " with size: " << sizef[0] << ", " << sizef[1] << ", " << sizef[2] << std::endl;
            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(posf, rotf, sizef));
            // Update bounds
            for (int i = 0; i < 2; ++i) {
                min_bound[i] = std::min(min_bound[i], posf[i] - sizef[i]);
                max_bound[i] = std::max(max_bound[i], posf[i] + sizef[i]);
            }
        } catch (const std::exception &e) {
            std::cerr << "Error reading object fields: " << e.what() << " -- skipping object" << std::endl;
            continue;
        }
    }

    return std::make_pair(min_bound, max_bound);
}




int main() {

	std::vector<Problem> problems;


    EnvironmentInput environment;

    auto bounds = load_cuboids_from_json(environment, "/src/myfork/vamp/resources/environments/cuboids/real_maze.json");
    std::cout << "Loaded environment bounds: x[" << bounds.first[0] << ", " << bounds.second[0] << "], y[" << bounds.first[1] << ", " << bounds.second[1] << "]" << std::endl;

    environment.sort();

    std::vector<vamp::collision::Sphere<float>> spheres;
    for(auto i=0U; i < 9; i++){
        spheres.push_back(vamp::collision::Sphere<float>(0.0, 0.0, i * 0.02, 0.01));
    }
    auto attach_transform = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    attach_transform.translation().z() = 0.0;
    AttachmentInput attachment(attach_transform);

    attachment.spheres.insert(attachment.spheres.end(), spheres.cbegin(), spheres.cend());
    environment.attach(attachment, 0);
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    // load the json file of problems from the specified path
    std::string problem_json_path = "/src/tsr_panda_maze_problems.json";
    load_problems_from_json(problems, problem_json_path);

    vamp::planning::RRTCSettings rrtc_settings;
    
    size_t total_num_problems = 0;
    size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;


    for(const auto &problem: problems){
        std::cout << "Planning problem " << total_num_problems + 1 << " / " << problems.size() << std::endl;
        total_num_problems++;

        Configuration start = Configuration(problem.problem_start);
        Configuration goal = Configuration(problem.problem_end);

        std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.01, -10.01, -0.01, -0.01, -0.01, -10.01
        };

        std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
            10.01, 10.01, 0.01, 0.01, 0.01, 10.01
        };

        std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783}};
        std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

        vamp::planning::TaskSpaceConstraint<Robot, rake> tsr_constraint(
            eef_transforms_ref_frame_w_world,
            eef_transforms,
            tsr_lower_bound,
            tsr_upper_bound
        );


        vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> task_constraint(
            tsr_constraint
        );

        rrtc_settings.range = 0.4;
        rrtc_settings.max_iterations = 200000;
        rrtc_settings.dynamic_domain = 0;
        rrtc_settings.projection_method = vamp::planning::ProjMethod::OuterLM;
        rrtc_settings.descend_rate = 1.0;
        rrtc_settings.radius = 1.0;
        rrtc_settings.num_projection_iterations = 10;
        rrtc_settings.insert_all_to_tree = 1;
        rrtc_settings.std_dev_scaling_factor = 0.01F;
        // std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
        vamp::planning::invalid_distance_counter_outside = 0;
        vamp::planning::invalid_distance_counter_inside = 0;
        vamp::planning::collision_counter = 0;
        vamp::planning::unable_to_project_counter = 0;

        auto result =
            vamp::planning::CRRTC<Robot, rake, Robot::resolution, decltype(tsr_constraint)>::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

        if(result.path.size() > 0)
        {
            successful_problems++;
            nanoseconds_per_problem.push_back(result.nanoseconds);
            iterations_per_problem.push_back(result.iterations);
            vamp::planning::SimplifySettings simplify_settings;
            auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, decltype(tsr_constraint)>(
                result.path, env_v, task_constraint, simplify_settings, rng);
    
            // std::cout << "Planning took " << result.nanoseconds / 1000000.0 << " ms and " << result.iterations << " iterations, original path size: " << result.path.size() << ", simplified took : " << simplify_result.nanoseconds / 1000000.0 << " ms, path size: " << simplify_result.path.size() << std::endl;
            // std::ofstream outfile("/src/trajectory.txt");
            // for (const auto &config : simplify_result.path)
            // {
            //     const auto &array = config.to_array();
            //     Robot::ConfigurationArray soln;
            //     bool first = true;
            //     for (auto i = 0U; i < Robot::dimension; ++i)
            //     {
            //         // std::cout << array[i] << ", ";
            //         soln[i] = array[i];

            //         if (!first) outfile << ",";
            //         outfile << array[i];
            //         first = false;
            //     }

            //     outfile << "\n";
            // }
            // outfile.close();
            // // sleep for a bit to ensure file is written before next problem (since we are overwriting the same file for each problem)
            // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        }
        else {
            std::cout << "Unable to solve problem with start and goal configs : " << start << " and " << goal << std::endl;
            // std::ofstream outfile("/src/trajectory.txt");
            // std::vector<Configuration> start_goal_configs = {start, goal};
            // for (const auto &config : start_goal_configs)
            // {
            //     const auto &array = config.to_array();
            //     Robot::ConfigurationArray soln;
            //     bool first = true;
            //     for (auto i = 0U; i < Robot::dimension; ++i)
            //     {
            //         // std::cout << array[i] << ", ";
            //         soln[i] = array[i];

            //         if (!first) outfile << ",";
            //         outfile << array[i];
            //         first = false;
            //     }

            //     outfile << "\n";
            // }
            // outfile.close();
            // // sleep for a bit to ensure file is written before next problem (since we are overwriting the same file for each problem)
            // std::this_thread::sleep_for(std::chrono::milliseconds(5000));


        }
    }

    // print summary of results
    std::cout << "Total problems: " << total_num_problems << std::endl
                << "Successful problems: " << successful_problems << std::endl
                << "Success rate: " << (static_cast<float>(successful_problems) / total_num_problems) * 100.0f << "%" << std::endl;
    
    if(successful_problems > 0){
        std::size_t total_nanoseconds = 0;
        std::size_t total_iterations = 0;
        for(size_t i = 0; i < successful_problems; i++){
            total_nanoseconds += nanoseconds_per_problem[i];
            total_iterations += iterations_per_problem[i];
        }
        std::cout << "Average time (ms) for successful problems: " << (total_nanoseconds / successful_problems) / 1000000.0 << std::endl;
        std::cout << "Average iterations for successful problems: " << total_iterations / successful_problems << std::endl;
        // compute median for time and iterations
        std::sort(nanoseconds_per_problem.begin(), nanoseconds_per_problem.end());
        std::sort(iterations_per_problem.begin(), iterations_per_problem.end());
        std::cout << "Median time (ms) for successful problems: " << (nanoseconds_per_problem[successful_problems / 2]) / 1000000.0 << std::endl;
        std::cout << "Median iterations for successful problems: " << iterations_per_problem[successful_problems / 2] << std::endl;
    }
    return 0;
}

