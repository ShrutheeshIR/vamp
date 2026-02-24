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

// #include <tqdm.hh>

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
	std::array<float, 6> tsr_lower_bound;
	std::array<float, 6> tsr_upper_bound;
	std::array<float, 7> eef_transforms_ref_frame_w_world;
	std::array<float, 7> eef_transforms;
	std::array<float, 7> problem_start;
	std::array<float, 7> problem_end;
	std::vector<std::array<float, 6>> cuboid_obstacles;
};


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
            p.tsr_lower_bound = item.at("tsr_lower_bound").get<std::array<float, 6>>();
            p.tsr_upper_bound = item.at("tsr_upper_bound").get<std::array<float, 6>>();
            p.eef_transforms_ref_frame_w_world = item.at("eef_transforms_ref_frame_w_world").get<std::array<float, 7>>();
            p.eef_transforms = item.at("eef_transforms").get<std::array<float, 7>>();
            p.cuboid_obstacles = item.at("cuboid_obstacles").get<std::vector<std::array<float, 6>>>();
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

// Equivalent of curobo's SolvedResult class
struct SolvedResult {
	bool success;
	std::vector<Configuration> plan;
	double solve_time;  // Single time in nanoseconds (instead of multiple independent times)
	std::size_t num_cuboid_obstacles;

	SolvedResult(bool success_ = false, std::vector<Configuration> plan_ = {}, 
	             double solve_time_ = 0.0, std::size_t num_cuboid_obstacles_ = 0)
		: success(success_), plan(plan_), solve_time(solve_time_), 
		  num_cuboid_obstacles(num_cuboid_obstacles_) {}

	// Convert to JSON for saving
	json to_json() const {
		json j;
		j["success"] = success;
		j["solve_time_ns"] = solve_time;
		j["total_solve_time"] = solve_time / 1000000.0;
		j["num_cuboid_obstacles"] = num_cuboid_obstacles;
		
		// Serialize trajectory
		std::vector<std::vector<float>> plan_data;
		for (const auto &config : plan) {
			auto array = config.to_array();
			std::vector<float> config_vec;
			for (auto i = 0U; i < Robot::dimension; ++i) {
				config_vec.push_back(array[i]);
			}
			plan_data.push_back(config_vec);
		}
		j["plan"] = plan_data;
		j["plan_length"] = plan.size();
		
		return j;
	}

	// Save results to JSON file
	static void save_solved_results(const std::vector<SolvedResult> &results, const std::string &save_path) {
		json plot_data = json::array();
		
		for (const auto &result : results) {
			plot_data.push_back(result.to_json());
		}
		
		std::ofstream output_file(save_path);
		if (!output_file.is_open()) {
			std::cerr << "Failed to open output file: " << save_path << std::endl;
			return;
		}
		
		output_file << plot_data.dump(4) << std::endl;
		output_file.close();
		
		std::cout << "Results saved to: " << save_path << std::endl;
	}

	// Save a single result to JSON file
	void save(const std::string &save_path) const {
		std::vector<SolvedResult> results = {*this};
		SolvedResult::save_solved_results(results, save_path);
	}
};




int main() {

	std::vector<Problem> problems;


    EnvironmentInput environment;


    // load the json file of problems from the specified path
    std::string problem_json_path = "/src/tsr_panda_problems_curobo_cuboid.json";
    load_problems_from_json(problems, problem_json_path);

    vamp::planning::RRTCSettings rrtc_settings;
    
    size_t total_num_problems = 0;
    size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;
    std::vector<SolvedResult> all_solved_results;


    for(const auto &problem: problems){
        // std::cout << "Planning problem " << total_num_problems + 1 << " / " << problems.size() << std::endl;
        total_num_problems++;
		auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


        Configuration start = Configuration(problem.problem_start);
        Configuration goal = Configuration(problem.problem_end);

        std::array<float, 6 * Robot::n_eef> tsr_lower_bound = problem.tsr_lower_bound;
        std::array<float, 6 * Robot::n_eef> tsr_upper_bound = problem.tsr_upper_bound;

        // std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = problem.eef_transforms;
        // std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = problem.eef_transforms_ref_frame_w_world;

        std::array<std::array<float, 7>, Robot::n_eef> eef_transforms;
        // set eef_transforms[0] to problem.eef_transforms
        eef_transforms[0] = problem.eef_transforms;
        std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world;
        // set eef_transforms_ref_frame_w_world[0] to problem.eef_transforms_ref_frame_w_world
        eef_transforms_ref_frame_w_world[0] = problem.eef_transforms_ref_frame_w_world;


        vamp::planning::TaskSpaceConstraint<Robot, rake> tsr_constraint(
            eef_transforms_ref_frame_w_world,
            eef_transforms,
            tsr_lower_bound,
            tsr_upper_bound
        );


        vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> task_constraint(
            tsr_constraint
        );

        rrtc_settings.range = 1.5;
        rrtc_settings.max_iterations = 200000;
        rrtc_settings.dynamic_domain = 0;
        rrtc_settings.projection_method = vamp::planning::ProjMethod::InnerLM;
        rrtc_settings.descend_rate = 0.75;
        rrtc_settings.radius = 1.0;
        rrtc_settings.num_projection_iterations = 20;
        rrtc_settings.insert_all_to_tree = 1;
        // std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
        vamp::planning::invalid_distance_counter_outside = 0;
        vamp::planning::invalid_distance_counter_inside = 0;
        vamp::planning::collision_counter = 0;
        vamp::planning::unable_to_project_counter = 0;


        EnvironmentInput environment;
        // now try to solve the problem with these obstacles
        for (const auto &cuboid : problem.cuboid_obstacles) {
            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({cuboid[0], cuboid[1], cuboid[2]}, {0.0, 0.0, 0.0}, {cuboid[3], cuboid[4], cuboid[5]}));
        }
        environment.sort();
        auto env_v = EnvironmentVector(environment);


        auto result =
            vamp::planning::CRRTC<Robot, rake, Robot::resolution, decltype(tsr_constraint)>::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

        if(result.path.size() > 0)
        {
            successful_problems++;
            nanoseconds_per_problem.push_back(result.nanoseconds);
            iterations_per_problem.push_back(result.iterations);
            vamp::planning::SimplifySettings simplify_settings;
            // auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, decltype(tsr_constraint)>(
            //     result.path, env_v, task_constraint, simplify_settings, rng);
    
            // Create SolvedResult object with the simplified path
            SolvedResult solved(
                true,                                   // success
                result.path,                   // plan (trajectories)
                static_cast<double>(result.nanoseconds), // solve_time
                problem.cuboid_obstacles.size()         // num_cuboid_obstacles
            );
            all_solved_results.push_back(solved);
    
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
    
    // Save all results to JSON
    SolvedResult::save_solved_results(all_solved_results, "/src/tsr_panda_problems_curobo_cuboid_plane_vamp_results.json");
    
    return 0;
}

