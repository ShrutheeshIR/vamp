#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <future>
#include <atomic>
#include <thread>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>

#include <Eigen/Geometry>
#include <Eigen/Dense>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/panda_curobo.hh>
#include <vamp/random/halton.hh>
#include <fstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;

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

// This helper function allows nlohmann::json to "just work" with your struct
void to_json(json& j, const Problem& p) {
	j = json{
		{"tsr_lower_bound", p.tsr_lower_bound},
		{"tsr_upper_bound", p.tsr_upper_bound},
		{"eef_transforms_ref_frame_w_world", p.eef_transforms_ref_frame_w_world},
		{"eef_transforms", p.eef_transforms},
		{"problem_start", p.problem_start},
		{"problem_end", p.problem_end},
		{"cuboid_obstacles", p.cuboid_obstacles}
	};
}

// Structure to hold parameter combinations
struct RRTCParams {
	float range;
	bool dynamic_domain;
	vamp::planning::ProjMethod projection_method;
	float descend_rate;
	int num_projection_iterations;
	bool insert_all_to_tree;
};

template <typename... Constraints>
bool run_crrtc_attempts(
	const Configuration &start,
	const Configuration &goal,
	const EnvironmentVector &environment,
	vamp::planning::ComposableConstraints<Robot, rake, Constraints...> &constraint,
	typename RNG::Ptr rng,
	int num_iterations = 100000
){
	// Generate all parameter combinations
	float ranges[] = {0.5, 1.0, 1.5, 2.0};
	bool dd[] = {false, true};
	vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM, vamp::planning::ProjMethod::OuterLM};
	float descend_rates[] = {0.75, 1.0};
	int num_projection_iterations[] = {100};
	bool insert_all_to_tree[] = {false};

	std::vector<RRTCParams> param_combinations;
	for(const auto range: ranges){
		for(const auto dyndom: dd){
			for(const auto &pm: projection_method){
				for(const auto descent_rate: descend_rates){
					for(const auto num_proj_iters: num_projection_iterations){
						for(const auto insert_all: insert_all_to_tree){
							param_combinations.push_back({
								range,
								dyndom,
								pm,
								descent_rate,
								num_proj_iters,
								insert_all
							});
						}
					}
				}
			}
		}
	}

	// Shared atomic flag to signal success
	std::atomic<bool> success_found{false};

	// Lambda function to run a single parameter combination
	auto run_single_attempt = [&](const RRTCParams& params) -> bool {
		// Early exit if another thread found a solution
		if (success_found.load()) {
			return false;
		}

		vamp::planning::RRTCSettings rrtc_settings;
		rrtc_settings.range = params.range;
		rrtc_settings.dynamic_domain = params.dynamic_domain;
		rrtc_settings.projection_method = params.projection_method;
		rrtc_settings.descend_rate = params.descend_rate;
		rrtc_settings.radius = 1.0;
		rrtc_settings.num_projection_iterations = params.num_projection_iterations;
		rrtc_settings.insert_all_to_tree = params.insert_all_to_tree;
		rrtc_settings.max_iterations = num_iterations;
		// rrtc_settings.std_dev_scaling_factor = 0.0F;

		vamp::planning::invalid_distance_counter_outside = 0;
		vamp::planning::invalid_distance_counter_inside = 0;
		vamp::planning::collision_counter = 0;
		vamp::planning::unable_to_project_counter = 0;

		// Create a new RNG instance for this thread
		auto thread_rng = std::make_shared<vamp::rng::Halton<Robot>>();

		auto result = vamp::planning::CRRTC<Robot, rake, Robot::resolution, vamp::planning::TaskSpaceConstraint<Robot, rake>>::solve(
			start, goal, environment, rrtc_settings, constraint, thread_rng);

		if(result.path.size() > 0) {
			success_found.store(true);
			return true;
		}

		return false;
	};

	// Launch async tasks for all parameter combinations
	std::vector<std::future<bool>> futures;
	futures.reserve(param_combinations.size());

	for (const auto& params : param_combinations) {
		futures.push_back(std::async(std::launch::async, run_single_attempt, params));
	}

	// Wait for results and return true as soon as any succeeds
	for (auto& future : futures) {
		if (success_found.load()) {
			return true;
		}
		if (future.get()) {
			return true;
		}
	}

	return false;
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

std::vector<std::array<float, 4>> generate_sphere_obstacles(const Configuration &start_config, const Configuration &goal_config, size_t num_obstacles, float obstacle_radius, Eigen::Vector3f lower_bound, Eigen::Vector3f upper_bound)
{
	size_t MAX_OBSTACLE_ATTEMPTS = 1000;
	std::vector<std::array<float, 4>> obstacles;
	size_t num_obstacles_added = 0;
	size_t num_attempts = 0;
	while (num_obstacles_added < num_obstacles && num_attempts++ < MAX_OBSTACLE_ATTEMPTS)
	{
		Eigen::Vector3f center = lower_bound + ((Eigen::Vector3f::Random() + Eigen::Vector3f::Ones()) / 2.0f).cwiseProduct(upper_bound - lower_bound);
        // std::cout << "Attempting to add obstacle at " << center.transpose() << std::endl;
		// check if it collides with the start or goal configuration, if so, resample
		EnvironmentInput environment;
		std::array<float, 3> sphere = {center[0], center[1], center[2]};
		environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, obstacle_radius));
		environment.sort();
		auto env_v = EnvironmentVector(environment);
		if (
			(vamp::planning::validate_motion<Robot, rake, 1>(start_config, start_config, env_v)) &&
			(vamp::planning::validate_motion<Robot, rake, 1>(goal_config, goal_config, env_v))
		)
			obstacles.push_back({center[0], center[1], center[2], obstacle_radius});
			num_obstacles_added++;
	}
	return obstacles;
}

std::vector<std::array<float, 6>> generate_cube_obstacles(
	const Configuration &start_config,
	const Configuration &goal_config,
	const Eigen::Vector3f &start_eef_pos,
	const Eigen::Vector3f &goal_eef_pos,
	size_t num_obstacles,
	float obstacle_half_length,
	Eigen::Vector3f lower_bound,
	Eigen::Vector3f upper_bound,
	vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> &task_constraint,
	typename RNG::Ptr rng
)
{
	size_t MAX_OBSTACLE_ATTEMPTS = 1000;
	std::vector<std::array<float, 6>> obstacles;
	size_t num_obstacles_added = 0;
	size_t num_attempts = 0;
	while (num_obstacles_added < num_obstacles && num_attempts++ < MAX_OBSTACLE_ATTEMPTS)
	{
		Eigen::Vector3f center = lower_bound + ((Eigen::Vector3f::Random() + Eigen::Vector3f::Ones()) / 2.0f).cwiseProduct(upper_bound - lower_bound);

		// check that the center does not intersect with the line between the start and goal configuration, if so, resample
		// first approximate the cube by a sphere, then check if the sphere intersects with the line segment between the start and goal eef positions
		Eigen::Vector3f line_vec = goal_eef_pos - start_eef_pos;

		const Eigen::Vector3f m = center - start_eef_pos;
		const float d2 = line_vec.squaredNorm();

		if (d2 == 0.0)
			continue;

		// Squared distance from point to infinite line
		const float dist2 = m.cross(line_vec).squaredNorm() / d2;

		if(dist2 <= (obstacle_half_length + 0.02F) * (obstacle_half_length + 0.02F)){
			continue;
		}


		EnvironmentInput environment;
		std::array<float, 3> sphere = {center[0], center[1], center[2]};
		const auto random_obstacle_half_length = ((Eigen::Vector3f::Random() + Eigen::Vector3f::Ones()) / 6.0f).norm() * obstacle_half_length;
		environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(sphere, {0.0, 0.0, 0.0}, {random_obstacle_half_length  + 0.01, random_obstacle_half_length  + 0.01, random_obstacle_half_length  + 0.01}));
		environment.sort();
		auto env_v = EnvironmentVector(environment);
		if (
			(vamp::planning::validate_motion<Robot, rake, 1>(start_config, start_config, env_v)) &&
			(vamp::planning::validate_motion<Robot, rake, 1>(goal_config, goal_config, env_v))
		)
		{
			bool any_successful_plan = run_crrtc_attempts(start_config, goal_config, env_v, task_constraint, rng, 100);
			if(!any_successful_plan){
				continue;
			}

			obstacles.push_back({center[0], center[1], center[2], random_obstacle_half_length, random_obstacle_half_length, random_obstacle_half_length});
			num_obstacles_added++;
		}
	}
	return obstacles;
}

auto potentially_add_start_goal_to_problems(Robot::Configuration start_config, Robot::Configuration goal_config, std::vector<Problem> &problems, const Problem &reference_problem, vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> &task_constraint, typename RNG::Ptr rng)
{
	auto start_eef_pose = Robot::eefk(config_to_array(start_config))[0].matrix();
	auto goal_eef_pose = Robot::eefk(config_to_array(goal_config))[0].matrix();
	Eigen::Vector3f start_eef_pos = start_eef_pose.block<3,1>(0,3);
	Eigen::Vector3f goal_eef_pos = goal_eef_pose.block<3,1>(0,3);


	// create bounding box for obstacle generation
	// Eigen::Vector3f lower_bound = start_eef_pos.cwiseMin(goal_eef_pos) - Eigen::Vector3f(0.5, 0.5, 0.5);
	// Eigen::Vector3f upper_bound = start_eef_pos.cwiseMax(goal_eef_pos) + Eigen::Vector3f(0.5, 0.5, 0.5);
	Eigen::Vector3f lower_bound = Eigen::Vector3f(-0.1, -0.8, -0.2);
	Eigen::Vector3f upper_bound = Eigen::Vector3f(0.8, 0.8, 0.7);
	size_t num_obstacle_problems_added = 0;

	std::vector<size_t> obstacle_counts;
	for (size_t num_obstacles = 0U; num_obstacles < 110; num_obstacles += 10) {
		obstacle_counts.push_back(num_obstacles);
	}

	bool obstacle_adding_failed = false;

	for (const auto num_obstacles : obstacle_counts) {
		// if (obstacle_adding_failed)
		//     break;
		// std::cout << "Attempting to add " << num_obstacles << " obstacles." << std::endl;
		for (size_t obstacle_attempt = 0U; obstacle_attempt < 25; ++obstacle_attempt) {
			auto cuboid_obstacles = generate_cube_obstacles(start_config, goal_config, start_eef_pos, goal_eef_pos, num_obstacles, 0.05f, lower_bound, upper_bound, task_constraint, rng);

			EnvironmentInput environment;
			// now try to solve the problem with these obstacles
			for (const auto &cuboid : cuboid_obstacles) {
				environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({cuboid[0], cuboid[1], cuboid[2]}, {0.0, 0.0, 0.0}, {cuboid[3], cuboid[4], cuboid[5]}));
			}
			environment.sort();
			auto env_v = EnvironmentVector(environment);

			bool any_successful_plan = run_crrtc_attempts(start_config, goal_config, env_v, task_constraint, rng);
			if (!any_successful_plan){
				obstacle_adding_failed = true;
				break;
			}

			Problem p{
				reference_problem.tsr_lower_bound,
				reference_problem.tsr_upper_bound,
				reference_problem.eef_transforms_ref_frame_w_world,
				reference_problem.eef_transforms,
				config_to_array(start_config),
				config_to_array(goal_config),
				cuboid_obstacles
			};
			std::cout << "Adding to problem set :) " << start_config << goal_config << p.cuboid_obstacles.size() << std::endl;
			problems.push_back(p);
			num_obstacle_problems_added++;
			break;
		}
	}
}



auto generate_constraint_and_start_goal(){

	// for the line constraint, first we want y line, i.e. x and z are fixed
	// for the line we choose 0, 1,0,0,   0.3486, 0.647752, 0.2399
	// from here, we generate n more lines, decreasing and increasing the z and the x by some amount
	// for this line, we also modify the orientation to be
	// get eefk of (0.0,0.0,0.0,0.0,0.0,1.57,0.82)

	std::vector<Problem> problems_initializer;
	std::array<std::array<float, 3>, 2> reference_positions = {{{0.3486, 0.647752, 0.2399}, {0.55, 0.4, 0.5}}};
	std::array<float, 9> pos_offsets = {0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2};
	std::array<std::array<float, 4>, 2> orientations = {{
		{0, 1,0,0},
		{0.5, -0.5, 0.5, 0.5}
	}};
	std::array<std::array<std::array<float, 7>, 2>, 2> reference_configs_to_start_projecting = {{
		{{
		{1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100},
		{-1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000} // this is for 0 1 0 0
		}},
		{{
		{-0.9463047, 0.53955036, 0.3271516, -1.353034, 1.1967324, 0.9882862, 2.0019703}, // this is for 0 0.7071 -0.0 0.7071
		{-0.47991845, 1.1638713, 1.6636035, -1.7775084, 0.40887687, 2.8879647, 2.3643737}
		}}
	}};


	for(const auto &x_pos_offset: pos_offsets){
		for(const auto &z_pos_offset: pos_offsets){
			size_t orientation_index = 0;
			for(const auto &orientation: orientations){
			std::array<float, 7> eef_transforms = {
					orientation[0], orientation[1], orientation[2], orientation[3],
					reference_positions[orientation_index][0] + x_pos_offset, reference_positions[orientation_index][1], reference_positions[orientation_index][2] + z_pos_offset
				};
				std::array<float, 7> eef_transforms_ref_frame_w_world = {1, 0, 0, 0, 0, 0, 0};
				Problem p;
				p.eef_transforms = eef_transforms;
				p.eef_transforms_ref_frame_w_world = eef_transforms_ref_frame_w_world;
				if(orientation_index == 0){
					p.tsr_lower_bound = {-10.001, -10.001, -0.001, -0.001, -0.001, -0.001};
					p.tsr_upper_bound = {10.001, 10.001, 0.001, 0.001, 0.001, 0.001};
				}
				else{
					p.tsr_lower_bound = {-10.001, -0.001, -10.001, -0.001, -0.001, -0.001};
					p.tsr_upper_bound = {10.001, 0.001, 10.001, 0.001, 0.001, 0.001};
				}
				p.problem_start = reference_configs_to_start_projecting[orientation_index][0];
				p.problem_end = reference_configs_to_start_projecting[orientation_index][1];
				problems_initializer.push_back(p);
				orientation_index++;
			}
		}
	}

	std::array<float, 3> reference_position = {0.35, 0.0, 0.2399};
	for(const auto &x_pos_offset: pos_offsets){
		for(const auto &y_pos_offset: pos_offsets){
			std::array<float, 7> eef_transforms = {
					0.0, -0.0, -0.7071, 0.7071,
					reference_position[0] + x_pos_offset, reference_position[1] + y_pos_offset, reference_position[2]
				};
			std::array<float, 7> eef_transforms_ref_frame_w_world = {1, 0, 0, 0, 0, 0, 0};
			Problem p;
			p.eef_transforms = eef_transforms;
			p.eef_transforms_ref_frame_w_world = eef_transforms_ref_frame_w_world;
			p.tsr_lower_bound = {-0.001, -10.001, -10.001, -0.001, -0.001, -0.001};
			p.tsr_upper_bound = {0.001, 10.001, 10.001, 0.001, 0.001, 0.001};
			p.problem_start = {1.2,-0.26,-0.75,-1.88,-1.24,1.26,-1.84};
			p.problem_end = {1.26,1.8326,-0.32,-1.22,-0.21,1.51,-1.08};
			problems_initializer.push_back(p);
		}
	}

	// now try to generate similarly for a top-down constraint

	return problems_initializer;
}


auto generate_prespecified_line_problems() {
	size_t TOT_NUM_START_GOAL_PAIRS_PER_CONSTRAINT = 100;
	const char *output_path = "scripts/cpp/benchmarks/line_plane_benchmark_problems/tsr_panda_problems_curobo_cuboid_prespecified_plane_curobo_likes.json";
	// std::ofstream file(output_path);


	size_t num_start_goal_sampling_attempts = 50;
	float sampling_std_dev = 0.1F;


	auto ref_problems_intialized = generate_constraint_and_start_goal();
	std::vector<Problem> problems;
	for(const auto &ref_problem: ref_problems_intialized){
		// if problems.size > 0 write to file
		if (problems.size() > 0){
			std::ofstream file(output_path, std::ios::out | std::ios::trunc);
			if (file && !problems.empty()) {
				file << json(problems).dump(4);
			}
			file.close();
		}



		// the start and goal are reference and we can generate variations around them by sampling around them in a gaussian manner and then project them
		vamp::planning::TaskSpaceConstraint<Robot, rake> tsr_constraint(
			{ref_problem.eef_transforms_ref_frame_w_world},
			{ref_problem.eef_transforms},
			ref_problem.tsr_lower_bound,
			ref_problem.tsr_upper_bound
		);
		EnvironmentInput environment;
		auto env_v = EnvironmentVector(environment);

		vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> task_constraint(
			tsr_constraint
		);

		auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


		for(size_t start_goal_config_attempt = 0U; start_goal_config_attempt < num_start_goal_sampling_attempts; ++start_goal_config_attempt){
			// convert the reference start and goal from ref_problem.problem_start and problem_end to Eigen vectors and add gaussian noise to them
			Eigen::VectorXf ref_start = Eigen::Map<const Eigen::VectorXf>(ref_problem.problem_start.data(), Robot::dimension);
			Eigen::VectorXf ref_goal = Eigen::Map<const Eigen::VectorXf>(ref_problem.problem_end.data(), Robot::dimension);
			Eigen::VectorXf sampled_start = ref_start + sampling_std_dev * Eigen::VectorXf::Random(Robot::dimension);
			Eigen::VectorXf sampled_goal = ref_goal + sampling_std_dev * Eigen::VectorXf::Random(Robot::dimension);

			// project the sampled start and goal onto the constraint
			std::array<float, Robot::dimension> float_sampled_start, float_sampled_goal;
			for (auto i = 0U; i < Robot::dimension; ++i)
			{
				float_sampled_start[i] = sampled_start[i];
				float_sampled_goal[i] = sampled_goal[i];
			}
			std::vector<typename Robot::Configuration> start_projected_vector, goal_projected_vector;
			if (
				(vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(
								Robot::Configuration(float_sampled_start),
								Robot::Configuration(float_sampled_start),
								start_projected_vector,
								task_constraint,
								env_v,
								vamp::planning::ProjMethod::InnerLM,
								0.75,
								100,
								false,
								true
					)
				)
				&&
				(vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(
								Robot::Configuration(float_sampled_goal),
								Robot::Configuration(float_sampled_goal),
								goal_projected_vector,
								task_constraint,
								env_v,
								vamp::planning::ProjMethod::InnerLM,
								0.75,
								100,
								false,
								true
					)
				)
			) {
					std::cout << "Successfully projected sampled start and goal onto constraint." << std::endl;
					auto start_config = start_projected_vector.back();
					auto goal_config = goal_projected_vector.back();
					if((goal_config - start_config).l2_norm() < 1.0){
						// std::cout << " Too close in configuration space, skipping." << std::endl;
						continue;
					}
					auto start_eef_pose = Robot::eefk(config_to_array(start_config))[0].matrix();
					auto goal_eef_pose = Robot::eefk(config_to_array(goal_config))[0].matrix();
					Eigen::Vector3f start_eef_pos = start_eef_pose.block<3,1>(0,3);
					Eigen::Vector3f goal_eef_pos = goal_eef_pose.block<3,1>(0,3);

					if((start_eef_pos - goal_eef_pos).norm() < 0.5){
						// std::cout << "Start and goal are too close in end-effector space, skipping." << std::endl;
						continue;
					}

					// now check if plan is feasible
					bool any_successful_plan = run_crrtc_attempts(start_config, goal_config, env_v, task_constraint, rng);
					if (!any_successful_plan){
						// std::cout << "Failed to find a successful plan for start config " << start_config << " and goal config " << goal_config << std::endl;
						continue;
					}
					std::cout << "Trying out " << start_config << " and " << goal_config << std::endl;
					potentially_add_start_goal_to_problems(start_config, goal_config, problems, ref_problem, task_constraint, rng);
					break;
			}
			else {
				std::cout << "Projection failed for sampled start or goal, skipping." << start_projected_vector.size() << " " << goal_projected_vector.size() << Robot::Configuration(float_sampled_start) << Robot::Configuration(float_sampled_goal) << std::endl;
			}




			if(problems.size() >= TOT_NUM_START_GOAL_PAIRS_PER_CONSTRAINT){
				break;
			}
		}




	}
	// close the file after writing all problems
	if (problems.size() > 0){
		std::ofstream file(output_path, std::ios::out | std::ios::trunc);
		if (file && !problems.empty()) {
			file << json(problems).dump(4);
		}
		file.close();
	}
	return -1;

}


// int old_main() {
// 	size_t num_constraints_added = 0;
// 	size_t TOT_NUM_CONSTRAINTS = 1;
// 	size_t TOT_NUM_START_GOAL_PAIRS_PER_CONSTRAINT = 100;


// 	std::vector<Problem> problems;
// 	const char *output_path = "/src/tsr_panda_problems_curobo_cuboid_prespecified_line.json";
// 	std::ofstream file(output_path);

//     // std::array<std::array<bool, 6>, 12> tsr_bound_combinations = {{
//     //     // true indicates no-constraint for that dimension, false indicates constraint
//     //     // x plane without orientation
//     //     {false, true, true, true, true, true},
//     //     // x plane with orientation
//     //     {false, true, true, false, false, false},
//     //     // y plane without orientation
//     //     {true, false, true, true, true, true},
//     //     // y plane with orientation
//     //     {true, false, true, false, false, false},
//     //     // z plane without orientation
//     //     {true, true, false, true, true, true},
//     //     // z plane with orientation
//     //     {true, true, false, false, false, false},
//     //     // xz-line without orientation
//     //     {false, true, false, true, true, true},
//     //     // xz-line with orientation
//     //     {false, true, false, false, false, false},
//     //     // xy line without orientation
//     //     {false, false, true, true, true, true},
//     //     // xy line with orientation
//     //     {false, false, true, false, false, false},
//     //     // yz line without orientation
//     //     {true, false, false, true, true, true},
//     //     // yz line with orientation
//     //     {true, false, false, false, false, false},
//     // }};


// 	auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

// 	while(num_constraints_added < TOT_NUM_CONSTRAINTS) {
// 		if (num_constraints_added > 0 && file.is_open()) {
// 			file.seekp(0);
// 			file << json(problems).dump(4);
// 			file.flush();
// 		}
// 		std::cout << "Attempting to generate problems for constraint " << num_constraints_added + 1 << std::endl;

//         // todo
//         // instead of doing this. Pick a specific line (along y)
//         // do with and without orientation for the line
//         // then pick along x and do with and without orientation
//         // then pick a diagonal from (-1, -1, -1) to (1, 1, 1) and do with and without orientation
//         // this way we can get a good distribution of problems along different parts of the space and with different types of constraints


// 		typedef Eigen::Quaterniond Quatd;
// 		// Generate a random unit quaternion
// 		Quatd random_quat = Quatd::UnitRandom();
// 		Eigen::VectorXf rand_translation = Eigen::VectorXf::Random(3);
// 		std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{random_quat.w(), random_quat.x(), random_quat.y() ,random_quat.z(), rand_translation[0], rand_translation[1], rand_translation[2] }};
// 		// std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0.0, 1.0, 0.0 ,0.0, rand_translation[0], rand_translation[1], rand_translation[2] }};
// 		std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};
//         // std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0, 1,0,0,   0.3486, 0.647752, 0.2399}};
//         // std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

//         std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
//             -0.0025, -10.0025, -10.0025, -0.0025, -0.0025, -0.0025
//         };
//         std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
//             0.0025, 10.0025, 10.0025, 0.0025, 0.0025, 0.0025
//         };
// 		vamp::planning::TaskSpaceConstraint<Robot, rake> tsr_constraint(
// 			eef_transforms_ref_frame_w_world,
// 			eef_transforms,
// 			tsr_lower_bound,
// 			tsr_upper_bound
// 		);

// 		EnvironmentInput environment;
// 		auto env_v = EnvironmentVector(environment);

// 		vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> task_constraint(
// 			tsr_constraint
// 		);
// 		bool goal_added_for_start = false;

//         // std::cout << "Attempting to generate problems for constraint " << num_constraints_added + 1 << std::endl;
// 		size_t num_start_goal_pairs_added = 0;
// 		for(size_t config_attempt = 0U; config_attempt < 1000; config_attempt++){
//             // std::cout << "Num start goal pairs added " << num_start_goal_pairs_added << std::endl;
// 			if(num_start_goal_pairs_added >= TOT_NUM_START_GOAL_PAIRS_PER_CONSTRAINT){
// 				break;
// 			}
// 			if (goal_added_for_start > 0)
// 			{
//     			if (file.is_open()) {
// 					file.seekp(0);
// 					file << json(problems).dump(4);
// 					file.flush();
//     			}

// 			}
// 			auto start = rng->next();
// 			typename Robot::template ConfigurationBlock<rake> block, projected_block;


// 			std::vector<typename Robot::Configuration> start_projected_vector;

// 			if (vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(
// 					start,
// 					start,
// 					start_projected_vector,
// 					task_constraint,
// 					env_v,
// 					vamp::planning::ProjMethod::InnerLM,
// 					0.75,
// 					100,
// 					false,
// 					true
// 				))
// 				{
// 					// std::cout << "Able to get start " << start << start_projected_vector.back() << std::endl;
// 					for(size_t goal_config_attempt = 0U; goal_config_attempt < 1000; goal_config_attempt++){
// 						auto goal = rng->next();
// 						typename Robot::template ConfigurationBlock<rake> block, projected_block;
// 						std::vector<typename Robot::Configuration> goal_projected_vector;

// 						if (vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(
// 								goal,
// 								goal,
// 								goal_projected_vector,
// 								task_constraint,
// 								env_v,
// 								vamp::planning::ProjMethod::InnerLM,
// 								0.75,
// 								100,
// 								false,
// 								true
// 							))
// 							{
// 								// std::cout << "Able to get goal " << goal << goal_projected_vector.back() << std::endl;

// 								auto start_config = start_projected_vector.back();
// 								auto goal_config = goal_projected_vector.back();
// 								if((goal_config - start_config).l2_norm() < 1.0){
// 									// std::cout << " Too close in configuration space, skipping." << std::endl;
// 									continue;
// 								}
// 								auto start_eef_pose = Robot::eefk(config_to_array(start_config))[0].matrix();
// 								auto goal_eef_pose = Robot::eefk(config_to_array(goal_config))[0].matrix();
// 								Eigen::Vector3f start_eef_pos = start_eef_pose.block<3,1>(0,3);
// 								Eigen::Vector3f goal_eef_pos = goal_eef_pose.block<3,1>(0,3);

// 								if((start_eef_pos - goal_eef_pos).norm() < 0.5){
// 									// std::cout << "Start and goal are too close in end-effector space, skipping." << std::endl;
// 									continue;
// 								}

// 								// now check if plan is feasible
// 								bool any_successful_plan = run_crrtc_attempts(start_config, goal_config, env_v, task_constraint, rng);
// 								if (!any_successful_plan){
//                                     // std::cout << "Failed to find a successful plan for start config " << start_config << " and goal config " << goal_config << std::endl;
// 									break;
//                                 }




// 								std::cout << start_eef_pos.transpose() << goal_eef_pos.transpose() << std::endl;

// 								// create bounding box for obstacle generation
// 								Eigen::Vector3f lower_bound = start_eef_pos.cwiseMin(goal_eef_pos) - Eigen::Vector3f(0.25, 0.25, 0.25);
// 								Eigen::Vector3f upper_bound = start_eef_pos.cwiseMax(goal_eef_pos) + Eigen::Vector3f(0.25, 0.25, 0.25);
//                                 std::cout << start_eef_pos.transpose() << " " << goal_eef_pos.transpose() << " " << lower_bound.transpose() << " " << upper_bound.transpose() << std::endl;

// 								size_t num_obstacle_problems_added = 0;

// 								if (start_projected_vector.empty() || goal_projected_vector.empty()) {
// 									std::cerr << "Error: Projected vectors are empty!" << std::endl;
// 									return 0; // or handle error
// 								}

// 								std::vector<size_t> obstacle_counts;
// 								for (size_t num_obstacles = 0U; num_obstacles < 200; num_obstacles += 20) {
// 									obstacle_counts.push_back(num_obstacles);
// 								}

// 								// std::vector<std::future<std::pair<bool, Problem>>> obstacle_futures;
// 								// obstacle_futures.reserve(obstacle_counts.size());
//                                 bool obstacle_adding_failed = false;

// 								for (const auto num_obstacles : obstacle_counts) {
//                                     // if (obstacle_adding_failed)
//                                     //     break;

//                                     // obstacle_futures.push_back(std::async(std::launch::async, [&, num_obstacles]() {
//                                         // std::cout << "Attempting with " << num_obstacles << " obstacles." << std::endl;
// 										for (size_t obstacle_attempt = 0U; obstacle_attempt < 10; ++obstacle_attempt) {
// 											auto cuboid_obstacles = generate_cube_obstacles(start_config, goal_config, num_obstacles, 0.05f, lower_bound, upper_bound);

// 											EnvironmentInput environment;
// 											// now try to solve the problem with these obstacles
// 											for (const auto &cuboid : cuboid_obstacles) {
// 												environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({cuboid[0], cuboid[1], cuboid[2]}, {0.0, 0.0, 0.0}, {cuboid[3], cuboid[4], cuboid[5]}));
// 											}
// 											environment.sort();
// 											auto env_v = EnvironmentVector(environment);

// 											bool any_successful_plan = run_crrtc_attempts(start_config, goal_config, env_v, task_constraint, rng);
//                                             if (!any_successful_plan){
//                                                 obstacle_adding_failed = true;
//                                                 break;
//                                             }

// 											Problem p{
// 												tsr_lower_bound,
// 												tsr_upper_bound,
// 												eef_transforms_ref_frame_w_world[0],
// 												eef_transforms[0],
// 												config_to_array(start_projected_vector.back()),
// 												config_to_array(goal_projected_vector.back()),
// 												cuboid_obstacles
// 											};
//                                             std::cout << "Adding to problem set :) " << start_projected_vector.back() << goal_projected_vector.back() << p.cuboid_obstacles.size() << std::endl;
//                                             problems.push_back(p);
//                                             num_obstacle_problems_added++;
//                                             goal_added_for_start = true;
//                                             break;

// 											// return std::make_pair(true, p);
// 										// }

// 										// return std::make_pair(false, Problem{});
// 									}
//                                 // ));
// 								}

// 								// for (auto &future : obstacle_futures) {
// 								// 	auto result = future.get();
// 								// 	if (!result.first) {
// 								// 		continue;
// 								// 	}

// 								// 	std::cout << "Adding start goal to problem set :) " << start_projected_vector.back() << goal_projected_vector.back() << result.second.obstacles.size() << std::endl;
// 								// 	problems.push_back(result.second);
// 								// 	num_obstacle_problems_added++;
// 								// 	goal_added_for_start = true;
// 								// }

// 								if (num_obstacle_problems_added > 0)
// 									num_start_goal_pairs_added++;

// 								break;
// 							}

// 					}

// 				}

// 		}
//         if(num_start_goal_pairs_added > 0){
//             num_constraints_added++;
//         }


// 	}

//     if(problems.empty()){
//         std::cerr << "No problems generated!" << std::endl;
//         return 0;
//     }
// 	json j_all_problems = problems;
// 	if (file.is_open()) {
// 		file << j_all_problems.dump(4); // '4' is the indentation level for readability
// 		std::cout << "Successfully saved to problems.json" << std::endl;
// 	}

// 	return 1;

// }


int main() {
	return generate_prespecified_line_problems();
}
