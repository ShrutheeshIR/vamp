#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/task_crrtc.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>
#include <vamp/parsers/mjcf_parser.hh>
#include <vamp/planning/simplify_constraints.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/digit.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::Digit;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
// using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution, 4>;
using AttachmentInput = vamp::collision::Attachment<float>;
using RNG = typename vamp::rng::RNG<Robot>;
static constexpr Robot::ConfigurationArray start = {-0.05070,0.00044,-0.51631,0.00265,0.19455,-0.00952,0.36900,-0.04901,-0.68524,-0.82693,-0.04935,0.86245,-0.62277,-0.06327,-0.36038,0.00939,0.65969,0.84571,0.01946,-0.89942,0.40086,0.05724,0.12483,-0.29145,0.21554,1.10831,-0.32827,0.21262,-0.21520,-1.03009};


// constexpr-friendly hash (works at runtime too)
constexpr unsigned hash(unsigned x)
{
    x ^= x >> 16;
    x *= 0x7feb352d;
    x ^= x >> 15;
    x *= 0x846ca68b;
    x ^= x >> 16;
    return x;
}


template <std::size_t n, std::size_t... I>
std::array<float, n> generate_random_percents(int attempt, std::index_sequence<I...>)
{
    return {
        (static_cast<void>(I),
         static_cast<float>(hash(attempt * 131543911u + I) % 1000) / 1000.0f - 0.5f)...
    };
}

template <std::size_t n>
struct RandomPercents
{
    static std::array<float, n> get(int attempt)
    {
        return generate_random_percents<n>(attempt, std::make_index_sequence<n>());
    }
};

template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
auto plan_to_pose_sampling(
    vamp::planning::ComposableConstraints<Robot, rake, Constraints...> &constraint,
    const typename Robot::Configuration &start_state,
    const EnvironmentVector &environment,
    typename RNG::Ptr rng,
    int max_attempts = 10,
    std::ofstream &outfile = std::ofstream("/src/trajectory.txt")
)
{
    vamp::planning::RRTCSettings rrtc_settings;

    std::array<vamp::FloatT, Robot::dimension> out_config;
    static std::random_device gen;
    std::uniform_real_distribution<float> dist_noise(-0.025, 0.025);
    for (int attempt = 0; attempt < max_attempts; ++attempt) {

        auto start_time = std::chrono::steady_clock::now();
        typename Robot::template ConfigurationBlock<rake> config_block;
        const auto random_percents = vamp::FloatVector<rake>(RandomPercents<rake>::get(attempt));

        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            config_block[i] = start_state.broadcast(i) + random_percents;
        }

        // std::cout << config_block << std::endl;

        typename Robot::ConfigurationBlock<rake> projected_block;
        auto success = constraint.projectConfiguration(config_block, projected_block, vamp::planning::ProjMethod::InnerLM, 35.0, 1.0, 100, false);
        // std::cout << projected_block << std::endl;
        std::cout << success << std::endl;
        // constraint.print_robot_tsr_error(projected_block);
        if (success >= 0) {
            for(auto d = 0U; d < Robot::dimension; d++)
                out_config[d] = projected_block[{d, success}];
            std::vector<typename Robot::Configuration> projected_vector;
            std::cout << typename Robot::Configuration(out_config) << std::endl;
            if (vamp::planning::validate_motion<Robot, rake, resolution>(typename Robot::Configuration(out_config), typename Robot::Configuration(out_config), environment))
            {
                std::cout << "Time taken: " << vamp::utils::get_elapsed_nanoseconds(start_time) << " ns" << std::endl;
                rrtc_settings.range = 0.75;
                rrtc_settings.max_iterations = 100000;
                rrtc_settings.dynamic_domain = false;
                rrtc_settings.projection_method = vamp::planning::ProjMethod::InnerLM;
                rrtc_settings.descend_rate = 0.75;


                std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
                    -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
                    -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
                    -0.001, -0.001, -0.001, -0.05, -0.05, -0.05,
                    -0.001, -0.001, -0.001, -0.05, -0.05, -0.05
                };

                std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
                    10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                    10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                    0.001, 0.001, 0.001, 0.05, 0.05, 0.05,
                    0.001, 0.001, 0.001, 0.05, 0.05, 0.05
                };


                std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{
                    {1, 0,0,0,   0, 0, 0},
                    {0.98436, 0.00282, -0.05503, 0.16732 ,  0.6824999999999999, -0.127, 0.765},
                    {1.0, 0.0, 0.0, 0.0, 0.12, 0.11, -0.0},
                    {1.0, 0.0, 0.0, 0.0, 0.12, -0.11, -0.0}
                }};
                std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{
                    {1, 0, 0, 0, 0, 0, 0},
                    {1, 0, 0, 0, 0, 0, 0},
                    {1, 0, 0, 0, 0, 0, 0},
                    {1, 0, 0, 0, 0, 0, 0}
                }};

                vamp::planning::TaskSpaceConstraint<Robot, rake> right_hand_constraint(
                    eef_transforms_ref_frame_w_world,
                    eef_transforms,
                    tsr_lower_bound,
                    tsr_upper_bound
                );

                std::array<float, 8> polygon_points = {
                    0.08, -0.045, 0.08, 0.045, 0.065, 0.045, 0.065, -0.045
                };
                vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(
                    polygon_points
                );


                vamp::planning::ComposableConstraints<Robot, rake, decltype(right_hand_constraint), decltype(com_constraint)> task_constraint(
                    right_hand_constraint,
                    com_constraint
                );


                auto result =
                    vamp::planning::CRRTC<Robot, rake, Robot::resolution, vamp::planning::TaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>>::solve(start_state, typename Robot::Configuration(out_config), environment, rrtc_settings, task_constraint, rng);

                if(result.path.size() > 0){
                    std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
                    vamp::planning::SimplifySettings simplify_settings;
                    auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, vamp::planning::TaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>>(
                        result.path, environment, task_constraint, simplify_settings, rng);
                    std::cout << "Simplify took " << result.nanoseconds / 1e6 << " ms" << std::endl;

                    // Output configurations of simplified path
                    std::cout << std::fixed << std::setprecision(3);
                    for (const auto &config : result.path)
                    {
                        const auto &array = config.to_array();
                        typename Robot::ConfigurationArray soln;
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
                    break;

                }


            }
            else{
                bool first = true;
                for (auto i = 0U; i < Robot::dimension; ++i)
                {
                    if (!first) outfile << ",";
                    outfile << config_block[{i, success}];
                    first = false;
                }
                outfile << "\n";
                first = true;
                for (auto i = 0U; i < Robot::dimension; ++i)
                {
                    if (!first) outfile << ",";
                    outfile << projected_block[{i, success}];
                    first = false;
                }
                outfile << "\n";

                std::cout << "Projection failed!" << std::endl;
            }

        }
    }
    return out_config;
}

template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
auto plan_to_pose(
    vamp::planning::ComposableConstraints<Robot, rake, Constraints...> &constraint,
    const typename Robot::Configuration &start_state,
    vamp::planning::TaskSpaceConstraint<Robot, rake> &goal,
    const EnvironmentVector &environment,
    typename RNG::Ptr rng,
    std::ofstream &outfile = std::ofstream("/src/trajectory.txt")
)
{
    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.75;
    rrtc_settings.max_iterations = 10000;
    rrtc_settings.dynamic_domain = true;
    rrtc_settings.projection_method = vamp::planning::ProjMethod::InnerLM;
    rrtc_settings.descend_rate = 0.75;

    std::cout << "Planning to goal " << std::endl;
    auto result =
        vamp::planning::TRCRRTC<Robot, rake, Robot::resolution, vamp::planning::TaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>>::solve(start_state, goal, environment, rrtc_settings, constraint, rng);


    if(result.path.size() > 0){
        std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
        vamp::planning::SimplifySettings simplify_settings;
        auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, vamp::planning::TaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>>(
            result.path, environment, constraint, simplify_settings, rng);
        std::cout << "Simplify took " << simplify_result.nanoseconds / 1e6 << " ms" << std::endl;

        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            typename Robot::ConfigurationArray soln;
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

    return result.path.back();
}


auto main(int, char **) -> int
{
    std::cout << std::fixed << std::setprecision(5);

    vamp::planning::RRTCSettings rrtc_settings;

    EnvironmentInput environment;


    // std::ifstream infile("/src/myfork/vamp/resources/environments/cuboids/shelf_drake.txt");
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
    //     // std::cout << x << ", " << y << ", " << z << ", " << dx << ", " << dy << ", " << dz << std::endl;
    //     // environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({x, y, z}, {0.0, 0.0, 0.0}, {dx/2, dy/2, dz/2}));
    // }
    // infile.close();

    // auto geoms = vamp::utils::parser::parseMJCF("/src/myfork/vamp/resources/environments/mjcf/bookshelf_simple.xml");
    // for (const auto& g : geoms) {
    //     if (g.type == vamp::utils::parser::GeomType::BOX){
    //         environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({g.world_pose.pos.x, g.world_pose.pos.y, g.world_pose.pos.z}, {0.0, 0.0, 0.0}, {g.size.x + 0.05, g.size.y, g.size.z}));
    //     }

    // }
    std::ofstream outfile("/src/trajectory.txt");

    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
        -10.001, -10.001, -10.001, -10.05, -10.05, -10.05,
        -0.001, -0.001, -0.001, -10.5, -10.5, -10.5,
        -0.001, -0.001, -0.001, -10.5, -10.5, -10.5
    };

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
        10.001, 10.001, 10.001, 10.05, 10.05, 10.05,
        0.001, 0.001, 0.001, 10.5, 10.5, 10.5,
        0.001, 0.001, 0.001, 10.5, 10.5, 10.5
    };


    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{
        {1, 0,0,0,   0, 0, 0},
        {1, 0,0,0,   0, 0, 0},
        {0.60346, 0.36840, 0.36843, 0.60364 , -0.04302, 0.10080, -0.96013},
        {0.60378, -0.36827, 0.36857, -0.60331 , -0.04288, -0.09895, -0.96033}
    }};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{
        {1, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0, 0}
    }};

    vamp::planning::TaskSpaceConstraint<Robot, rake> feet_constraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        tsr_lower_bound,
        tsr_upper_bound
    );

    std::array<float, 8> polygon_points = {
        0.03, -0.08, 0.03, 0.08, -0.03, 0.08, -0.03, -0.08
    };
    vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(
        polygon_points
    );


    vamp::planning::ComposableConstraints<Robot, rake, decltype(feet_constraint), decltype(com_constraint)> task_constraint(
        feet_constraint,
        com_constraint
    );



    // eef constraint
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_book_1 = {{
        {0.74111, -0.67082, 0.02585, -0.00909 , 0.55250, 0.16273, 0.29131},
        {0.74176, 0.67012, 0.02553, 0.00870 ,  0.55570, -0.15810,  0.29023},
        {0.60346, 0.36840, 0.36843, 0.60364 , -0.04302, 0.10080, -0.96013},
        {0.60378, -0.36827, 0.36857, -0.60331 , -0.04288, -0.09895, -0.96033}
    }};

    // std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_book_2 = {{
    //     {1, 0,0,0,   0, 0, 0},
    //     {0.92436, 0.00282, -0.05503, 0.16732 ,  0.54, -0.2, 1.05},
    //     {1.0, 0.0, 0.0, 0.0, 0.12, 0.11, -0.0},
    //     {1.0, 0.0, 0.0, 0.0, 0.12, -0.11, -0.0}
    // }};

    std::array<float, 6 * Robot::n_eef> goal_pose_tsr_lower_bound = {
        -0.1, -0.1, -0.1, -10.05, -10.05, -10.05,
        -0.1, -0.1, -0.1, -10.05, -10.05, -10.05,
        -0.001, -0.001, -0.001, -10.5, -10.5, -10.5,
        -0.001, -0.001, -0.001, -10.5, -10.5, -10.5
    };

    std::array<float, 6 * Robot::n_eef> goal_pose_tsr_upper_bound = {
        0.1, 0.1, 0.1, 10.05, 10.05, 10.05,
        0.1, 0.1, 0.1, 10.05, 10.05, 10.05,
        0.001, 0.001, 0.001, 10.5, 10.5, 10.5,
        0.001, 0.001, 0.001, 10.5, 10.5, 10.5
    };



    vamp::planning::TaskSpaceConstraint<Robot, rake> right_hand_constraint_book_1(
        eef_transforms_ref_frame_w_world,
        eef_transforms_book_1,
        goal_pose_tsr_lower_bound,
        goal_pose_tsr_upper_bound
    );

    // vamp::planning::TaskSpaceConstraint<Robot, rake> right_hand_constraint_book_2(
    //     eef_transforms_ref_frame_w_world,
    //     eef_transforms_book_2,
    //     goal_pose_tsr_lower_bound,
    //     goal_pose_tsr_upper_bound
    // );


    auto start_time = std::chrono::steady_clock::now();

    auto sampled_valid_config = plan_to_pose<Robot, rake, Robot::resolution>(task_constraint, typename Robot::Configuration(start), right_hand_constraint_book_1, env_v, rng, outfile);


    // auto sampled_valid_config2 = plan_to_pose<Robot, rake, Robot::resolution>(task_constraint, sampled_valid_config, right_hand_constraint_book_2, env_v, rng, outfile);
    std::cout << "Total planning time for both motions : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() << " ms" << std::endl;


    // std::array<vamp::FloatT, Robot::dimension * rake> config_block_arr;
    // for(auto i = 0U; i < rake; i++) {
    //     auto sampled_config = rng->next();
    //     std::cout << "Sampled config : " << sampled_config << std::endl;
    //     typename Robot::ConfigurationBuffer temp_array;
    //     sampled_config.to_array(temp_array.data());
    //     for (auto j = 0U; j < Robot::dimension; j++)
    //         config_block_arr[i + j * rake] = temp_array[j];
    // }
    // typename Robot::template ConfigurationBlock<rake> config_block = typename Robot::template ConfigurationBlock<rake>(config_block_arr);
    // std::cout << "Sampled config block : " << config_block << std::endl;
    // typename Robot::ConfigurationBlock<rake> projected_block;
    // bool success = task_constraint.projectConfiguration(config_block, projected_block, vamp::planning::ProjMethod::InnerLM, 35.0, 1.0, 100, false);
    // std::cout << "Projected config block : " << projected_block << std::endl;
    // std::cout << success << std::endl;

    // std::ofstream outfile("/src/trajectory.txt");
    // for(auto r = 0U; r < rake; r++){
    //     bool first = true;
    //     for (auto i = 0U; i < Robot::dimension; ++i)
    //     {
    //         if (!first) outfile << ",";
    //         outfile << projected_block[{i, r}];
    //         first = false;
    //     }

    //     outfile << "\n";
    // }
    // outfile.close();

    std::array<float, Robot::dimension> pose_1;
    sampled_valid_config.to_array(pose_1.data());

    // std::array<float, Robot::dimension> pose_2;
    // sampled_valid_config2.to_array(pose_2.data());

    auto start_fk0 = Robot::eefk(pose_1)[0];
         // auto fka = Robot::eefk(goal);
    std::cout << std::endl << start_fk0.matrix() << std::endl;
    Eigen::Quaternionf q1(start_fk0.linear());
    std::cout << q1.w() << ", " << q1.x() << ", " << q1.y() << ", " << q1.z() << " , " << start_fk0.translation().transpose() << std::endl;

    auto start_fk1 = Robot::eefk(pose_1)[1];
    std::cout << std::endl << start_fk1.matrix() << std::endl;
    Eigen::Quaternionf q3(start_fk1.linear());
    std::cout << q3.w() << ", " << q3.x() << ", " << q3.y() << ", " << q3.z() << " , " << start_fk1.translation().transpose() << std::endl;

    // auto goal_fk0 = Robot::eefk(pose_2)[0];
    //      // auto fka = Robot::eefk(goal);
    // std::cout << std::endl << goal_fk0.matrix() << std::endl;
    // Eigen::Quaternionf q_goal1(goal_fk0.linear());
    // std::cout << q_goal1.w() << ", " << q_goal1.x() << ", " << q_goal1.y() << ", " << q_goal1.z() << " , " << goal_fk0.translation().transpose() << std::endl;

    // auto goal_fk1 = Robot::eefk(pose_2)[1];
    // std::cout << std::endl << goal_fk1.matrix() << std::endl;
    // Eigen::Quaternionf q_goal2(goal_fk1.linear());
    // std::cout << q_goal2.w() << ", " << q_goal2.x() << ", " << q_goal2.y() << ", " << q_goal2.z() << " , " << goal_fk1.translation().transpose() << std::endl;

    outfile.close();



}
