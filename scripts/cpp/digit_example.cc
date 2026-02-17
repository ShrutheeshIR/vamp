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

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {
    0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31794,-0.01125,-0.28643,0.11635,-0.00983,0.08084,-0.26989,-0.01151,0.12815,-0.36544,0.00508,-0.29158,-0.32097,0.00882,0.28598,-0.11670,0.01291,0.15245,0.28130,0.06021,-0.10471
};

static constexpr Robot::ConfigurationArray goal = {
    0.00306,0.01556,-0.48345,-0.01063,0.00173,-0.00292,0.37003,0.00751,-0.22884,-0.85230,-0.02549,0.90911,-0.43355,0.01078,-0.17309,-0.36296,0.06140,1.08973,-0.33766,0.01570,0.22586,0.84365,0.02174,-0.90312,0.42744,0.06025,-0.29859,0.37721,-0.15511,-1.16677
};

// static constexpr Robot::ConfigurationArray goal = {
//     0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31631,-0.01347,-0.28807,0.11635,-0.00983,0.02358,-0.00579,0.01520,0.09714,-0.36544,0.00508,-0.29158,-0.31758,0.01345,0.28939,-0.11670,0.01291,0.00435,0.00820,-0.01199,-0.09980
// };

// static constexpr Robot::ConfigurationArray start = {
//     0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31631,-0.01347,-0.28807,0.11635,-0.00983,-0.10504,0.88852,-0.00624,0.37778,-0.36544,0.00508,-0.29158,-0.31758,0.01345,0.28939,-0.11670,0.01291,0.10456,-0.89396,0.00550,-0.36663
// };

// static constexpr Robot::ConfigurationArray start = {-0.148774,1.59886,1.36434,-2.75007,0.544898,2.51704,-1.4485,2.07773,1.0024,-0.823622,-1.69743,-0.625681,2.59153,0.7243};
// static constexpr Robot::ConfigurationArray goal = {-1.62706,-0.100903,2.59477,-2.09287,1.2912,1.8256,-0.7243,1.75215,1.5907,-2.0261,-1.8123,-0.119768,2.50718,-0.7243};


struct Attempt {
    float range;
    bool dynamic_domain;
    vamp::planning::ProjMethod proj_method;
    float descend_rate;
    int num_projection_iterations;
    bool insert_all_to_tree;
    bool success;
    std::size_t planning_time;
    std::size_t planning_iterations;
    std::size_t path_length;

    bool operator<(const Attempt& other) const {
        return planning_time < other.planning_time;
    }
};


auto main(int, char **) -> int
{

    vamp::planning::RRTCSettings rrtc_settings;

    float ranges[] = {0.5, 0.75, 1.0, 1.5, 2.0};
    // float ranges[] = {1.0};
    // float ranges[] = {0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0};
    // bool dd[] = {false};
    bool dd[] = {false, true};
    // vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM, vamp::planning::ProjMethod::OuterLM, vamp::planning::ProjMethod::GradDesc};
    vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM, vamp::planning::ProjMethod::OuterLM};
    // vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM};

    // float descend_rates[] = {0.25, 0.5, 0.75, 1.0};
    float descend_rates[] = {0.75, 1.0};
    int num_projection_iterations[] = {5, 10, 25, 50, 100};
    bool insert_all_to_tree[] = {false, true};

    // float descend_rates[] = {1.0};
    std::vector<Attempt> succ_attempts;
    for(const auto range: ranges){
        for(const auto dyndom: dd){
            for(const auto &pm: projection_method){
                for(const auto descent_rate: descend_rates){
                    for(const auto num_projection_iterations: num_projection_iterations){
                        for(const auto insert_all_to_tree: insert_all_to_tree){

                // if(pm < 2) continue;

    // Build sphere cage environment
    EnvironmentInput environment;
    // std::ofstream outfile_sph("spheres.txt");
    // for (const auto &sphere : problem)
    // {
    //     outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }
    // outfile_sph.close();
    std::ifstream infile("/src/myfork/vamp/resources/environments/cuboids/humanoid_digit_shelf.txt");
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
        // std::cout << x << ", " << y << ", " << z << ", " << dx << ", " << dy << ", " << dz << std::endl;
        // environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({x, y, z}, {0.0, 0.0, 0.0}, {dx/2, dy/2, dz/2}));
    }
    infile.close();



    environment.sort();


    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


    std::array<float, 8> polygon_points = {
        0.03, -0.075, 0.03, 0.075, -0.04, 0.075, -0.04, -0.075
    };

    std::array<float, 6> lower_bound = {
        -0.001, -0.001, -0.001, -10.1, -10.1, -10.1
    };
    std::array<float, 6> upper_bound = {
        0.001, 0.001, 0.001, 10.1, 10.1, 10.1
    };

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01
    };

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05
    };


    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    // Eigen::Matrix<float, 4, 4> T;
    // T << 0.9999990, -0.0009995,  0.0010005, 0.0, 0.0010005,  0.9999990, -0.0009995, -0.303, -0.0009995,  0.0010005,  0.9999990, 0.0, 0.0, 0.0, 0.0, 1.0;
    // const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);

    // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms;

    // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990, -0.0009995, 0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0;
    // eef_transforms[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990, -0.0009995, 0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0;
    // eef_transforms[1] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990, -0.0009995, 0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0;
    // eef_transforms[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990, -0.0009995, -0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0;
    // eef_transforms[3] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

    // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms_ref_frame_w_world;
    // T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // eef_transforms_ref_frame_w_world[1] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // eef_transforms_ref_frame_w_world[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // eef_transforms_ref_frame_w_world[3] = Eigen::Transform<float, 3, Eigen::Isometry>(T);


    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{{1, 0,0,0,   0, 0, 0}, {1, 0,0,0,   0.0, 0.0, 0.0}, {0.603, 0.36, 0.36, 0.603, -0.04302,  0.10080, -0.96013}, {0.603, -0.36, 0.36, -0.603 , -0.04288, -0.09895, -0.96033}}};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{{1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}}};

    vamp::planning::FeetTaskSpaceConstraint<Robot, rake> feet_tsr_constraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        tsr_lower_bound,
        tsr_upper_bound
    );

    // vamp::planning::TaskSpaceConstraint<Robot, rake> feet_tsr_constraint(
    //     eef_transforms_ref_frame_w_world,
    //     eef_transforms,
    //     std::make_pair(tsr_lower_bound, tsr_upper_bound)
    // );

    std::array<float, 7> transform = {0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.36356};
    vamp::planning::BimanualTaskSpaceConstraint<Robot, rake> bimanual_task_constraint(transform, lower_bound, upper_bound);

    // vamp::planning::BimanualTaskSpaceConstraint<Robot, rake> bimanual_constraint(
    //     target_pose,
    //     std::make_pair(lower_bound, upper_bound)
    // );
    vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(
        polygon_points
    );

    // vamp::planning::SelfCollisionConstraint<Robot, rake> self_collision_constraint;
    vamp::planning::ClosedLinkConstraint<Robot, rake> closed_link_constraint;

    vamp::planning::ComposableConstraints<Robot, rake, decltype(feet_tsr_constraint), decltype(com_constraint), decltype(bimanual_task_constraint), decltype(closed_link_constraint)> task_constraint(
    // vamp::planning::ComposableConstraints<Robot, rake, decltype(feet_tsr_constraint), decltype(com_constraint), decltype(closed_link_constraint)> task_constraint(
        feet_tsr_constraint,
        com_constraint,
        bimanual_task_constraint,
        closed_link_constraint
    );    // vamp::planning::ComposableConstraints<Robot, rake, decltype(bimanual_constraint), decltype(com_constraint)> task_constraint(
    //     bimanual_constraint,
    //     com_constraint
    // );

    rrtc_settings.range = range;
    rrtc_settings.max_iterations = 100000;
    rrtc_settings.dynamic_domain = dyndom;
    rrtc_settings.projection_method = pm;
    rrtc_settings.descend_rate = descent_rate;

    rrtc_settings.radius = 10.0;
    rrtc_settings.num_projection_iterations = num_projection_iterations;
    rrtc_settings.insert_all_to_tree = insert_all_to_tree;
    // std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
    std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";
    vamp::planning::invalid_distance_counter_outside = 0;
    vamp::planning::invalid_distance_counter_inside = 0;
    vamp::planning::collision_counter = 0;
    vamp::planning::unable_to_project_counter = 0;




    // std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
    std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";
    auto result =
        vamp::planning::CRRTC<Robot, rake, Robot::resolution, decltype(feet_tsr_constraint), decltype(com_constraint), decltype(bimanual_task_constraint), decltype(closed_link_constraint)>::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);
        // vamp::planning::CRRTC<Robot, rake, Robot::resolution, decltype(feet_tsr_constraint), decltype(com_constraint), decltype(closed_link_constraint)>::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);
    // auto result =
    //     vamp::planning::CRRTC<Robot, rake, Robot::resolution, decltype(bimanual_constraint), decltype(com_constraint)>::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

    if(result.path.size() > 0)
    {
        Attempt a{
            range,
            dyndom,
            pm,
            descent_rate,
            num_projection_iterations,
            insert_all_to_tree,
            true,
            result.nanoseconds,
            result.iterations,
            result.path.size()
        };



        if((succ_attempts.size() == 0) || (succ_attempts.size() > 0 && a < succ_attempts[0])){

        vamp::planning::SimplifySettings simplify_settings;
        auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, decltype(feet_tsr_constraint), decltype(com_constraint), decltype(bimanual_task_constraint), decltype(closed_link_constraint)>(
        // auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, decltype(feet_tsr_constraint), decltype(com_constraint), decltype(closed_link_constraint)>(
            result.path, env_v, task_constraint, simplify_settings, rng);
        std::cout << "Simplify took " << result.nanoseconds / 1e6 << " ms" << std::endl;

        std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
        simplify_result.path.interpolate_to_resolution(Robot::resolution);
        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        std::ofstream outfile("/src/trajectory.txt");
        for (const auto &config : simplify_result.path)
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
            std::cout << std::endl;
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
    }
    std::cout << "------Final Result --------" << std::endl;
    std::sort(succ_attempts.rbegin(), succ_attempts.rend());
    for (const auto &a : succ_attempts) {
        std::cout << a.range << ", " << a.dynamic_domain << ", " << a.proj_method << ", " << a.descend_rate << ", " << a.num_projection_iterations << ", " << a.insert_all_to_tree << ", " << a.planning_time/1e6 << ", " << a.planning_iterations << ", " << a.path_length << std::endl;

    }
    std::cout << "---------------------------" << std::endl;


    return 0;
}
