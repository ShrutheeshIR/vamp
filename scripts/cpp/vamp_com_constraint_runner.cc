#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>
#include <vamp/planning/constraints/CoM_task_space_constraint.hh>
#include <vamp/planning/constraints/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/constraints/self_collision_constraint.hh>
// #include <vamp/planning/simplify.hh>
#include <vamp/robots/g1_unitree.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::G1Unitree;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
// using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution, 4>;
using AttachmentInput = vamp::collision::Attachment<float>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {
    0.0,  0.0, -0.2, 0.0,  0.0,      0.0, -0.9,  0.0,    0.0,  1.75, -0.87267, 0.0,
    -0.9, 0.0, 0.0,  1.75, -0.87267, 0.0, 1.595, -0.277, 0.52, 0.0,  0.0,      0.0,
    0.0,  0.0, 0.0,  0.0,  0.0,      0.0, 0.0,   0.0,    0.0,  0.0,  0.0};
static constexpr Robot::ConfigurationArray goal = {
    0.035, 0.0, 0.031, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,   0.0, 0.0,   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// static constexpr Robot::ConfigurationArray start =
// {-0.148774,1.59886,1.36434,-2.75007,0.544898,2.51704,-1.4485,2.07773,1.0024,-0.823622,-1.69743,-0.625681,2.59153,0.7243};
// static constexpr Robot::ConfigurationArray goal =
// {-1.62706,-0.100903,2.59477,-2.09287,1.2912,1.8256,-0.7243,1.75215,1.5907,-2.0261,-1.8123,-0.119768,2.50718,-0.7243};

auto main(int, char **) -> int
{
    vamp::planning::RRTCSettings rrtc_settings;

    EnvironmentInput environment;
    // std::ofstream outfile_sph("spheres.txt");
    // for (const auto &sphere : problem)
    // {
    //     outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }
    // outfile_sph.close();
    std::ifstream infile("/src/myfork/vamp/resources/environments/cuboids/shelf_drake.txt");
    if (!infile.is_open())
    {
        std::cerr << "Failed to open file!" << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        char delim;
        float x, y, z, dx, dy, dz;

        if (!(iss >> x >> delim >> y >> delim >> z >> delim >> dx >> delim >> dy >> delim >> dz))
        {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        }
        // std::cout << x << ", " << y << ", " << z << ", " << dx << ", " << dy << ", " << dz << std::endl;
        environment.cuboids.emplace_back(
            vamp::collision::factory::cuboid::array({x, y, z}, {0.0, 0.0, 0.0}, {dx / 2, dy / 2, dz / 2}));
    }
    infile.close();

    environment.sort();

    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    std::array<float, 8> polygon_points = {
        // 1.0, -0.05, 1.0, 0.05, 0.0, 0.05, 0.0, -0.05
        0.20,
        -0.25,
        0.20,
        0.25,
        0.00,
        0.25,
        0.00,
        -0.25
        // 10.20, -10.25, 10.20, 10.25, -10.05, 10.25, -10.05, -10.25
        // 0.0, 0.2, 1.0, 0.2, 1.0, 1.0, 0.0, 1.0
    };

    std::array<float, 6> lower_bound = {-0.001, -0.001, -0.001, -0.001, -0.001, -0.001};
    std::array<float, 6> upper_bound = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
        -0.02, -0.01, -0.01, -0.05, -0.05, -0.05, -0.02, -0.01, -0.01, -0.05, -0.05, -0.05};

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                                                           10.0, 10.0, 10.0, 10.0, 0.02, 0.01, 0.01, 0.05,
                                                           0.05, 0.05, 0.02, 0.01, 0.01, 0.05, 0.05, 0.05};

    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    Eigen::Matrix<float, 4, 4> T;
    T << 0.9999990, -0.0009995, 0.0010005, 0.0, 0.0010005, 0.9999990, -0.0009995, -0.303, -0.0009995,
        0.0010005, 0.9999990, 0.0, 0.0, 0.0, 0.0, 1.0;
    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);

    std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms;

    T << 0.9999990, -0.0009995, 0.0010005, 0.13, 0.0010005, 0.9999990, -0.0009995, 0.11, -0.0009995,
        0.0010005, 0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0;
    eef_transforms[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    T << 0.9999990, -0.0009995, 0.0010005, 0.13, 0.0010005, 0.9999990, -0.0009995, 0.11, -0.0009995,
        0.0010005, 0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0;
    eef_transforms[1] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    T << 0.9999990, -0.0009995, 0.0010005, 0.13, 0.0010005, 0.9999990, -0.0009995, 0.11, -0.0009995,
        0.0010005, 0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0;
    eef_transforms[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    T << 0.9999990, -0.0009995, 0.0010005, 0.13, 0.0010005, 0.9999990, -0.0009995, -0.11, -0.0009995,
        0.0010005, 0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0;
    eef_transforms[3] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

    std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms_ref_frame_w_world;
    T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    eef_transforms_ref_frame_w_world[1] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    eef_transforms_ref_frame_w_world[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    eef_transforms_ref_frame_w_world[3] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

    vamp::planning::constraint::TaskSpaceConstraint<Robot, rake> feet_tsr_constraint(
        eef_transforms_ref_frame_w_world, eef_transforms, std::make_pair(tsr_lower_bound, tsr_upper_bound));

    vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake> bimanual_constraint(
        target_pose, std::make_pair(lower_bound, upper_bound));
    vamp::planning::constraint::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(polygon_points);

    vamp::planning::constraint::SelfCollisionConstraint<Robot, rake> self_collision_constraint;

    vamp::planning::constraint::ComposableConstraints<
        Robot,
        rake,
        decltype(feet_tsr_constraint),
        decltype(com_constraint),
        decltype(bimanual_constraint)>
        task_constraint(feet_tsr_constraint, com_constraint, bimanual_constraint);
    // vamp::planning::constraint::ComposableConstraints<Robot, rake, decltype(bimanual_constraint),
    // decltype(com_constraint)> task_constraint(
    //     bimanual_constraint,
    //     com_constraint
    // );

    rrtc_settings.range = 1.0;
    rrtc_settings.max_iterations = 100000;
    rrtc_settings.dynamic_domain = true;
    rrtc_settings.projection_method = vamp::planning::constraint::ProjMethod::InnerLM;
    rrtc_settings.descend_rate = 1.0;
    auto result = vamp::planning::CRRTC<
        Robot,
        rake,
        Robot::resolution,
        decltype(feet_tsr_constraint),
        decltype(com_constraint),
        decltype(bimanual_constraint)>::
        solve(
            Robot::Configuration(start),
            Robot::Configuration(goal),
            env_v,
            rrtc_settings,
            task_constraint,
            rng);

    if (result.path.size() > 0)
    {
        std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        std::ofstream outfile("/src/trajectory.txt");
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            Robot::ConfigurationArray soln;
            bool first = true;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                soln[i] = array[i];

                if (!first)
                {
                    outfile << ",";
                }
                outfile << array[i];
                first = false;
            }

            outfile << "\n";
        }
        outfile.close();
    }
    return 0;
}
