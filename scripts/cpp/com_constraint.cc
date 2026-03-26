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
// static constexpr Robot::ConfigurationArray start = {0.697778, -0.5024, -1.256, -1.94109, -2.12554,
// -2.36424, -2.44589, -0.204884, -2.35822, 0.113728, -0.793422, -0.234981, -2.26647, -2.81113, -2.53907,
// 0.0183405, -0.824602, -0.245389, -2.46313, -0.490082, -0.492354, -2.94348, -1.4976, -2.50464, -0.98188,
// -1.8918, -1.55106, -1.55222, -2.98216, -2.18801, -2.53406, -0.999044, -1.91263, -1.56892, -1.56953};

static constexpr Robot::ConfigurationArray start = {
    -0.01691, -0.00008, 0.52435, -0.00002, 0.00087,  0.00058,  -0.89966, -0.00864, 0.01527,
    1.75038,  -0.87267, 0.01757, -0.89936, 0.00875,  -0.01522, 1.75040,  -0.87267, -0.01753,
    1.59564,  0.14700,  0.37000, 0.00002,  0.00014,  0.00009,  -0.00003, -0.00006, 0.00000,
    -0.00001, -0.00001, 0.00017, 0.00007,  -0.00000, 0.00003,  0.00000,  0.00002};
static constexpr Robot::ConfigurationArray goal = {
    -0.0804749, -0.222722, 0.329562,    0.0752235, -0.234729, -0.187489, -0.540513,  0.159474,  0.036218,
    1.20727,    -0.508012, -0.00816858, -0.55574,  -0.25357,  0.255191,  1.33009,    -0.561027, -0.0319013,
    0.578079,   0.0344014, 0.0243313,   -0.355914, -0.118091, -0.25972,  -0.0948007, -0.253484, 0.0772362,
    0.174869,   0.171759,  0.0892086,   0.124599,  0.0630061, 0.118393,  0.0219054,  0.0661802};

// static constexpr Robot::ConfigurationArray goal =
// {-0.045,0.0,-0.03,0.0,-0.264,0.0,0.0,0.0,0.0,0.456,0.0,0.0,0.0,0.0,0.0,0.486,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

struct Attempt
{
    float range;
    bool dynamic_domain;
    vamp::planning::constraint::ProjMethod proj_method;
    float descend_rate;
    bool success;
    std::size_t planning_time;
    std::size_t planning_iterations;
    std::size_t path_length;

    bool operator<(const Attempt &other) const
    {
        return planning_time < other.planning_time;
    }
};

auto main(int, char **) -> int
{
    std::cout << std::fixed << std::setprecision(5);

    vamp::planning::RRTCSettings rrtc_settings;

    EnvironmentInput environment;

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
        // environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({x, y, z}, {0.0, 0.0,
        // 0.0}, {dx/2, dy/2, dz/2}));
    }
    infile.close();

    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    auto isometries = Robot::eefk(start);
    std::cout << "Start pose eef poses " << std::endl;
    std::cout << "Left hand " << isometries[0].matrix() << std::endl;
    std::cout << "Right hand " << isometries[1].matrix() << std::endl;
    std::cout << "Left foot " << isometries[2].matrix() << std::endl;
    std::cout << "Right foot " << isometries[3].matrix() << std::endl;

    std::cout << "Relative pose between left and right hand "
              << (isometries[0].inverse() * isometries[1]).matrix() << std::endl;

    isometries = Robot::eefk(goal);
    std::cout << "Goal pose eef poses " << std::endl;
    std::cout << "Left hand " << isometries[0].matrix() << std::endl;
    std::cout << "Right hand " << isometries[1].matrix() << std::endl;
    std::cout << "Left foot " << isometries[2].matrix() << std::endl;
    std::cout << "Right foot " << isometries[3].matrix() << std::endl;

    std::cout << "Relative pose between left and right hand "
              << (isometries[0].inverse() * isometries[1]).matrix() << std::endl;

    // std::cout << isometries[0].matrix() << std::endl<< isometries[1].matrix() << std::endl<<
    // isometries[2].matrix() << std::endl<< isometries[3].matrix() << std::endl;

    std::cout << "Hands : " << (isometries[0].inverse() * isometries[1]).matrix() << std::endl;
    std::array<float, 8> polygon_points = {0.08, -0.045, 0.08, 0.045, 0.06, 0.045, 0.06, -0.045};

    std::array<float, 6> lower_bound = {-0.001, -0.001, -0.001, -0.5, -0.001, -0.001};
    std::array<float, 6> upper_bound = {0.001, 0.001, 0.001, 0.5, 0.001, 0.001};

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
        -0.01, -0.01, -0.01, -0.05, -0.05, -0.05, -0.01, -0.01, -0.01, -0.05, -0.05, -0.05};

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                                                           10.0, 10.0, 10.0, 10.0, 0.01, 0.01, 0.01, 0.05,
                                                           0.05, 0.05, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05};

    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    // Eigen::Matrix<float, 4, 4> T;
    // T << 0.9999990, -0.0009995,  0.0010005, 0.0, 0.0010005,  0.9999990, -0.0009995, -0.303, -0.0009995,
    // 0.0010005,  0.9999990, 0.0, 0.0, 0.0, 0.0, 1.0; const Eigen::Transform<float, 3, Eigen::Isometry>
    // target_pose(T);

    // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms;

    // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990, -0.0009995, 0.11, -0.0009995,
    // 0.0010005,  0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0; eef_transforms[0] = Eigen::Transform<float, 3,
    // Eigen::Isometry>(T); T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990, -0.0009995,
    // 0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0; eef_transforms[1] =
    // Eigen::Transform<float, 3, Eigen::Isometry>(T); T << 0.9999990, -0.0009995,  0.0010005, 0.13,
    // 0.0010005,  0.9999990, -0.0009995, 0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0,
    // 0.0, 1.0; eef_transforms[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T); T << 0.9999990,
    // -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990, -0.0009995, -0.11, -0.0009995,  0.0010005,
    // 0.9999990, -0.725, 0.0, 0.0, 0.0, 1.0; eef_transforms[3] = Eigen::Transform<float, 3,
    // Eigen::Isometry>(T);

    // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms_ref_frame_w_world;
    // T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // eef_transforms_ref_frame_w_world[1] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // eef_transforms_ref_frame_w_world[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // eef_transforms_ref_frame_w_world[3] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {
        {{1, 0, 0, 0, 0, 0, 0},
         {1, 0, 0, 0, 0.0, 0.0, 0.0},
         {1.0, 0.0, 0.0, 0.0, 0.12, 0.11, -0.0},
         {1.0, 0.0, 0.0, 0.0, 0.12, -0.11, -0.0}}};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {
        {{1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}}};

    vamp::planning::constraint::FeetTaskSpaceConstraint<Robot, rake> feet_tsr_constraint(
        eef_transforms_ref_frame_w_world, eef_transforms, tsr_lower_bound, tsr_upper_bound);

    // vamp::planning::constraint::TaskSpaceConstraint<Robot, rake> feet_tsr_constraint(
    //     eef_transforms_ref_frame_w_world,
    //     eef_transforms,
    //     std::make_pair(tsr_lower_bound, tsr_upper_bound)
    // );

    std::array<float, 7> transform = {1., 0.0005, 0.0005, 0.0005, 0.0, -0.303, 0.0};
    vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake> bimanual_task_constraint(
        transform, lower_bound, upper_bound);

    // vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake> bimanual_constraint(
    //     target_pose,
    //     std::make_pair(lower_bound, upper_bound)
    // );
    vamp::planning::constraint::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(polygon_points);

    // vamp::planning::constraint::SelfCollisionConstraint<Robot, rake> self_collision_constraint;

    vamp::planning::constraint::ComposableConstraints<
        Robot,
        rake,
        decltype(feet_tsr_constraint),
        decltype(com_constraint),
        decltype(bimanual_task_constraint)>
        task_constraint(feet_tsr_constraint, com_constraint, bimanual_task_constraint);
    // vamp::planning::constraint::TaskSpaceConstraint<Robot, rake> task_constraint(
    //     eef_transforms_ref_frame_w_world,
    //     eef_transforms,
    //     std::make_pair(tsr_lower_bound, tsr_upper_bound)
    // );

    // vamp::planning::SETaskSpaceConstraint<Robot, rake> task_constraint2(
    //     eef_transforms_ref_frame_w_world[2],
    //     eef_transforms[2],
    //     std::make_pair(slower_bound, supper_bound)
    // );

    // vamp::planning::BimanualCoMTaskSpaceConstraint<Robot, rake, 4> task_constraint(
    //     polygon_points,
    //     target_pose,
    //     std::make_pair(lower_bound, upper_bound)
    // );

    // vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake> task_constraint(
    //     target_pose,
    //     std::make_pair(lower_bound, upper_bound)
    // );

    // // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    // Eigen::Matrix<float, 4, 4> T;
    // T << -0.719427, 0.694568, 6.59173e-05, -2.2769e-05, 0.694568, 0.719427, -2.26738e-05, -0.000370264,
    // -6.31774e-05, 2.94462e-05, -1, 0.171814, 0, 0, 0, 1; const Eigen::Transform<float, 3, Eigen::Isometry>
    // target_pose(T); vamp::planning::BimanualCoMTaskSpaceConstraint<Robot, rake, 4>
    // task_constraint(polygon_points, target_pose, std::make_pair(lower_bound, upper_bound));

    // // vamp::planning::constraint::CoMTaskSpaceConstraint<Robot, rake, 4> task_constraint(polygon_points);
    // auto startcc = rng->next();
    bool success;
    Robot::ConfigurationArray test = {-0.05756, -0.10361, -0.30607, -0.22192, -0.22951, -0.23933, -1.01621,
                                      -0.03210, -0.22167, 1.59964,  -0.86941, -0.02150, -1.02993, -0.24976,
                                      -0.22910, 1.59571,  -0.87069, -0.02193, 1.23762,  -0.29675, 0.43134,
                                      -0.26073, -0.13340, -0.22137, -0.08773, -0.16697, -0.13678, -0.13683,
                                      -0.26232, -0.19178, -0.22258, -0.08843, -0.16783, -0.13752, -0.13754};
    typename Robot::template ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = Robot::Configuration(start).broadcast(i);
    }

    bool goal_valid = Robot::template fkcc<rake>(env_v, block);
    std::cout << "Goal valid: " << goal_valid << std::endl;

    task_constraint.distanceToConstraint(block);
    task_constraint.print_robot_tsr_error(block);

    std::cout << "----Projecting ----" << std::endl;
    typename Robot::template ConfigurationBlock<rake> projected_block;

    // for (auto i = 0U; i < Robot::dimension; ++i)
    //     block[i] = Robot::Configuration(start).broadcast(i);
    // goal_valid = Robot::template fkcc<rake>(env_v, block);
    // std::cout << "Goal valid: " << goal_valid << std::endl;
    // std::cout << "Distance to constraint: " << task_constraint.distanceToConstraint(block) << std::endl;

    // for (auto i = 0U; i < Robot::dimension; ++i)
    //     block[i] = Robot::Configuration(goal).broadcast(i);
    // goal_valid = Robot::template fkcc<rake>(env_v, block);
    // std::cout << "Goal valid: " << goal_valid << std::endl;
    // std::cout << "Distance to constraint: " << task_constraint.distanceToConstraint(block) << std::endl;

    // task_constraint.projectStep(block, projected_block);
    // for(auto i=0U; i < Robot::dimension; i++)
    //     std::cout << projected_block[{i, 0}] << " ";
    // std::cout << std::endl;

    bool first = true;
    success = task_constraint.projectConfiguration(
        block, projected_block, vamp::planning::constraint::ProjMethod::InnerLM, 35.0, 1.0);
    for (auto i = 0U; i < Robot::dimension; i++)
    {
        std::cout << projected_block[{i, 0}] << ",";
    }
    std::cout << success << std::endl;
    std::ofstream outfile("/src/trajectory.txt");
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        if (!first)
        {
            outfile << ",";
        }
        outfile << block[{i, 0}];
        first = false;
    }

    outfile << "\n";

    first = true;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        if (!first)
        {
            outfile << ",";
        }
        outfile << projected_block[{i, 0}];
        first = false;
    }

    outfile << "\n";

    std::cout << " \n\n ------------Goal-----------------\n\n";

    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = Robot::Configuration(goal).broadcast(i);
    }

    first = true;
    success = task_constraint.projectConfiguration(
        block, projected_block, vamp::planning::constraint::ProjMethod::InnerLM, 1.0, 1.0);
    for (auto i = 0U; i < Robot::dimension; i++)
    {
        std::cout << projected_block[{i, 0}] << ",";
    }
    std::cout << success << std::endl;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        if (!first)
        {
            outfile << ",";
        }
        outfile << block[{i, 0}];
        first = false;
    }

    outfile << "\n";

    first = true;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        if (!first)
        {
            outfile << ",";
        }
        outfile << projected_block[{i, 0}];
        first = false;
    }

    outfile << "\n";
    outfile.close();

    task_constraint.distanceToConstraint(block);
    task_constraint.print_robot_tsr_error(block);

    // // std::cout << fks[0].matrix() << std::endl;
    // // std::cout << fks[1].matrix() << std::endl;
    // // for (int i = 0; i < err.rows(); ++i) {
    // //     for (int j = 0; j < err.cols(); ++j) {
    // //     std::cout << err(i, j);
    // //     std::cout << ", ";
    // // }
    // // }
    // // std::cout << std::endl;

    // auto startc = rng->next();
    // auto goalc = rng->next();
    // // bool success;

    // auto startc = Robot::Configuration(start);
    // // auto goalc = Robot::Configuration(goal);

    // std::cout << startc << ", " << goalc << std::endl;
    // auto extension_vector = goalc - startc;
    // std::cout << extension_vector << std::endl;
    // extension_vector = extension_vector / extension_vector.l2_norm();
    // std::cout << extension_vector << std::endl;

    // // std::cout << std::fixed << std::setprecision(3);
    // // std::ofstream outfile("/src/trajectory.txt");

    // for(size_t ext = 0; ext < 11; ext++){
    //     std::cout << "\nExtension attempt " << ext << " " << std::endl;
    //     auto cfg = startc + extension_vector * ext / 10;
    //     for (auto i = 0U; i < Robot::dimension; ++i)
    //         block[i] = cfg.broadcast(i) + 0.0;
    //     for(auto i=0U; i < Robot::dimension; i++)
    //         std::cout << block[{i, 0}] << ", ";
    //     std::cout << std::endl;

    //     typename Robot::template ConfigurationBlock<rake> projected_block;
    //     success = task_constraint.projectConfiguration(block, projected_block,
    //     vamp::planning::constraint::ProjMethod::InnerLM, 10.0, 1.0);

    //     bool valid = Robot::template fkcc<rake>(env_v, projected_block);
    //     bool valid_og = Robot::template fkcc<rake>(env_v, block);

    //     for(auto i=0U; i < Robot::dimension; i++)
    //         std::cout << projected_block[{i, 0}] << ", ";
    //     std::cout << " --> " << success << " -- " << valid << "-- " << valid_og << std::endl;

    //     bool first = true;
    //     for (auto i = 0U; i < Robot::dimension; ++i)
    //     {
    //         if (!first) outfile << ",";
    //         outfile << block[{i, 0}];
    //         first = false;
    //     }

    //     outfile << "\n";

    //     first = true;
    //     for (auto i = 0U; i < Robot::dimension; ++i)
    //     {
    //         if (!first) outfile << ",";
    //         outfile << projected_block[{i, 0}];
    //         first = false;
    //     }

    //     // auto fka = Robot::eefk(soln);
    //     // std::cout <<std::endl << fka.matrix() <<std::endl;
    //     // std::cout << std::endl;
    //     outfile << "\n";
    //     // task_constraint.distanceToConstraint(projected_block);
    // }
    // outfile.close();
}
