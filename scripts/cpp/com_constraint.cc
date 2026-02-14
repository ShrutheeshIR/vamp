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
#include <vamp/robots/digit.hh>
#include <vamp/random/halton.hh>
#include <fstream>
#include <random>


using Robot = vamp::robots::Digit;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
// using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution, 4>;
using AttachmentInput = vamp::collision::Attachment<float>;

// static constexpr Robot::ConfigurationArray start = {
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // floating joints
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0
// };

// static constexpr Robot::ConfigurationArray goal = {
//     -0.03391,0.00026,-0.48973,-0.00640,0.22621,0.01055,
//     0.35910,-0.03271,-0.64898,-0.84229,0.87402,-0.55091,-0.06622,
//     -0.15184,-0.56563,0.24405,0.97277,
//     -0.36672,0.01895,0.66644,0.86380,-0.86609,0.50342,0.05169,
//     -0.17592,0.44021,0.19196,-0.87314
// };


// static constexpr Robot::ConfigurationArray start = {
//     0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,
//     0.36547,-0.00525,0.29127,0.31631,-0.0134694,-0.28807,0.11635,-0.00983,
//     -0.105038, 0.888521, -0.00624406, 0.377775,
//     -0.36544,0.00508,-0.29158,-0.31758,0.0134478,0.28939,-0.11670,0.01291,
//     0.104559, -0.893957, 0.00550343, -0.366626,
// };

// static constexpr Robot::ConfigurationArray goal = {
//     0.00358,0.01604,-0.48348,-0.00927,0.00030,-0.00362,
//     0.37249,0.00746,-0.22666,-0.85028, -0.02148, 0.91164,-0.43089,0.01204,
//     -0.18033,-0.49388,0.29491,1.24416,
//     -0.34264,0.01123,0.22809,0.84257, 0.021335, -0.90335,0.43159,0.05288,
//     -0.38174,0.50399,-0.22872,-1.31295
// };


// static constexpr Robot::ConfigurationArray start = {
//     0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,
//     0.36547,-0.00525,0.29127,0.31631, -0.0134694,-0.28807,0.11635,-0.00983,
//     0.02358,-0.00579,0.01520,0.09714,
//     -0.36544,0.00508,-0.29158,-0.31758, 0.0134478, 0.28939,-0.11670,0.01291,
//     0.00435,0.00820,-0.01199,-0.09980
// };

// static constexpr Robot::ConfigurationArray goal = {
//     0.00358,0.01604,-0.48348,-0.00927,0.00030,-0.00362,
//     0.37249,0.00746,-0.22666,-0.85028, -0.02148, 0.91164,-0.43089,0.01204,
//     -0.17608,-0.40123,0.03686,1.07578,
//     -0.34264,0.01123,0.22809,0.84257, 0.021335, -0.90335,0.43159,0.05288,
//     -0.28604,0.42121,-0.16502,-1.13728
// };
// static constexpr Robot::ConfigurationArray goal = {-0.045,0.0,-0.03,0.0,-0.264,0.0,0.0,0.0,0.0,0.456,0.0,0.0,0.0,0.0,0.0,0.486,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

static constexpr Robot::ConfigurationArray start = {
    0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31631,-0.01347,-0.28807,0.11635,-0.00983,0.02358,-0.00579,0.01520,0.09714,-0.36544,0.00508,-0.29158,-0.31758,0.01345,0.28939,-0.11670,0.01291,0.00435,0.00820,-0.01199,-0.09980
};

static constexpr Robot::ConfigurationArray goal = {
    0.00358,0.01604,-0.48348,-0.00927,0.00030,-0.00362,0.37249,0.00746,-0.22666,-0.85028,-0.02148,0.91164,-0.43089,0.01204,-0.17608,-0.40123,0.03686,1.07578,-0.34264,0.01123,0.22809,0.84257,0.02134,-0.90335,0.43159,0.05288,-0.28604,0.42121,-0.16502,-1.13728
};


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


auto main(int, char **) -> int
{
    std::cout << std::fixed << std::setprecision(5);

    vamp::planning::RRTCSettings rrtc_settings;

    EnvironmentInput environment;


    std::ifstream infile("/src/myfork/vamp/resources/environments/cuboids/shelf_drake.txt");
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

    auto isometries = Robot::eefk(start);
    std::cout << "Start pose eef poses " << std::endl;
    // std::cout << "Left hand " << isometries[0].matrix() << std::endl;
    // std::cout << "Right hand " << isometries[1].matrix() << std::endl;
    // std::cout << "Left foot " << isometries[2].matrix() << std::endl;
    // std::cout << "Right foot " << isometries[3].matrix() << std::endl;
    Eigen::Quaternionf q_hand(isometries[0].linear());
    std::cout << q_hand.w() << ", " << q_hand.x() << ", " << q_hand.y() << ", " << q_hand.z() << " , " << isometries[0].translation().transpose() << std::endl;
    q_hand = Eigen::Quaternionf(isometries[1].linear());
    std::cout << q_hand.w() << ", " << q_hand.x() << ", " << q_hand.y() << ", " << q_hand.z() << " , " << isometries[1].translation().transpose() << std::endl;


    // std::cout << "Relative pose between left and right hand " << (isometries[0].inverse() * isometries[1]).matrix() << std::endl;
    Eigen::Quaternionf q_foot(isometries[2].linear());
    std::cout << q_foot.w() << ", " << q_foot.x() << ", " << q_foot.y() << ", " << q_foot.z() << " , " << isometries[2].translation().transpose() << std::endl;
    q_foot = Eigen::Quaternionf(isometries[3].linear());
    std::cout << q_foot.w() << ", " << q_foot.x() << ", " << q_foot.y() << ", " << q_foot.z() << " , " << isometries[3].translation().transpose() << std::endl;

    auto left_right = isometries[0].inverse() * isometries[1];
    Eigen::Quaternionf q_left_right(left_right.linear());
    std::cout << q_left_right.w() << ", " << q_left_right.x() << ", " << q_left_right.y() << ", " << q_left_right.z() << left_right.translation().transpose() << std::endl;


    isometries = Robot::eefk(goal);
    std::cout << "Goal pose eef poses " << std::endl;
    // std::cout << "Left hand " << isometries[0].matrix() << std::endl;
    // std::cout << "Right hand " << isometries[1].matrix() << std::endl;
    // std::cout << "Left foot " << isometries[2].matrix() << std::endl;
    // std::cout << "Right foot " << isometries[3].matrix() << std::endl;

    q_hand = Eigen::Quaternionf(isometries[0].linear());
    std::cout << q_hand.w() << ", " << q_hand.x() << ", " << q_hand.y() << ", " << q_hand.z() << " , " << isometries[0].translation().transpose() << std::endl;
    q_hand = Eigen::Quaternionf(isometries[1].linear());
    std::cout << q_hand.w() << ", " << q_hand.x() << ", " << q_hand.y() << ", " << q_hand.z() << " , " << isometries[1].translation().transpose() << std::endl;

    // std::cout << "Relative pose between left and right hand " << (isometries[0].inverse() * isometries[1]).matrix() << std::endl;
    q_foot = Eigen::Quaternionf(isometries[2].linear());
    std::cout << q_foot.w() << ", " << q_foot.x() << ", " << q_foot.y() << ", " << q_foot.z() << " , " << isometries[2].translation().transpose() << std::endl;
    q_foot = Eigen::Quaternionf(isometries[3].linear());
    std::cout << q_foot.w() << ", " << q_foot.x() << ", " << q_foot.y() << ", " << q_foot.z() << " , " << isometries[3].translation().transpose() << std::endl;

    left_right = isometries[0].inverse() * isometries[1];
    q_left_right = Eigen::Quaternionf(left_right.linear());
    std::cout << q_left_right.w() << ", " << q_left_right.x() << ", " << q_left_right.y() << ", " << q_left_right.z() << " , " << left_right.translation().transpose() << std::endl;

    std::array<float, 8> polygon_points = {
        0.03, -0.075, 0.03, 0.075, -0.04, 0.075, -0.04, -0.1
    };

    std::array<float, 6> lower_bound = {
        -0.001, -0.001, -0.001, -1.0, -1.0, -1.0
    };
    std::array<float, 6> upper_bound = {
        0.001, 0.001, 0.001, 1.0, 1.0, 1.0
    };

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -0.01, -0.01, -0.01, -0.05, -0.05, -0.05, -0.001, -0.001, -0.001, -0.05, -0.05, -0.05
    };

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.001, 0.001, 0.001, 0.05, 0.05, 0.05
    };


    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{{1, 0,0,0,   0, 0, 0}, {1, 0,0,0,   0.0, 0.0, 0.0}, {0.603, 0.36, 0.36, 0.603, -0.04302,  0.10080, -0.96013}, {0.603, -0.36, 0.36, -0.603 , -0.04288, -0.09895, -0.96033}}};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{{1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}}};

    vamp::planning::TaskSpaceConstraint<Robot, rake> feet_tsr_constraint(
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

    vamp::planning::ClosedLinkConstraint<Robot, rake> closed_link_constraint;

    // vamp::planning::SelfCollisionConstraint<Robot, rake> self_collision_constraint;


    vamp::planning::ComposableConstraints<Robot, rake, decltype(feet_tsr_constraint), decltype(com_constraint), decltype(bimanual_task_constraint), decltype(closed_link_constraint)> task_constraint(
        feet_tsr_constraint,
        com_constraint,
        bimanual_task_constraint,
        closed_link_constraint
    );
    // vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(
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

    // vamp::planning::BimanualTaskSpaceConstraint<Robot, rake> task_constraint(
    //     target_pose,
    //     std::make_pair(lower_bound, upper_bound)
    // );


    // // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    // Eigen::Matrix<float, 4, 4> T;
    // T << -0.719427, 0.694568, 6.59173e-05, -2.2769e-05, 0.694568, 0.719427, -2.26738e-05, -0.000370264, -6.31774e-05, 2.94462e-05, -1, 0.171814, 0, 0, 0, 1;
    // const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    // vamp::planning::BimanualCoMTaskSpaceConstraint<Robot, rake, 4> task_constraint(polygon_points, target_pose, std::make_pair(lower_bound, upper_bound));



    // // vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4> task_constraint(polygon_points);
    // auto startcc = rng->next();
    bool success;
    // Robot::ConfigurationArray test = {-0.05756, -0.10361, -0.30607, -0.22192, -0.22951, -0.23933, -1.01621, -0.03210, -0.22167, 1.59964, -0.86941, -0.02150, -1.02993, -0.24976, -0.22910, 1.59571, -0.87069, -0.02193, 1.23762, -0.29675, 0.43134, -0.26073, -0.13340, -0.22137, -0.08773, -0.16697, -0.13678, -0.13683, -0.26232, -0.19178, -0.22258, -0.08843, -0.16783, -0.13752, -0.13754};
    typename Robot::template ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(start).broadcast(i);


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
    success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 5.0, 1.0, 100, true);
    for(auto i=0U; i < Robot::dimension; i++)
        std::cout << projected_block[{i, 0}] << ",";
    std::cout << success <<std::endl;
    std::ofstream outfile("/src/trajectory.txt");
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        if (!first) outfile << ",";
        outfile << block[{i, 0}];
        first = false;
    }

    outfile << "\n";

    first = true;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        if (!first) outfile << ",";
        outfile << projected_block[{i, 0}];
        first = false;
    }

    outfile << "\n";

    std::cout << " \n\n ------------Goal-----------------\n\n";

    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i);

    first = true;
    success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 5.0, 1.0, 100, true);
    for(auto i=0U; i < Robot::dimension; i++)
        std::cout << projected_block[{i, 0}] << ",";
    std::cout << success <<std::endl;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        if (!first) outfile << ",";
        outfile << block[{i, 0}];
        first = false;
    }

    outfile << "\n";

    first = true;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        if (!first) outfile << ",";
        outfile << projected_block[{i, 0}];
        first = false;
    }

    outfile << "\n";

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


    auto startc = Robot::Configuration(start);
    auto goalc = Robot::Configuration(goal);


    // std::cout << startc << ", " << goalc << std::endl;
    auto extension_vector = goalc - startc;
    // std::cout << extension_vector << std::endl;
    // extension_vector = extension_vector / extension_vector.l2_norm();
    // std::cout << extension_vector << std::endl;


    // // std::cout << std::fixed << std::setprecision(3);
    // // std::ofstream outfile("/src/trajectory.txt");

    std::default_random_engine generator;
    std::normal_distribution<double> dist(0.0, 0.01);

    for(size_t ext = 0; ext < 11; ext++){
        std::cout << "\nExtension attempt " << ext << " " << std::endl;
        auto cfg = startc + extension_vector * ext / 10;

        std::array<float, Robot::dimension> config_for_eefk;
        for (auto i = 0U; i < Robot::dimension; ++i)
            config_for_eefk[i] = start[i] + (goal[i] - start[i]) * ext / 10;


        isometries = Robot::eefk(config_for_eefk);
        std::cout << "Interm pose eef poses " << std::endl;
        // std::cout << "Left hand " << isometries[0].matrix() << std::endl;
        // std::cout << "Right hand " << isometries[1].matrix() << std::endl;
        // std::cout << "Left foot " << isometries[2].matrix() << std::endl;
        // std::cout << "Right foot " << isometries[3].matrix() << std::endl;

        q_hand = Eigen::Quaternionf(isometries[0].linear());
        std::cout << q_hand.w() << ", " << q_hand.x() << ", " << q_hand.y() << ", " << q_hand.z() << " , " << isometries[0].translation().transpose() << std::endl;
        q_hand = Eigen::Quaternionf(isometries[1].linear());
        std::cout << q_hand.w() << ", " << q_hand.x() << ", " << q_hand.y() << ", " << q_hand.z() << " , " << isometries[1].translation().transpose() << std::endl;

        // std::cout << "Relative pose between left and right hand " << (isometries[0].inverse() * isometries[1]).matrix() << std::endl;
        q_foot = Eigen::Quaternionf(isometries[2].linear());
        std::cout << q_foot.w() << ", " << q_foot.x() << ", " << q_foot.y() << ", " << q_foot.z() << " , " << isometries[2].translation().transpose() << std::endl;
        q_foot = Eigen::Quaternionf(isometries[3].linear());
        std::cout << q_foot.w() << ", " << q_foot.x() << ", " << q_foot.y() << ", " << q_foot.z() << " , " << isometries[3].translation().transpose() << std::endl;

        left_right = isometries[0].inverse() * isometries[1];
        q_left_right = Eigen::Quaternionf(left_right.linear());
        std::cout << q_left_right.w() << ", " << q_left_right.x() << ", " << q_left_right.y() << ", " << q_left_right.z() << " , " << left_right.translation().transpose() << std::endl;


        for (auto i = 0U; i < Robot::dimension; ++i)
            block[i] = cfg.broadcast(i);// + dist(generator);
        for(auto i=0U; i < Robot::dimension; i++)
            std::cout << block[{i, 0}] << ", ";
        std::cout << std::endl;

        typename Robot::template ConfigurationBlock<rake> projected_block;
        success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 5.0, 1.0, 100, true);

        bool valid = Robot::template fkcc<rake>(env_v, projected_block);
        bool valid_og = Robot::template fkcc<rake>(env_v, block);

        for(auto i=0U; i < Robot::dimension; i++)
            std::cout << projected_block[{i, 0}] << ", ";
        std::cout << " --> " << success << " -- " << valid << "-- " << valid_og << std::endl;


        bool first = true;
        // for (auto i = 0U; i < Robot::dimension; ++i)
        // {
        //     if (!first) outfile << ",";
        //     outfile << block[{i, 0}];
        //     first = false;
        // }

        // outfile << "\n";

        // first = true;
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            if (!first) outfile << ",";
            outfile << projected_block[{i, 0}];
            first = false;
        }

        // auto fka = Robot::eefk(soln);
        // std::cout <<std::endl << fka.matrix() <<std::endl;
        // std::cout << std::endl;
        outfile << "\n";
        // task_constraint.distanceToConstraint(projected_block);
    }
    outfile.close();



}
