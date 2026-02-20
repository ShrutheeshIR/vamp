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
#include "vamp/parsers/mjcf_parser.hh"
#include <fstream>
#include <sstream>
#include <string>



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

// static constexpr Robot::ConfigurationArray start = {
//     0.0,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31631,-0.0134694,-0.28807,0.11635,-0.00983,0.02358,-0.304,0.0152,0.472,-0.36544,0.00508,-0.29158,-0.31758,0.0134478,0.28939,-0.1167,0.01291,0.00435,0.304,-0.01199,-0.472
// };

// static constexpr Robot::ConfigurationArray goal = {
//     0.00305,0.01553,-0.48345,-0.01069,0.00173,-0.00299,0.36992,0.00741,-0.22885,-0.85230,-0.02549,0.90910,-0.43356,0.01072,-0.10347,-0.22115,0.08292,1.04140,-0.33777,0.01560,0.22590,0.84363,0.02173,-0.90310,0.42748,0.06019,-0.22174,0.23454,-0.12485,-1.19928
// };

static constexpr Robot::ConfigurationArray standing_pose = {0.0287480000000000, -0.0189052000000000, -0.0145299000000000, 0.00967345000000000, 0.00825856000000000, -0.00215209000000000,
0.397209000000000, -0.00820680000000000, 0.284002000000000, 0.290042000000000, -0.0213535000000000, -0.235481000000000, -0.0319210000000000, 0.00127043000000000,
-0.0782953000000000, 1.04159000000000, 0.0284093000000000, -0.0123198000000000,
-0.358284000000000, -0.0133610000000000, -0.278441000000000, -0.268447000000000, 0.0177607000000000, 0.212721000000000, -0.0692397000000000, -0.000343739000000000,
0.0689620000000000, -1.23829000000000, 0.0369306000000000, -0.00862385000000000};
// static constexpr Robot::ConfigurationArray box_top_shelf_pickup = {0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31794,-0.01125,-0.28643,0.11635,-0.00983,-0.15597,-0.29369,-0.00103,0.35552,-0.36544,0.00508,-0.29158,-0.32097,0.00882,0.28598,-0.11670,0.01291,-0.12971,0.29191,0.02145,-0.35453};
// static constexpr Robot::ConfigurationArray rack_2 = {0.00305,0.01553,-0.48345,-0.01069,0.00173,-0.00299,0.36991,0.00741,-0.22885,-0.85230,-0.02549,0.90910,-0.43356,0.01072,-0.02876,-0.17999,0.16228,0.99774,-0.33776,0.01561,0.22590,0.84363,0.02173,-0.90309,0.42748,0.06021,-0.16520,0.35173,-0.06290,-1.30637};

static constexpr Robot::ConfigurationArray box_top_shelf_pickup = {
    0.00779,-0.02074,0.00461,-0.00428,-0.00896,-0.00528,0.38120,0.00154,0.28644,0.32323,-0.00734,-0.29696,0.07485,0.01125,0.08865,-0.26348,-0.00645,0.11539,-0.37495,-0.00330,-0.29074,-0.31914,0.01103,0.28118,-0.13812,0.00576,0.16191,0.27674,0.06755,-0.09470
};

static constexpr Robot::ConfigurationArray rack_2 = {
    0.00927,-0.01219,-0.47283,-0.01411,-0.02955,-0.00937,0.40005,0.01408,-0.23717,-0.84704,-0.02429,0.90190,-0.48186,0.04543,-0.17059,-0.36363,0.06204,1.09005,-0.36164,-0.00267,0.22962,0.84538,0.02448,-0.90900,0.40052,0.03680,-0.29646,0.37895,-0.15478,-1.16821
};

static constexpr Robot::ConfigurationArray rack_3 = {
    0.00927,-0.01219,-0.47283,-0.01411,-0.02955,-0.00937,0.40005,0.01408,-0.23717,-0.84704,-0.02429,0.90190,-0.48186,0.04543,-0.17059,-0.36363,0.06204,1.09005,-0.36164,-0.00267,0.22962,0.84538,0.02448,-0.90900,0.40052,0.03680,-0.29646,0.37895,-0.15478,-1.16821
};


static constexpr Robot::ConfigurationArray start = {
    0.0287480000000000, -0.0189052000000000, -0.0145299000000000, 0.00967345000000000, 0.00825856000000000, -0.00215209000000000,
0.397209000000000, -0.00820680000000000, 0.284002000000000, 0.290042000000000, -0.0213535000000000, -0.235481000000000, -0.0319210000000000, 0.00127043000000000,
-0.0782953000000000, 1.04159000000000, 0.0284093000000000, -0.0123198000000000,
-0.358284000000000, -0.0133610000000000, -0.278441000000000, -0.268447000000000, 0.0177607000000000, 0.212721000000000, -0.0692397000000000, -0.000343739000000000,
0.0689620000000000, -1.23829000000000, 0.0369306000000000, -0.00862385000000000
};

static constexpr Robot::ConfigurationArray goal = {
    0.0144636,-0.0135279,-0.317291,0.0638242,-0.252667,0.0139239,0.394165,-0.0164428,0.295529,-0.379852,-0.118226,0.559355,-0.225703,-0.0560423,-0.222843,0.45205,0.426306,0.05304,-0.327884,-0.0220936,-0.365103,0.200247,0.272734,-0.609867,0.231366,-0.0179224,-0.0699971,-0.613628,-0.0186893,-0.217534
};

// static constexpr Robot::ConfigurationArray goal = {0.0,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31631,-0.01347,-0.28807,0.11635,-0.00983,-0.10504,-0.4,0.174,0.37778,-0.36544,0.00508,-0.29158,-0.31758,0.01345,0.28939,-0.1167,0.01291,0.10456,0.4,-0.324,-0.36663};


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

    auto geoms = vamp::utils::parser::parseMJCF("/src/myfork/vamp/resources/environments/cuboids/wooden_shelf.xml");
    for (const auto& g : geoms) {
        if (g.type == vamp::utils::parser::GeomType::BOX){
            std::cout << "Adding cuboid with pos " << g.world_pose.pos.x << ", " << g.world_pose.pos.y << ", " << g.world_pose.pos.z << " and size " << g.size.x << ", " << g.size.y << ", " << g.size.z << std::endl;
            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({g.world_pose.pos.x, g.world_pose.pos.y, g.world_pose.pos.z}, {0.0, 0.0, 0.0}, {g.size.x, g.size.y, g.size.z}));
        }

    }



    std::array<float, 8> polygon_points = {
        0.01, -0.01, 0.01, 0.03, -0.05, 0.03, -0.05, -0.01
    };

    std::array<float, 6> lower_bound = {
        -0.001, -0.001, -0.001, -0.1, -10.1, -10.1
    };
    std::array<float, 6> upper_bound = {
        0.001, 0.001, 0.001, 0.1, 10.1, 10.1
    };

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, 
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, 
        -0.001, -0.001, -0.001, -0.1, -0.1, -0.1, 
        -0.001, -0.001, -0.001, -0.1, -0.1, -0.1
    };

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        0.001, 0.001, 0.001, 0.1, 0.1, 0.1, 
        0.001, 0.001, 0.001, 0.1, 0.1, 0.1
    };



    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{{1, 0,0,0,   0, 0, 0}, {1, 0,0,0,   0.0, 0.0, 0.0}, {0.59, 0.38, 0.4, 0.59 , -0.02070, 0.06015, -0.95335}, {0.61, -0.36, 0.35, -0.61 , -0.02228, -0.11609, -0.94832}}};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{{1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0}}};

    vamp::planning::FeetTaskSpaceConstraint<Robot, rake> feet_tsr_constraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        tsr_lower_bound,
        tsr_upper_bound
    );

    std::array<float, 7> transform = {0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.36356};
    vamp::planning::BimanualTaskSpaceConstraint<Robot, rake> bimanual_task_constraint(transform, lower_bound, upper_bound);
    vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(
        polygon_points
    );

    vamp::planning::ClosedLinkConstraint<Robot, rake> closed_link_constraint;

    vamp::planning::ComposableConstraints<Robot, rake, 
        vamp::planning::FeetTaskSpaceConstraint<Robot, rake>,
        vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>, 
        vamp::planning::ClosedLinkConstraint<Robot, rake>, 
        vamp::planning::BimanualTaskSpaceConstraint<Robot, rake>>task_constraint(
        feet_tsr_constraint,
        com_constraint,
        closed_link_constraint,
        bimanual_task_constraint
    );

    vamp::planning::ComposableConstraints<Robot, rake, 
        vamp::planning::FeetTaskSpaceConstraint<Robot, rake>,
        vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>, 
        vamp::planning::ClosedLinkConstraint<Robot, rake>>non_transport_task_constraint(
        feet_tsr_constraint,
        com_constraint,
        closed_link_constraint
    );


    // 1. Extract the pointer to the data
    const float* data = transform.data();

    // 2. Create the Quaternion (w, x, y, z)
    // Note: We cast to double because Isometry3d expects doubles
    Eigen::Quaternionf bimanual_quat(
        static_cast<double>(data[0]), // w
        static_cast<double>(data[1]), // x
        static_cast<double>(data[2]), // y
        static_cast<double>(data[3])  // z
    );

    std::cout << "Adding transforms : " << data[4] << ", " << data[5] << ", " << data[6] << std::endl;
    // 3. Create the Translation vector (x, y, z)
    Eigen::Vector3f bimanual_trans(
        static_cast<double>(data[4]), 
        static_cast<double>(data[5]), 
        static_cast<double>(data[6])
    );

    // 4. Combine into an Isometry3d
    Eigen::Isometry3f T_bim_goal = Eigen::Isometry3f::Identity();
    T_bim_goal.rotate(bimanual_quat);
    T_bim_goal.pretranslate(bimanual_trans);

    std::cout << "Bimanual constraint goal transform: " << T_bim_goal.matrix() << std::endl;


    std::vector<std::array<float, 3>> attach_spheres = {
        {-0.02, 0.0, -0.09},
        {-0.02, -0.02, -0.18},
        {-0.02, -0.04, -0.26},
        {-0.02, 0.05, -0.09},
        {-0.02, 0.03, -0.18},
        {-0.02, 0.01, -0.26},
        {0.04, 0.0, -0.09},
        {0.04, -0.02, -0.18},
        {0.04, -0.04, -0.26},
        {0.04, 0.05, -0.09},
        {0.04, 0.03, -0.18},
        {0.04, 0.01, -0.26},
    };
    std::vector<vamp::collision::Sphere<float>> spheres;
    for(const auto& s : attach_spheres){
        std::cout << "Adding sphere with center " << s[0] << ", " << s[1] << ", " << s[2] << " and radius " << 0.05 << std::endl;
        spheres.emplace_back(vamp::collision::Sphere<float>(s[0], s[1], s[2], 0.05));
        }
    auto attach_transform = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    AttachmentInput attachment(attach_transform);

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

    auto left_right_goal = left_right * T_bim_goal.inverse();
    Eigen::Quaternionf q_left_right_goal(left_right_goal.linear());
    std::cout << q_left_right_goal.w() << ", " << q_left_right_goal.x() << ", " << q_left_right_goal.y() << ", " << q_left_right_goal.z() << ", " << left_right_goal.translation().transpose() << std::endl;


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

    left_right_goal = left_right * T_bim_goal.inverse();
    q_left_right_goal = Eigen::Quaternionf(left_right_goal.linear());
    std::cout << q_left_right_goal.w() << ", " << q_left_right_goal.x() << ", " << q_left_right_goal.y() << ", " << q_left_right_goal.z() << ", " <<  left_right_goal.translation().transpose() << std::endl;



    bool success;
    // Robot::ConfigurationArray test = {-0.05756, -0.10361, -0.30607, -0.22192, -0.22951, -0.23933, -1.01621, -0.03210, -0.22167, 1.59964, -0.86941, -0.02150, -1.02993, -0.24976, -0.22910, 1.59571, -0.87069, -0.02193, 1.23762, -0.29675, 0.43134, -0.26073, -0.13340, -0.22137, -0.08773, -0.16697, -0.13678, -0.13683, -0.26232, -0.19178, -0.22258, -0.08843, -0.16783, -0.13752, -0.13754};
    typename Robot::template ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i);
    bool goal_valid = Robot::template fkcc<rake>(env_v, block);
    std::cout << "Goal valid: " << goal_valid << std::endl;

    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(start).broadcast(i);
    bool start_valid = Robot::template fkcc<rake>(env_v, block);
    std::cout << "Start valid: " << start_valid << std::endl;

    task_constraint.distanceToConstraint(block);
    task_constraint.print_robot_tsr_error(block);

    std::cout << "----Projecting ----" << std::endl;
    typename Robot::template ConfigurationBlock<rake> projected_block;


    // Insert after line 320
    std::ifstream traj_file("/src/trajectory.txt");
    std::string line;
    while (std::getline(traj_file, line)) {
        Robot::ConfigurationArray config;

        std::stringstream ss(line);
        std::string value;
        size_t dim = 0;
        // Split by comma
        while (std::getline(ss, value, ',')) {
            config[dim++] = std::stof(value);
        }


        isometries = Robot::eefk(config);
        // std::cout << "Goal pose eef poses " << std::endl;

        // q_hand = Eigen::Quaternionf(isometries[0].linear());
        // std::cout << q_hand.w() << ", " << q_hand.x() << ", " << q_hand.y() << ", " << q_hand.z() << " , " << isometries[0].translation().transpose() << std::endl;
        // q_hand = Eigen::Quaternionf(isometries[1].linear());
        // std::cout << q_hand.w() << ", " << q_hand.x() << ", " << q_hand.y() << ", " << q_hand.z() << " , " << isometries[1].translation().transpose() << std::endl;

        // std::cout << "Relative pose between left and right hand " << (isometries[0].inverse() * isometries[1]).matrix() << std::endl;
        // q_foot = Eigen::Quaternionf(isometries[2].linear());
        // std::cout << isometries[2].translation().transpose()  << " " << isometries[3].translation().transpose() << std::endl;
        // q_foot = Eigen::Quaternionf(isometries[3].linear());
        // std::cout << q_foot.w() << ", " << q_foot.x() << ", " << q_foot.y() << ", " << q_foot.z() << " , " << isometries[3].translation().transpose() << std::endl;

        // left_right = isometries[0].inverse() * isometries[1];
        // q_left_right = Eigen::Quaternionf(left_right.linear());
        // std::cout << q_left_right.w() << ", " << q_left_right.x() << ", " << q_left_right.y() << ", " << q_left_right.z() << " , " << left_right.translation().transpose() << std::endl;

        // left_right_goal = left_right * T_bim_goal.inverse();
        // q_left_right_goal = Eigen::Quaternionf(left_right_goal.linear());
        // std::cout << q_left_right_goal.w() << ", " << q_left_right_goal.x() << ", " << q_left_right_goal.y() << ", " << q_left_right_goal.z() << ", " <<  left_right_goal.translation().transpose() << std::endl;



    }

    size_t pose_iterator = 0;
    // for each of standing_pose, box_top_shelf_pickup, rack_2, rack_3, do projectConfiguration and print out projected block
    for (const auto& config : {standing_pose, box_top_shelf_pickup, rack_2, rack_3}) {
        // if standing pose, then use non_transport_task_constraint, else use task_constraint

        std::cout << "----Projecting config----" << std::endl;
        typename Robot::template ConfigurationBlock<rake> block;
        for (auto i = 0U; i < Robot::dimension; ++i)
            block[i] = Robot::Configuration(config).broadcast(i);
        bool valid = Robot::template fkcc<rake>(env_v, block);
        std::cout << "Config valid: " << valid << std::endl;

        if (pose_iterator == 0) {
            std::cout << "Using non-transport task constraint for standing pose" << std::endl;
            non_transport_task_constraint.distanceToConstraint(block);
            // non_transport_task_constraint.print_robot_tsr_error(block);

            typename Robot::template ConfigurationBlock<rake> projected_block;
            bool success = non_transport_task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 5.0, 1.0, 100, true);
            for(auto i=0U; i < Robot::dimension; i++)
                std::cout << projected_block[{i, 0}] << ",";
            std::cout << success <<std::endl;
        } else {
            std::cout << "Using full task constraint for other poses" << std::endl;
            task_constraint.distanceToConstraint(block);
            // task_constraint.print_robot_tsr_error(block);

            typename Robot::template ConfigurationBlock<rake> projected_block;
            bool success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 5.0, 1.0, 100, true);
            for(auto i=0U; i < Robot::dimension; i++)
                std::cout << projected_block[{i, 0}] << ",";
            std::cout << success <<std::endl;
        }
        pose_iterator++;
    }


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
    std::ofstream outfile("/src/trajectory_com.txt");
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

    // for 


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
