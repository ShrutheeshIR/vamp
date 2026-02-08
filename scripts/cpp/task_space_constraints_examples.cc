#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
// #include <vamp/planning/cbirrt.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/bimanual_iiwa.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::BimanualIiwa;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Start and goal configurations
// static constexpr Robot::ConfigurationArray start = {1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891};
// static constexpr Robot::ConfigurationArray goal = {-0.154247,-1.38187,1.88281,-1.42206,-0.691598,3.52511,1.43219};

// static constexpr Robot::ConfigurationArray start = {-1.314, 1.339, 1.095, -2.495, 0.513, 2.551, -1.495, 1.294, 1.309, -1.055, -2.493, -0.713, 2.500, -3.014 };
// static constexpr Robot::ConfigurationArray goal = {-1.068, 0.602, 1.329, -1.797, 1.059, 1.959, -1.149, 1.033, 0.446, -1.224, -1.837, -1.286, 1.917, 2.813};

static constexpr Robot::ConfigurationArray start = {-0.64309101,  1.9156121 , -1.79682547,  1.29454471, -0.02383453,
       -0.87696681, -1.70416432,  0.71370579,  1.96751046,  1.72862129,
        1.29729566,  0.16350904, -0.933994  ,  2.38605378};
static constexpr Robot::ConfigurationArray goal = {-0.19949942,  0.914074  , -2.23661832,  0.52388792,  0.79984419,
       -1.3575398 , -1.01530928,  0.2416075 ,  0.90226654,  2.28974135,
        0.52868543, -0.8636816 , -1.41231345,  1.79618995};


// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.35, 0.35, 0.25},
    // {0, 0.55, 0.25},
    // {-0.55, 0, 0.25},
    // {-0.35, -0.35, 0.25},
    // {0, -0.55, 0.25},
    // {0.35, -0.35, 0.25},
    // {0.35, 0.35, 0.8},
    // {0, 0.55, 0.8},
    // {-0.35, 0.35, 0.8},
    // {-0.55, 0, 0.8},
    // {-0.35, -0.35, 0.8},
    // {0, -0.55, 0.8},
    // {0.35, -0.35, 0.8},
};
// Radius for obstacle spheres
static constexpr float radius = 0.2;


auto main(int, char **) -> int
{
    // Build sphere cage environment
    EnvironmentInput environment;
    for (const auto &sphere : problem)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    std::array<float, 6> tsr_lower_bound = {
        -0.005, -10.01, -0.005, -0.01, -10.01, -10.01
    };
    std::array<float, 6> tsr_upper_bound = {
        0.005, 10.01, 0.005, 0.01, 10.01, 10.01
    };


    // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms;
    // Eigen::Matrix<float, 4, 4> T;
    // T << 1,0,0,   0.3486,   0,-1,0,      0.647752,   0,0,-1,    0.2399,          0,           0,           0,           1;
    // eef_transforms[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms_ref_frame_w_world;
    // T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

    // vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(
    //     eef_transforms_ref_frame_w_world,
    //     eef_transforms,
    //     std::make_pair(tsr_lower_bound, tsr_upper_bound)
    // );
    std::array<float, 6> lower_bound = {
        -0.001, -0.001, -0.001, -0.001, -0.001, -0.001
    };
    std::array<float, 6> upper_bound = {
        0.001, 0.001, 0.001, 0.001, 0.001, 0.001
    };


    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;

    std::array<float, 7> transform = {-0.0005, 0.927184, -0.364607, 0.0009 , 5.96046e-08 ,          0 ,        0.6};
    vamp::planning::BimanualTaskSpaceConstraint<Robot, rake> bimanual_task_constraint(transform, lower_bound, upper_bound);

    vamp::planning::ComposableConstraints<Robot, rake, decltype(bimanual_task_constraint)> task_constraint(
        bimanual_task_constraint
    );

    auto start_time = std::chrono::steady_clock::now();



    // Robot::ConfigurationArray test = {-0.154247,-1.38187,1.88281,-1.42206,-0.691598,3.52511,1.43219};
    // typename Robot::template ConfigurationBlock<rake> block;
    // for (auto i = 0U; i < Robot::dimension; ++i)
    //     block[i] = Robot::Configuration(test).broadcast(i) + 0.0;
    // auto dist = task_constraint.distanceToConstraint(block);
    // // std::cout << "Dist to constraint" << "->" << vamp::utils::get_elapsed_nanoseconds(start_time);
    // std::cout << "From block : " << dist << std::endl;
    // task_constraint.print_robot_tsr_error(block);
    // Eigen::Quaternionf qstart(Robot::eefk(start)[0].linear());
    // Eigen::Quaternionf qgoal(Robot::eefk(goal)[0].linear());
    std::cout << Robot::eefk(start)[0].matrix() << std::endl;
    std::cout << Robot::eefk(goal)[0].matrix() << std::endl;

    auto start_eefk = Robot::eefk(start);
    std::cout << (start_eefk[0].inverse() * start_eefk[1]).matrix()  << std::endl;
    auto goal_eefk = Robot::eefk(goal);
    std::cout << (goal_eefk[0].inverse() * goal_eefk[1]).matrix()  << std::endl;

    auto lTr_start = start_eefk[0].inverse() * start_eefk[1];
    Eigen::Quaternionf q1(lTr_start.linear());
    std::cout << q1.w() << ", " << q1.x() << ", " << q1.y() << ", " << q1.z() << " , " << lTr_start.translation().transpose() << std::endl;


    auto lTr_goal = goal_eefk[0].inverse() * goal_eefk[1];
    Eigen::Quaternionf q2(lTr_goal.linear());
    std::cout << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << ", " << lTr_goal.translation().transpose() << std::endl;


    // std::cout << qstart.w() << ", " << qstart.x() << ", " << qstart.y() << ", " << qstart.z() << std::endl;
    // std::cout << qgoal.w() << ", " << qgoal.x() << ", " << qgoal.y() << ", " << qgoal.z() << std::endl;


    auto vector = Robot::Configuration(goal) - Robot::Configuration(start);

    auto vector_norm = vector.l2_norm();
    auto distance = vector_norm;
    vector = vector / vector_norm;
    typename Robot::template ConfigurationBlock<rake> block, projected_block, direction_vector_block, initial_projected_block;
    // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
    const auto percents = vamp::FloatVector<rake>(vamp::planning::Percents<rake>::percents);

    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = Robot::Configuration(start).broadcast(i) + (vector.broadcast(i) * percents);
        direction_vector_block[i] = vector.broadcast(i);
    }

    std::cout << "Block values: ";
    for (auto i = 0U; i < Robot::dimension; ++i){
        std::cout << block[{i, 0}] << ", ";
    }
    std::cout << std::endl;


    bool ableToProject = task_constraint.projectConfiguration(block, initial_projected_block, vamp::planning::ProjMethod::InnerLM, vector_norm, 1.0);

    std::cout << std::endl;

    for (auto i = 0U; i < rake-1; i++)
    {
        float inter_distance = 0.F;
        for (auto j = 0U; j < Robot::dimension; j++)
        {
            // std::cout << i << " " << j << " " << initial_projected_block[{j, i+1}] << ", " << initial_projected_block[{j, i}] << " " << initial_projected_block[{j, i+1}] - initial_projected_block[{j, i}] << std::endl;
            inter_distance = inter_distance + std::pow(initial_projected_block[{j, i+1}] - initial_projected_block[{j, i}], 2);
        }
        std::cout << "Distance between points " << i << " and " << i+1 << ": " << std::sqrt(inter_distance) << " " << (distance / rake)<< std::endl;
        // if (inter_distance > (distance / rake) * (distance / rake))
        // {
        //     return false;
        // }
    }

    // // auto diff_block = configuration_block_difference(initial_projected_block, Robot::Configuration(start));


    std::array<vamp::FloatT, Robot::dimension * rake> diff_arr;
    std::cout <<diff_arr.size() << std::endl;
    std::cout << "Computing diffs : " << std::endl;

    auto lane_dist_block = vamp::planning::inter_lane_distance_block(initial_projected_block, Robot::Configuration(start));


    typename Robot::template ConfigurationBlock<rake> start_block;
    float max_inter_dist = 0.F;

    // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        start_block[i] = Robot::Configuration(start).broadcast(i);
    }
    for (auto i = 0U; i < rake; i++)
    {
        std::cout << std::endl << i << " : ";
        float inter_distance = 0.F;
        for (auto j = 0U; j < Robot::dimension; j++)
        {
            if (i == 0)
            {
                diff_arr[i + j * rake] = initial_projected_block[{j, i}] - start_block[{j, i}];
            }
            else
            {
                diff_arr[i + j * rake] = initial_projected_block[{j, i}] - initial_projected_block[{j, i-1}];
            }

            if (abs(diff_arr[i + j * rake] - lane_dist_block[{j, i}]) > 1e-6)
            {
                std::cout << "Error at " << i << ", " << j << ": " << diff_arr[i + j * rake] << " != " << lane_dist_block[{j, i}] << std::endl;
            }
            inter_distance = inter_distance + diff_arr[i + j * rake] * diff_arr[i + j * rake];
        }
        std::cout << "Inter Distance: " << inter_distance <<std::endl;
        max_inter_dist = std::max(max_inter_dist, inter_distance);
    }
    max_inter_dist = std::sqrt(max_inter_dist);

    std::cout << initial_projected_block << Robot::Configuration(start) << std::endl;
    std::cout << std::endl << "Lane Distance Block: " << lane_dist_block << std::endl;

    auto inter_rake_distance = lane_dist_block[0] * lane_dist_block[0];
    for (auto dim = 1U; dim < Robot::dimension; dim++)
    {
        std::cout << Robot::Configuration(start).element(dim) << " ";
        inter_rake_distance = inter_rake_distance + lane_dist_block[dim] * lane_dist_block[dim];
    }
    std::cout << std::endl;
    std::cout << "Inter Rake Distance: " << inter_rake_distance << " , " << std::sqrt(inter_rake_distance.hmax()) << ", " << max_inter_dist << std::endl;



    // typename Robot::template ConfigurationBlock<rake> shifted_block = typename Robot::template ConfigurationBlock<rake>(diff_arr);

    // std::cout << Robot::Configuration(start) << std::endl;
    // std::cout << initial_projected_block << std::endl;
    // std::cout << shifted_block << std::endl;
    // std::cout << initial_projected_block - shifted_block << std::endl;
    // auto q_dist = shifted_block[0] * shifted_block[0];
    // for(auto j = 1U; j < Robot::dimension; j++)
    // {
    //     q_dist = q_dist + shifted_block[j] * shifted_block[j];
    // }
    // std::cout << q_dist << " " << q_dist.sqrt() << " " << std::endl;

    // for(auto j = 1U; j < Robot::dimension; j++)
    // {
    //     shifted_block[{j, 0}] = initial_projected_block[{j-1, 0}];
    // }


    // shifted_block[0] = Robot::Configuration(start).broadcast(0);
    // shifted_block[1] = initial_projected_block[0];
    // shifted_block[2] = initial_projected_block[1];
    // shifted_block[3] = initial_projected_block[2];

    // std::cout << diff_block[0].l2_norm() << " , " << diff_block[0].squared_l2_norm() << std::endl;

    // std::cout << initial_projected_block << std::endl;
    // std::cout <<  shifted_block << std::endl;
    // std::cout <<  shifted_block << std::endl;

    std::cout << std::fixed << std::setprecision(5);
    std::ofstream outfile("/src/trajectory.txt");
    // for (const auto &config : result.path)
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

    //     // auto fka = Robot::eefk(soln);
    //     // std::cout <<std::endl << fka.matrix() <<std::endl;
    //     // std::cout << std::endl;
    //     outfile << "\n";
    // }


    Robot::ConfigurationArray holder;
    // typename Robot::template ConfigurationBlock<rake> block, projected_block, direction_vector_block;
    for (auto i = 0U; i < Robot::dimension; ++i){
        block[i] = Robot::Configuration(start).broadcast(i);
    }
    bimanual_task_constraint.print_robot_tsr_error(block);
    bool startsuccess = bimanual_task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
    std::cout << startsuccess <<std::endl;
    bool first = true;
    for (auto i = 0U; i < Robot::dimension; ++i){
        holder[i] = projected_block[{i, 0}];
        std::cout << holder[i] << ", ";
        if (!first) outfile << ",";
        outfile << holder[i];
        first = false;

    }
    outfile << "\n";
    std::cout << std::endl;
    // // Eigen::Quaternionf q(Robot::eefk(holder)[0].linear());
    // // std::cout << Robot::eefk(holder)[0].matrix() << std::endl;
    // // std::cout << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;

    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i);
    bimanual_task_constraint.print_robot_tsr_error(block);
    bool goalsuccess = bimanual_task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
    std::cout << goalsuccess << std::endl;

    first = true;
    for (auto i = 0U; i < Robot::dimension; ++i){
        holder[i] = projected_block[{i, 0}];
        std::cout << holder[i] << ", ";
        if (!first) outfile << ",";
        outfile << holder[i];
        first = false;

    }
    outfile << "\n";
    std::cout << std::endl;
    // std::cout << Robot::eefk(holder)[0].matrix() << std::endl;
    // Eigen::Quaternionf q2(Robot::eefk(holder)[0].linear());
    // std::cout << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << std::endl;

    auto vector2 = Robot::Configuration(start) - Robot::Configuration(goal);
    std::cout << vector2 << " with norm : " << vector2.l2_norm() << std::endl;
    std::vector<Robot::Configuration> projected_vector2;
    auto ret = vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(Robot::Configuration(start), Robot::Configuration(goal), projected_vector2, task_constraint, env_v);

    // Robot::Configuration start_plus_percent = {-1.289,  1.039,  1.079, -2.12 ,  0.553,  2.082, -1.181,  1.165, 1.11 , -1.048, -2.128, -0.5  ,  2.138, -0.202};
    Robot::ConfigurationArray test = {-1.289,  1.039,  1.079, -2.12 ,  0.553,  2.082, -1.181,  1.165, 1.11 , -1.048, -2.128, -0.5  ,  2.138, -0.202};


    for (auto i = 0U; i < Robot::dimension; ++i){
        block[i] = Robot::Configuration(test).broadcast(i);
    }
    std::cout << "\n\n Running project for a single config : \n\n";
    task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
    std::cout << std::endl;
    first = true;
    for (auto i = 0U; i < Robot::dimension; ++i){
        holder[i] = projected_block[{i, 0}];
        std::cout << holder[i] << ", ";
        if (!first) outfile << ",";
        outfile << holder[i];
        first = false;
    }
    outfile << "\n";
    std::cout << std::endl;


    // std::array<float, rake * Robot::dimension> diff_arr = {
    //     -0.675179, -0.707267, -0.739355, -0.771444, -0.803532, -0.83562, -0.867708, -0.899796,
    //     1.9179, 1.9202, 1.92249, 1.92478, 1.92707, 1.92937, 1.93166, 1.93395,
    //     -1.74784, -1.69885, -1.64987, -1.60088, -1.5519, -1.50291, -1.45392, -1.40494,
    //     1.29645, 1.29835, 1.30025, 1.30215, 1.30405, 1.30595, 1.30785, 1.30975,
    //     0.011661, 0.0471565, 0.0826521, 0.118148, 0.153643, 0.189139, 0.224634, 0.26013,
    //     -0.847029, -0.81709, -0.787152, -0.757214, -0.727276, -0.697338, -0.6674, -0.637461,
    //     -1.66942, -1.63468, -1.59994, -1.5652, -1.53045, -1.49571, -1.46097, -1.42623,
    //     0.701158, 0.68861, 0.676063, 0.663515, 0.650967, 0.638419, 0.625871, 0.613324,
    //     1.93658, 1.90565, 1.87472, 1.84379, 1.81285, 1.78192, 1.75099, 1.72006,
    //     1.68064, 1.63266, 1.58467, 1.53669, 1.48871, 1.44073, 1.39274, 1.34476,
    //     1.24989, 1.20248, 1.15508, 1.10767, 1.06027, 1.01286, 0.965455, 0.918049,
    //     0.150976, 0.138444, 0.125911, 0.113378, 0.100845, 0.0883125, 0.0757797, 0.063247,
    //     -0.935916, -0.937838, -0.93976, -0.941682, -0.943604, -0.945526, -0.947448, -0.94937,
    //     2.33129, 2.27653, 2.22176, 2.167, 2.11224, 2.05747, 2.00271, 1.94795 };

    // typename Robot::template ConfigurationBlock<rake> configblock = typename Robot::template ConfigurationBlock<rake>(diff_arr);
    // std::cout << configblock << std::endl;
    // std::cout << "-----------------" << std::endl;
    // bimanual_task_constraint.print_robot_tsr_error(block);
    // task_constraint.print_robot_tsr_error(block);
    // std::cout << "-----------------" << std::endl;

    // task_constraint.projectConfiguration(configblock, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);

    // for (auto r = 0U; r < rake; ++r){
    //     bool first = true;
    //     for (auto i = 0U; i < Robot::dimension; ++i){
    //         holder[i] = projected_block[{i, r}];
    //         std::cout << holder[i] << ", ";
    //         if (!first) outfile << ",";
    //         outfile << holder[i];
    //         first = false;
    //     }
    //     outfile << "\n";

    //     std::cout << std::endl;
    // }

    outfile.close();

    return 0;
}
