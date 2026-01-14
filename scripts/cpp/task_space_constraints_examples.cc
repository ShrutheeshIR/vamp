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

#include <vamp/planning/simplify.hh>
#include <vamp/robots/bimanual_panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::BimanualPanda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Start and goal configurations
// static constexpr Robot::ConfigurationArray start = {1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891};
// static constexpr Robot::ConfigurationArray goal = {-0.154247,-1.38187,1.88281,-1.42206,-0.691598,3.52511,1.43219};

static constexpr Robot::ConfigurationArray start = {-1.334, 1.550, 0.960, -2.372, 0.230, 2.632, 1.963, 1.330, 1.555, -0.957, -2.412, -0.232, 2.622, -0.325 };
static constexpr Robot::ConfigurationArray goal = {-1.951, 0.078, 1.857, -1.970, 1.473, 1.657, 2.161, 1.019, 0.509, -1.371, -1.750, -1.111, 1.985, -0.609};

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
        -0.02, -0.02, -0.02, -0.1, -0.1, -0.1
    };
    std::array<float, 6> upper_bound = {
        0.02, 0.02, 0.02, 0.1, 0.1, 0.1
    };


    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;

    std::array<float, 7> transform = {0.00, 1.0, 0.00, 0.00, 0.0, 0.0, 0.171814};
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
    // std::cout << qstart.w() << ", " << qstart.x() << ", " << qstart.y() << ", " << qstart.z() << std::endl;
    // std::cout << qgoal.w() << ", " << qgoal.x() << ", " << qgoal.y() << ", " << qgoal.z() << std::endl;


    // auto vector = Robot::Configuration(goal) - Robot::Configuration(start);

    // auto vector_norm = vector.l2_norm();
    // auto distance = vector_norm;
    // vector = vector / vector_norm;
    // typename Robot::template ConfigurationBlock<rake> block, projected_block, direction_vector_block, initial_projected_block;
    // // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
    // const auto percents = vamp::FloatVector<rake>(vamp::planning::Percents<rake>::percents);

    // for (auto i = 0U; i < Robot::dimension; ++i)
    // {
    //     block[i] = Robot::Configuration(start).broadcast(i) + (vector.broadcast(i) * percents);
    //     direction_vector_block[i] = vector.broadcast(i);
    // }

    // std::cout << "Block values: ";
    // for (auto i = 0U; i < Robot::dimension; ++i){
    //     std::cout << block[{i, 0}] << ", ";
    // }
    // std::cout << std::endl;


    // bool ableToProject = task_constraint.projectConfiguration(block, initial_projected_block, vamp::planning::ProjMethod::InnerLM, vector_norm, 1.0);

    // std::cout << std::endl;

    // for (auto i = 0U; i < rake-1; i++)
    // {
    //     float inter_distance = 0.F;
    //     for (auto j = 0U; j < Robot::dimension; j++)
    //     {
    //         // std::cout << i << " " << j << " " << initial_projected_block[{j, i+1}] << ", " << initial_projected_block[{j, i}] << " " << initial_projected_block[{j, i+1}] - initial_projected_block[{j, i}] << std::endl;
    //         inter_distance = inter_distance + std::pow(initial_projected_block[{j, i+1}] - initial_projected_block[{j, i}], 2);
    //     }
    //     std::cout << "Distance between points " << i << " and " << i+1 << ": " << std::sqrt(inter_distance) << " " << (distance / rake)<< std::endl;
    //     // if (inter_distance > (distance / rake) * (distance / rake))
    //     // {
    //     //     return false;
    //     // }
    // }

    // // auto diff_block = configuration_block_difference(initial_projected_block, Robot::Configuration(start));


    // std::array<vamp::FloatT, Robot::dimension * rake> diff_arr;
    // std::cout <<diff_arr.size() << std::endl;

    // for (auto j = 0U; j < Robot::dimension; j++)
    //     // diff_arr[j * rake] = 0.0;
    //     diff_arr[j * rake] = initial_projected_block[{j, 0}] - Robot::Configuration(start).broadcast(j)[{j, 0}];

    // // Dimensions are the rows and rake are the columns
    // // for (auto i = 1U; i < rake; i++)
    // // {
    // //     for (auto j = 0U; j < Robot::dimension; j++)
    // //     {
    // //         diff_arr[i * Robot::dimension + j] = initial_projected_block[{j, i}] - initial_projected_block[{j, i-1}];
    // //         // std::cout << j << " " << i << " " << diff_arr[j + i * Robot::dimension] << std::endl;
    // //     }
    // // }

    // for (auto i = 1U; i < rake; i++)
    // {
    //     for (auto j = 0U; j < Robot::dimension; j++)
    //     {
    //         diff_arr[i + j * rake] = initial_projected_block[{j, i}] - initial_projected_block[{j, i-1}];
    //         // std::cout << j << " " << i << " " << diff_arr[j + i * Robot::dimension] << std::endl;
    //     }
    // }

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

    std::cout << std::fixed << std::setprecision(3);
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
    typename Robot::template ConfigurationBlock<rake> block, projected_block, direction_vector_block;
    for (auto i = 0U; i < Robot::dimension; ++i){
        block[i] = Robot::Configuration(start).broadcast(i);
    }
    bimanual_task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
    std::cout << std::endl;
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
    bimanual_task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
    std::cout << std::endl;

    first = true;
    for (auto i = 0U; i < Robot::dimension; ++i){
        holder[i] = projected_block[{i, 0}];
        std::cout << holder[i] << ", ";
        if (!first) outfile << ",";
        outfile << holder[i];
        first = false;

    }
    std::cout << std::endl;
    // std::cout << Robot::eefk(holder)[0].matrix() << std::endl;
    // Eigen::Quaternionf q2(Robot::eefk(holder)[0].linear());
    // std::cout << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << std::endl;

    outfile.close();

    return 0;
}
