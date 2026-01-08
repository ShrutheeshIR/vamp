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
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891};
static constexpr Robot::ConfigurationArray goal = {-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24};

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
        -0.001, -10.01, -0.001, -10.1, -0.1, -10.1
    };
    std::array<float, 6> tsr_upper_bound = {
        0.001, 10.01, 0.001, 10.1, 0.1, 10.1
    };


    std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms;
    Eigen::Matrix<float, 4, 4> T;
    T << 1,0,0,   0.50,   0,-1,0,      0.570738,   0,0,-1,    0.121557,          0,           0,           0,           1;;
    eef_transforms[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms_ref_frame_w_world;
    T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        std::make_pair(tsr_lower_bound, tsr_upper_bound)
    );

    auto start_time = std::chrono::steady_clock::now();


    float delta[] = {-1.0, -0.5, -0.3, -0.2, -0.1, 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.5, 0.1, 0.2, 0.3, 0.5, 1.0};

    for(auto del_ind = 0U; del_ind < 17; del_ind++){
        ;
        // typename Robot::template ConfigurationBlock<rake> block;
        // for (auto i = 0U; i < Robot::dimension; ++i)
        //     block[i] = Robot::Configuration(goal).broadcast(i) + delta[del_ind];

        // start_time = std::chrono::steady_clock::now();
        // auto dist = task_constraint.distanceToConstraint(block);
        // // std::cout << "Dist to constraint" << "->" << vamp::utils::get_elapsed_nanoseconds(start_time);
        // // std::cout << "From block : " << dist << std::endl;
        // // task_constraint.print_robot_tsr_error(block);

        // typename Robot::template ConfigurationBlock<rake> projected_block;
        // bool success;


        // std::cout << " Output of project step inner ";
        // start_time = std::chrono::steady_clock::now();
        // task_constraint.projectStep(block, projected_block);
        // // for(auto i=0U; i < Robot::dimension; i++)
        // //     std::cout << projected_block[{i, 0}] << " ";
        // std::cout << "->" << vamp::utils::get_elapsed_nanoseconds(start_time);

        // std::cout << " Output of project step outer ";
        // start_time = std::chrono::steady_clock::now();
        // task_constraint.projectStep(block, projected_block, vamp::planning::ProjMethod::OuterLM);
        // // for(auto i=0U; i < Robot::dimension; i++)
        // //     std::cout << projected_block[{i, 0}] << " ";
        // std::cout << "->" << vamp::utils::get_elapsed_nanoseconds(start_time);

        // std::cout << " Output of project step Jac ";
        // start_time = std::chrono::steady_clock::now();
        // task_constraint.projectStep(block, projected_block, vamp::planning::ProjMethod::GradDesc);
        // for(auto i=0U; i < Robot::dimension; i++)
        //     std::cout << projected_block[{i, 0}] << " ";
        // // std::cout << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << " " << std::endl;
        // std::cout << " " << std::endl;

        // start_time = std::chrono::steady_clock::now();
        // success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::OuterLM);
        // // for(auto i=0U; i < Robot::dimension; i++)
        // //     std::cout << projected_block[{i, 0}] << " ";
        // std::cout << "Outer " << success << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << " ";

        // start_time = std::chrono::steady_clock::now();
        // success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM);
        // // for(auto i=0U; i < Robot::dimension; i++)
        // //     std::cout << projected_block[{i, 0}] << " ";
        // std::cout << "Inner " << success << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << " ";

        // start_time = std::chrono::steady_clock::now();
        // success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::GradDesc);
        // for(auto i=0U; i < Robot::dimension; i++)
        //     std::cout << projected_block[{i, 0}] << " ";
        // // std::cout << std::endl;
        // // std::cout << "Jac " << success << "->" << vamp::utils::get_elapsed_nanoseconds(start_time) << std::endl;
        // std::cout << success << " " << std::endl;

    }

    // Robot::ConfigurationArray test = {-0.154247,-1.38187,1.88281,-1.42206,-0.691598,3.52511,1.43219};
    // typename Robot::template ConfigurationBlock<rake> block;
    // for (auto i = 0U; i < Robot::dimension; ++i)
    //     block[i] = Robot::Configuration(test).broadcast(i) + 0.0;
    // auto dist = task_constraint.distanceToConstraint(block);
    // // std::cout << "Dist to constraint" << "->" << vamp::utils::get_elapsed_nanoseconds(start_time);
    // std::cout << "From block : " << dist << std::endl;
    // task_constraint.print_robot_tsr_error(block);
    Eigen::Quaternionf qstart(Robot::eefk(start)[0].linear());
    Eigen::Quaternionf qgoal(Robot::eefk(goal)[0].linear());
    std::cout << Robot::eefk(start)[0].matrix() << std::endl;
    std::cout << Robot::eefk(goal)[0].matrix() << std::endl;
    std::cout << qstart.w() << ", " << qstart.x() << ", " << qstart.y() << ", " << qstart.z() << std::endl;
    std::cout << qgoal.w() << ", " << qgoal.x() << ", " << qgoal.y() << ", " << qgoal.z() << std::endl;

    Robot::ConfigurationArray holder;
    typename Robot::template ConfigurationBlock<rake> block, projected_block;
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(start).broadcast(i);
    task_constraint.projectConfiguration(block, projected_block);
    std::cout << std::endl;
    for (auto i = 0U; i < Robot::dimension; ++i){
        holder[i] = projected_block[{i, 0}];
        std::cout << holder[i] << ", ";
    }
    std::cout << std::endl;
    Eigen::Quaternionf q(Robot::eefk(holder)[0].linear());
    std::cout << Robot::eefk(holder)[0].matrix() << std::endl;
    std::cout << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;

    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i);
    task_constraint.projectConfiguration(block, projected_block);
    std::cout << std::endl;
    for (auto i = 0U; i < Robot::dimension; ++i){
        holder[i] = projected_block[{i, 0}];
        std::cout << holder[i] << ", ";
    }
    std::cout << std::endl;
    std::cout << Robot::eefk(holder)[0].matrix() << std::endl;
    Eigen::Quaternionf q2(Robot::eefk(holder)[0].linear());
    std::cout << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << std::endl;


    return 0;
}
