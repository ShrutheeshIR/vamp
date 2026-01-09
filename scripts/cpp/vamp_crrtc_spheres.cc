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
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>
#include "problem_setup/constrained_easier_cage_problem_setup.hh"

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;
using ConfigurationBlock = typename Robot::template ConfigurationBlock<rake>;
using Configuration = typename Robot::Configuration;
ConfigurationBlock turn_configuration_into_configuration_block(const Robot::Configuration &c) {
        typename Robot::template ConfigurationBlock<rake> block;

        for (auto i = 0U; i < Robot::dimension; ++i) {
            block[i] = c.broadcast(i) + 0.0;
        }
        return block;
}

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

// std::ostream& dump(std::ostream &o, const Attempt& a)
// {
//     return o << a.range << ", " << a.dynamic_domain << ", " << a.proj_method << ", " << a.descend_rate << ", " << a.planning_time/1e6 << ", " << a.planning_iterations << std::endl;
// }

auto main(int, char **) -> int
{


    // Setup RRTC and plan
    vamp::planning::RRTCSettings rrtc_settings;
    float ranges[] = {1.0};
    //float ranges[] = {0.5, 1.0, 1.5, 2.0};
    // float ranges[] = {0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0};
    bool dd[] = {false};
    vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM};

    // float descend_rates[] = {0.1, 0.25, 0.5, 0.75, 1.0};
    //float descend_rates[] = {0.75, 1.0};
    float descend_rates[] = {1.0};


    std::vector<Attempt> succ_attempts;

    for(const auto range: ranges){
        for(const auto dyndom: dd){
            for(const auto &pm: projection_method){
                for(const auto descent_rate: descend_rates){

                // if(pm < 2) continue;

    // Build sphere cage environment
    EnvironmentInput environment;
    std::ofstream outfile_sph("/src/spheres.txt");
    for (const auto &sphere : problem)
    {
        outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }
    outfile_sph.close();

    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    
    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(in_hand_pose, target_pose, std::make_pair(lower_bound, upper_bound));
    int x;


    //attempt to project start down to the manifold.
    

    std::vector<Configuration> projected_vector;
    Robot::ConfigurationArray zero_vector = {0.0,0.0,0.0,-0.0,0.0,0.0,0.0};
    bool success = vamp::planning::project_constraint_vector<Robot, rake, Robot::resolution>(
        Robot::Configuration(goal),
        Robot::Configuration(zero_vector),
        range,
        projected_vector,
        task_constraint,
        env_v
    );

    std::cout << "projection success: " << success << "\n";
    
    std::cout << "size of projected_vector is " << projected_vector.size();
    /*for(auto proj_vector : projected_vector)
    {

        for (std::size_t i = 0; i < Robot::dimension; ++i) {
            std::cout << proj_vector.broadcast(i) << " "; // extract scalar
        }
        std::cout << std::endl;
    }*/
    Robot::ConfigurationArray new_start = {-0.855292,1.12975,0.0380571,-0.69921,0.0141439,1.73613,0.0000393391};
    Robot::ConfigurationArray new_goal = {0.841046,1.13069,-0.0242863,-0.697919,-0.00982904,1.74158,0.000353813};
    auto block = turn_configuration_into_configuration_block(Robot::Configuration(new_start));
    auto simd_float_vector = task_constraint.distanceToConstraint(block);
    std::cout << "distanceToConstraint returned " << simd_float_vector << "\n";
    std::cout << "is the start position on the manifold?? Answer: " << simd_float_vector.test_all_less_equal(0.0001F) << "\n";
    block = turn_configuration_into_configuration_block(Robot::Configuration(new_goal));
    simd_float_vector = task_constraint.distanceToConstraint(block);
    std::cout << "distanceToConstraint returned " << simd_float_vector << "\n";
    std::cout << "is the end position on the manifold?? Answer: " << simd_float_vector.test_all_less_equal(0.0001F) << "\n";

    //THis is an intermediate result from ompl RRT connect
     Robot::ConfigurationArray intermed = {0.484433, 0.275282, -0.617731, -2.3295, 0.301566, 2.64362, -0.689576};
    block = turn_configuration_into_configuration_block(Robot::Configuration(intermed));
    simd_float_vector = task_constraint.distanceToConstraint(block);
    std::cout << "distanceToConstraint returned " << simd_float_vector << "\n";
    std::cout << "is the intermediate position on the manifold?? Answer: " << simd_float_vector.test_all_less_equal(0.0001F) << "\n";
    std::vector<Configuration> projected_vector_dummy;
    //Now, test motion between start and intermediate
    bool result_start_intermed = vamp::planning::project_constraint_motion<Robot, rake, 1>(
            Robot::Configuration(new_start),
            Robot::Configuration(intermed),
            projected_vector_dummy,
            task_constraint,
            env_v
    );
    std::cout << "The result of project_constraint_motion between start and intermed is " << result_start_intermed << "\n";
    bool result_intermed_goal = vamp::planning::project_constraint_motion<Robot, rake, 1>(
            Robot::Configuration(intermed),
            Robot::Configuration(new_goal),
            projected_vector_dummy,
            task_constraint,
            env_v
    );
    std::cout << "The result of project_constraint_motion between intermed and goal is " << result_intermed_goal << "\n";

    bool result1 = vamp::planning::project_constraint_motion<Robot, rake, 1>(
            Robot::Configuration(new_start),
            Robot::Configuration(new_goal),
            projected_vector,
            task_constraint,
            env_v
        );
        std::cout << "The result of project_constraint_motion is " << result1 << "\n";
    rrtc_settings.range = range;
    // rrtc_settings.max_iterations = 20000;
    rrtc_settings.dynamic_domain = dyndom;
    rrtc_settings.projection_method = pm;
    rrtc_settings.descend_rate = descent_rate;
    // std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
    std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";

    auto result =
        CRRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

    if(result.path.size() > 0)
    {
        Attempt a{
            range,
            dyndom,
            pm,
            descent_rate,
            true,
            result.nanoseconds,
            result.iterations,
            result.path.size()
        };


        if((succ_attempts.size() == 0) || (succ_attempts.size() > 0 && a < succ_attempts[0])){

        std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        std::ofstream outfile("/src/trajectory.txt");
        for (const auto &config : result.path)
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
            // std::cout << std::endl;
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
    std::cout << "------Final Result --------" << std::endl;
    std::sort(succ_attempts.rbegin(), succ_attempts.rend());
    for (const auto &a : succ_attempts) {
        std::cout << a.range << ", " << a.dynamic_domain << ", " << a.proj_method << ", " << a.descend_rate << ", " << a.planning_time/1e6 << ", " << a.planning_iterations << ", " << a.path_length << std::endl;

    }
    std::cout << "---------------------------" << std::endl;



    return 0;
}
