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

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;


static constexpr Robot::ConfigurationArray start = {0.88,1.05,0.0,-0.66,0.0,1.73,0.0};
static constexpr Robot::ConfigurationArray goal = {-0.92,1.05,0.0,-0.66,0.0,1.73,0.0};



// static constexpr Robot::ConfigurationArray goal = {-0.839708,  0.496555, -0.630832, -0.573204,  0.232247,  1.8259,   -0.467584};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.55, 0, 0.50},
    {0.55, 0, 0.60},
    {0.65, 0, 0.50},
    {0.75, 0, 0.50},
    {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, -0.35, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
};
// Radius for obstacle spheres
static constexpr float radius = 0.15;

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

    float ranges[] = {0.5, 1.0, 1.5, 2.0};
    // float ranges[] = {0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0};
    bool dd[] = {false, true};
    vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM};

    // float descend_rates[] = {0.1, 0.25, 0.5, 0.75, 1.0};
    float descend_rates[] = {0.75, 1.0};
    // float descend_rates[] = {1.0};


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


    std::array<float, 6> lower_bound = {
        -10.01, -10.01, -0.03, -0.1, -0.1, -3.14
    };
    std::array<float, 6> upper_bound = {
        10.03, 10.01, 0.03, 0.1, 0.1, 3.14
    };

    Eigen::Matrix<float, 4, 4> T;
    T << 1,0,0,   0.543325,   0,-1,0,      0.570738,   0,0,-1,    0.121557,          0,           0,           0,           1;

    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(in_hand_pose, target_pose, std::make_pair(lower_bound, upper_bound));
    int x;


    
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
