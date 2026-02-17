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
using AttachmentInput = vamp::collision::Attachment<float>;

static constexpr Robot::ConfigurationArray standing_pose = {0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31631,-0.01347,-0.28807,0.11635,-0.00983,-0.10504,0.88852,-0.00624,0.37778,-0.36544,0.00508,-0.29158,-0.31758,0.01345,0.28939,-0.11670,0.01291,0.10456,-0.89396,0.00550,-0.36663};
static constexpr Robot::ConfigurationArray box_top_shelf_pickup = {0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31794,-0.01125,-0.28643,0.11635,-0.00983,-0.15597,-0.29369,-0.00103,0.35552,-0.36544,0.00508,-0.29158,-0.32097,0.00882,0.28598,-0.11670,0.01291,-0.12971,0.29191,0.02145,-0.35453};
static constexpr Robot::ConfigurationArray rack_2 = {0.00305,0.01553,-0.48345,-0.01069,0.00173,-0.00299,0.36991,0.00741,-0.22885,-0.85230,-0.02549,0.90910,-0.43356,0.01072,-0.02876,-0.17999,0.16228,0.99774,-0.33776,0.01561,0.22590,0.84363,0.02173,-0.90309,0.42748,0.06021,-0.16520,0.35173,-0.06290,-1.30637};

template <typename... Constraints>
bool compute_plan(
    const Robot::ConfigurationArray &start,
    const Robot::ConfigurationArray &goal,
    EnvironmentVector &env_v,
    vamp::planning::RRTCSettings &rrtc_settings,
    vamp::planning::ComposableConstraints<Robot, rake, Constraints...> &task_constraint,
    std::vector<Robot::Configuration>& path_out,
    vamp::rng::RNG<Robot>::Ptr rng
){
    auto result =
        vamp::planning::CRRTC<Robot, rake, Robot::resolution, Constraints...>::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);
    if(result.path.size() > 0)
    {

        vamp::planning::SimplifySettings simplify_settings;
        // auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, decltype(feet_tsr_constraint), decltype(com_constraint), decltype(bimanual_task_constraint), decltype(closed_link_constraint)>(
        auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, Constraints...>(
            result.path, env_v, task_constraint, simplify_settings, rng);
        std::cout << "Planning took " << result.nanoseconds << " Simplify took " << simplify_result.nanoseconds / 1e6 << " ms" << std::endl;
        simplify_result.path.interpolate_to_resolution(Robot::resolution);
        path_out = std::move(simplify_result.path);
        return true;
    }
    return false;
}



int main(){
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

    std::array<float, 7> transform = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -0.36356};
    vamp::planning::BimanualTaskSpaceConstraint<Robot, rake> bimanual_task_constraint(transform, lower_bound, upper_bound);
    vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(
        polygon_points
    );

    vamp::planning::ClosedLinkConstraint<Robot, rake> closed_link_constraint;

    vamp::planning::ComposableConstraints<Robot, rake, 
        vamp::planning::TaskSpaceConstraint<Robot, rake>,
        vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>, 
        vamp::planning::ClosedLinkConstraint<Robot, rake>, 
        vamp::planning::BimanualTaskSpaceConstraint<Robot, rake>>transport_task_constraint(
        feet_tsr_constraint,
        com_constraint,
        closed_link_constraint,
        bimanual_task_constraint
    );

    vamp::planning::ComposableConstraints<Robot, rake, 
        vamp::planning::TaskSpaceConstraint<Robot, rake>,
        vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>, 
        vamp::planning::ClosedLinkConstraint<Robot, rake>>non_transport_task_constraint(
        feet_tsr_constraint,
        com_constraint,
        closed_link_constraint
    );


    vamp::planning::RRTCSettings rrtc_settings;

    float ranges[] = {0.5, 0.75, 1.0, 1.5, 2.0};
    bool dd[] = {false, true};
    vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM, vamp::planning::ProjMethod::OuterLM};

    float descend_rates[] = {0.75, 1.0};
    int num_projection_iterations[] = {5, 10, 25, 50, 100};
    bool insert_all_to_tree[] = {false, true};

    // float descend_rates[] = {1.0};
    for(const auto range: ranges){
        for(const auto dyndom: dd){
            for(const auto &pm: projection_method){
                for(const auto descent_rate: descend_rates){
                    for(const auto num_projection_iterations: num_projection_iterations){
                        for(const auto insert_all_to_tree: insert_all_to_tree){

                            EnvironmentInput environment;
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
                                // environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({x, y, z}, {0.0, 0.0, 0.0}, {dx/2, dy/2, dz/2}));
                            }
                            infile.close();
                            environment.sort();
                            auto env_v = EnvironmentVector(environment);


                            // plan from standing to box pickup with non-transport constraints
                            // Create RNG for planning
                            auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
                            std::vector<Robot::Configuration> path_non_transport;
                            bool success_non_transport = compute_plan<vamp::planning::TaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>,vamp::planning::ClosedLinkConstraint<Robot, rake>>(
                                standing_pose,
                                box_top_shelf_pickup,
                                env_v,
                                rrtc_settings,
                                non_transport_task_constraint,
                                path_non_transport,
                                rng
                            );
                            if (success_non_transport){
                                std::cout << "Found non-transport path of length " << path_non_transport.size() << std::endl;
                            } else {
                                std::cout << "Failed to find non-transport path" << std::endl;
                                break;
                            }

                            // plan from box pickup to rack 2 with transport constraints
                            std::vector<Robot::Configuration> path_transport;
                            bool success_transport = compute_plan<vamp::planning::TaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>,vamp::planning::ClosedLinkConstraint<Robot, rake>, vamp::planning::BimanualTaskSpaceConstraint<Robot, rake>>(
                                box_top_shelf_pickup,
                                rack_2,
                                env_v,
                                rrtc_settings,
                                transport_task_constraint,
                                path_transport,
                                rng
                            );
                            if (success_transport){
                                std::cout << "Found transport path of length " << path_transport.size() << std::endl;
                            } else {
                                std::cout << "Failed to find transport path" << std::endl;
                                break;
                            }

                            // // plan from rack 2 to standing with non-transport constraints
                            std::vector<Robot::Configuration> path_non_transport_2;
                            bool success_non_transport_2 = compute_plan<vamp::planning::TaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>,vamp::planning::ClosedLinkConstraint<Robot, rake>>(
                                rack_2,
                                standing_pose,
                                env_v,
                                rrtc_settings,
                                non_transport_task_constraint,
                                path_non_transport_2,
                                rng
                            );
                            if (success_non_transport_2){
                                std::cout << "Found non-transport path back to standing of length " << path_non_transport_2.size() << std::endl;
                                // now combine the paths from all 3 segments and validate the entire trajectory with all constraints
                                std::vector<Robot::Configuration> full_trajectory;
                                full_trajectory.insert(full_trajectory.end(), path_non_transport.begin(), path_non_transport.end());
                                full_trajectory.insert(full_trajectory.end(), path_transport.begin(), path_transport.end());
                                full_trajectory.insert(full_trajectory.end(), path_non_transport_2.begin(), path_non_transport_2.end());

                                std::cout << std::fixed << std::setprecision(3);
                                std::ofstream outfile("/src/trajectory.txt");
                                for (const auto &config : full_trajectory)
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
                                    outfile << "\n";
                                }
                                outfile.close();



                                return 1;
                            } else {
                                std::cout << "Failed to find non-transport path back to standing" << std::endl;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    return 1;





}