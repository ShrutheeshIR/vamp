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
#include "vamp/parsers/mjcf_parser.hh"


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
// static constexpr Robot::ConfigurationArray box_top_shelf_pickup = {0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31794,-0.01125,-0.28643,0.11635,-0.00983,-0.15597,-0.29369,-0.00103,0.35552,-0.36544,0.00508,-0.29158,-0.32097,0.00882,0.28598,-0.11670,0.01291,-0.12971,0.29191,0.02145,-0.35453};
// static constexpr Robot::ConfigurationArray rack_2 = {0.00305,0.01553,-0.48345,-0.01069,0.00173,-0.00299,0.36991,0.00741,-0.22885,-0.85230,-0.02549,0.90910,-0.43356,0.01072,-0.02876,-0.17999,0.16228,0.99774,-0.33776,0.01561,0.22590,0.84363,0.02173,-0.90309,0.42748,0.06021,-0.16520,0.35173,-0.06290,-1.30637};

static constexpr Robot::ConfigurationArray box_top_shelf_pickup = {
    0.01535,0.00223,-0.00520,-0.00184,0.00960,0.00021,0.36475,-0.00394,0.29881,0.31744,-0.00933,-0.28854,0.11234,-0.00974,0.08865,-0.26356,-0.00642,0.11516,-0.36586,0.00529,-0.29317,-0.32211,0.00711,0.28940,-0.09870,0.01280,0.16197,0.27678,0.06764,-0.09442
};

static constexpr Robot::ConfigurationArray rack_2 = {
    0.00649,0.01494,-0.48312,-0.01123,0.00484,-0.00190,0.37187,0.00985,-0.22625,-0.85267,-0.02488,0.90858,-0.43197,0.01244,-0.17075,-0.36408,0.06200,1.08998,-0.33958,0.01487,0.22498,0.84377,0.02152,-0.90293,0.43374,0.05814,-0.29661,0.37934,-0.15480,-1.16806
};

static constexpr Robot::ConfigurationArray rack_3 = {
    0.01172,0.00883,-0.28551,-0.00360,-0.00775,-0.00259,0.36978,0.00088,-0.01740,-0.40609,-0.03825,0.47305,-0.23680,-0.00017,-0.09672,-0.29005,-0.02483,0.49941,-0.35131,0.00681,0.01679,0.40305,0.03835,-0.47680,0.23502,0.03439,-0.07496,0.30373,-0.05558,-0.51322
};


template <typename... Constraints>
inline auto compute_plan(
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
        std::cout << "Planning took " << result.nanoseconds / 1e6 << " Simplify took " << simplify_result.nanoseconds / 1e6 << " ms" << std::endl;
        simplify_result.path.interpolate_to_resolution(Robot::resolution);
        path_out = std::move(simplify_result.path);
        return std::make_pair(true, result.nanoseconds);
    }
    return std::make_pair(false, result.nanoseconds);
}



int main(){
    std::array<float, 8> polygon_points = {
        0.01, -0.06, 0.01, 0.06, -0.04, 0.06, -0.04, -0.06
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
        -0.001, -0.001, -0.001, -0.01, -0.01, -0.01, 
        -0.001, -0.001, -0.001, -0.01, -0.01, -0.01
    };

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 
        0.001, 0.001, 0.001, 0.01, 0.01, 0.01
    };



    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{{1, 0,0,0,   0, 0, 0}, {1, 0,0,0,   0.0, 0.0, 0.0}, {0.603, 0.36, 0.36, 0.603, -0.04302,  0.10080, -0.96013}, {0.603, -0.36, 0.36, -0.603 , -0.04288, -0.09895, -0.96033}}};
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
        vamp::planning::BimanualTaskSpaceConstraint<Robot, rake>>transport_task_constraint(
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


    vamp::planning::RRTCSettings rrtc_settings;


    struct Attempt {
        float range;
        bool dynamic_domain;
        vamp::planning::ProjMethod proj_method;
        float descend_rate;
        int num_projection_iterations;
        bool insert_all_to_tree;
        bool success;
        std::size_t planning_time;
        std::size_t planning_iterations;
        std::size_t path_length;

        bool operator<(const Attempt& other) const {
            return planning_time < other.planning_time;
        }
    };

    std::vector<Attempt> succ_attempts;

    float ranges[] = {0.75, 0.5};
    bool dd[] = {false, true};
    vamp::planning::ProjMethod projection_method[] = {vamp::planning::ProjMethod::InnerLM};

    float descend_rates[] = {0.75, 1.0};
    int num_projection_iterations[] = {10, 25};
    bool insert_all_to_tree[] = {true};

    // float descend_rates[] = {1.0};
    for(const auto range: ranges){
        for(const auto dyndom: dd){
            for(const auto &pm: projection_method){
                for(const auto descent_rate: descend_rates){
                    for(const auto num_projection_iterations: num_projection_iterations){
                        for(const auto insert_all_to_tree: insert_all_to_tree){
                            rrtc_settings.range = range;
                            rrtc_settings.max_iterations = 100000;
                            rrtc_settings.dynamic_domain = dyndom;
                            rrtc_settings.projection_method = pm;
                            rrtc_settings.descend_rate = descent_rate;

                            rrtc_settings.radius = 10.0;
                            rrtc_settings.num_projection_iterations = num_projection_iterations;
                            rrtc_settings.insert_all_to_tree = insert_all_to_tree;
                            // std::cout << "\n\n-----------------Starting to cbirrt------------ " << std::endl;
                            // std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";
                            vamp::planning::invalid_distance_counter_outside = 0;
                            vamp::planning::invalid_distance_counter_inside = 0;
                            vamp::planning::collision_counter = 0;
                            vamp::planning::unable_to_project_counter = 0;


                            EnvironmentInput environment;
                            auto geoms = vamp::utils::parser::parseMJCF("/src/myfork/vamp/resources/environments/cuboids/wooden_shelf.xml");
                            for (const auto& g : geoms) {
                                if (g.type == vamp::utils::parser::GeomType::BOX){
                                    std::cout << "Adding cuboid with pos " << g.world_pose.pos.x << "," << g.world_pose.pos.y << "," << g.world_pose.pos.z << " and size " << g.size.x << "," << g.size.y << "," << g.size.z << std::endl;
                                    environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array({g.world_pose.pos.x, g.world_pose.pos.y, g.world_pose.pos.z}, {0.0, 0.0, 0.0}, {g.size.x, g.size.y, g.size.z}));
                                }

                            }

                            environment.sort();
                            auto env_v = EnvironmentVector(environment);


                            // plan from standing to box pickup with non-transport constraints
                            // Create RNG for planning
                            auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
                            std::vector<Robot::Configuration> path_non_transport;
                            auto [success_non_transport, non_transport_ns] = compute_plan<vamp::planning::FeetTaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>,vamp::planning::ClosedLinkConstraint<Robot, rake>>(
                                standing_pose,
                                box_top_shelf_pickup,
                                env_v,
                                rrtc_settings,
                                non_transport_task_constraint,
                                path_non_transport,
                                rng
                            );
                            if (success_non_transport){
                                // std::cout << "Found non-transport path of length " << path_non_transport.size() << std::endl;
                                ;
                            } else {
                                std::cout << "Failed to find non-transport path" << std::endl;
                                break;
                            }


                            vamp::planning::invalid_distance_counter_outside = 0;
                            vamp::planning::invalid_distance_counter_inside = 0;
                            vamp::planning::collision_counter = 0;
                            vamp::planning::unable_to_project_counter = 0;

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

                            attachment.spheres.insert(attachment.spheres.end(), spheres.cbegin(), spheres.cend());
                            environment.attach(attachment, 0);
                            environment.sort();
                            env_v = EnvironmentVector(environment);


                            // plan from box pickup to rack 2 with transport constraints
                            std::vector<Robot::Configuration> path_transport;
                            auto [success_transport, transport_ns] = compute_plan<vamp::planning::FeetTaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>,vamp::planning::ClosedLinkConstraint<Robot, rake>, vamp::planning::BimanualTaskSpaceConstraint<Robot, rake>>(
                                box_top_shelf_pickup,
                                rack_2,
                                env_v,
                                rrtc_settings,
                                transport_task_constraint,
                                path_transport,
                                rng
                            );
                            if (success_transport){
                                // std::cout << "Found transport path of length " << path_transport.size() << std::endl;
                                ;
                            } else {
                                std::cout << "Failed to find transport path" << std::endl;
                                break;
                            }
                            environment.detach(0);

                            std::vector<Robot::Configuration> path_non_transport_2;
                            auto [success_non_transport_2, non_transport_ns_2] = compute_plan<vamp::planning::FeetTaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>,vamp::planning::ClosedLinkConstraint<Robot, rake>>(
                                rack_2,
                                rack_3,
                                env_v,
                                rrtc_settings,
                                non_transport_task_constraint,
                                path_non_transport_2,
                                rng
                            );
                            if (success_non_transport_2){
                                // std::cout << "Found non-transport path of length " << path_non_transport.size() << std::endl;
                                ;
                            } else {
                                std::cout << "Failed to find non-transport path" << std::endl;
                                break;
                            }

                            attach_transform = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
                            attachment = AttachmentInput(attach_transform);

                            attachment.spheres.insert(attachment.spheres.end(), spheres.cbegin(), spheres.cend());
                            environment.attach(attachment, 0);
                            environment.sort();
                            env_v = EnvironmentVector(environment);


                            //  plan to pickup from rack 2 to rack 3 with transport constraints to validate that we can plan with the attachment in the environment

                            std::vector<Robot::Configuration> path_transport_2;
                            auto [success_transport_2, transport_ns_2] = compute_plan<vamp::planning::FeetTaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>,vamp::planning::ClosedLinkConstraint<Robot, rake>, vamp::planning::BimanualTaskSpaceConstraint<Robot, rake>>(
                                rack_3,
                                box_top_shelf_pickup,
                                env_v,
                                rrtc_settings,
                                transport_task_constraint,
                                path_transport_2,
                                rng
                            );
                            if (success_transport_2){
                                // std::cout << "Found transport path of length " << path_transport.size() << std::endl;
                                ;
                            } else {
                                std::cout << "Failed to find transport path" << std::endl;
                                break;
                            }


                            environment.detach(0);

                            environment.sort();
                            env_v = EnvironmentVector(environment);

                            vamp::planning::invalid_distance_counter_outside = 0;
                            vamp::planning::invalid_distance_counter_inside = 0;
                            vamp::planning::collision_counter = 0;
                            vamp::planning::unable_to_project_counter = 0;


                            // // plan from rack 2 to standing with non-transport constraints
                            std::vector<Robot::Configuration> path_non_transport_3;
                            auto [success_non_transport_3, non_transport_ns_3] = compute_plan<vamp::planning::FeetTaskSpaceConstraint<Robot, rake>, vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4>,vamp::planning::ClosedLinkConstraint<Robot, rake>>(
                                box_top_shelf_pickup,
                                standing_pose,
                                env_v,
                                rrtc_settings,
                                non_transport_task_constraint,
                                path_non_transport_3,
                                rng
                            );
                            if (success_non_transport_3){


                                // std::cout << "Found non-transport path back to standing of length " << path_non_transport_2.size() << std::endl;
                                // now combine the paths from all 3 segments and validate the entire trajectory with all constraints
                                std::vector<Robot::Configuration> full_trajectory;
                                full_trajectory.insert(full_trajectory.end(), path_non_transport.begin(), path_non_transport.end());
                                full_trajectory.insert(full_trajectory.end(), path_transport.begin(), path_transport.end());
                                full_trajectory.insert(full_trajectory.end(), path_non_transport_2.begin(), path_non_transport_2.end());
                                full_trajectory.insert(full_trajectory.end(), path_transport_2.begin(), path_transport_2.end());
                                full_trajectory.insert(full_trajectory.end(), path_non_transport_3.begin(), path_non_transport_3.end());

                                Attempt a{
                                    range,
                                    dyndom,
                                    pm,
                                    descent_rate,
                                    num_projection_iterations,
                                    insert_all_to_tree,
                                    true,
                                    non_transport_ns + transport_ns + transport_ns_2 + non_transport_ns_2 + non_transport_ns_3,
                                    0,
                                    full_trajectory.size()
                                };


                                if((succ_attempts.size() == 0) || (succ_attempts.size() > 0 && a < succ_attempts[0])){
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
                                }
                                succ_attempts.push_back(a);
                                std::sort(succ_attempts.begin(), succ_attempts.end());



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

    std::cout << "------Final Result --------" << std::endl;
    std::sort(succ_attempts.rbegin(), succ_attempts.rend());
    for (const auto &a : succ_attempts) {
        std::cout << a.range << ", " << a.dynamic_domain << ", " << a.proj_method << ", " << a.descend_rate << ", " << a.num_projection_iterations << ", " << a.insert_all_to_tree << ", " << a.planning_time/1e6 << ", " << a.planning_iterations << ", " << a.path_length << std::endl;

    }
    std::cout << "---------------------------" << std::endl;

    return 1;





}