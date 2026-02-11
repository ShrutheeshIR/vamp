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
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

#include <sstream>
#include <string>
#include <nlohmann/json.hpp>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using AttachmentInput = vamp::collision::Attachment<float>;

// Start and goal configurations
// static constexpr Robot::ConfigurationArray start = {1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891};
// static constexpr Robot::ConfigurationArray goal = {-0.154247,-1.38187,1.88281,-1.42206,-0.691598,3.52511,1.43219};

// static constexpr Robot::ConfigurationArray start = {-1.314, 1.339, 1.095, -2.495, 0.513, 2.551, -1.495, 1.294, 1.309, -1.055, -2.493, -0.713, 2.500, -3.014 };
// static constexpr Robot::ConfigurationArray goal = {-1.068, 0.602, 1.329, -1.797, 1.059, 1.959, -1.149, 1.033, 0.446, -1.224, -1.837, -1.286, 1.917, 2.813};

static constexpr Robot::ConfigurationArray start = {-1.45508 , -0.718918,  2.46299 , -0.29207 , -0.396585,  2.28344 , -0.613603};
static constexpr Robot::ConfigurationArray goal = {0.2013, 0.0540317, -2.7026, -0.138382, -0.902814, 3.35919, -1.78867};

static bool load_cuboids_from_json(EnvironmentInput &environment, const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return false;
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception &e) {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return false;
    }

    if (!j.is_array()) {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return false;
    }

    for (const auto &obj : j) {
        if (!obj.is_object()) {
            std::cerr << "Skipping non-object element in array" << std::endl;
            continue;
        }

        // required fields
        if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z") ||
            !obj.contains("dx") || !obj.contains("dy") || !obj.contains("dz")) {
            std::cerr << "Skipping object missing required fields (x,y,z,dx,dy,dz)" << std::endl;
            continue;
        }

        try {
            float x = obj.at("x").get<float>();
            float y = obj.at("y").get<float>();
            float z = obj.at("z").get<float>();
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>();

            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (obj.contains("roll")) roll = obj.at("roll").get<float>();
            if (obj.contains("pitch")) pitch = obj.at("pitch").get<float>();
            if (obj.contains("yaw")) yaw = obj.at("yaw").get<float>();

            std::array<float,3> posf = {x, y, z};
            std::array<float,3> rotf = {roll, pitch, yaw};
            std::array<float,3> sizef = {dx / 2, dy / 2, dz / 2};
            // std::cout << "Creating cuboid at position: " << posf[0] << ", " << posf[1] << ", " << posf[2] << " with size: " << sizef[0] << ", " << sizef[1] << ", " << sizef[2] << std::endl;
            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(posf, rotf, sizef));
        } catch (const std::exception &e) {
            std::cerr << "Error reading object fields: " << e.what() << " -- skipping object" << std::endl;
            continue;
        }
    }

    return true;
}


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

    // Try a few likely paths for the JSON file
    const std::vector<std::string> candidate_paths = {
        "/src/myfork/vamp/resources/environments/cuboids/real_maze.json",
    };

    bool loaded = false;
    for (const auto &p : candidate_paths) {
        if (load_cuboids_from_json(environment, p)) {
            std::cout << "Loaded cuboids from: " << p << std::endl;
            loaded = true;
            break;
        }
    }
    if (!loaded) {
        std::cerr << "Failed to load cuboids JSON from any candidate path. Exiting." << std::endl;
        return 1;
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -0.01, -10.01, -0.01, -0.01, -0.01, -0.01
    };

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        0.01, 10.01, 0.01, 0.01, 0.01, 0.01
    };
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0, 1,0,0,   0.3486, 0.647752, 0.2399}};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

    vamp::planning::TaskSpaceConstraint<Robot, rake> tsr_constraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        tsr_lower_bound,
        tsr_upper_bound
    );
    vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> task_constraint(
        tsr_constraint
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
    Eigen::Quaternionf qstart(Robot::eefk(start)[0].linear());
    Eigen::Quaternionf qgoal(Robot::eefk(goal)[0].linear());
    std::cout << Robot::eefk(start)[0].matrix() << std::endl;
    std::cout << Robot::eefk(goal)[0].matrix() << std::endl;
    std::cout << "-----------------" << std::endl;

    // auto start_eefk = Robot::eefk(start);
    // std::cout << (start_eefk[0].inverse() * start_eefk[1]).matrix()  << std::endl;
    // auto goal_eefk = Robot::eefk(goal);
    // std::cout << (goal_eefk[0].inverse() * goal_eefk[1]).matrix()  << std::endl;

    // auto lTr_start = start_eefk[0].inverse() * start_eefk[1];
    // Eigen::Quaternionf q1(lTr_start.linear());
    // std::cout << q1.w() << ", " << q1.x() << ", " << q1.y() << ", " << q1.z() << " , " << lTr_start.translation().transpose() << std::endl;


    // auto lTr_goal = goal_eefk[0].inverse() * goal_eefk[1];
    // Eigen::Quaternionf q2(lTr_goal.linear());
    // std::cout << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << ", " << lTr_goal.translation().transpose() << std::endl;


    std::cout << qstart.w() << ", " << qstart.x() << ", " << qstart.y() << ", " << qstart.z() << std::endl;
    std::cout << qgoal.w() << ", " << qgoal.x() << ", " << qgoal.y() << ", " << qgoal.z() << std::endl;


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

    std::cout << "Projecting start and goal\n\n";
    Robot::ConfigurationArray holder;
    // typename Robot::template ConfigurationBlock<rake> block, projected_block, direction_vector_block;
    for (auto i = 0U; i < Robot::dimension; ++i){
        block[i] = Robot::Configuration(start).broadcast(i);
    }
    task_constraint.print_robot_tsr_error(block);
    bool startsuccess = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
    std::cout << "Start success : " <<startsuccess <<std::endl;
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
    std::cout << "\n\nProjecting goal\n\n";
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(goal).broadcast(i);
    task_constraint.print_robot_tsr_error(block);
    bool goalsuccess = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
    std::cout << "Goal success : " << goalsuccess << std::endl;

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

    // auto vector2 = Robot::Configuration(start) - Robot::Configuration(goal);
    // std::cout << vector2 << " with norm : " << vector2.l2_norm() << std::endl;
    // std::vector<Robot::Configuration> projected_vector2, projected_vector_inside;
    // auto ret = vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(Robot::Configuration(start), Robot::Configuration(goal), projected_vector2, task_constraint, env_v);
    // std::cout << ret << std::endl;


    // for(auto& proj_vec : projected_vector2){
    //     first = true;
    //     for (auto i = 0U; i < Robot::dimension; ++i){
    //         holder[i] = proj_vec.broadcast(i)[{0, 0}];
    //         std::cout << holder[i] << ", ";
    //         if (!first) outfile << ",";
    //         outfile << holder[i];
    //         first = false;
    //     }
    //     outfile << "\n";
    //     std::cout << std::endl;
    // }

    // // now try with attachment
    // std::vector<vamp::collision::Sphere<float>> spheres;
    // for(auto i=0U; i < 10; i++){
    //     spheres.push_back(vamp::collision::Sphere<float>(0.0, 0.0, i * 0.02, 0.02));
    // }
    // auto attach_transform = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    // attach_transform.translation().z() = 0.0;
    // AttachmentInput attachment(attach_transform);

    // attachment.spheres.insert(attachment.spheres.end(), spheres.cbegin(), spheres.cend());
    // environment.attach(attachment, 0);
    // env_v = EnvironmentVector(environment);

    // ret = vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(Robot::Configuration(start), Robot::Configuration(goal), projected_vector2, task_constraint, env_v);
    // std::cout << ret << std::endl;
    // std::cout << Robot::eefk(start)[0].matrix() << std::endl;
    // std::cout << Robot::eefk(goal)[0].matrix() << std::endl;

    // // testing start
    // std::cout << "Testing start configuration:" << std::endl;
    // for (auto i = 0U; i < Robot::dimension; ++i){
    //     block[i] = Robot::Configuration(start).broadcast(i);
    // }
    // std::cout << Robot::fkcc_attach(env_v, block) << std::endl;

    // // testing goal
    // std::cout << "Testing goal configuration:" << std::endl;
    // for (auto i = 0U; i < Robot::dimension; ++i){
    //     block[i] = Robot::Configuration(goal).broadcast(i);
    // }
    // std::cout << Robot::fkcc_attach(env_v, block) << std::endl;


    // for(auto& proj_vec : projected_vector2){
    //     ret = vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(proj_vec, proj_vec, projected_vector_inside, task_constraint, env_v);
    //     std::cout << "Projected Motion: " << ret << " -- ";

    //     first = true;
    //     for (auto i = 0U; i < Robot::dimension; ++i){
    //         holder[i] = proj_vec.broadcast(i)[{0, 0}];
    //         std::cout << holder[i] << ", ";
    //         if (!first) outfile << ",";
    //         outfile << holder[i];
    //         first = false;
    //     }

    //     outfile << "\n";
    //     std::cout << std::endl;
    // }


    // Robot::Configuration start_plus_percent = {-1.289,  1.039,  1.079, -2.12 ,  0.553,  2.082, -1.181,  1.165, 1.11 , -1.048, -2.128, -0.5  ,  2.138, -0.202};
    // Robot::ConfigurationArray test = {-1.289,  1.039,  1.079, -2.12 ,  0.553,  2.082, -1.181,  1.165, 1.11 , -1.048, -2.128, -0.5  ,  2.138, -0.202};


    // for (auto i = 0U; i < Robot::dimension; ++i){
    //     block[i] = Robot::Configuration(test).broadcast(i);
    // }
    // std::cout << "\n\n Running project for a single config : \n\n";
    // task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
    // std::cout << std::endl;
    // first = true;
    // for (auto i = 0U; i < Robot::dimension; ++i){
    //     holder[i] = projected_block[{i, 0}];
    //     std::cout << holder[i] << ", ";
    //     if (!first) outfile << ",";
    //     outfile << holder[i];
    //     first = false;
    // }
    // outfile << "\n";
    // std::cout << std::endl;



    outfile.close();

    return 0;
}
