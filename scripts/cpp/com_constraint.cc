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
#include <vamp/robots/g1_unitree.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::G1Unitree;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
// using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution, 4>;
using AttachmentInput = vamp::collision::Attachment<float>;

// Start and goal configurations
// static constexpr Robot::ConfigurationArray start = {0.697778, -0.5024, -1.256, -1.94109, -2.12554, -2.36424, -2.44589, -0.204884, -2.35822, 0.113728, -0.793422, -0.234981, -2.26647, -2.81113, -2.53907, 0.0183405, -0.824602, -0.245389, -2.46313, -0.490082, -0.492354, -2.94348, -1.4976, -2.50464, -0.98188, -1.8918, -1.55106, -1.55222, -2.98216, -2.18801, -2.53406, -0.999044, -1.91263, -1.56892, -1.56953};

static constexpr Robot::ConfigurationArray start = {0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.767,-0.16,0.52,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
static constexpr Robot::ConfigurationArray goal = {0.0, 0.0, 0.0, -2.14842, -0.0682913, -2.18706, 0.199869, -0.759459, -0.223488, -2.15322, -2.74429, -2.44542, 0.0636008, -0.804001, -0.238355, -2.39676, -0.47726, -0.480506, -2.88102, -1.45877, -2.45606, -0.953885, -1.85734, -1.5239, -1.52556, -2.93629, -2.1608, -2.49809, -0.978406, -1.88709, -1.54942, -1.55028};
// static constexpr Robot::ConfigurationArray goal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.702,-0.16,0.52,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};


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

    // auto isometries = Robot::eefk(start);
    // std::cout << isometries[0].matrix() << std::endl<< isometries[1].matrix() << std::endl<< isometries[2].matrix() << std::endl<< isometries[3].matrix() << std::endl;


    // std::array<float, 8> polygon_points = {
    //     // 1.0, -0.3, 1.0, 0.3, 0.0, 0.3, 0.0, -0.3
    //     // 0.0, 0.2, 1.0, 0.2, 1.0, 1.0, 0.0, 1.0
    //     1.0, -0.05, 1.0, 0.05, 0.0, 0.05, 0.0, -0.05
    // };

    // std::array<float, 6> lower_bound = {
    //     -0.00001, -0.00001, -0.00001, -0.00001, -0.00001, -0.00001
    // };
    // std::array<float, 6> upper_bound = {
    //     0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001
    // };
    std::array<float, 8> polygon_points = {
        // 1.0, -0.05, 1.0, 0.05, 0.0, 0.05, 0.0, -0.05
        10.10, -10.15, 10.10, 10.15, -10.10, 10.15, -10.10, -10.15
        // 0.0, 0.2, 1.0, 0.2, 1.0, 1.0, 0.0, 1.0
    };

    std::array<float, 6> lower_bound = {
        -0.00001, -0.00001, -0.00001, -0.00001, -0.00001, -0.00001
    };
    std::array<float, 6> upper_bound = {
        0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001
    };

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -0.01, -0.01, -0.01, -10.0, -10.0, -10.0, -0.01, -0.01, -0.01, -10.0, -10.0, -10.0
    };

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
        0.01, 0.01, 0.01,
        10.0, 10.0, 10.0,
        0.01, 0.01, 0.01,
        10.0, 10.0, 10.0
    };


    std::array<float, 6> slower_bound = {
        -0.01, -0.01, -0.01, -10.0, -10.0, -10.0
    };
    std::array<float, 6> supper_bound = {
        0.01, 0.01, 0.01, 10.0, 10.0, 10.0
    };


    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    Eigen::Matrix<float, 4, 4> T;
    T << 1, 0, 0, 0, 0, 1, 0, -0.3, 0, 0, 1, 0, 0, 0, 0, 1;
    const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);

    std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms;

    T << 1, 0, 0, 0.09, 0, 1, 0, 0.11, 0, 0, 1, -0.75, 0, 0, 0, 1;
    eef_transforms[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    T << 1, 0, 0, 0.09, 0, 1, 0, 0.11, 0, 0, 1, -0.75, 0, 0, 0, 1;
    eef_transforms[1] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    T << 1, 0, 0, 0.09, 0, 1, 0, 0.11, 0, 0, 1, -0.75, 0, 0, 0, 1;
    eef_transforms[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    T << 1, 0, 0, 0.09, 0, 1, 0, -0.11, 0, 0, 1, -0.75, 0, 0, 0, 1;
    eef_transforms[3] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

    std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms_ref_frame_w_world;
    T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    eef_transforms_ref_frame_w_world[1] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    eef_transforms_ref_frame_w_world[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    eef_transforms_ref_frame_w_world[3] = Eigen::Transform<float, 3, Eigen::Isometry>(T);


    // vamp::planning::BimanualCoMTSRTaskSpaceConstraint<Robot, rake, 4> task_constraint(
    //     polygon_points, 
    //     target_pose, 
    //     std::make_pair(lower_bound, upper_bound), 
    //     eef_transforms_ref_frame_w_world,
    //     eef_transforms, 
    //     std::make_pair(tsr_lower_bound, tsr_upper_bound)
    // );

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

    vamp::planning::BimanualTaskSpaceConstraint<Robot, rake> task_constraint(
        target_pose, 
        std::make_pair(lower_bound, upper_bound)
    );


    // // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    // Eigen::Matrix<float, 4, 4> T;
    // T << -0.719427, 0.694568, 6.59173e-05, -2.2769e-05, 0.694568, 0.719427, -2.26738e-05, -0.000370264, -6.31774e-05, 2.94462e-05, -1, 0.171814, 0, 0, 0, 1;
    // const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    // vamp::planning::BimanualCoMTaskSpaceConstraint<Robot, rake, 4> task_constraint(polygon_points, target_pose, std::make_pair(lower_bound, upper_bound));



    // // vamp::planning::CoMTaskSpaceConstraint<Robot, rake, 4> task_constraint(polygon_points);
    // auto startcc = rng->next();
    bool success;
    typename Robot::template ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
        block[i] = Robot::Configuration(start).broadcast(i);

    // task_constraint.print_robot_tsr_error(block);

    // // Robot::ConfigurationArray test = {-0.894,  0.391, -0.591, -1.742, -0.776,  1.65,  -0.978,  0.596,  0.49,  -0.002, -1.672,  0.039,  1.981, -0.502};
    // // for (auto i = 0U; i < Robot::dimension; ++i)
    // //     block[i] = Robot::Configuration(test).broadcast(i);

    // task_constraint.distanceToConstraint(block);
    // task_constraint2.print_robot_tsr_error(block);

    std::cout << "----Projecting ----" << std::endl;
    typename Robot::template ConfigurationBlock<rake> projected_block;
    typename Robot::template ConfigurationBlock<rake> projected_block2;

    // task_constraint2.projectStep(block, projected_block2);
    // for(auto i=0U; i < Robot::dimension; i++)
    //     std::cout << projected_block2[{i, 0}] << " ";
    // std::cout << std::endl;

    // task_constraint.projectStep(block, projected_block);
    // for(auto i=0U; i < Robot::dimension; i++)
    //     std::cout << projected_block[{i, 0}] << " ";
    // std::cout << std::endl;
    // task_constraint.projectStep(projected_block, projected_block);
    // for(auto i=0U; i < Robot::dimension; i++)
    //     std::cout << projected_block[{i, 0}] << " ";
    // std::cout << std::endl;
    // success = task_constraint2.projectConfiguration(block, projected_block2);
    // for(auto i=0U; i < Robot::dimension; i++)
    //     std::cout << projected_block2[{i, 0}] << ",";
    // std::cout << std::endl;


    success = task_constraint.projectConfiguration(block, projected_block);
    for(auto i=0U; i < Robot::dimension; i++)
        std::cout << projected_block[{i, 0}] << ",";
    std::cout << std::endl;


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
    // bool success;

    // std::cout << startc << ", " << goalc << std::endl;

    auto startc = Robot::Configuration(start);
    auto goalc = Robot::Configuration(goal);


    auto extension_vector = goalc - startc;
    std::cout << extension_vector << std::endl;
    extension_vector = extension_vector / extension_vector.l2_norm();
    std::cout << extension_vector << std::endl;


    std::cout << std::fixed << std::setprecision(3);
    std::ofstream outfile("/src/trajectory.txt");


    for(size_t ext = 0; ext < 11; ext++){
        std::cout << "\nExtension attempt " << ext << " " << std::endl;
        auto cfg = startc + extension_vector * ext / 10;
        for (auto i = 0U; i < Robot::dimension; ++i)
            block[i] = cfg.broadcast(i) + 0.0;
        for(auto i=0U; i < Robot::dimension; i++)
            std::cout << block[{i, 0}] << ", ";
        std::cout << std::endl;
        // for(auto i=0U; i < Robot::dimension; i++)
        //     std::cout << block[{i, 0}] << " ";
        // std::cout << " --> " ;

        // task_constraint.distanceToConstraint(block);

        // typename Robot::template ConfigurationBlock<rake> projected_block2;

        // success = task_constraint2.projectConfiguration(block, projected_block2, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);
        // for(auto i=0U; i < Robot::dimension; i++)
        //     std::cout << projected_block2[{i, 0}] << " ";
        // std::cout << " --> " << success << " -- "<< std::endl;

        // for (auto i = 0U; i < Robot::dimension; ++i)
        //     block[i] = cfg.broadcast(i) + 0.0;

        typename Robot::template ConfigurationBlock<rake> projected_block;
        success = task_constraint.projectConfiguration(block, projected_block, vamp::planning::ProjMethod::InnerLM, 10.0, 1.0);

        bool valid = Robot::template fkcc<rake>(env_v, projected_block);

        for(auto i=0U; i < Robot::dimension; i++)
            std::cout << projected_block[{i, 0}] << ", ";
        std::cout << " --> " << success << " -- " << valid << std::endl;


        bool first = true;
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

        // auto fka = Robot::eefk(soln);
        // std::cout <<std::endl << fka.matrix() <<std::endl;
        // std::cout << std::endl;
        outfile << "\n";
        // task_constraint.distanceToConstraint(projected_block);
    }
    outfile.close();

    // Robot::eefk(start)



}
