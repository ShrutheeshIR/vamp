#include <vector>
#include <array>
#include <utility>
#include <csignal>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vamp/vector.hh>
#include <vamp/vector/interface.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <nlohmann/json.hpp>

#include <vamp/robots/panda.hh>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
// #include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Exception.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/log.hpp>

 #include "pinocchio/spatial/explog.hpp"
 #include "pinocchio/algorithm/joint-configuration.hpp"

using namespace pinocchio;


namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using ConfigurationBlock = typename Robot::template ConfigurationBlock<rake>;
using AttachmentInput = vamp::collision::Attachment<float>;


// Maximum planning time
static constexpr float planning_time = 100.0;
static constexpr int maxIterations = 1000000;
// Maximum simplification time
static constexpr float simplification_time = 1.0;



namespace ob = ompl::base;

struct TSR
{
    Eigen::Isometry3d T_ref;               // reference pose
    Eigen::Matrix<double,6,1> lower;       // lower bounds in task-space log coordinates
    Eigen::Matrix<double,6,1> upper;       // upper bounds
};


class SE3Constraint : public ob::Constraint
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SE3Constraint(const std::string& urdf_path, 
                const TSR &tsr)
        : ob::Constraint(get_model(urdf_path)->nq, 6, 1e-3),
        tsr_(tsr)
        {
            // 1. Get the shared model (loads only once)
            model_ptr_ = get_model(urdf_path);
            
            // 2. Get the Frame ID
            eeFrame_ = model_ptr_->getFrameId("panda_grasptarget");
            // pinocchio::FrameIndex eeFrame = model.getFrameId("panda_grasptarget");
            // 3. Setup Reference
            T_ref_pin_ = pinocchio::SE3(tsr.T_ref.rotation(), tsr.T_ref.translation());
        }

        void function(const Eigen::Ref<const Eigen::VectorXd> &q,
                        Eigen::Ref<Eigen::VectorXd> out) const override
            {
                static thread_local pinocchio::Data data(*model_ptr_);
                // Use a local data object if multi-threading, or lock this section
                pinocchio::forwardKinematics(*model_ptr_, data, q);
                pinocchio::updateFramePlacements(*model_ptr_, data);

                // Compute error in local-world aligned or local frame
                // T_err = T_ref^-1 * T_ee
                const pinocchio::SE3 &T_ee = data.oMf[eeFrame_];
                const pinocchio::SE3 T_err = T_ref_pin_.actInv(T_ee);
                
                // Log map gives the 6D error vector
                Eigen::VectorXd error = pinocchio::log6(T_err).toVector();

                // Apply TSR bounds: (val - upper).max(0) + (val - lower).min(0)
                for(int i=0; i<6; ++i) {
                    if (error[i] > tsr_.upper[i]) out[i] = error[i] - tsr_.upper[i];
                    else if (error[i] < tsr_.lower[i]) out[i] = error[i] - tsr_.lower[i];
                    else out[i] = 0.0;

                }
                // std::cout << out.transpose() << " " << q.transpose() << std::endl;
            }

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &q,
                    Eigen::Ref<Eigen::MatrixXd> out) const override
        {

            static thread_local pinocchio::Data data(*model_ptr_);

            pinocchio::forwardKinematics(*model_ptr_, data, q);
            pinocchio::computeJointJacobians(*model_ptr_, data, q);
            pinocchio::updateFramePlacements(*model_ptr_, data);

            pinocchio::Data::Matrix6x J(6, model_ptr_->nv);
            // Get Jacobian in the LOCAL frame of the end effector
            pinocchio::getFrameJacobian(*model_ptr_, data, eeFrame_, pinocchio::LOCAL, J);
            const pinocchio::SE3 &T_ee = data.oMf[eeFrame_];
            const pinocchio::SE3 T_err = T_ref_pin_.actInv(T_ee);

            pinocchio::Data::Matrix6 Jlog;
            pinocchio::Jlog6(T_err, Jlog);
            out = -Jlog * J;

        }

        // Optional LM projection
        bool project (Eigen::Ref< Eigen::VectorXd > x) const override
        {
            Eigen::VectorXd f(6);
            Eigen::MatrixXd J(6, model_ptr_->nv);

            for(int i=0;i<max_iters_;++i)
            {
                // std::cout << i << std::endl;
                function(x, f);
                if(f.norm() < dist_tol) {
                    // std::cout << "Project succeeded : " << x.transpose() << std::endl;
                    return true;
                }

                jacobian(x, J);


                Eigen::VectorXd delta(model_ptr_->nv);
                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += lambda_;
                delta.noalias() = -J.transpose() * JJt.ldlt().solve(f);

                // Eigen::MatrixXd H = J.transpose()*J + lambda*Eigen::MatrixXd::Identity(model_.nv, model_.nv);
                // Eigen::VectorXd delta = H.ldlt().solve(-J.transpose()*f);
                x = pinocchio::integrate(*model_ptr_, x, -delta * 0.25);
                // x += delta;  

                if(delta.norm() < dist_tol) {
                    return true;
                }

            }
            // std::cout << "Projection failed " << std::endl;
            return false; // did not converge
        }

        double distance(const Eigen::Ref<const Eigen::VectorXd> &x) const override
        {
            Eigen::VectorXd out(6);   // ✅ correct size
            function(x, out);
            return out.squaredNorm();
        }

        bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
        {
            bool result = distance(x) < 0.0001;
            std::cout << "our custom implementation of isSatisfied is called " << result << std::endl;
            return result;
        }


    private:
        // std::shared_ptr<const pinocchio::Model> model_ptr_; 
        static std::shared_ptr<pinocchio::Model> get_model(const std::string& path) {
            static std::shared_ptr<pinocchio::Model> shared_model;
            if (!shared_model) {
                shared_model = std::make_shared<pinocchio::Model>();
                pinocchio::urdf::buildModel(path, *shared_model);
            }
            return shared_model;
        }
        std::shared_ptr<const pinocchio::Model> model_ptr_;
        pinocchio::FrameIndex eeFrame_;
        pinocchio::SE3 T_ref_pin_;
        TSR tsr_;
        size_t max_iters_ = 50;
        double dist_tol = 1e-3;
        double lambda_ = 1e-3;
};



struct VAMPStateValidator : public ob::StateValidityChecker
{
    VAMPStateValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        // Convert OMPL to VAMP vector and validate
        // auto configuration = ompl_to_vamp(state);

        const Eigen::Map<Eigen::VectorXd> &x = *state->as<ob::ConstrainedStateSpace::StateType>();

        std::array<float, Robot::dimension> float_config_from_x;
        for (auto i = 0U; i < Robot::dimension; ++i) {
            // cast x[i] to float
            float_config_from_x[i] = static_cast<float>(x[i]);
        }
        Configuration robot_config(float_config_from_x);
        bool val_res = vamp::planning::validate_motion<Robot, rake, 1>(robot_config, robot_config, env_v);
        // std::cout << "Running is valid " << val_res << std::endl;
        return val_res;
    }
    const EnvironmentVector &env_v;
    // vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint;

};


// Helper function to convert projected state into a double veector
std::vector<double> extractStateReals(
    const ob::State *state,
    const ob::ProjectedStateSpace *space)
{
    // Cast the top-level state space to ProjectedStateSpace

    std::size_t dim = space->getDimension();

    std::vector<double> values(dim);
    space->copyToReals(values, state);

    return values;
}


//Helper function to turn double vector into vamp vector
inline static auto double_vector_to_vamp(const std::vector<double> &values) -> Configuration
{
    if (values.size() != dimension)
        throw std::runtime_error("Input vector size does not match robot dimension");

    // Create an aligned buffer for the VAMP configuration
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    // Copy the vector values into the aligned buffer
    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(values[i]);
    }

    // Construct a Configuration from the aligned buffer
    return Configuration(aligned_buffer.data());
}


struct OMPLMotionValidator : public ob::MotionValidator
{
    const EnvironmentVector &env_v;
    inline static size_t project_constrained_motion_failed_counter = 0;
    OMPLMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    OMPLMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        // std::cout << "Calling geodesic checkMotion\n";
        auto *css = dynamic_cast<ob::ProjectedStateSpace*>(si_->getStateSpace().get());
        // if (!css)
        //     throw ompl::Exception("Expected ProjectedStateSpace");

        // auto constraint_base = css->getConstraint();

        // auto *custom_constraint = dynamic_cast<CustomConstraint*>(constraint_base.get());
        // if (!custom_constraint)
        //     throw ompl::Exception("Expected CustomConstraint");

        // Configuration configuration2 = double_vector_to_vamp(extractStateReals(s2, css));
        // if (!(custom_constraint->isSatisfied(configuration2))) {
        //     std::cout << "configuration2 is not satisfied within checkMotion!\n";
        //     return false;
        // }

        auto discrete_geodesic = css->discreteGeodesic(s1, s2, false);
        if (!discrete_geodesic) {
            // std::cout << "discrete_geodesic is not satisfied within checkMotion!\n";
            // css->printState(s1, std::cout);
            // css->printState(s2, std::cout);

            project_constrained_motion_failed_counter++;
        }
        else {
            ;
            // std::cout << "discrete_geodesic is satisfied within checkMotion!\n";
            // css->printState(s1, std::cout);
            // css->printState(s2, std::cout);
        }
        // std::cout << project_constrained_motion_failed_counter << std::endl;
        return discrete_geodesic;


    }

    auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const
        -> bool override
    {
        //Intentionally not implemented
        throw ompl::Exception("Not implemented!");
    }
};



// State validator using VAMP
inline static auto ompl_to_vamp(const ob::State *state) -> Configuration
{
    // Create an aligned memory buffer to load VAMP vector from
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    // Copy OMPL data into aligned buffer
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(as->values[i]);
    }

    // Create configuration from aligned buffer data
    return Configuration(aligned_buffer.data());
}

// Convert a VAMP vector to an OMPL state
inline static auto vamp_to_ompl(const Configuration &c, ob::State *state)
{
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        as->values[i] = static_cast<double>(c[{i, 0}]);
    }
}

// Define your problem structure
struct Problem {
	std::array<float, 7> problem_start;
	std::array<float, 7> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
};

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


void load_problems_from_json(std::vector<Problem> &problems, const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return;
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception &e) {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return;
    }

    if (!j.is_array()) {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return;
    }

    for (const auto &item : j) {
        try {
            Problem p;
            p.problem_start = item.at("problem_start").get<std::array<float, 7>>();
            p.problem_end = item.at("problem_end").get<std::array<float, 7>>();
            p.start_eef_pos = item.at("start_eef_pos").get<std::array<float, 3>>();
            p.goal_eef_pos = item.at("goal_eef_pos").get<std::array<float, 3>>();
            problems.push_back(p);
        } catch (const std::exception &e) {
            std::cerr << "Error reading problem fields: " << e.what() << " -- skipping problem" << std::endl;
            continue;
        }
    }
}

auto run_rrtc(const Problem &problem) {
        std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};
        //used for constraint
        std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -10.01, -10.01, -0.01, -0.01, -0.01, -10.01
        };

        std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
            10.01, 10.01, 0.01, 0.01, 0.01, 10.01
        };
        std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783}};


        // 1. Extract the pointer to the data
        const float* data = eef_transforms[0].data();

        // 2. Create the Quaternion (w, x, y, z)
        // Note: We cast to double because Isometry3d expects doubles
        Eigen::Quaterniond q(
            static_cast<double>(data[0]), // w
            static_cast<double>(data[1]), // x
            static_cast<double>(data[2]), // y
            static_cast<double>(data[3])  // z
        );

        // 3. Create the Translation vector (x, y, z)
        Eigen::Vector3d t(
            static_cast<double>(data[4]), 
            static_cast<double>(data[5]), 
            static_cast<double>(data[6])
        );

        // 4. Combine into an Isometry3d
        Eigen::Isometry3d T_ref = Eigen::Isometry3d::Identity();
        T_ref.rotate(q);
        T_ref.pretranslate(t);

        TSR tsr;
        tsr.T_ref = T_ref;
        tsr.lower = Eigen::Map<const Eigen::Matrix<float, 6, 1>>(tsr_lower_bound.data()).cast<double>();
        tsr.upper = Eigen::Map<const Eigen::Matrix<float, 6, 1>>(tsr_upper_bound.data()).cast<double>();

        std::string urdf_file = "/src/myfork/vamp/resources/panda/panda_spherized.urdf";
        // pinocchio::Model model;
        // pinocchio::GeometryModel collision_model;

        // pinocchio::urdf::buildModel(urdf_file, model);
        // pinocchio::urdf::buildGeom(model, urdf_file, pinocchio::COLLISION, collision_model);
        // pinocchio::FrameIndex eeFrame = model.getFrameId("panda_grasptarget");

        auto rvss = std::make_shared<ob::RealVectorStateSpace>(dimension);

        // ob::RealVectorBounds bounds(dimension);
        // bounds.setLow(-2);
        // bounds.setHigh(2);
        // Get bounds from VAMP Robot information, scale 0/1 config to min/max
        static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
        static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

        auto zero_v = Configuration(zeros);
        auto one_v = Configuration(ones);
        Robot::scale_configuration(zero_v);
        Robot::scale_configuration(one_v);


        ob::RealVectorBounds bounds(dimension);
        for (auto i = 0U; i < dimension; ++i)
        {
            bounds.setLow(i, zero_v[{0, i}]);
            bounds.setHigh(i, one_v[{0, i}]);
        }

        rvss->setBounds(bounds);


        // Create a shared pointer to our constraint.
        // auto constraint = std::make_shared<SphereConstraint>();
        // auto constraint = std::make_shared<CustomConstraint>(task_constraint);

        // Combine the ambient space and the constraint into a constrained state space.
        // auto constraint = std::make_shared<SE3Constraint>(model, eeFrame , tsr);
        // 2. Pass the model_ptr to your new constructor
        auto constraint = std::make_shared<SE3Constraint>(urdf_file, tsr);

        auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);

        // Define the constrained space information for this constrained state space.
        auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);

        // auto ss = std::make_shared<og::SimpleSetup>(csi);

        // Set the state validity checker in simple setup.
        // csi->setStateValidityChecker(obstacle);
        //
        //
        EnvironmentInput environment;

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
        environment.sort();

        std::vector<vamp::collision::Sphere<float>> spheres;
        for(auto i=0U; i < 9; i++){
            spheres.push_back(vamp::collision::Sphere<float>(0.0, 0.0, i * 0.02, 0.01));
        }
        auto attach_transform = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
        attach_transform.translation().z() = 0.0;
        AttachmentInput attachment(attach_transform);

        attachment.spheres.insert(attachment.spheres.end(), spheres.cbegin(), spheres.cend());
        environment.attach(attachment, 0);

        auto env_v = EnvironmentVector(environment);

        csi->setStateValidityChecker(std::make_shared<VAMPStateValidator>(csi, env_v));
        csi->setMotionValidator(std::make_shared<OMPLMotionValidator>(csi, env_v));

        csi->setup();

        // si->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si, env_v));
        // si->setMotionValidator(std::make_shared<VAMPMotionValidator>(si, env_v));
        // si->setup();

        // Start and goal vectors.
        Eigen::VectorXd sv(dimension), gv(dimension);
        // sv << 1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100;
        // gv << -1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000;
        for (size_t i = 0; i < dimension; ++i) {
            sv[i] = problem.problem_start[i];
            gv[i] = problem.problem_end[i];
        }

        // Scoped states that we will add to simple setup.
        ob::ScopedState<> start(css);
        ob::ScopedState<> goal(css);

        // Copy the values from the vectors into the start and goal states.
        start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
        goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);

        // If we were using an Atlas or TangentBundleStateSpace, we would also have to anchor these states to charts:
        //   css->anchorChart(start.get());
        //   css->anchorChart(goal.get());
        // Which gives a starting point for the atlas to grow.
        // css->setBackoff(0.05);
        // css->setAtlasBoundary(0.1);


        auto pdef = std::make_shared<ob::ProblemDefinition>(csi);
        pdef->setStartAndGoalStates(start, goal);

        // ss->setStartAndGoalStates(start, goal);

        auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
        pdef->setOptimizationObjective(obj);
        obj->setCostThreshold(obj->infiniteCost());


        auto planner = std::make_shared<og::RRTConnect>(csi);

        planner->setProblemDefinition(pdef);
        planner->setRange(0.4);
        planner->setup();


        // auto pp = std::make_shared<og::RRTConnect>(csi);
        // ss->setPlanner(pp);
        // ss->setup();
        // Solve a problem like normal, for 5 seconds.
        auto start_time = std::chrono::steady_clock::now();
        ob::PlannerStatus stat = planner->ob::Planner::solve(planning_time);
        auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

        planner->clear();

        // 2. Clear the problem definition
        pdef->clearSolutionPaths();
        pdef->clearStartStates();
        pdef->clearGoal();

        planner.reset();
        pdef.reset();
        csi.reset();
        css.reset();
        constraint.reset();
        
        std::cout << "Planning time: " << nanoseconds / 1e6 << "ms" << std::endl;
        return std::make_pair(stat == ob::PlannerStatus::EXACT_SOLUTION, nanoseconds);

}


auto main(int argc, char **) -> int
{

	std::vector<Problem> problems;

    size_t total_num_problems = 0;
    size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;
    std::string problem_json_path = "/src/tsr_panda_maze_problems.json";
    load_problems_from_json(problems, problem_json_path);


    for(const auto &problem: problems){
        std::cout << "Planning problem " << total_num_problems + 1 << " / " << problems.size() << std::endl;
        total_num_problems++;

        auto [result, nanoseconds] = run_rrtc(const_cast<Problem&>(problem));


        if (result)
        {
            successful_problems++;
            nanoseconds_per_problem.push_back(nanoseconds);
            iterations_per_problem.push_back(0);
        std::cout << "Found solution in " <<    nanoseconds / 1e6 << "ms"<< std::endl;

        }
        else
            OMPL_WARN("No solution found!");
        
        std::cout << "Moving onto next problem...\n" << std::endl;
    }
    std::cout << "Total problems: " << total_num_problems << std::endl
                << "Successful problems: " << successful_problems << std::endl
                << "Success rate: " << (static_cast<float>(successful_problems) / total_num_problems) * 100.0f << "%" << std::endl;
    
    if(successful_problems > 0){
        std::size_t total_nanoseconds = 0;
        std::size_t total_iterations = 0;
        for(size_t i = 0; i < successful_problems; i++){
            total_nanoseconds += nanoseconds_per_problem[i];
            total_iterations += iterations_per_problem[i];
        }
        std::cout << "Average time (ms) for successful problems: " << (total_nanoseconds / successful_problems) / 1000000.0 << std::endl;
        std::cout << "Average iterations for successful problems: " << total_iterations / successful_problems << std::endl;
        // compute median for time and iterations
        std::sort(nanoseconds_per_problem.begin(), nanoseconds_per_problem.end());
        std::sort(iterations_per_problem.begin(), iterations_per_problem.end());
        std::cout << "Median time (ms) for successful problems: " << (nanoseconds_per_problem[successful_problems / 2]) / 1000000.0 << std::endl;
        std::cout << "Median iterations for successful problems: " << iterations_per_problem[successful_problems / 2] << std::endl;
    }


}
