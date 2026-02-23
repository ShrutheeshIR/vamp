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

#include <vamp/robots/panda.hh>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
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


// Maximum planning time
static constexpr float planning_time = 50.0;
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
                // Eigen::VectorXd error = pinocchio::log6(T_err).toVector();
                //
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
                    return true;
                }

                jacobian(x, J);


                Eigen::VectorXd delta(model_ptr_->nv);
                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += lambda_;
                delta.noalias() = -J.transpose() * JJt.ldlt().solve(f);

                if (delta.norm() > 10.0)
                    return false;

                // Eigen::MatrixXd H = J.transpose()*J + lambda*Eigen::MatrixXd::Identity(model_.nv, model_.nv);
                // Eigen::VectorXd delta = H.ldlt().solve(-J.transpose()*f);
                x = pinocchio::integrate(*model_ptr_, x, -delta * 1.0);
                // x += delta;  

                if(delta.norm() < dist_tol) {
                    return true;
                }

            }
            return false; // did not converge
        }

        double distance(const Eigen::Ref<const Eigen::VectorXd> &x) const override
        {
            Eigen::VectorXd out(6);   // ✅ correct size
            function(x, out);
            // std::cout << "Distance called, value: " << (out.squaredNorm() < 1e-3) << std::endl;
            return out.squaredNorm();
        }

        bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
        {
            bool result = distance(x) < 0.0001;
            // std::cout << "our custom implementation of isSatisfied is called " << result << std::endl;
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


class ParallelSE3Constraint : public ob::Constraint
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ParallelSE3Constraint(
        std::shared_ptr<vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>> task_constraint)
        : ob::Constraint(7, 1, 1e-3),
          task_constraint_(task_constraint)
        {
            ;

        }

        ConfigurationBlock turn_configuration_into_configuration_block(const Robot::Configuration &c) const{
            typename Robot::template ConfigurationBlock<rake> block;

            for (auto i = 0U; i < Robot::dimension; ++i) {
                block[i] = c.broadcast(i) + 0.0;
            }
            return block;
        }

        void function(const Eigen::Ref<const Eigen::VectorXd> &q,
                        Eigen::Ref<Eigen::VectorXd> out) const override
            {
                // need to convert x to a configuration block
                std::array<float, Robot::dimension> float_config_from_x;
                for (auto i = 0U; i < Robot::dimension; ++i) {
                    // cast x[i] to float
                    float_config_from_x[i] = static_cast<float>(q[i]);
                }
                auto config_block = turn_configuration_into_configuration_block(Configuration(float_config_from_x));
                vamp::FloatVector<rake, 1> distance = task_constraint_->distanceToConstraint(config_block);
                out[0] = std::sqrt(distance[{0, 0}]);
            }


        // Optional LM projection
        bool project (Eigen::Ref< Eigen::VectorXd > x) const override
        {
            std::array<float, Robot::dimension> float_config_from_x;
            for (auto i = 0U; i < Robot::dimension; ++i) {
                // cast x[i] to float
                float_config_from_x[i] = static_cast<float>(x[i]);
            }
            auto config_block = turn_configuration_into_configuration_block(Configuration(float_config_from_x));
            ConfigurationBlock last_projected_block;
            bool result = task_constraint_->projectConfiguration(config_block, last_projected_block, vamp::planning::ProjMethod::OuterLM, 5.0, 1.0, max_iters_, false);
            if (result){
                for (auto i = 0U; i < Robot::dimension; ++i) {
                    x[i] = static_cast<double>(last_projected_block[{i, rake - 1}]);
                    // std::cout << x[i] << " ";
                }
            }
            return result;


        }

        double distance(const Eigen::Ref<const Eigen::VectorXd> &x) const override
        {
            Eigen::VectorXd out(1);   // ✅ correct size
            function(x, out);
            // std::cout << "Distance called, value: " << (out.squaredNorm() < 1e-3) << std::endl;
            return out.squaredNorm();
        }

        bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
        {
            bool result = distance(x) < 0.0001;
            // std::cout << "our custom implementation of isSatisfied is called " << result << std::endl;
            return result;
        }


    private:
        
        std::shared_ptr<vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>> task_constraint_;
        size_t max_iters_ = 50;
        double dist_tol = 1e-3;
        double lambda_ = 1e-3;
};




struct VAMPStateValidator : public ob::StateValidityChecker
{
    VAMPStateValidator(ob::SpaceInformation *si, std::shared_ptr<ob::Constraint> constraint, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), constraint_(constraint), env_v(env_v)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr &si, std::shared_ptr<ob::Constraint> constraint, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), constraint_(constraint), env_v(env_v)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        // Convert OMPL to VAMP vector and validate
        // auto configuration = ompl_to_vamp(state);

        const Eigen::Map<Eigen::VectorXd> &x = *state->as<ob::ConstrainedStateSpace::StateType>();

        bool constraint_satisfied = constraint_->isSatisfied(x);
        if (!constraint_satisfied) {
            // std::cout << "State is invalid, constraint not satisfied" << std::endl;
            return false;
        }

        std::array<float, Robot::dimension> float_config_from_x;
        for (auto i = 0U; i < Robot::dimension; ++i) {
            float_config_from_x[i] = static_cast<float>(x[i]);
        }
        Configuration robot_config(float_config_from_x);
        bool val_res = vamp::planning::validate_motion<Robot, rake, 1>(robot_config, robot_config, env_v);
        return val_res;
    }
    const EnvironmentVector &env_v;
    std::shared_ptr<ob::Constraint> constraint_;
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
        // std::cout << "Calling geodesic checkMotion between " << s1->as<ob::RealVectorStateSpace::StateType>()->values[0] << " and " << s2->as<ob::RealVectorStateSpace::StateType>()->values[0] << std::endl;

        auto *css = dynamic_cast<ob::ProjectedStateSpace*>(si_->getStateSpace().get());

        // make a pointer to std::Vector<State *>* geodesic path, so that can be passed into the geodesic function
        std::vector<ob::State*> geodesic_path;

        auto discrete_geodesic = css->discreteGeodesic(s1, s2, false, &geodesic_path);
        if (!discrete_geodesic) {
            project_constrained_motion_failed_counter++;
        }
        else {
            ;
        }
        // now free the geodesic path

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

struct VAMPMotionValidator : public ob::MotionValidator
{
    VAMPMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v, vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint)
      : ob::MotionValidator(si), env_v(env_v), task_constraint(task_constraint)
    {
    }

    VAMPMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v, vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint)
      : ob::MotionValidator(si), env_v(env_v), task_constraint(task_constraint)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {

        std::array<float, Robot::dimension> float_config_from_x1, float_config_from_x2;
        const Eigen::Map<Eigen::VectorXd> &x1 = *s1->as<ob::ConstrainedStateSpace::StateType>();
        const Eigen::Map<Eigen::VectorXd> &x2 = *s2->as<ob::ConstrainedStateSpace::StateType>();


        for (auto i = 0U; i < Robot::dimension; ++i) {
            // cast x[i] to float
            float_config_from_x1[i] = static_cast<float>(x1[i]);
            float_config_from_x2[i] = static_cast<float>(x2[i]);
        }
        Configuration robot_config_1(float_config_from_x1);
        Configuration robot_config_2(float_config_from_x2);

        std::vector <typename Robot::Configuration> projected_vector;
        bool projection_result = vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(robot_config_1, robot_config_2, projected_vector, task_constraint, env_v, vamp::planning::ProjMethod::InnerLM, 0.75, 25, false);
        return projection_result;
    }

    auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const
        -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }

    const EnvironmentVector &env_v;
    vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>&task_constraint;
};


auto main(int argc, char **) -> int
{

    // const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
    // const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    // vamp::planning::TaskSpaceConstraint<Robot, rake> task_constraint(in_hand_pose, target_pose, std::make_pair(lower_bound, upper_bound));

    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};
    //used for constraint
    const std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
        -0.01, -10.01, -0.01, -0.01, -0.01, -0.01};
    const std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
        0.01, 10.01, 0.01, 0.01, 0.01, 0.01};

    const std::array<std::array<float, Robot::dimension>, Robot::n_eef> eef_transforms = {{0, 1,0,0,   0.3486, 0.647752, 0.2399}};

    vamp::planning::TaskSpaceConstraint<Robot, rake> tsr_constraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        tsr_lower_bound,
        tsr_upper_bound
    );
    vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> task_constraint(
        tsr_constraint
    );
    
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
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(dimension);
    ompl::RNG::setSeed(42);   // fixed seed

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
    auto constraint = std::make_shared<SE3Constraint>(urdf_file, tsr);
    // auto constraint = std::make_shared<ParallelSE3Constraint>(std::make_shared<vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>>>(task_constraint));

    auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);
    css->setDelta(1.0F/Robot::resolution); // Set the projection step size (delta) to control the resolution of the projection

    // Define the constrained space information for this constrained state space.
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);

    // auto ss = std::make_shared<og::SimpleSetup>(csi);

    // Set the state validity checker in simple setup.
    // csi->setStateValidityChecker(obstacle);
    //
    //
    EnvironmentInput environment;

    static const std::vector<std::array<float, 3>> problem = {
        // {0.55, 0, 0.25},
        // {0.55, 0, 0.50},
        // {0.55, 0, 0.60},
        {0.56, 0, 0.450},
        {0.1, 0, 0.7},
        // {0.35, 0.35, 0.25},
        {0, 0.55, 0.25},
        {-0.55, 0, 0.25},
        {-0.35, -0.35, 0.25},
        {0, -0.55, 0.25},
        // {0.35, -0.35, 0.25},
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
    for (const auto &sphere : problem)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }

    // Try a few likely paths for the JSON file
    environment.sort();
    auto env_v = EnvironmentVector(environment);

    csi->setStateValidityChecker(std::make_shared<VAMPStateValidator>(csi, constraint, env_v));
    csi->setMotionValidator(std::make_shared<OMPLMotionValidator>(csi, env_v));
    // csi->setMotionValidator(std::make_shared<VAMPMotionValidator>(csi, env_v, task_constraint));

    std::cout << css->getDelta() << " : " << 1.0F/Robot::resolution << std::endl;

    csi->setup();

    // si->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si, env_v));
    // si->setup();

    // Start and goal vectors.
    Eigen::VectorXd sv(dimension), gv(dimension);
    sv << 1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100;
    gv << -1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000;

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
    planner->setRange(1.0);
    planner->setup();


    // auto pp = std::make_shared<og::RRTConnect>(csi);
    // ss->setPlanner(pp);
    // ss->setup();
    // Solve a problem like normal, for 5 seconds.
    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerStatus stat = planner->ob::Planner::solve(planning_time);
    auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);


    if (stat)
    {
        std::cout << "Found solution in " << nanoseconds / 1e6 << "ms! Simplfying..." << std::endl;

        const ob::PathPtr &path = pdef->getSolutionPath();
        og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);

        auto initial_cost = path_geometric.cost(obj);

        og::PathSimplifier simplifier(csi, pdef->getGoal(), obj);


        std::ofstream outfile("/src/trajectory.txt");
        outfile << std::fixed << std::setprecision(10);
        // std::cout << "Raw path length: " << path->length() << std::endl;

        // path->print(std::cout);
        for (std::size_t i = 0; i < path_geometric.getStateCount(); ++i)
        {
            std::vector<double> result = extractStateReals(path_geometric.getState(i), css.get());

            for (std::size_t j = 0; j < dimension; ++j)
            {
                if (j > 0)
                    outfile << ",";
                outfile << result[j];
            }
            outfile << "\n";
        }
        outfile.close();

        // if (not simplifier.simplify(path_geometric, simplification_time))
        // {
        //     std::cout << "Path not valid!" << std::endl;
        // }

        // auto simplified_cost = path_geometric.cost(obj);

        // // Output statistics
        // std::cout << "Found initial solution with cost " << initial_cost.value() << std::endl;
        // std::cout << "Simplified solution to cost " << simplified_cost.value() << std::endl;
        // std::cout << "Simplified solution:" << std::endl;

        path_geometric.print(std::cout);


        // // Path simplification also works when using a constrained state space!
        // ss->simplifySolution(5.);

        // // Get solution path.
        // auto path = ss->getSolutionPath();

        // // Interpolation also works on constrained state spaces, and is generally required.
        // path.interpolate();

        // // Then do whatever you want with the path, like normal!
        // for (auto state : path.getStates())
        // {
        //     std::cout << state << std::endl;
        // }
    }
    else
        OMPL_WARN("No solution found!");

}
