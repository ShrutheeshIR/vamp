#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
// #include <vamp/planning/cbirrt.hh>
// #include <vamp/planning/task_space_constraints.hh>

#include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
// using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};


// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
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
static constexpr float radius = 0.2;


auto main(int, char **) -> int
{


    // Build sphere cage environment
    // EnvironmentInput environment;
    // for (const auto &sphere : problem)
    // {
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }

    // environment.sort();
    // auto env_v = EnvironmentVector(environment);




    // Create RNG for planning
    // auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
    std::array<float, 12> out;

    // auto idx = Robot::fk(start, out);
    // std::cout << "idx : " << idx << std::endl;

    // auto fkg = Robot::eefk(goal);
    // for (auto i = 0U; i < 12; i++)
    //     std::cout << out[i] << ", ";
    // std::cout << std::endl;


    Eigen::Vector<float, 6> lower_bound = {
        -3.14, -3.14, -3.14, -10.25, -10.0, -0.09
    };
    Eigen::Vector<float, 6> upper_bound = {
        3.14, 3.14, 3.14, 10.25, 10.0, 0.09
    };

    // const auto in_hand_pose = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();

    // fk(2, 3) -= 0.08;
    // const auto eef_target_pose = fk;

    // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
    Eigen::Matrix<float, 4, 4> T;
    T << 1,  0.000398163,  4.62412e-17, 0.30702, 0.000398163, -1, -6.92765e-12, -5.94873e-12, -2.7121e-15,  6.92765e-12, -1, 0.48527, 0.0, 0.0, 0.0, 1;
    Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);

    // const auto jacobian = Robot::jacobian(start);
    // std::cout << "-----Printing Jacobian------" << std::endl;
    // std::cout << jacobian << std::endl;
    // std::cout << "-----Printed Jacobian------" << std::endl;


    Eigen::Transform<float, 3, Eigen::Isometry> efk;
    Eigen::Matrix<float, 6, Eigen::Dynamic> J;

    Robot::jacobian_eefk(goal, J, efk);
    std::cout << "-----Printing Jacobian------" << std::endl;
    std::cout << J << std::endl;
    std::cout << "-----Printed Jacobian------" << std::endl;
    std::cout << efk.matrix() << std::endl;
    std::cout << target_pose.matrix() << std::endl;



    // Eigen::Matrix<float, 4, 4> T2;
    // T2 << -0.537531,    -0.544099,    -0.644218 , -0.753648 , 0.711539, -0.702646, -0.000256504 , 0.218136 , -0.452518, -0.458524, 0.764842 , -0.848243 , 0.0, 0.0, 0.0, 1;
    // T2 << -0.537748, 0.711259, -0.4527, -0.637404, 0.543885, 0.70293, 0.458344, 0.64535, 0.644218, 0.000256504, -0.764842, 0.321956, 0.0, 0.0, 0.0, 1.0;
    // Eigen::Transform<float, 3, Eigen::Isometry> og_pose(T2);
    const auto og_pose = Robot::eefk(goal);

    const auto error_se3 = og_pose.inverse() * target_pose;

    // Eigen::Map<Eigen::Vector<float, 16>> error_se3_vec(error_se3.matrix(), 16);

    std::cout << " --- EEF trans ---" << std::endl << og_pose.matrix() << std::endl;
    std::cout << " --- Goal trans ---" << std::endl << target_pose.matrix() << std::endl;
    std::cout << " --- Error trans ---" << std::endl << error_se3.matrix() << std::endl;

    std::array<float, 16> error_se3_array;
    for (auto i = 0U; i < 4; i++)
        for (auto j = 0U; j < 4; j++)
            error_se3_array[i*4 + j] = error_se3(j, i);


    std::array<float, Robot::dimension * 6 > jacobian_array;
    for (auto j = 0U; j < Robot::dimension; j++)
        for (auto i = 0U; i < 6; i++)
            jacobian_array[j*6 + i] = J(i, j);
    // for (auto i = 0U; i < 6; i++)
    //     for (auto j = 0U; j < Robot::dimension; j++)
    //         jacobian_array[i*Robot::dimension + j] = jacobian(i, j);


    // std::cout << error_se3_array[3] << ", " << error_se3_array[12] << std::endl;


    std::array<float, Robot::dimension> grad;
    std::array<float, 6> error_vec;

    const auto d = Robot::jacobian_solve(jacobian_array, error_se3_array, grad, error_vec);
    std::cout << d << std::endl;
    std::cout << "Print error " << std::endl;
    for (auto i = 0U; i < 6; i++)
        std::cout << error_vec[i] << ", ";
    std::cout << "Printed error " << std::endl;
    std::cout << "Print grad " << std::endl;
    for (auto i = 0U; i < 7; i++)
        std::cout << grad[i] << ", ";
    std::cout << "Printed grad " << std::endl;



    // Eigen::Transform<float, 3, Eigen::Isometry> efk;
    // Eigen::Matrix<float, 6, Eigen::Dynamic> J;

    std::cout << "Starting to project " << std::endl;
    Robot::ConfigurationArray q_ar = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};
    // Robot::Configuration q(q_ar);

    // for (auto k = 0U; k < Robot::dimension; k++)
    //     std::cout << q[k] << ", ";
    // std::cout << std::endl;

    for (auto i = 0U; i < 325; i++)
    {
        Robot::jacobian_eefk(q_ar, J, efk);

        // convert jacobian to array

        for (auto k = 0U; k < Robot::dimension; k++)
            for (auto j = 0U; j < 6; j++)
                jacobian_array[k*6 + j] = J(j, k);

        
        // // compute error matrix
        const auto error_se3_matrix = efk.inverse() * target_pose;

        // // convert to arr
        for (auto k = 0U; k < 4; k++)
            for (auto j = 0U; j < 4; j++)
                error_se3_array[k*4 + j] = error_se3_matrix(j, k);



        const auto err_norm = Robot::jacobian_solve(jacobian_array, error_se3_array, grad, error_vec);
        // for (auto k = 0U; k < Robot::dimension; k++)
        //     std::cout << grad[k] << ", ";
        // std::cout << std::endl;

        for (auto k = 0U; k < Robot::dimension; k++)
            q_ar[k] = q_ar[k] + grad[k] * 0.5;





        q_ar[0] = 0.16851471364498138 * (q_ar[0] - -2.967099905014038);
        q_ar[1] = 0.2728364169597626 * (q_ar[1] - -1.8325999975204468);
        q_ar[2] = 0.16851471364498138 * (q_ar[2] - -2.967099905014038);
        q_ar[3] = 0.30970299243927 * (q_ar[3] - -3.1415998935699463);
        q_ar[4] = 0.16851471364498138 * (q_ar[4] - -2.967099905014038);
        q_ar[5] = 0.2557806372642517 * (q_ar[5] - -0.08730000257492065);
        q_ar[6] = 0.16851471364498138 * (q_ar[6] - -2.967099905014038);
        for (auto k = 0U; k < Robot::dimension; k++)
        {
            if( q_ar[k] < 0.0)
                q_ar[k] = 0.0;
            if (q_ar[k] > 1.0)
                q_ar[k] = 1.0;

        }


        q_ar[0] = -2.967099905014038 + (q_ar[0] * 5.934199810028076);
        q_ar[1] = -1.8325999975204468 + (q_ar[1] * 3.6651999950408936);
        q_ar[2] = -2.967099905014038 + (q_ar[2] * 5.934199810028076);
        q_ar[3] = -3.1415998935699463 + (q_ar[3] * 3.2288999557495117);
        q_ar[4] = -2.967099905014038 + (q_ar[4] * 5.934199810028076);
        q_ar[5] = -0.08730000257492065 + (q_ar[5] * 3.909600019454956);
        q_ar[6] = -2.967099905014038 + (q_ar[6] * 5.934199810028076);

        // Robot::descale_configuration(q);
        // q.clamp(0.F, 1.F);
        // Robot::scale_configuration(q);
        // for (auto k = 0U; k < Robot::dimension; k++)
        //     std::cout << q_ar[k] << ", ";
        // std::cout << std::endl;
        



    }

    for (auto k = 0U; k < Robot::dimension; k++)
        std::cout << q_ar[k] << ", ";
    std::cout << std::endl;

    Robot::KinJacBlock<rake> JB;
    Robot::SE3Block<rake> TB;
    Robot::ConfigurationBlock<rake> grad_out;
    // vamp::FloatVector<rake, 6> error_out;
    Robot::ConfigurationBlock<rake> error_out;

    // std::vector<Robot::ConfigurationBlock<rake>> content = {grad_out, error_out};
    // std::vector<size_t> inds = {7, 7};


    // vamp::VecContainer c= {content, inds};
    // auto val = c[10];





    // const auto val = Robot::jacobian_solve_config(JB, TB, grad_out, error_out);



    // std::cout << grad;









    // vamp::planning::TaskSpaceConstraint<Robot> task_constraint(eef_target_pose, in_hand_pose, std::make_pair(lower_bound, upper_bound));


    // const Eigen::Vector<float, 6> distance_vec = task_constraint.distanceToConstraint(start);

    // std::cout << distance_vec << std::endl;

    // Robot::ConfigurationArray q_new;


    // const auto jacobian = Robot::jacobian(start);
    // std::cout << "-----Printing Jacobian------";
    // std::cout << jacobian << std::endl;


    // Eigen::Matrix<float, 6, Eigen::Dynamic> J;
    // Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;

    // Robot::jacobian_eefk(start, J, eef_pose);
    // std::cout << "-----Printing Jacobian EEFK ------";
    // std::cout << J << std::endl;
    // std::cout << eef_pose.matrix() << std::endl;
    // std::cout << "-----Printed Jacobian EEFK ------";


    // std::cout << "projecting" << std::endl;

    // double distance = task_constraint.projectStep(start, q_new, true);
    // std::cout << distance << std::endl;

    // // task_constraint.project(start, q_new);
    // for (float element : q_new) {
    //     std::cout << element << " ";
    // }
    // std::cout << std::endl;


    // distance = task_constraint.projectStepOld(start, q_new, true);
    // std::cout << distance << std::endl;

    // // task_constraint.project(start, q_new);
    // for (float element : q_new) {
    //     std::cout << element << " ";
    // }
    // std::cout << std::endl;

    // distance = task_constraint.projectStepJT(start, q_new, true);
    // std::cout << distance << std::endl;

    // // task_constraint.project(start, q_new);
    // for (float element : q_new) {
    //     std::cout << element << " ";
    // }
    // std::cout << std::endl;

    // bool success = task_constraint.project(start, q_new);
    // std::cout << success << std::endl;

    // // task_constraint.project(start, q_new);
    // for (float element : q_new) {
    //     std::cout << element << " ";
    // }
    // std::cout << std::endl;


    // // Setup RRTC and plan
    // vamp::planning::RRTCSettings rrtc_settings;
    // rrtc_settings.range = 1.0;

    // auto result =
    //     CRRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

    // if (result.path.size() > 0)
    // {

    //     // Output configurations of simplified path
    //     std::cout << std::fixed << std::setprecision(3);
    //     for (const auto &config : result.path)
    //     {
    //         const auto &array = config.to_array();
    //         Robot::ConfigurationArray soln;
    //         for (auto i = 0U; i < Robot::dimension; ++i)
    //         {
    //             std::cout << array[i] << ", ";
    //             soln[i] = array[i];
    //         }

    //         auto fka = Robot::eefk(soln);
    //         std::cout <<std::endl << fka.matrix() <<std::endl;

    //         std::cout << std::endl;
    //     }
    // }

    // Robot::ConfigurationBlock<rake> q;
    

    // std::vector<vamp::FloatVector<rake>> combined_vec;
    // std::array<float, vamp::FloatVector<rake>> combined_arr;
    // for (auto k = 0U; k < Robot::dimension; k++)
    // {
    //     const auto r = q.row(k);
    //     std::cout << r << std::endl;
    //     combined_arr[k] = r;
    // }
    // for (auto k = 0U; k < Robot::dimension; k++)
    // {
    //     const auto r = q.row(k);
    //     std::cout << r << std::endl;
    //     combined_arr[k] = r;
    // }

    // vamp::FloatVector<rake, Robot::dimension * 2> q2(combined_arr);

    

    return 0;
}
