#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>
#include <vamp/planning/simplify_constraints.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;

int main()
{
    // read Configs from /src/trajectory.txt
    vamp::planning::Path<Robot> planned_path;

    // Try the same path used elsewhere; fall back to the repo copy if needed
    std::ifstream infile("/src/trajectory.txt");
    if (!infile.is_open()) {
        infile.open("vamp/trajectory.txt");
    }

    if (!infile.is_open()) {
        std::cerr << "Failed to open trajectory file '/src/trajectory.txt' or 'vamp/trajectory.txt'\n";
        return 1;
    }

    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        Robot::ConfigurationArray arr{};
        std::string token;
        std::size_t idx = 0;

        while (std::getline(ss, token, ',') && idx < Robot::dimension) {
            // simple parsing, matches how the file was written
            try {
                arr[idx++] = std::stof(token);
            } catch (...) {
                // malformed token -> skip this line
                idx = 0;
                break;
            }
        }

        if (idx != Robot::dimension) {
            std::cerr << "Skipping line: expected " << Robot::dimension << " values, got " << idx << "\n";
            continue;
        }

        planned_path.emplace_back(Robot::Configuration(arr));
    }

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


    EnvironmentInput environment;
    environment.sort();
    auto env_v = EnvironmentVector(environment);
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


    vamp::planning::SimplifySettings simplify_settings;
    auto simplify_result = vamp::planning::constraint::simplify_with_constraints<Robot, rake, Robot::resolution, decltype(tsr_constraint)>(
        planned_path, env_v, task_constraint, simplify_settings, rng, vamp::planning::ProjMethod::InnerLM, 0.75, 10, 0.001F, false);


    std::cout << "Read " << planned_path.size() << " configurations into planned_path\n";



    return 0;
}
