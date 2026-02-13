#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/vector.hh>
// #include <vamp/planning/simplify.hh>
#include <vamp/robots/digit.hh>

using Robot = vamp::robots::Digit;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;

static constexpr Robot::ConfigurationArray start = {
    0.00438741,    0.00952297,    -0.149783,    -0.00444245,    0.00167457,    -0.0013295,
    0.366443,    -0.000246321,    0.094638,    -0.102846,     0.139485,   -0.087064,    -0.00352648,
    -0.102535,    0.843245,    -0.010127,    0.450957,
    -0.360956,    0.00656994,    -0.0956402,    0.0981786,    -0.134632,    0.0859524,    0.0232191,
    0.103937,     -0.891019, 0.00527577,  -0.369154,
};

static constexpr Robot::ConfigurationArray goal = {
    0.00352543,    0.0160413,    -0.48349,    -0.00927372,    0.000426254,    -0.00362023,
    0.372489,    0.007467,    -0.226769,    -0.850073,    0.911696,    -0.430518,    0.0120371,
    -0.110171,    0.82905,    0.0310924,    0.456459,
    -0.342646,    0.0112654,    0.228212,    0.842371,    -0.903408,   0.431292,    0.0528911,
    0.102618,    -0.891391, 0.00519742,    -0.369172,
};

int main() {
    std::cout << std::fixed << std::setprecision(5);

    auto fixed = Robot::add_closed_link_joints(goal);

    for (auto& joint : fixed) {
        std::cout << joint << " ";
    }
    std::cout << std::endl;

}
