#pragma once
#include <vector>
#include <array>
#include <filesystem>


inline std::vector<std::array<float, 3>> problem(const std::string& filename)
{
    //std::cout << "Current working directory: "
    //          << std::filesystem::current_path() << std::endl;
    std::cout << "Building the obstacles using the file " << filename << "\n";
    std::ifstream file(filename);
    if (!file)
        throw std::runtime_error("Could not open obstacle file: " + filename);

    std::vector<std::array<float, 3>> obstacles;
    std::string line;
    float x, y, z;

   while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::stringstream ss(line);
        float x, y, z;

        if (!(ss >> x >> y >> z))
            throw std::runtime_error("Parse error in obstacle file: " + line);

        obstacles.push_back({x, y, z});
    }

    return obstacles;
}
