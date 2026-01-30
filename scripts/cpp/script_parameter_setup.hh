#pragma once
#include <string>
#include <stdexcept>
#include <iostream>
#include <cstdlib>
#include "problem_setup/line_constraint_no_orientation_problem_setup.hh"
#include "problem_setup/parse_obstacle_textfile.hh"

struct ProgramParameters
{
    bool optimize = false;
    std::string obstacle_file = default_obstacle_filepath;
};

inline ProgramParameters parse_args(int argc, char **argv)
{
    ProgramParameters param;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "--optimize")
        {
            param.optimize = true;
        }
        else if (arg == "--obstacles")
        {
            if (i + 1 >= argc)
                throw std::runtime_error("--obstacles requires a filename");

            param.obstacle_file = argv[++i];
        }
        else if (arg == "--help")
        {
            std::cout << "Usage:\n"
                      << "  --optimize\n"
                      << "  --obstacles <file>\n";
            std::exit(0);
        }
        else
        {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    return param;
}