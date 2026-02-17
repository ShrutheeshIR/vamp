#include <iostream>
#include <string>
#include <vector>
#include "mjcf_parser.hh"

/* =========================
   Example main
   ========================= */

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: mjcf_parser model.xml\n";
        return 1;
    }
    std::cout << "Parsing MJCF file...\n";

    auto geoms = vamp::utils::parser::parseMJCF(argv[1]);
    std::cout << "Parsing complete with " << geoms.size() << " geoms.\n";

    for (const auto& g : geoms) {
        std::cout << "Geom " << g.body_name << ", " << g.geom_name << " ";
        if (g.type == vamp::utils::parser::GeomType::BOX) std::cout << "BOX";
        else if (g.type == vamp::utils::parser::GeomType::SPHERE) std::cout << "SPHERE";
        else if (g.type == vamp::utils::parser::GeomType::CYLINDER) std::cout << "CYLINDER";

        std::cout << "\n  size: "
                  << g.size.x << " "
                  << g.size.y << " "
                  << g.size.z << "\n";

        std::cout << "  pos:  "
                  << g.world_pose.pos.x << " "
                  << g.world_pose.pos.y << " "
                  << g.world_pose.pos.z << "\n";

        std::cout << "  quat: "
                  << g.world_pose.rot.w << " "
                  << g.world_pose.rot.x << " "
                  << g.world_pose.rot.y << " "
                  << g.world_pose.rot.z << "\n\n";
    }

    return 0;
}
