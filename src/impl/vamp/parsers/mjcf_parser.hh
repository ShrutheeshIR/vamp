#include <tinyxml2.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace tinyxml2;

/* =========================
   Math types
   ========================= */

namespace vamp::utils::parser{

struct Vec3 {
    double x{0}, y{0}, z{0};

    Vec3 operator+(const Vec3& o) const {
        return {x + o.x, y + o.y, z + o.z};
    }
};

struct Quat {
    double w{1}, x{0}, y{0}, z{0};
};

struct Pose {
    Vec3 pos;
    Quat rot;
};

/* =========================
   Geometry
   ========================= */

enum class GeomType {
    BOX,
    SPHERE,
    CYLINDER,
    UNKNOWN
};

struct Geom {
    GeomType type;
    Vec3 size;        // box: half-extents, sphere: radius in x, cylinder: (r, h/2)
    Pose world_pose;
    std::string body_name;   // parent body
    std::string geom_name;   // optional
};

/* =========================
   Math helpers
   ========================= */

Quat quatMul(const Quat& a, const Quat& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

Vec3 rotate(const Quat& q, const Vec3& v) {
    Quat p{0, v.x, v.y, v.z};
    Quat qi{q.w, -q.x, -q.y, -q.z};
    Quat r = quatMul(quatMul(q, p), qi);
    return {r.x, r.y, r.z};
}

/* =========================
   Parsing helpers
   ========================= */

Vec3 parseVec3(const char* s) {
    Vec3 v;
    if (!s) return v;
    std::stringstream ss(s);
    ss >> v.x >> v.y >> v.z;
    return v;
}

Quat parseQuat(const char* s) {
    Quat q;
    if (!s) return q;
    std::stringstream ss(s);
    ss >> q.w >> q.x >> q.y >> q.z;
    return q;
}

// MJCF euler = XYZ, radians
Quat eulerToQuat(const Vec3& e) {
    double cx = cos(e.x * 0.5), sx = sin(e.x * 0.5);
    double cy = cos(e.y * 0.5), sy = sin(e.y * 0.5);
    double cz = cos(e.z * 0.5), sz = sin(e.z * 0.5);

    return {
        cx*cy*cz + sx*sy*sz,
        sx*cy*cz - cx*sy*sz,
        cx*sy*cz + sx*cy*sz,
        cx*cy*sz - sx*sy*cz
    };
}

Pose parsePose(XMLElement* e) {
    Pose p;
    p.pos = parseVec3(e->Attribute("pos"));

    if (const char* q = e->Attribute("quat")) {
        p.rot = parseQuat(q);
    } else if (const char* r = e->Attribute("euler")) {
        p.rot = eulerToQuat(parseVec3(r));
    }
    // else identity

    return p;
}

GeomType parseGeomType(const char* s) {
    if (!s) return GeomType::UNKNOWN;
    if (strcmp(s, "box") == 0) return GeomType::BOX;
    if (strcmp(s, "sphere") == 0) return GeomType::SPHERE;
    if (strcmp(s, "cylinder") == 0) return GeomType::CYLINDER;
    return GeomType::UNKNOWN;
}

/* =========================
   Core traversal
   ========================= */

void parseBody(
    XMLElement* body,
    const Pose& parent_world,
    std::vector<Geom>& out_geoms)
{
    Pose body_local = parsePose(body);

    Pose body_world;
    body_world.rot = quatMul(parent_world.rot, body_local.rot);
    body_world.pos = parent_world.pos +
                     rotate(parent_world.rot, body_local.pos);
    const char* body_name_attr = body->Attribute("name");
    std::string body_name = body_name_attr ? body_name_attr : "";
    // Parse geoms
    for (XMLElement* geom = body->FirstChildElement("geom");
         geom;
         geom = geom->NextSiblingElement("geom")) {

        const char* cls = geom->Attribute("class");
        if (cls && std::string(cls) == "visual")
            continue;

        Geom g;
        g.type = parseGeomType(geom->Attribute("type"));
        if (g.type == GeomType::UNKNOWN)
            continue;

        g.size = parseVec3(geom->Attribute("size"));

        Pose geom_local = parsePose(geom);

        g.world_pose.rot = quatMul(body_world.rot, geom_local.rot);
        g.world_pose.pos = body_world.pos +
                           rotate(body_world.rot, geom_local.pos);

        const char* geom_name_attr = geom->Attribute("name");
        g.geom_name = geom_name_attr ? geom_name_attr : "";
        g.body_name = body_name;

        out_geoms.push_back(g);
    }

    // Recurse
    for (XMLElement* child = body->FirstChildElement("body");
         child;
         child = child->NextSiblingElement("body")) {

        parseBody(child, body_world, out_geoms);
    }
}

/* =========================
   Public API
   ========================= */

std::vector<Geom> parseMJCF(const std::string& filename) {
    XMLDocument doc;
    std::vector<Geom> geoms;

    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
        std::cerr << "Failed to load MJCF file\n";
        return geoms;
    }

    XMLElement* root = doc.FirstChildElement("mujoco");
    if (!root) {
        root = doc.FirstChildElement("mujocoinclude");
    }
    if (!root) {
        std::cerr << "No <mujoco> or <mujocoinclude> root found\n";
        return geoms;
    }

    XMLElement* worldbody = root->FirstChildElement("worldbody");
    if (!worldbody) {
        std::cerr << "No <worldbody> found\n";
        return geoms;
    }

    Pose world; // identity

    for (XMLElement* body = worldbody->FirstChildElement("body");
         body;
         body = body->NextSiblingElement("body")) {

        parseBody(body, world, geoms);
    }

    return geoms;
}

}
