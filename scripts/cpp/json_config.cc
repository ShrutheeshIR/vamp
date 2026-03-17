#include "json_config.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <fstream>
#include <iterator>
#include <sstream>

#include <nlohmann/json.hpp>

namespace vamp
{
    namespace config
    {

        namespace
        {

            // Strips // line comments and /* block comments */ from the input.
            // Preserves content within JSON strings.
            std::string StripJsonComments(std::string_view in)
            {
                std::string out;
                out.reserve(in.size());

                enum class State
                {
                    kNormal,
                    kInString,
                    kInLineComment,
                    kInBlockComment
                };
                State state = State::kNormal;
                bool prev_was_backslash = false;

                for (size_t i = 0; i < in.size(); ++i)
                {
                    char c = in[i];
                    char next = (i + 1 < in.size()) ? in[i + 1] : '\0';

                    switch (state)
                    {
                        case State::kNormal:
                            if (c == '/' && next == '/')
                            {
                                state = State::kInLineComment;
                                ++i;  // consume second slash
                            }
                            else if (c == '/' && next == '*')
                            {
                                state = State::kInBlockComment;
                                ++i;  // consume '*'
                            }
                            else if (c == '"')
                            {
                                out.push_back(c);
                                state = State::kInString;
                                prev_was_backslash = false;
                            }
                            else
                            {
                                out.push_back(c);
                            }
                            break;

                        case State::kInString:
                            out.push_back(c);
                            if (c == '"' && !prev_was_backslash)
                            {
                                state = State::kNormal;
                            }
                            if (c == '\\' && !prev_was_backslash)
                            {
                                prev_was_backslash = true;
                            }
                            else
                            {
                                prev_was_backslash = false;
                            }
                            break;

                        case State::kInLineComment:
                            if (c == '\n')
                            {
                                out.push_back(c);
                                state = State::kNormal;
                            }
                            break;

                        case State::kInBlockComment:
                            if (c == '*' && next == '/')
                            {
                                ++i;  // consume '/'
                                state = State::kNormal;
                            }
                            break;
                    }
                }

                return out;
            }

            // Removes trailing commas before closing ']' or '}'.
            // Only operates outside JSON string literals.
            std::string RemoveTrailingCommas(std::string_view in)
            {
                std::string out;
                out.reserve(in.size());

                bool in_string = false;
                bool prev_was_backslash = false;

                for (size_t i = 0; i < in.size(); ++i)
                {
                    char c = in[i];

                    if (!in_string)
                    {
                        if (c == '"')
                        {
                            in_string = true;
                            prev_was_backslash = false;
                            out.push_back(c);
                            continue;
                        }
                        if (c == ',')
                        {
                            // Lookahead to see if next non-space is ] or }
                            size_t j = i + 1;
                            while (j < in.size() && std::isspace(static_cast<unsigned char>(in[j])))
                            {
                                ++j;
                            }
                            if (j < in.size() && (in[j] == ']' || in[j] == '}'))
                            {
                                // Skip this trailing comma.
                                continue;
                            }
                        }
                        out.push_back(c);
                    }
                    else
                    {
                        // In string literal
                        out.push_back(c);
                        if (c == '"' && !prev_was_backslash)
                        {
                            in_string = false;
                        }
                        if (c == '\\' && !prev_was_backslash)
                        {
                            prev_was_backslash = true;
                        }
                        else
                        {
                            prev_was_backslash = false;
                        }
                    }
                }

                return out;
            }

            bool JsonNumberToDouble(const nlohmann::json &j, double &out)
            {
                if (j.is_number_float())
                {
                    out = j.get<double>();
                    return true;
                }
                if (j.is_number_integer())
                {
                    out = static_cast<double>(j.get<long long>());
                    return true;
                }
                if (j.is_number_unsigned())
                {
                    out = static_cast<double>(j.get<unsigned long long>());
                    return true;
                }
                return false;
            }

            bool ParseDoubleArray(
                const nlohmann::json &j,
                std::vector<double> &out,
                std::string *err,
                const char *field_name)
            {
                if (!j.is_array())
                {
                    if (err)
                    {
                        *err = std::string("Field '") + field_name + "' must be an array.";
                    }
                    return false;
                }
                out.clear();
                out.reserve(j.size());
                for (size_t i = 0; i < j.size(); ++i)
                {
                    const auto &elt = j[i];
                    double v;
                    if (!JsonNumberToDouble(elt, v))
                    {
                        if (err)
                        {
                            std::ostringstream oss;
                            oss << "Element " << i << " of '" << field_name << "' must be a number.";
                            *err = oss.str();
                        }
                        return false;
                    }
                    out.push_back(v);
                }
                return true;
            }

            bool ParseSpheres(const nlohmann::json &spheres_obj, std::vector<Sphere> &out, std::string *err)
            {
                out.clear();
                if (spheres_obj.is_null())
                {
                    return true;
                }
                if (!spheres_obj.is_object())
                {
                    if (err)
                    {
                        *err = "env.spheres must be an object mapping id -> [x, y, z, r].";
                    }
                    return false;
                }

                for (auto it = spheres_obj.begin(); it != spheres_obj.end(); ++it)
                {
                    const std::string id = it.key();
                    const auto &arr = it.value();
                    if (!arr.is_array() || arr.size() != 4)
                    {
                        if (err)
                        {
                            std::ostringstream oss;
                            oss << "env.spheres['" << id << "'] must be an array of length 4: [x,y,z,r].";
                            *err = oss.str();
                        }
                        return false;
                    }
                    Sphere s;
                    s.id = id;
                    double v;
                    for (int i = 0; i < 4; ++i)
                    {
                        if (!JsonNumberToDouble(arr[i], v))
                        {
                            if (err)
                            {
                                std::ostringstream oss;
                                oss << "env.spheres['" << id << "'] element " << i << " must be a number.";
                                *err = oss.str();
                            }
                            return false;
                        }
                        switch (i)
                        {
                            case 0:
                                s.x = v;
                                break;
                            case 1:
                                s.y = v;
                                break;
                            case 2:
                                s.z = v;
                                break;
                            case 3:
                                s.r = v;
                                break;
                        }
                    }
                    out.push_back(std::move(s));
                }
                return true;
            }

            bool ParseCuboids(const nlohmann::json &cuboids_obj, std::vector<Cuboid> &out, std::string *err)
            {
                out.clear();
                if (cuboids_obj.is_null())
                {
                    return true;
                }
                if (!cuboids_obj.is_object())
                {
                    if (err)
                    {
                        *err = "env.cuboids must be an object mapping id -> [x, y, z, yaw, pitch, roll, dx, "
                               "dy, dz].";
                    }
                    return false;
                }

                for (auto it = cuboids_obj.begin(); it != cuboids_obj.end(); ++it)
                {
                    const std::string id = it.key();
                    const auto &arr = it.value();
                    if (!arr.is_array() || arr.size() != 9)
                    {
                        if (err)
                        {
                            std::ostringstream oss;
                            oss << "env.cuboids['" << id
                                << "'] must be an array of length 9: [x,y,z,yaw,pitch,roll,dx,dy,dz].";
                            *err = oss.str();
                        }
                        return false;
                    }
                    Cuboid c;
                    c.id = id;
                    double v;
                    for (int i = 0; i < 9; ++i)
                    {
                        if (!JsonNumberToDouble(arr[i], v))
                        {
                            if (err)
                            {
                                std::ostringstream oss;
                                oss << "env.cuboids['" << id << "'] element " << i << " must be a number.";
                                *err = oss.str();
                            }
                            return false;
                        }
                        switch (i)
                        {
                            case 0:
                                c.x = v;
                                break;
                            case 1:
                                c.y = v;
                                break;
                            case 2:
                                c.z = v;
                                break;
                            case 3:
                                c.yaw = v;
                                break;
                            case 4:
                                c.pitch = v;
                                break;
                            case 5:
                                c.roll = v;
                                break;
                            case 6:
                                c.dx = v;
                                break;
                            case 7:
                                c.dy = v;
                                break;
                            case 8:
                                c.dz = v;
                                break;
                        }
                    }
                    out.push_back(std::move(c));
                }
                return true;
            }

        }  // namespace

        bool ParseConfigFromJson(
            std::string_view json_text,
            Config &out,
            std::string *error_message,
            const ParseOptions &options)
        {
            std::string text(json_text);
            // Best-effort preprocessing according to options.
            if (options.allow_comments)
            {
                text = StripJsonComments(text);
            }
            if (options.allow_trailing_commas)
            {
                text = RemoveTrailingCommas(text);
            }

            nlohmann::json j;
            try
            {
                // Note: nlohmann::json provides limited parser options; our preprocessing above
                // handles comments/trailing commas best-effort.
                j = nlohmann::json::parse(text);
            }
            catch (const nlohmann::json::exception &e)
            {
                if (error_message)
                {
                    *error_message = std::string("JSON parse error: ") + e.what();
                }
                return false;
            }

            // Required top-level keys: "constraint", "problem", "env" (env may be optional contents).
            if (!j.is_object())
            {
                if (error_message)
                {
                    *error_message = "Top-level JSON must be an object.";
                }
                return false;
            }

            // constraint
            if (!j.contains("constraint") || !j["constraint"].is_object())
            {
                if (error_message)
                {
                    *error_message = "Missing or invalid 'constraint' object.";
                }
                return false;
            }
            const auto &jc = j["constraint"];
            if (!jc.contains("constraint_type") || !jc["constraint_type"].is_string())
            {
                if (error_message)
                {
                    *error_message = "constraint.constraint_type must be a string.";
                }
                return false;
            }
            out.constraint.constraint_type = jc["constraint_type"].get<std::string>();

            if (!jc.contains("lower_bound") || !jc.contains("upper_bound"))
            {
                if (error_message)
                {
                    *error_message = "constraint.lower_bound and constraint.upper_bound are required.";
                }
                return false;
            }
            {
                std::string err;
                if (!ParseDoubleArray(
                        jc["lower_bound"], out.constraint.lower_bound, &err, "constraint.lower_bound"))
                {
                    if (error_message)
                    {
                        *error_message = err;
                    }
                    return false;
                }
                if (!ParseDoubleArray(
                        jc["upper_bound"], out.constraint.upper_bound, &err, "constraint.upper_bound"))
                {
                    if (error_message)
                    {
                        *error_message = err;
                    }
                    return false;
                }
            }

            // problem
            if (!j.contains("problem") || !j["problem"].is_object())
            {
                if (error_message)
                {
                    *error_message = "Missing or invalid 'problem' object.";
                }
                return false;
            }
            const auto &jp = j["problem"];
            if (!jp.contains("start") || !jp.contains("goal"))
            {
                if (error_message)
                {
                    *error_message = "problem.start and problem.goal are required.";
                }
                return false;
            }
            {
                std::string err;
                if (!ParseDoubleArray(jp["start"], out.problem.start, &err, "problem.start"))
                {
                    if (error_message)
                    {
                        *error_message = err;
                    }
                    return false;
                }
                if (!ParseDoubleArray(jp["goal"], out.problem.goal, &err, "problem.goal"))
                {
                    if (error_message)
                    {
                        *error_message = err;
                    }
                    return false;
                }
            }

            // env
            if (!j.contains("env") || !j["env"].is_object())
            {
                if (error_message)
                {
                    *error_message = "Missing or invalid 'env' object.";
                }
                return false;
            }
            const auto &je = j["env"];
            {
                std::string err;
                if (je.contains("spheres"))
                {
                    if (!ParseSpheres(je["spheres"], out.env.spheres, &err))
                    {
                        if (error_message)
                        {
                            *error_message = err;
                        }
                        return false;
                    }
                }
                else
                {
                    out.env.spheres.clear();
                }

                if (je.contains("cuboids"))
                {
                    if (!ParseCuboids(je["cuboids"], out.env.cuboids, &err))
                    {
                        if (error_message)
                        {
                            *error_message = err;
                        }
                        return false;
                    }
                }
                else
                {
                    out.env.cuboids.clear();
                }
            }

            // Optional semantic validation here (separate API also provided).
            std::string validation_error;
            if (!ValidateConfig(out, &validation_error))
            {
                if (error_message)
                {
                    *error_message = validation_error;
                }
                return false;
            }

            return true;
        }

        bool ParseConfigFromFile(
            const std::string &file_path,
            Config &out,
            std::string *error_message,
            const ParseOptions &options)
        {
            std::ifstream ifs(file_path, std::ios::in | std::ios::binary);
            if (!ifs)
            {
                if (error_message)
                {
                    std::ostringstream oss;
                    oss << "Failed to open file '" << file_path << "'.";
                    *error_message = oss.str();
                }
                return false;
            }
            std::string content;
            ifs.seekg(0, std::ios::end);
            content.reserve(static_cast<size_t>(ifs.tellg()));
            ifs.seekg(0, std::ios::beg);
            content.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

            return ParseConfigFromJson(content, out, error_message, options);
        }

        bool ValidateConfig(const Config &cfg, std::string *error_message)
        {
            const auto &lb = cfg.constraint.lower_bound;
            const auto &ub = cfg.constraint.upper_bound;

            if (lb.size() != ub.size())
            {
                if (error_message)
                {
                    *error_message = "constraint.lower_bound and constraint.upper_bound must have the same "
                                     "size.";
                }
                return false;
            }
            if (lb.empty())
            {
                if (error_message)
                {
                    *error_message = "constraint bounds must not be empty.";
                }
                return false;
            }
            const size_t dim = lb.size();

            if (cfg.problem.start.size() != cfg.problem.goal.size())
            {
                if (error_message)
                {
                    *error_message = "problem.start and problem.goal must have the same size.";
                }
                return false;
            }
            if (cfg.problem.start.size() != dim)
            {
                if (error_message)
                {
                    std::ostringstream oss;
                    oss << "problem.start/problem.goal must have size " << dim
                        << " to match constraint bounds.";
                    *error_message = oss.str();
                }
                return false;
            }

            // Check bounds consistency: lb[i] <= ub[i]
            for (size_t i = 0; i < dim; ++i)
            {
    if (lb
