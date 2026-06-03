#include "config.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace {

V7d ParseV7Deg(const YAML::Node& node) {
    V7d v = V7d::Zero();
    if (!node || !node.IsSequence() || node.size() < DOF) {
        return v;
    }
    for (int i = 0; i < DOF; ++i) {
        v(i) = node[i].as<double>() * D2R;  // yaml 中 home/work 为度
    }
    return v;
}

V7d ParseV7(const YAML::Node& node) {
    V7d v = V7d::Zero();
    if (!node || !node.IsSequence() || node.size() < DOF) {
        return v;
    }
    for (int i = 0; i < DOF; ++i) {
        v(i) = node[i].as<double>();
    }
    return v;
}

JointLimit ParseJointLimit(const YAML::Node& node) {
    JointLimit lim;
    if (node["max_v"]) {
        lim.max_v = ParseV7(node["max_v"]);
    }
    if (node["max_a"]) {
        lim.max_a = ParseV7(node["max_a"]);
    }
    if (node["max_j"]) {
        lim.max_j = ParseV7(node["max_j"]);
    }
    return lim;
}

CartLimit ParseCartLimit(const YAML::Node& node) {
    CartLimit lim;
    if (node["max_line_v"]) {
        lim.max_line_v = node["max_line_v"].as<double>();
    }
    if (node["max_line_a"]) {
        lim.max_line_a = node["max_line_a"].as<double>();
    }
    if (node["max_line_j"]) {
        lim.max_line_j = node["max_line_j"].as<double>();
    }
    if (node["max_angle_v"]) {
        lim.max_angle_v = node["max_angle_v"].as<double>();
    }
    if (node["max_angle_a"]) {
        lim.max_angle_a = node["max_angle_a"].as<double>();
    }
    if (node["max_angle_j"]) {
        lim.max_angle_j = node["max_angle_j"].as<double>();
    }
    return lim;
}

void ParseArm(const YAML::Node& node, ArmConfig& arm) {
    if (node["arm_serial"]) {
        arm.arm_serial = node["arm_serial"].as<int>();
    }
    if (node["home_q"]) {
        arm.home_q = ParseV7Deg(node["home_q"]);
    }
    if (node["work_q"]) {
        arm.work_q = ParseV7Deg(node["work_q"]);
    }
    if (node["joint_limit"]) {
        arm.joint_limit = ParseJointLimit(node["joint_limit"]);
    }
    if (node["cart_limit"]) {
        arm.cart_limit = ParseCartLimit(node["cart_limit"]);
    }
}

}  // namespace

bool LoadMvConfig(const char* yaml_path, MvConfig& out) {
    try {
        {
            const YAML::Node root = YAML::LoadFile(yaml_path);
            if (root["connect"]) {
                const auto& c = root["connect"];
                if (c["ip"] && c["ip"].IsSequence() && c["ip"].size() >= 4) {
                    for (int i = 0; i < 4; ++i) {
                        out.connect_ip[i] = static_cast<uint8_t>(c["ip"][i].as<int>());
                    }
                }
                if (c["log_switch"]) {
                    out.log_switch = c["log_switch"].as<int>();
                }
            }
            if (root["sdk"] && root["sdk"]["kine_cfg"]) {
                out.kine_cfg_path = root["sdk"]["kine_cfg"].as<std::string>();
                const std::filesystem::path base =
                    std::filesystem::path(yaml_path).parent_path();
                const std::filesystem::path resolved = base / out.kine_cfg_path;
                if (std::filesystem::exists(resolved)) {
                    out.kine_cfg_path = resolved.string();
                }
            }
            if (root["left"]) {
                ParseArm(root["left"], out.left);
            }
            if (root["right"]) {
                ParseArm(root["right"], out.right);
            }
        }
        return true;
    } catch (const YAML::Exception&) {
        return false;
    }
}
