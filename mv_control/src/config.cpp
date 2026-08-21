#include "config.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace {

V7d ParseV7Deg(const YAML::Node& node) {
    V7d v = V7d::Zero();
    if (!node || !node.IsSequence() || node.size() < DOF) {
        return v;
    }
    for (int i = 0; i < DOF; ++i) {
        v(i) = node[i].as<double>() * D2R;
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
        lim.max_v = ParseV7Deg(node["max_v"]);
    }
    if (node["max_a"]) {
        lim.max_a = ParseV7Deg(node["max_a"]);
    }
    if (node["max_j"]) {
        lim.max_j = ParseV7Deg(node["max_j"]);
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
    // YAML 单位 deg/s、deg/s²、deg/s³；内部 rad（与 joint_limit 一致）
    if (node["max_angle_v"]) {
        lim.max_angle_v = node["max_angle_v"].as<double>() * D2R;
    }
    if (node["max_angle_a"]) {
        lim.max_angle_a = node["max_angle_a"].as<double>() * D2R;
    }
    if (node["max_angle_j"]) {
        lim.max_angle_j = node["max_angle_j"].as<double>() * D2R;
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

ServoPdGain ParseServoPd(const YAML::Node& node, const ServoPdGain& fallback) {
    ServoPdGain g = fallback;
    if (node["p_gain"]) {
        g.p_gain = node["p_gain"].as<double>();
    }
    if (node["d_gain"]) {
        g.d_gain = node["d_gain"].as<double>();
    }
    return g;
}

void ParseServo(const YAML::Node& node, ServoConfig& servo) {
    if (node["servoj"]) {
        servo.servoj = ParseServoPd(node["servoj"], servo.servoj);
    }
    if (node["servop"]) {
        servo.servop = ParseServoPd(node["servop"], servo.servop);
    }
}

void ParseConnect(const YAML::Node& node, ConnectConfig& connect) {
    if (node["ip"] && node["ip"].IsSequence() && node["ip"].size() >= 4) {
        for (int i = 0; i < 4; ++i) {
            connect.ip[i] = static_cast<uint8_t>(node["ip"][i].as<int>());
        }
    }
    if (node["log_switch"]) {
        connect.log_switch = node["log_switch"].as<int>();
    }
    if (node["vel_ratio"]) {
        connect.vel_ratio = node["vel_ratio"].as<int>();
    }
    if (node["acc_ratio"]) {
        connect.acc_ratio = node["acc_ratio"].as<int>();
    }
    if (node["mode_transition_timeout_ms"]) {
        connect.mode_transition_timeout_ms =
            node["mode_transition_timeout_ms"].as<int>();
    }
    if (node["strict_init_state"]) {
        connect.strict_init_state = node["strict_init_state"].as<bool>();
    }
    if (node["servo_err_poll_cycles"]) {
        connect.servo_err_poll_cycles = node["servo_err_poll_cycles"].as<int>();
    }
}

void ParseImp(const YAML::Node& node, ImpConfig& imp) {
    if (node["joint"]) {
        const YAML::Node j = node["joint"];
        if (j["K"]) {
            imp.joint.K = ParseV7(j["K"]);
        }
        if (j["D"]) {
            imp.joint.D = ParseV7(j["D"]);
        }
    }
    if (node["cart"]) {
        const YAML::Node c = node["cart"];
        if (c["K"]) {
            imp.cart.K = ParseV7(c["K"]);
        }
        if (c["D"]) {
            imp.cart.D = ParseV7(c["D"]);
        }
        if (c["rot_type"]) {
            imp.cart.rot_type = c["rot_type"].as<int>();
        }
        if (c["cart_ctrl_para"] && c["cart_ctrl_para"].IsSequence()) {
            for (size_t i = 0; i < imp.cart.cart_ctrl_para.size() &&
                               i < c["cart_ctrl_para"].size();
                 ++i) {
                imp.cart.cart_ctrl_para[i] = c["cart_ctrl_para"][i].as<double>();
            }
        }
    }
    if (node["force"]) {
        const YAML::Node f = node["force"];
        if (f["fx_dir"] && f["fx_dir"].IsSequence() && f["fx_dir"].size() >= 6) {
            for (int i = 0; i < 6; ++i) {
                imp.force.fx_dir(i) = f["fx_dir"][i].as<double>();
            }
        }
        if (f["fc_adj_lmt"]) {
            imp.force.fc_adj_lmt = f["fc_adj_lmt"].as<double>();
        }
    }
    for (int i = 0; i < DOF; ++i) {
        imp.joint.K(i) = std::max(0.0, imp.joint.K(i));
        imp.joint.D(i) = std::clamp(imp.joint.D(i), 0.0, 1.0);
        imp.cart.K(i) = std::max(0.0, imp.cart.K(i));
        imp.cart.D(i) = std::clamp(imp.cart.D(i), 0.0, 1.0);
    }
}

}  // namespace

bool LoadMvConfig(const char* yaml_path, MvConfig& out) {
    try {
        const YAML::Node root = YAML::LoadFile(yaml_path);
        if (root["connect"]) {
            ParseConnect(root["connect"], out.connect);
        }
        if (root["sdk"] && root["sdk"]["urdf"]) {
            out.urdf_path = root["sdk"]["urdf"].as<std::string>();
            const std::filesystem::path base =
                std::filesystem::path(yaml_path).parent_path();
            const std::filesystem::path resolved = base / out.urdf_path;
            if (std::filesystem::exists(resolved)) {
                out.urdf_path = resolved.string();
            }
        }
        if (root["servo"]) {
            ParseServo(root["servo"], out.servo);
        }
        if (root["imp"]) {
            ParseImp(root["imp"], out.imp);
        }
        if (root["left"]) {
            ParseArm(root["left"], out.left);
        }
        if (root["right"]) {
            ParseArm(root["right"], out.right);
        }
        return true;
    } catch (const YAML::Exception&) {
        return false;
    }
}

bool ResolveRobotModelUrdf(const char* config_path, const char* model,
                           std::string& out) {
    if (config_path == nullptr || model == nullptr) {
        return false;
    }
    const std::filesystem::path cfg(config_path);
    const std::filesystem::path repo_root = cfg.parent_path().parent_path().parent_path();
    std::filesystem::path urdf;
    if (std::strcmp(model, "696") == 0) {
        urdf = repo_root / "urdf/Marvin_M6S_CCS_696_ urdf/urdf/"
                         "Marvin M6-S-CCS-696-V4.0_Base_and_Stand_Asm urdf.urdf";
    } else if (std::strcmp(model, "615") == 0) {
        urdf = repo_root / "urdf/Marvin_M3S_CCS_615_urdf/urdf/"
                         "Marvin_M3-S-CCS-615-V2.0_Base_and_Stand_Asm.urdf";
    } else {
        return false;
    }
    urdf = urdf.lexically_normal();
    if (!std::filesystem::is_regular_file(urdf)) {
        return false;
    }
    out = urdf.string();
    return true;
}
