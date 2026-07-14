#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "auv_msgs/msg/control_command.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

struct RobotContext {
    rclcpp::Node::SharedPtr node;
    std::mutex mtx;
    sensor_msgs::msg::Imu::SharedPtr latest_imu;
    double latest_altimeter = 0.0;
    double target_depth = 0.0;
    vision_msgs::msg::Detection3DArray::SharedPtr latest_detections;

    bool imu_received = false;
    bool altimeter_received = false;
    bool zed_ok = true;
    bool image_received = false;
    double last_image_t = 0.0;

    rclcpp::Publisher<auv_msgs::msg::ControlCommand>::SharedPtr pico_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr alt_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr zed_diag_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    // --- MISSION CONFIGURATION (LOADED VIA YAML) ---
    float base_surge_speed = 1.5f;
    float base_yaw_speed = 0.1f;
    float gate_conf_thresh = 0.6f;
    float pole_conf_thresh = 0.3f;
    float depth_tolerance = 0.15f;
    float pole_approach_threshold = 1.0f;
    float gate_staystill_time = 3.0f;
    float align_obj_threshold = 0.04f;
    float align_yaw_threshold = 0.10f;
    float camera_alpha = 0.25f;
    int align_confirm_frames = 5;
    int gate_lost_timeout_frames = 30;

    double getCurrentYaw() {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_imu)
            return 0.0;
        tf2::Quaternion q(
                latest_imu->orientation.x,
                latest_imu->orientation.y,
                latest_imu->orientation.z,
                latest_imu->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool isObjectSeen(const std::string &object) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections)
            return false;
        for (const auto &det : latest_detections->detections) {
            if (det.results.empty())
                continue;
            const auto &hyp = det.results[0].hypothesis;
            const auto &id = hyp.class_id;
            const auto &score = hyp.score;

            float thresh = (id.find("pole") != std::string::npos) ? pole_conf_thresh : gate_conf_thresh;

            if ((object == "GATE" || object == "main_gate") && id == "main_gate" && score >= thresh)
                return true;
            if (object == id && score >= thresh)
                return true;
        }
        return false;
    }

    bool getObjectPosition(const std::string &object, double &ox, double &oy,
                           double &oz, double *score_out = nullptr) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections)
            return false;

        for (const auto &det : latest_detections->detections) {
            if (det.results.empty())
                continue;
            const auto &hyp = det.results[0].hypothesis;
            const auto &id = hyp.class_id;
            const auto &score = hyp.score;

            float thresh = (id.find("pole") != std::string::npos) ? pole_conf_thresh : gate_conf_thresh;

            bool match = false;
            if ((object == "GATE" || object == "main_gate") && id == "main_gate" && score >= thresh) {
                match = true;
            } else if (object == id && score >= thresh) {
                match = true;
            }

            if (match) {
                ox = det.bbox.center.position.x;
                oy = det.bbox.center.position.y;
                oz = det.bbox.center.position.z;
                if (score_out) *score_out = score;
                return true;
            }
        }
        return false;
    }

    void publishToPico(float delta_theta, float delta_distance,
                                         float target_depth_val, uint8_t stop_thrusters) {
        auv_msgs::msg::ControlCommand msg;
        msg.delta_theta = delta_theta;
        msg.delta_distance = delta_distance;
        msg.target_depth = target_depth_val;
        msg.stop_thrusters = stop_thrusters;
        pico_pub->publish(msg);
    }

    void stopMotion() {
        publishToPico(0.0f, 0.0f, target_depth, 1);
    }
};

// --- Math Utilities --------------------------------------------------------

inline double clampVal(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double normalizeAngle(double a) {
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

// --- Condition Nodes -------------------------------------------------------

class AllSystemsOK : public BT::StatefulActionNode {
public:
    AllSystemsOK(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("timeout_s", 20.0,
                                                                 "Seconds to wait before giving up")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::chrono::steady_clock::time_point start_time_;
    double timeout_s_ = 20.0;
};

class IsObjectSeen : public BT::ConditionNode {
public:
    IsObjectSeen(const std::string &name, const BT::NodeConfig &config)
            : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("object")};
    }
    BT::NodeStatus tick() override;
};

// --- Action Nodes ----------------------------------------------------------

class DiveToDepth : public BT::StatefulActionNode {
public:
    DiveToDepth(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("target_depth")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double target_z_ = 0.0;
};

class Do360Turn : public BT::StatefulActionNode {
public:
    Do360Turn(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("success_when_seen"),
                BT::InputPort<int>("max_turns", 2, "Max full rotations before stopping")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string target_object_;
    double prev_yaw_ = 0.0;
    double accumulated_yaw_ = 0.0;
    int confirm_frames_ = 0;
    int max_turns_ = 2;
    bool object_found_ = false;
};

class AlignAndApproachObject : public BT::StatefulActionNode {
public:
    AlignAndApproachObject(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("object")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string target_object_;
    int lost_frames_ = 0;
    bool has_seen_ = false;
};

inline void registerAllNodes(BT::BehaviorTreeFactory &factory) {
    factory.registerNodeType<AllSystemsOK>("AllSystemsOK");
    factory.registerNodeType<DiveToDepth>("DiveToDepth");
    factory.registerNodeType<IsObjectSeen>("IsObjectSeen");
    factory.registerNodeType<Do360Turn>("Do360Turn");
    factory.registerNodeType<AlignAndApproachObject>("AlignAndApproachObject");
}
