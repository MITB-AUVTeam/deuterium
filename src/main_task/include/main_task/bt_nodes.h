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
#include <rclcpp_action/rclcpp_action.hpp>
#include "auv_msgs/action/surge.hpp"

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

    double locked_yaw = 0.0;
    bool use_locked_yaw = false;

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

            if ((object == "GATE" || object == "main_gate") && id == "main_gate" && score >= gate_conf_thresh)
                return true;
            if (object == "repair_and_survey" && id == "repair_and_survey" && score >= gate_conf_thresh)
                return true;
            if (object == "search_and_rescue" && id == "search_and_rescue" && score >= gate_conf_thresh)
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

            if ((object == "GATE" || object == "main_gate") && id == "main_gate" && score >= gate_conf_thresh) {
                ox = det.bbox.center.position.x;
                oy = det.bbox.center.position.y;
                oz = det.bbox.center.position.z;
                if (score_out) *score_out = score;
                return true;
            }
            if (object == "repair_and_survey" && id == "repair_and_survey" && score >= gate_conf_thresh) {
                ox = det.bbox.center.position.x;
                oy = det.bbox.center.position.y;
                oz = det.bbox.center.position.z;
                if (score_out) *score_out = score;
                return true;
            }
            if (object == "search_and_rescue" && id == "search_and_rescue" && score >= gate_conf_thresh) {
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
        return {BT::InputPort<std::string>("success_when_seen")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string target_object_;
    double prev_yaw_ = 0.0;
    double accumulated_yaw_ = 0.0;
    int confirm_frames_ = 0;
};

class CenterObject : public BT::StatefulActionNode {
public:
    CenterObject(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("object")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string target_object_;
    int align_confirm_frames_ = 0;
    double filtered_yaw_err_ = 0.0;
    bool is_holding_ = false;
    std::chrono::steady_clock::time_point hold_start_time_;
};

class FindAnyObject : public BT::ConditionNode {
public:
    FindAnyObject(const std::string &name, const BT::NodeConfig &config)
            : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("objects"),
                BT::OutputPort<std::string>("found_object")};
    }
    BT::NodeStatus tick() override;
};

class Do360TurnAny : public BT::StatefulActionNode {
public:
    Do360TurnAny(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("objects"),
                BT::OutputPort<std::string>("found_object")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::vector<std::string> targets_;
    double prev_yaw_ = 0.0;
    double accumulated_yaw_ = 0.0;
    int confirm_frames_ = 0;
    std::string found_target_;
};

class AlignWithObject : public BT::StatefulActionNode {
public:
    AlignWithObject(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("object")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string target_object_;
    int align_confirm_frames_ = 0;
    int gate_lost_frames_ = 0;
    double filtered_yaw_err_ = 0.0;
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
};

class StayStill : public BT::StatefulActionNode {
public:
    StayStill(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("duration", 3.0,
                                      "Seconds to hold still")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::chrono::steady_clock::time_point start_time_;
    double duration_ = 0.0;
};

class SurgeForwardDistance : public BT::StatefulActionNode {
public:
    SurgeForwardDistance(const std::string & name, const BT::NodeConfig & config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("distance", 1.0, "metres to surge forward")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double target_distance_ = 0.0;
    rclcpp_action::Client<auv_msgs::action::Surge>::SharedPtr surge_client_ = nullptr;
    rclcpp_action::ClientGoalHandle<auv_msgs::action::Surge>::SharedPtr goal_handle_ = nullptr;
    std::shared_future<rclcpp_action::ClientGoalHandle<auv_msgs::action::Surge>::WrappedResult> result_future_;
    bool action_sent_ = false;
};

class SaveCurrentYaw : public BT::SyncActionNode {
public:
    SaveCurrentYaw(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class AlignToLockedYaw : public BT::StatefulActionNode {
public:
    AlignToLockedYaw(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    int align_confirm_frames_ = 0;
};

inline void registerAllNodes(BT::BehaviorTreeFactory &factory) {
    factory.registerNodeType<AllSystemsOK>("AllSystemsOK");
    factory.registerNodeType<DiveToDepth>("DiveToDepth");
    factory.registerNodeType<IsObjectSeen>("IsObjectSeen");
    factory.registerNodeType<Do360Turn>("Do360Turn");
    factory.registerNodeType<CenterObject>("CenterObject");
    factory.registerNodeType<FindAnyObject>("FindAnyObject");
    factory.registerNodeType<Do360TurnAny>("Do360TurnAny");
    factory.registerNodeType<AlignWithObject>("AlignWithObject");
    factory.registerNodeType<AlignAndApproachObject>("AlignAndApproachObject");
    factory.registerNodeType<StayStill>("StayStill");
    factory.registerNodeType<SurgeForwardDistance>("SurgeForwardDistance");
    factory.registerNodeType<SaveCurrentYaw>("SaveCurrentYaw");
    factory.registerNodeType<AlignToLockedYaw>("AlignToLockedYaw");
}
