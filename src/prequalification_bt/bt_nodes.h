/**
 * @file bt_nodes.h
 * @brief Behavior Tree node declarations for the RoboSub pre-qualification
 * mission.
 * @license Apache-2.0
 */

#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "auv_msgs/msg/control_command.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "auv_msgs/action/surge.hpp"
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
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det_sub;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    zed_diag_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

  // --- MISSION CONFIGURATION (LOADED VIA YAML) ---
  float base_surge_speed = 0.1f;
  float base_yaw_speed = 0.1f;
  float gate_conf_thresh = 0.6f;
  float pole_conf_thresh = 0.3f;
  float gate_lock_thresh = 0.70f;  // min YOLO score to commit to gate approach
  float pole_lock_thresh = 0.45f;  // min score to commit to pole approach (HSV always = 0.5)
  float depth_tolerance = 0.15f;
  float gate_align_deadband = 0.04f;
  float pole_align_deadband = 0.06f;
  float orbit_surge_duration = 4.0f;
  float orbit_step_angle = 85.0f;

  double locked_yaw = 0.0;
  bool use_locked_yaw = false;

  /**
   * @brief Retrieves the current heading estimated from IMU.
   */
  double getCurrentYaw() {
    std::lock_guard<std::mutex> g(mtx);
    if (latest_imu) {
      tf2::Quaternion q(latest_imu->orientation.x, latest_imu->orientation.y,
        latest_imu->orientation.z, latest_imu->orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return yaw;
    }
    return 0.0;
  }

  /**
   * @brief Checks if a specific object is currently detected.
   */
  bool isObjectSeen(const std::string& object) {
    std::lock_guard<std::mutex> g(mtx);
    if (!latest_detections)
      return false;
    for (const auto& det : latest_detections->detections) {
      if (det.results.empty())
        continue;
      const auto& hyp = det.results[0].hypothesis;
      const auto& id = hyp.class_id;
      const auto& score = hyp.score;

      if (object == "GATE" && id == "preq_gate" && score >= gate_conf_thresh)
        return true;
      if (object == "POLE" && id == "preq_pole" && score >= pole_conf_thresh)
        return true;
    }
    return false;
  }

  /**
   * @brief Gets the 3D position of a detected object.
   */
  bool getObjectPosition(const std::string& object, double& ox, double& oy,
    double& oz, double* score_out = nullptr) {
    std::lock_guard<std::mutex> g(mtx);
    if (!latest_detections)
      return false;

    for (const auto& det : latest_detections->detections) {
      if (det.results.empty())
        continue;
      const auto& hyp = det.results[0].hypothesis;
      const auto& id = hyp.class_id;
      const auto& score = hyp.score;

      if (((object == "GATE" && id == "preq_gate" &&
        score >= gate_conf_thresh) ||
        (object == "POLE" && id == "preq_pole" &&
          score >= pole_conf_thresh))) {
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

  void stopMotion() { publishToPico(0.0f, 0.0f, (float)target_depth, 1); }
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
  AllSystemsOK(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("timeout_s", 20.0,
                                 "Seconds to wait before giving up") };
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
  IsObjectSeen(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("object") };
  }
  BT::NodeStatus tick() override;
};

// --- Action Nodes ----------------------------------------------------------

class DiveToDepth : public BT::StatefulActionNode {
public:
  DiveToDepth(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("target_depth") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  double target_z_ = 0.0;
};

class Do360Turn : public BT::StatefulActionNode {
public:
  Do360Turn(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("success_when_seen") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::string target_object_;
  double prev_yaw_ = 0.0, accumulated_yaw_ = 0.0;
  int confirm_frames_ = 0;
};

class DriveThruGate : public BT::StatefulActionNode {
public:
  DriveThruGate(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int gate_lost_frames_ = 0;
  double target_yaw_ = 0.0;
};

class ApproachObject : public BT::StatefulActionNode {
public:
  ApproachObject(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("object"),
      BT::InputPort<double>("threshold"),
      BT::InputPort<double>("angle", 0.0, "Optional angle parameter") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::string target_object_;
  double angle_ = 0.0;
  double threshold_ = 1.5;
  double target_yaw_ = 0.0;
  float smoothed_norm_x_ = 0.0f;
  bool locked_ = false;
};

class OrbitPole : public BT::StatefulActionNode {
public:
  OrbitPole(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("object"),
            BT::InputPort<double>("staystill", 0.0,
                                  "Seconds to pause at threshold before orbiting") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { STAY_STILL, TURN, SURGE };
  Phase phase_ = Phase::STAY_STILL;
  std::string target_object_;
  double target_yaw_ = 0.0;
  double surge_start_ = 0.0;
  int steps_completed_ = 0;
  double staystill_ = 0.0;
  bool turn_target_set_ = false;
  std::chrono::steady_clock::time_point stay_still_start_;
};

class StayStill : public BT::StatefulActionNode {
public:
  StayStill(const std::string &name, const BT::NodeConfig &config)
      : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("duration", 3.0,
                                  "Seconds to hold still")} ;
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  double duration_ = 0.0;
};

class SurgeForwardDistance : public BT::StatefulActionNode
{
public:
  SurgeForwardDistance(const std::string & name, const BT::NodeConfig & config)
    : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "metres to surge forward"),
    };
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

/**
 * @brief Registration helper for the Behavior Tree factory.
 */
inline void registerAllNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<AllSystemsOK>("AllSystemsOK");
  factory.registerNodeType<DiveToDepth>("DiveToDepth");
  factory.registerNodeType<IsObjectSeen>("IsObjectSeen");
  factory.registerNodeType<Do360Turn>("Do360Turn");
  factory.registerNodeType<DriveThruGate>("DriveThruGate");
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<OrbitPole>("OrbitPole");
  factory.registerNodeType<StayStill>("StayStill");
  factory.registerNodeType<SurgeForwardDistance>("SurgeForwardDistance");
}