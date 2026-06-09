/**
 * @file bt_nodes.cpp
 * @brief Implementation of Behavior Tree nodes for the RoboSub
 * pre-qualification mission.
 * @license Apache-2.0
 */

#include "bt_nodes.h"

/**
 * @brief Retrieves the shared context from the blackboard.
 */
static std::shared_ptr<RobotContext> getCtx(const BT::NodeConfig &cfg) {
  std::shared_ptr<RobotContext> ctx;
  if (!cfg.blackboard->get("robot_context", ctx)) {
    throw BT::RuntimeError("MISSING robot_context on blackboard.");
  }
  return ctx;
}

// --- AllSystemsOK -----------------------------------------------------------

BT::NodeStatus AllSystemsOK::tick() {
  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  // Evaluate every sensor condition into a single flag.
  // All failures are logged simultaneously so you can see everything that's
  // wrong at once.
  bool all_ok = true;

  if (!ctx->imu_received) {
    RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                         "[AllSystemsOK] Waiting for /imu ...");
    all_ok = false;
  }

  if (!ctx->altimeter_received) {
    RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                         "[AllSystemsOK] Waiting for /altimeter ...");
    all_ok = false;
  }

  if (!ctx->zed_ok) {
    RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                         "[AllSystemsOK] ZED camera not healthy.");
    all_ok = false;
  }

  if (!ctx->image_received) {
    RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                         "[AllSystemsOK] Waiting for image stream ...");
    all_ok = false;
  }

  double image_age =
      ctx->node->get_clock()->now().seconds() - ctx->last_image_t;
  if (ctx->image_received && image_age > 1.0) {
    RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                         "[AllSystemsOK] Image stream stale (%.1fs ago).",
                         image_age);
    all_ok = false;
  }

  if (!all_ok) {
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ctx->node->get_logger(), "[AllSystemsOK] All systems OK.");
  return BT::NodeStatus::SUCCESS;
}

// --- DiveToDepth ------------------------------------------------------------

BT::NodeStatus DiveToDepth::onStart() {
  auto depth_in = getInput<double>("target_depth");
  if (!depth_in)
    throw BT::RuntimeError("DiveToDepth: missing [target_depth]");
  target_z_ = depth_in.value();

  auto ctx = getCtx(config());
  ctx->target_depth = target_z_;
  RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Diving to z = %.2f m",
              target_z_);

  ctx->publishToPico(0.0f, 0.0f, (float)target_z_, 0);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DiveToDepth::onRunning() {
  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  double current_z = ctx->latest_altimeter;
  if (std::abs(target_z_ - current_z) < ctx->depth_tolerance) {
    RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Target depth reached.");
    return BT::NodeStatus::SUCCESS;
  }

  ctx->publishToPico(0.0f, 0.0f, (float)target_z_, 0);
  return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() { getCtx(config())->stopMotion(); }

// --- IsObjectSeen -----------------------------------------------------------

BT::NodeStatus IsObjectSeen::tick() {
  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  auto obj = getInput<std::string>("object");
  if (!obj)
    throw BT::RuntimeError("IsObjectSeen: missing [object]");

  return ctx->isObjectSeen(obj.value()) ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
}

// --- Do360Turn --------------------------------------------------------------

BT::NodeStatus Do360Turn::onStart() {
  auto obj = getInput<std::string>("success_when_seen");
  if (!obj)
    throw BT::RuntimeError("Do360Turn: missing [success_when_seen]");
  target_object_ = obj.value();

  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  prev_yaw_ = ctx->getCurrentYaw();
  accumulated_yaw_ = 0.0;

  RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] Searching for %s...",
              target_object_.c_str());
  ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360Turn::onRunning() {
  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  if (ctx->isObjectSeen(target_object_)) {
    ctx->stopMotion();
    RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s found.",
                target_object_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  double current_yaw = ctx->getCurrentYaw();
  accumulated_yaw_ += std::abs(normalizeAngle(current_yaw - prev_yaw_));
  prev_yaw_ = current_yaw;

  if (accumulated_yaw_ >= (2.0 * M_PI)) {
    ctx->stopMotion();
    RCLCPP_WARN(ctx->node->get_logger(),
                "[Do360Turn] Full rotation complete. %s not found.",
                target_object_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
  return BT::NodeStatus::RUNNING;
}

void Do360Turn::onHalted() { getCtx(config())->stopMotion(); }

// --- ApproachObject ---------------------------------------------------------

BT::NodeStatus ApproachObject::onStart() {
  auto obj = getInput<std::string>("object");
  auto thr = getInput<double>("threshold");
  if (!obj || !thr)
    throw BT::RuntimeError("ApproachObject: missing [object] or [threshold]");
  target_object_ = obj.value();
  threshold_ = thr.value();

  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  phase_ = Phase::ALIGN;
  smoothed_norm_x_ = 0.0f;

  RCLCPP_INFO(ctx->node->get_logger(),
              "[ApproachObject] Approaching %s to %.1f m",
              target_object_.c_str(), threshold_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachObject::onRunning() {
  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  double ox, oy, oz;
  bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz);

  if (phase_ == Phase::ALIGN) {
    if (!seen) {
      ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth,
                         0);
      return BT::NodeStatus::RUNNING;
    }

    double raw_norm_x = ox / std::max(oz, 0.5);
    smoothed_norm_x_ = 0.7f * smoothed_norm_x_ + 0.3f * (float)raw_norm_x;

    float deadband = (target_object_ == "GATE") ? ctx->gate_align_deadband
                                                : ctx->pole_align_deadband;
    if (std::abs(smoothed_norm_x_) < deadband) {
      phase_ = Phase::APPROACH;
      RCLCPP_INFO(ctx->node->get_logger(),
                  "[ApproachObject] Aligned. Moving to threshold.");
    } else {
      ctx->publishToPico(-(float)smoothed_norm_x_, 0.0f,
                         (float)ctx->target_depth, 0);
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::APPROACH) {
    if (!seen) {
      phase_ = Phase::ALIGN;
      return BT::NodeStatus::RUNNING;
    }

    if (oz < threshold_) {
      ctx->stopMotion();
      return BT::NodeStatus::SUCCESS;
    }

    double raw_norm_x = ox / std::max(oz, 0.5);
    smoothed_norm_x_ = 0.7f * smoothed_norm_x_ + 0.3f * (float)raw_norm_x;
    ctx->publishToPico(-(float)smoothed_norm_x_, ctx->base_surge_speed,
                       (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void ApproachObject::onHalted() { getCtx(config())->stopMotion(); }

// --- DriveThruGate ----------------------------------------------------------

BT::NodeStatus DriveThruGate::onStart() {
  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  locked_heading_ = ctx->getCurrentYaw();
  phase_ = Phase::ALIGN;
  gate_lost_frames_ = 0;
  smoothed_norm_x_ = 0.0f;

  RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Starting alignment.");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveThruGate::onRunning() {
  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);

  double ox, oy, oz;
  bool gate_seen = ctx->getObjectPosition("GATE", ox, oy, oz);

  if (phase_ == Phase::ALIGN) {
    if (!gate_seen) {
      ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth,
                         0);
      smoothed_norm_x_ = 0.0f;
      return BT::NodeStatus::RUNNING;
    }

    double raw_norm_x = ox / std::max(oz, 0.5);
    smoothed_norm_x_ = 0.7f * smoothed_norm_x_ + 0.3f * (float)raw_norm_x;

    if (std::abs(smoothed_norm_x_) < ctx->gate_align_deadband) {
      phase_ = Phase::DRIVE;
      locked_heading_ = ctx->getCurrentYaw();
      gate_lost_frames_ = 0;
      RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Aligned. Driving.");
    } else {
      ctx->publishToPico(-(float)smoothed_norm_x_, 0.0f,
                         (float)ctx->target_depth, 0);
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::DRIVE) {
    if (!gate_seen) {
      gate_lost_frames_++;

      // NOTE: After 8 missed frames, we assume the gate is cleared.
      if (gate_lost_frames_ >= 8) {
        RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Gate cleared.");
        ctx->stopMotion();
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      gate_lost_frames_ = 0;
    }

    double yaw_err = normalizeAngle(locked_heading_ - ctx->getCurrentYaw());
    ctx->publishToPico((float)yaw_err, ctx->base_surge_speed,
                       (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void DriveThruGate::onHalted() { getCtx(config())->stopMotion(); }

// --- OrbitPole --------------------------------------------------------------

BT::NodeStatus OrbitPole::onStart() {
  auto obj = getInput<std::string>("object");
  if (!obj)
    throw BT::RuntimeError("OrbitPole: missing [object]");
  target_object_ = obj.value();
  staystill_ = getInput<double>("staystill").value_or(0.0);

  steps_completed_ = 0;
  phase_ = Phase::TURN; // Starts with turn, assuming ApproachObject ran first.
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OrbitPole::onRunning() {
  auto ctx = getCtx(config());
  rclcpp::spin_some(ctx->node);
  double cur_yaw = ctx->getCurrentYaw();

  if (phase_ == Phase::STAY_STILL) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      stay_still_start_)
            .count() >= staystill_) {
      return BT::NodeStatus::SUCCESS;
    }
    ctx->stopMotion();
    return BT::NodeStatus::RUNNING;
  }

  if (steps_completed_ >= 8) {
    if (staystill_ > 0.01) {
      phase_ = Phase::STAY_STILL;
      stay_still_start_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(ctx->node->get_logger(),
                  "[OrbitPole] Orbit complete. Staying still.");
      ctx->stopMotion();
      return BT::NodeStatus::RUNNING;
    }
    ctx->stopMotion();
    return BT::NodeStatus::SUCCESS;
  }

  if (phase_ == Phase::TURN) {
    // Orbit logic: turn tangent (relative angle from YAML) then surge.
    if (start_time_ == 0.0) { // First time in this step's turn
      target_yaw_ =
          normalizeAngle(cur_yaw - (ctx->orbit_step_angle * M_PI / 180.0));
      start_time_ = 1.0; // flag
    }

    double yaw_err = normalizeAngle(target_yaw_ - cur_yaw);
    if (std::abs(yaw_err) < 0.08) {
      phase_ = Phase::SURGE;
      start_time_ = ctx->node->get_clock()->now().seconds();
    } else {
      ctx->publishToPico((float)yaw_err, 0.0f, (float)ctx->target_depth, 0);
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::SURGE) {
    if (ctx->node->get_clock()->now().seconds() - start_time_ >=
        ctx->orbit_surge_duration) {
      phase_ = Phase::TURN;
      start_time_ = 0.0; // reset flag
      steps_completed_++;
      RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Step %d/8 complete.",
                  steps_completed_);
      ctx->stopMotion();
    } else {
      ctx->publishToPico(0.0f, ctx->base_surge_speed, (float)ctx->target_depth,
                         0);
    }
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::RUNNING;
}

void OrbitPole::onHalted() { getCtx(config())->stopMotion(); }
