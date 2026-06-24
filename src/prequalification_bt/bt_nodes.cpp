/**
 * @file bt_nodes.cpp
 * @brief Implementation of Behavior Tree nodes for the RoboSub pre-qualification mission.
 * @license Apache-2.0
 */

#include "bt_nodes.h"

 /**
  * @brief Retrieves the shared context from the blackboard.
  */
static std::shared_ptr<RobotContext> getCtx(const BT::NodeConfig& cfg) {
    std::shared_ptr<RobotContext> ctx;
    if (!cfg.blackboard->get("robot_context", ctx)) {
        throw BT::RuntimeError("MISSING robot_context on blackboard.");
    }
    return ctx;
}
double turn_start_ = 0.0;
double target_yaw_ = 0;
// --- AllSystemsOK -----------------------------------------------------------

BT::NodeStatus AllSystemsOK::onStart() {
    auto timeout_in = getInput<double>("timeout_s");
    timeout_s_ = timeout_in ? timeout_in.value() : 20.0;
    start_time_ = std::chrono::steady_clock::now();

    auto ctx = getCtx(config());
    RCLCPP_INFO(ctx->node->get_logger(),
        "[AllSystemsOK] Waiting for all systems (timeout %.0f s)...", timeout_s_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AllSystemsOK::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();

    bool all_ok = true;

    if (!ctx->imu_received) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
            "[AllSystemsOK] Waiting for /imu ... (%.0fs elapsed)", elapsed);
        all_ok = false;
    }

    // if (!ctx->altimeter_received) {
    //     RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
    //                          "[AllSystemsOK] Waiting for /pressure ... (%.0fs elapsed)", elapsed);
    //     all_ok = false;
    // }

    if (!ctx->zed_ok) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
            "[AllSystemsOK] ZED camera not healthy. (%.0fs elapsed)", elapsed);
        all_ok = false;
    }

    if (!ctx->image_received) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
            "[AllSystemsOK] Waiting for image stream ... (%.0fs elapsed)", elapsed);
        all_ok = false;
    }

    double image_age = ctx->node->get_clock()->now().seconds() - ctx->last_image_t;
    if (ctx->image_received && image_age > 1.0) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
            "[AllSystemsOK] Image stream stale (%.1fs ago).", image_age);
        all_ok = false;
    }

    if (all_ok) {
        RCLCPP_INFO(ctx->node->get_logger(),
            "[AllSystemsOK] All systems OK after %.1fs.", elapsed);
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsed >= timeout_s_) {
        RCLCPP_ERROR(ctx->node->get_logger(),
            "[AllSystemsOK] Timeout after %.0fs — aborting mission.", timeout_s_);
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void AllSystemsOK::onHalted() {
    RCLCPP_WARN(getCtx(config())->node->get_logger(), "[AllSystemsOK] Halted.");
}

// --- DiveToDepth ------------------------------------------------------------

BT::NodeStatus DiveToDepth::onStart() {
    auto depth_in = getInput<double>("target_depth");
    if (!depth_in) throw BT::RuntimeError("DiveToDepth: missing [target_depth]");
    target_z_ = depth_in.value();

    auto ctx = getCtx(config());
    ctx->target_depth = target_z_;
    RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Diving to z = %.2f m", target_z_);

    ctx->publishToPico(0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
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
    if (!obj) throw BT::RuntimeError("IsObjectSeen: missing [object]");

    return ctx->isObjectSeen(obj.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --- Do360Turn --------------------------------------------------------------

BT::NodeStatus Do360Turn::onStart() {
    auto obj = getInput<std::string>("success_when_seen");
    if (!obj) throw BT::RuntimeError("Do360Turn: missing [success_when_seen]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_ = ctx->getCurrentYaw();
    accumulated_yaw_ = 0.0;
    confirm_frames_ = 0;

    RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] Searching for %s...", target_object_.c_str());
    ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360Turn::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (ctx->isObjectSeen(target_object_)) {
        confirm_frames_++;

        if (confirm_frames_ == 1) {
            // First sighting: slow the turn so inertia doesn't carry us past the object.
            ctx->publishToPico(ctx->base_yaw_speed * 0.3f, 0.0f, (float)ctx->target_depth, 0);
            RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s spotted, slowing to confirm...", target_object_.c_str());
        }

        if (confirm_frames_ >= 4) {
            // Object has been stably visible for 4 consecutive ticks (~400 ms) — commit.
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s confirmed (%d frames).", target_object_.c_str(), confirm_frames_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    // Object not seen (or lost mid-confirmation) — reset counter and keep spinning.
    if (confirm_frames_ > 0) {
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] %s lost after %d confirm frames, resuming spin.", target_object_.c_str(), confirm_frames_);
        confirm_frames_ = 0;
    }

    double current_yaw = ctx->getCurrentYaw();
    accumulated_yaw_ += std::abs(normalizeAngle(current_yaw - prev_yaw_));
    prev_yaw_ = current_yaw;

    if (accumulated_yaw_ >= (2.0 * M_PI)) {
        ctx->stopMotion();
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] Full rotation complete. %s not found.", target_object_.c_str());
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
    auto ang = getInput<double>("angle");
    // if (!obj || !thr) throw BT::RuntimeError("ApproachObject: missing [object] or [threshold]");
    // target_object_ = obj.value();
    threshold_ = thr ? thr.value() : threshold_;
    angle_ = ang ? ang.value() : angle_;

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    smoothed_norm_x_ = 0.0f;
    locked_ = false;
    target_yaw_ = angle_;
    RCLCPP_INFO(ctx->node->get_logger(), "hahaha %f", target_yaw_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachObject::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    for (int i = 0;i < threshold_;i++) {
        rclcpp::spin_some(ctx->node);
        double current_yaw = ctx->getCurrentYaw();
        float yaw_cmd = (float)normalizeAngle(target_yaw_ - current_yaw);
        ctx->publishToPico(yaw_cmd, 5.0f, (float)ctx->target_depth, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    RCLCPP_INFO(ctx->node->get_logger(), "done surge1");

    for (int i = 0;i < 25;i++) {
        rclcpp::spin_some(ctx->node);
        double current_yaw = ctx->getCurrentYaw();
        float yaw_cmd = (float)normalizeAngle(target_yaw_ - current_yaw);
        ctx->publishToPico(yaw_cmd, 0.0f, (float)ctx->target_depth, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    RCLCPP_INFO(ctx->node->get_logger(), "done stay still");

    return BT::NodeStatus::SUCCESS;
}

void ApproachObject::onHalted() { getCtx(config())->stopMotion(); }

// --- DriveThruGate ----------------------------------------------------------

BT::NodeStatus DriveThruGate::onStart() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);


}

BT::NodeStatus DriveThruGate::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz, score = 0.0;
    bool gate_seen = ctx->getObjectPosition("GATE", ox, oy, oz, &score);

    double current_yaw = ctx->getCurrentYaw();
    float yaw_cmd = (float)normalizeAngle(target_yaw_ - current_yaw);

    if (!gate_seen) {
        gate_lost_frames_++;
        if (gate_lost_frames_ >= 8) {
            RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Gate cleared.");
            ctx->stopMotion();
            return BT::NodeStatus::SUCCESS;
        }
        ctx->publishToPico(yaw_cmd, ctx->base_surge_speed, (float)ctx->target_depth, 0);
        return BT::NodeStatus::RUNNING;
    }

    gate_lost_frames_ = 0;
    ctx->publishToPico(yaw_cmd, ctx->base_surge_speed, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}
void DriveThruGate::onHalted() { getCtx(config())->stopMotion(); }

// --- OrbitPole --------------------------------------------------------------
//
// Square orbit: 4 legs, all left (CCW) turns of 90°.
//   Leg 0: surge X   (half-side — starts at midpoint of one side)
//   Leg 1: surge 2X  (full side)
//   Leg 2: surge 2X  (full side)
//   Leg 3: surge 2X  (full side)
// where X = orbit_surge_duration from YAML.

BT::NodeStatus OrbitPole::onStart() {
    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("OrbitPole: missing [object]");
    target_object_   = obj.value();
    staystill_       = getInput<double>("staystill").value_or(0.0);
    steps_completed_ = 0;
    turn_target_set_ = false;

    if (staystill_ > 0.01) {
        phase_ = Phase::STAY_STILL;
        stay_still_start_ = std::chrono::steady_clock::now();
        auto ctx = getCtx(config());
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[OrbitPole] Pausing at threshold for %.1f s.", staystill_);
    } else {
        phase_ = Phase::TURN;
        RCLCPP_INFO(getCtx(config())->node->get_logger(),
                    "[OrbitPole] Starting square orbit.");
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OrbitPole::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    // --- Initial pause at threshold point ---
    if (phase_ == Phase::STAY_STILL) {
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - stay_still_start_).count();
        if (elapsed >= staystill_) {
            phase_ = Phase::TURN;
            turn_target_set_ = false;
            RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Starting square orbit.");
        }
        ctx->stopMotion();
        return BT::NodeStatus::RUNNING;
    }

    // --- All legs complete ---
    if (steps_completed_ >= 6) {
        ctx->stopMotion();
        ctx->locked_yaw = ctx->getCurrentYaw();
        ctx->use_locked_yaw = true;
        RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Square orbit complete. Locking yaw for future surges at %.2f rad", ctx->locked_yaw);
        return BT::NodeStatus::SUCCESS;
    }

    double cur_yaw = ctx->getCurrentYaw();

    // --- TURN: right 90° for leg 0, left 90° for legs 1-4, right 90° for leg 5 ---
    if (phase_ == Phase::TURN) {
        if (!turn_target_set_) {
            double turn_angle = (steps_completed_ == 0 || steps_completed_ == 5) ? -M_PI / 2.0 : M_PI / 2.0;
            target_yaw_ = normalizeAngle(cur_yaw + turn_angle);
            turn_target_set_ = true;
        }

        double yaw_err = normalizeAngle(target_yaw_ - cur_yaw);
        if (std::abs(yaw_err) < 0.08) {
            if (steps_completed_ == 5) {
                steps_completed_++;
                RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Final realign turn done.");
            } else {
                phase_ = Phase::SURGE;
                surge_start_ = ctx->node->get_clock()->now().seconds();
                RCLCPP_INFO(ctx->node->get_logger(),
                            "[OrbitPole] Turn complete, surging (leg %d/5).", steps_completed_ + 1);
            }
        } else {
            ctx->publishToPico((float)yaw_err, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    // --- SURGE: X for leg 0 and leg 4, 2X for legs 1-3 ---
    if (phase_ == Phase::SURGE) {
        double duration = (steps_completed_ == 0 || steps_completed_ == 4)
            ? ctx->orbit_surge_duration
            : ctx->orbit_surge_duration * 2.0;

        if (ctx->node->get_clock()->now().seconds() - surge_start_ >= duration) {
            steps_completed_++;
            phase_ = Phase::TURN;
            turn_target_set_ = false;
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(),
                        "[OrbitPole] Leg %d/5 complete.", steps_completed_);
        } else {
            ctx->publishToPico(0.0f, ctx->base_surge_speed*3, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
}

// void OrbitPole::onHalted() { getCtx(config())->stopMotion(); }

BT::NodeStatus StayStill::onStart() {
    auto duration_in = getInput<double>("duration");
    duration_ = duration_in ? duration_in.value() : duration_;
    start_time_ = std::chrono::steady_clock::now();
    getCtx(config())->stopMotion();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StayStill::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (ctx->use_locked_yaw) {
        double yaw_err = normalizeAngle(ctx->locked_yaw - ctx->getCurrentYaw());
        ctx->publishToPico(yaw_err, 0.0f, (float)ctx->target_depth, 0);
    } else {
        ctx->stopMotion();
    }
    
    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();

    if (elapsed >= duration_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void StayStill::onHalted() { getCtx(config())->stopMotion(); }


BT::NodeStatus SurgeForwardDistance::onStart()
{
    auto distance_in = getInput<double>("distance");
    if (!distance_in) {
        throw BT::RuntimeError("SurgeForwardDistance: missing [distance]");
    }
    target_distance_ = distance_in.value();
    if (target_distance_ <= 0.0) {
        auto ctx = getCtx(config());
        RCLCPP_ERROR(ctx->node->get_logger(), "[SurgeForwardDistance] distance must be > 0");
        return BT::NodeStatus::FAILURE;
    }

    auto ctx = getCtx(config());
    if (!surge_client_) {
        surge_client_ = rclcpp_action::create_client<auv_msgs::action::Surge>(
            ctx->node, "/surge_distance");
    }

    if (!surge_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(
            ctx->node->get_logger(),
            "[SurgeForwardDistance] action server /surge_distance not available");
        return BT::NodeStatus::FAILURE;
    }

    auv_msgs::action::Surge::Goal goal;
    goal.distance = target_distance_;

    auto goal_future = surge_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(ctx->node, goal_future, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
            ctx->node->get_logger(),
            "[SurgeForwardDistance] failed to send goal to /surge_distance");
        return BT::NodeStatus::FAILURE;
    }

    goal_handle_ = goal_future.get();
    if (!goal_handle_) {
        RCLCPP_ERROR(
            ctx->node->get_logger(),
            "[SurgeForwardDistance] goal rejected by /surge_distance");
        return BT::NodeStatus::FAILURE;
    }

    result_future_ = surge_client_->async_get_result(goal_handle_);
    action_sent_ = true;

    RCLCPP_INFO(
        ctx->node->get_logger(),
        "[SurgeForwardDistance] sent surge goal for %.2f m",
        target_distance_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SurgeForwardDistance::onRunning()
{
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (!action_sent_ || !goal_handle_ || !result_future_.valid()) {
        RCLCPP_ERROR(
            ctx->node->get_logger(),
            "[SurgeForwardDistance] action state invalid while waiting for result");
        return BT::NodeStatus::FAILURE;
    }

    if (result_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    auto wrapped_result = result_future_.get();
    action_sent_ = false;
    goal_handle_.reset();

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        const auto message = wrapped_result.result
            ? wrapped_result.result->message
            : std::string("no result message");
        RCLCPP_ERROR(
            ctx->node->get_logger(),
            "[SurgeForwardDistance] action did not succeed: %s",
            message.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!wrapped_result.result || !wrapped_result.result->success) {
        const auto message = wrapped_result.result
            ? wrapped_result.result->message
            : std::string("surge action returned no result");
        RCLCPP_ERROR(
            ctx->node->get_logger(),
            "[SurgeForwardDistance] surge failed: %s",
            message.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
        ctx->node->get_logger(),
        "[SurgeForwardDistance] surge complete: %s",
        wrapped_result.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
}

void SurgeForwardDistance::onHalted()
{
    auto ctx = getCtx(config());

    if (surge_client_ && goal_handle_) {
        RCLCPP_INFO(ctx->node->get_logger(), "[SurgeForwardDistance] halting, canceling goal");
        surge_client_->async_cancel_goal(goal_handle_);
    }
    action_sent_ = false;
    goal_handle_.reset();
}

void OrbitPole::onHalted() { getCtx(config())->stopMotion(); }
