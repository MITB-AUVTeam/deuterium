#include "main_task/bt_nodes.h"

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

// --- AllSystemsOK -----------------------------------------------------------

BT::NodeStatus AllSystemsOK::onStart() {
    auto timeout_in = getInput<double>("timeout_s");
    timeout_s_  = timeout_in ? timeout_in.value() : 20.0;
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

    if (!ctx->zed_ok) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                             "[AllSystemsOK] ZED camera not healthy. (%.0fs elapsed)", elapsed);
        all_ok = false;
    }

    if (all_ok) {
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[AllSystemsOK] All systems OK after %.1fs.", elapsed);
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsed >= timeout_s_) {
        RCLCPP_ERROR(ctx->node->get_logger(),
                     "[AllSystemsOK] Timeout after %.0fs systems check failed, aborting mission.", timeout_s_);
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

void DiveToDepth::onHalted() { 
    getCtx(config())->stopMotion(); 
}

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

    auto turns_in = getInput<int>("max_turns");
    max_turns_ = turns_in ? turns_in.value() : 2;

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_        = ctx->getCurrentYaw();
    accumulated_yaw_ = 0.0;
    confirm_frames_  = 0;
    object_found_    = false;

    RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] Scanning for %s (max %d turns)...",
                target_object_.c_str(), max_turns_);
    ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360Turn::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (ctx->isObjectSeen(target_object_)) {
        confirm_frames_++;

        if (confirm_frames_ == 1) {
            ctx->publishToPico(ctx->base_yaw_speed * 0.3f, 0.0f, (float)ctx->target_depth, 0);
            RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s spotted, slowing to confirm...", target_object_.c_str());
        }

        if (confirm_frames_ >= 4) {
            ctx->stopMotion();
            object_found_ = true;
            RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s confirmed (%d frames). Stopping early.",
                        target_object_.c_str(), confirm_frames_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    if (confirm_frames_ > 0) {
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] %s lost after %d confirm frames, resuming spin.",
                    target_object_.c_str(), confirm_frames_);
        confirm_frames_ = 0;
    }

    double current_yaw = ctx->getCurrentYaw();
    accumulated_yaw_ += std::abs(normalizeAngle(current_yaw - prev_yaw_));
    prev_yaw_ = current_yaw;

    if (accumulated_yaw_ >= (max_turns_ * 2.0 * M_PI)) {
        ctx->stopMotion();
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] %d full rotations complete. %s not found. Returning SUCCESS anyway.",
                    max_turns_, target_object_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void Do360Turn::onHalted() {
    getCtx(config())->stopMotion();
}

// --- AlignAndApproachObject --------------------------------------------------

BT::NodeStatus AlignAndApproachObject::onStart() {
    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("AlignAndApproachObject: missing [object]");
    target_object_ = obj.value();

    lost_frames_ = 0;
    has_seen_ = false;

    auto ctx = getCtx(config());
    RCLCPP_INFO(ctx->node->get_logger(), "[AlignAndApproachObject] Starting approach to %s (pass-through)", target_object_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignAndApproachObject::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz, score = 0.0;
    bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz, &score);

    if (!seen) {
        // Not yet locked on: hold still and wait for the target to appear.
        if (!has_seen_) {
            ctx->publishToPico(0.0f, 0.0f, (float)ctx->target_depth, 0);
            RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 1000,
                                 "[AlignAndApproachObject] Target %s not acquired yet, holding still.", target_object_.c_str());
            return BT::NodeStatus::RUNNING;
        }

        // Locked on before: target left the frame -> we are passing through it.
        // Keep surging straight ahead to clear the gate while confirming.
        lost_frames_++;
        ctx->publishToPico(0.0f, ctx->base_surge_speed, (float)ctx->target_depth, 0);

        if (lost_frames_ >= 4) {
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(), "[AlignAndApproachObject] Target %s out of view for %d frames. Passed through. SUCCESS.",
                        target_object_.c_str(), lost_frames_);
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(ctx->node->get_logger(), "[AlignAndApproachObject] %s lost (%d/4), surging through...", target_object_.c_str(), lost_frames_);
        return BT::NodeStatus::RUNNING;
    }

    has_seen_ = true;
    lost_frames_ = 0;

    double raw_norm_x = ox / std::max(oz, 0.5);
    double error = -raw_norm_x;
    float yaw_cmd = (float)error;
    yaw_cmd = std::max(-ctx->base_yaw_speed, std::min(ctx->base_yaw_speed, yaw_cmd));

    ctx->publishToPico(yaw_cmd, ctx->base_surge_speed, (float)ctx->target_depth, 0);
    RCLCPP_INFO_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 500,
                         "[AlignAndApproachObject] Approaching %s: dist = %.2f m, yaw_cmd = %.3f",
                         target_object_.c_str(), oz, yaw_cmd);

    return BT::NodeStatus::RUNNING;
}

void AlignAndApproachObject::onHalted() {
    getCtx(config())->stopMotion();
}

