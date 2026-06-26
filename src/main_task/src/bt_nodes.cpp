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

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_       = ctx->getCurrentYaw();
    accumulated_yaw_ = 0.0;
    confirm_frames_  = 0;

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
            ctx->publishToPico(ctx->base_yaw_speed * 0.3f, 0.0f, (float)ctx->target_depth, 0);
            RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s spotted, slowing to confirm...", target_object_.c_str());
        }

        if (confirm_frames_ >= 4) {
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s confirmed (%d frames).", target_object_.c_str(), confirm_frames_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

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

void Do360Turn::onHalted() { 
    getCtx(config())->stopMotion(); 
}

// --- AlignWithObject -----------------------------------------------------------

BT::NodeStatus AlignWithObject::onStart() {
    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("AlignWithObject: missing [object]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    align_confirm_frames_ = 0;
    gate_lost_frames_ = 0; // using this variable from the struct if available
    filtered_yaw_err_ = 0.0;

    RCLCPP_INFO(ctx->node->get_logger(), "[AlignWithObject] Starting alignment with %s.", target_object_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignWithObject::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz, score = 0.0;
    bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz, &score);

    if (!seen) {
        gate_lost_frames_++;
        ctx->publishToPico(0.0f, 0.0f, (float)ctx->target_depth, 0);
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 1000,
                             "[AlignWithObject] %s not seen (%d frames), holding still.", target_object_.c_str(), gate_lost_frames_);
        
        if (gate_lost_frames_ > ctx->gate_lost_timeout_frames) {
            RCLCPP_ERROR(ctx->node->get_logger(), "[AlignWithObject] Target completely lost. Returning FAILURE.");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    
    gate_lost_frames_ = 0;

    double raw_norm_x = ox / std::max(oz, 0.5);

    if (std::abs(raw_norm_x) < ctx->align_obj_threshold) {
        align_confirm_frames_++;
        if (align_confirm_frames_ >= ctx->align_confirm_frames) {
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(), "[AlignWithObject] Aligned with %s.", target_object_.c_str());
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        align_confirm_frames_ = 0;
    }

    double error = -raw_norm_x;
    if (filtered_yaw_err_ == 0.0) {
        filtered_yaw_err_ = error;
    } else {
        filtered_yaw_err_ = ctx->camera_alpha * error + (1.0 - ctx->camera_alpha) * filtered_yaw_err_;
    }

    float yaw_cmd = (float)filtered_yaw_err_;
    yaw_cmd = std::max(-ctx->base_yaw_speed, std::min(ctx->base_yaw_speed, yaw_cmd));

    ctx->publishToPico(yaw_cmd, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void AlignWithObject::onHalted() {
    getCtx(config())->stopMotion();
}

// --- AlignAndApproachObject --------------------------------------------------

BT::NodeStatus AlignAndApproachObject::onStart() {
    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("AlignAndApproachObject: missing [object]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    RCLCPP_INFO(ctx->node->get_logger(), "[AlignAndApproachObject] Starting approach to %s", target_object_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignAndApproachObject::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz, score = 0.0;
    bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz, &score);

    if (!seen) {
        ctx->publishToPico(0.0f, 0.0f, (float)ctx->target_depth, 0);
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 1000,
                             "[AlignAndApproachObject] Target %s not seen, holding still.", target_object_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    if (oz <= ctx->pole_approach_threshold) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[AlignAndApproachObject] Target %s reached threshold distance %.2f m <= %.2f m. SUCCESS.",
                    target_object_.c_str(), oz, ctx->pole_approach_threshold);
        return BT::NodeStatus::SUCCESS;
    }

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

// --- CenterObject -----------------------------------------------------------

BT::NodeStatus CenterObject::onStart() {
    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("CenterObject: missing [object]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    align_confirm_frames_ = 0;
    filtered_yaw_err_ = 0.0;
    is_holding_ = false;

    RCLCPP_INFO(ctx->node->get_logger(), "[CenterObject] Starting to center on %s.", target_object_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CenterObject::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz, score = 0.0;
    bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz, &score);

    if (!seen) {
        ctx->publishToPico(0.0f, 0.0f, (float)ctx->target_depth, 0);
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 1000,
                             "[CenterObject] %s not seen, holding still.", target_object_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    if (is_holding_) {
        ctx->publishToPico(0.0f, 0.0f, (float)ctx->target_depth, 0);
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - hold_start_time_).count();
        if (elapsed >= ctx->gate_staystill_time) {
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(), "[CenterObject] Hold complete. Centering finished!");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 500,
                             "[CenterObject] Holding position for %.1f / %.1f sec...", 
                             elapsed, ctx->gate_staystill_time);
        return BT::NodeStatus::RUNNING;
    }

    double raw_norm_x = ox / std::max(oz, 0.5);
    if (std::abs(raw_norm_x) < ctx->align_obj_threshold) {
        align_confirm_frames_++;
        if (align_confirm_frames_ >= ctx->align_confirm_frames) {
            is_holding_ = true;
            hold_start_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(ctx->node->get_logger(), "[CenterObject] Centered on %s. Starting hold for %.1f seconds.", 
                        target_object_.c_str(), ctx->gate_staystill_time);
            return BT::NodeStatus::RUNNING;
        }
    } else {
        align_confirm_frames_ = 0;
    }

    double error = -raw_norm_x;
    if (filtered_yaw_err_ == 0.0) {
        filtered_yaw_err_ = error;
    } else {
        filtered_yaw_err_ = ctx->camera_alpha * error + (1.0 - ctx->camera_alpha) * filtered_yaw_err_;
    }

    float yaw_cmd = (float)filtered_yaw_err_;
    yaw_cmd = std::max(-ctx->base_yaw_speed, std::min(ctx->base_yaw_speed, yaw_cmd));

    ctx->publishToPico(yaw_cmd, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void CenterObject::onHalted() {
    getCtx(config())->stopMotion();
}

// --- FindAnyObject ----------------------------------------------------------

BT::NodeStatus FindAnyObject::tick() {
    auto objs_str = getInput<std::string>("objects");
    if (!objs_str) throw BT::RuntimeError("FindAnyObject: missing [objects]");

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    std::vector<std::string> targets;
    std::stringstream ss(objs_str.value());
    std::string item;
    while (std::getline(ss, item, ',')) {
        targets.push_back(item);
    }

    for (const auto& t : targets) {
        if (ctx->isObjectSeen(t)) {
            setOutput("found_object", t);
            RCLCPP_INFO(ctx->node->get_logger(), "[FindAnyObject] Spotted %s!", t.c_str());
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;
}

// --- Do360TurnAny -----------------------------------------------------------

BT::NodeStatus Do360TurnAny::onStart() {
    auto objs_str = getInput<std::string>("objects");
    if (!objs_str) throw BT::RuntimeError("Do360TurnAny: missing [objects]");

    std::stringstream ss(objs_str.value());
    std::string item;
    targets_.clear();
    while (std::getline(ss, item, ',')) {
        targets_.push_back(item);
    }

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_ = ctx->getCurrentYaw();
    accumulated_yaw_ = 0.0;
    confirm_frames_ = 0;
    found_target_ = "";

    RCLCPP_INFO(ctx->node->get_logger(), "[Do360TurnAny] Spinning 360 to search for multiple targets...");
    ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360TurnAny::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    for (const auto& t : targets_) {
        if (ctx->isObjectSeen(t)) {
            if (found_target_ != t) {
                found_target_ = t;
                confirm_frames_ = 1;
            } else {
                confirm_frames_++;
            }
            break;
        }
    }

    if (confirm_frames_ > 0) {
        if (!ctx->isObjectSeen(found_target_)) {
            confirm_frames_ = 0;
            found_target_ = "";
        } else {
            if (confirm_frames_ == 1) {
                ctx->publishToPico(ctx->base_yaw_speed * 0.3f, 0.0f, (float)ctx->target_depth, 0);
                RCLCPP_INFO(ctx->node->get_logger(), "[Do360TurnAny] %s spotted, slowing...", found_target_.c_str());
            }

            if (confirm_frames_ >= 4) {
                ctx->stopMotion();
                setOutput("found_object", found_target_);
                RCLCPP_INFO(ctx->node->get_logger(), "[Do360TurnAny] %s confirmed! Stopping spin.", found_target_.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }
    }

    double current_yaw = ctx->getCurrentYaw();
    accumulated_yaw_ += std::abs(normalizeAngle(current_yaw - prev_yaw_));
    prev_yaw_ = current_yaw;

    if (accumulated_yaw_ >= (2.0 * M_PI)) {
        ctx->stopMotion();
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360TurnAny] Full rotation complete. Targets not found.");
        return BT::NodeStatus::FAILURE;
    }

    ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void Do360TurnAny::onHalted() {
    getCtx(config())->stopMotion();
}

// --- SaveCurrentYaw -----------------------------------------------------------

BT::NodeStatus SaveCurrentYaw::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    
    ctx->locked_yaw = ctx->getCurrentYaw();
    ctx->use_locked_yaw = true;
    
    RCLCPP_INFO(ctx->node->get_logger(), "[SaveCurrentYaw] Saved locked_yaw as %.2f rad.", ctx->locked_yaw);
    return BT::NodeStatus::SUCCESS;
}

// --- AlignToLockedYaw -------------------------------------------------------

BT::NodeStatus AlignToLockedYaw::onStart() {
    align_confirm_frames_ = 0;
    auto ctx = getCtx(config());
    if (!ctx->use_locked_yaw) {
        RCLCPP_WARN(ctx->node->get_logger(), "[AlignToLockedYaw] use_locked_yaw is false. Proceeding anyway.");
    }
    RCLCPP_INFO(ctx->node->get_logger(), "[AlignToLockedYaw] Aligning back to %.2f rad.", ctx->locked_yaw);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignToLockedYaw::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double current_yaw = ctx->getCurrentYaw();
    double yaw_err = normalizeAngle(ctx->locked_yaw - current_yaw);
    
    if (std::abs(yaw_err) < ctx->align_yaw_threshold) {
        align_confirm_frames_++;
        if (align_confirm_frames_ >= 10) {
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(), "[AlignToLockedYaw] Successfully aligned to locked yaw.");
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        align_confirm_frames_ = 0;
    }

    float yaw_cmd = (float)yaw_err;
    yaw_cmd = std::max(-ctx->base_yaw_speed, std::min(ctx->base_yaw_speed, yaw_cmd));

    ctx->publishToPico(yaw_cmd, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void AlignToLockedYaw::onHalted() {
    getCtx(config())->stopMotion();
}

// --- StayStill --------------------------------------------------------------

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

// --- SurgeForwardDistance ---------------------------------------------------

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
