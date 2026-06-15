
/**
 * @file main.cpp
 * @brief Main entry point for the pre-qualification behavior tree mission.
 * @license Apache-2.0
 */

#include "bt_nodes.h"
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("prequalification_bt");

    // --- Shared robot context -----------------------------------------------
    auto ctx       = std::make_shared<RobotContext>();
    ctx->node      = node;

    // Declare and Load Parameters (with default values)
    node->declare_parameter("base_surge_speed", 0.9);
    node->declare_parameter("base_yaw_speed", 0.1);
    node->declare_parameter("gate_conf_thresh", 0.5);
    node->declare_parameter("pole_conf_thresh", 0.3);
    node->declare_parameter("gate_lock_thresh", 0.50);
    node->declare_parameter("pole_lock_thresh", 0.45);
    node->declare_parameter("depth_tolerance", 0.15);
    node->declare_parameter("gate_align_deadband", 0.04);
    node->declare_parameter("pole_align_deadband", 0.06);
    node->declare_parameter("orbit_surge_duration", 4.0);
    node->declare_parameter("orbit_step_angle", 70.0);

    auto update_ctx_params = [node, ctx]() {
        ctx->base_surge_speed = node->get_parameter("base_surge_speed").as_double();
        ctx->base_yaw_speed   = node->get_parameter("base_yaw_speed").as_double();
        ctx->gate_conf_thresh = node->get_parameter("gate_conf_thresh").as_double();
        ctx->pole_conf_thresh = node->get_parameter("pole_conf_thresh").as_double();
        ctx->gate_lock_thresh = node->get_parameter("gate_lock_thresh").as_double();
        ctx->pole_lock_thresh = node->get_parameter("pole_lock_thresh").as_double();
        ctx->depth_tolerance  = node->get_parameter("depth_tolerance").as_double();
        ctx->gate_align_deadband = node->get_parameter("gate_align_deadband").as_double();
        ctx->pole_align_deadband = node->get_parameter("pole_align_deadband").as_double();
        ctx->orbit_surge_duration = node->get_parameter("orbit_surge_duration").as_double();
        ctx->orbit_step_angle     = node->get_parameter("orbit_step_angle").as_double();
    };

    update_ctx_params(); // Initial load

    // Add callback for live parameter updates
    auto param_callback_handle = node->add_on_set_parameters_callback(
        [update_ctx_params](const std::vector<rclcpp::Parameter> & parameters) {
            auto result = rcl_interfaces::msg::SetParametersResult();
            result.successful = true;
            for (const auto & parameter : parameters) {
                // We'll update the context after the parameter is set
            }
            // Use a timer or a delayed call to update context because the parameter 
            // is not yet fully updated in the node's internal storage during this callback.
            // But for simplicity in this setup, we'll just trigger the update.
            return result;
        });

    // Actuator publishers
    ctx->pico_pub = node->create_publisher<auv_msgs::msg::ControlCommand>("/control_cmd", 10);

    // IMU subscription (orientation / yaw)
    ctx->imu_sub =
        node->create_subscription<sensor_msgs::msg::Imu>(
            "/zed2i_front/zed_node/imu/data", 10,
            [ctx](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_imu   = msg;
                ctx->imu_received = true;
                 }); 

    // Altimeter subscription (altitude above pool floor)
    ctx->alt_sub =
        node->create_subscription<std_msgs::msg::Float32>(
            "/pressure", 10,
            [ctx](const std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                // NOTE: The BT expects positive values for depth (positive-down).
                // If your sensor outputs negative values underwater, flip the sign here:
                // ctx->latest_altimeter = -msg->data;
                ctx->latest_altimeter = msg->data;
                ctx->altimeter_received = true;
            });

    // 3D Detections subscription (YOLO + depth fusion)
    ctx->det_sub =
        node->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/detections_3d", 10,
            [ctx](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_detections = msg;
            });

    // ZED Diagnostics subscription
    ctx->zed_diag_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/zed2i_front/zed_node/diagnostic", 10,
        [ctx](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
            std::lock_guard<std::mutex> g(ctx->mtx);
            for (const auto& status : msg->status) {
                if (status.name.find("zed") != std::string::npos ||
                    status.name.find("ZED") != std::string::npos) {
                    ctx->zed_ok = (status.level <= 1); // 0 = OK, 1 = WARN
                }
            }
        });

    // ZED Image stream subscription 
    ctx->image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/zed2i_front/zed_node/rgb/color/rect/image", 10,
        [ctx](const sensor_msgs::msg::Image::SharedPtr) {
            std::lock_guard<std::mutex> g(ctx->mtx);
            ctx->image_received = true;
            ctx->last_image_t   = ctx->node->get_clock()->now().seconds();
        });

    // --- Behavior Tree Initialization ---------------------------------------
    BT::BehaviorTreeFactory factory;
    registerAllNodes(factory);

    // Locate the XML mission file
    std::string xml_path;
    auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    if (non_ros_args.size() > 1) {
        xml_path = non_ros_args[1];
    } else {
        try {
            xml_path = ament_index_cpp::get_package_share_directory("prequalification_bt")
                       + "/prequalification.xml";
        } catch (const std::exception& e) {
            RCLCPP_FATAL(node->get_logger(), "Cannot find prequalification.xml: %s", e.what());
            rclcpp::shutdown();
            return 1;
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "Loading behavior tree: %s", xml_path.c_str());
    auto tree = factory.createTreeFromFile(xml_path);

    // Start Groot2 Publisher on default port 1667
    RCLCPP_INFO(node->get_logger(), "Starting Groot2 Publisher on port 1667...");
    BT::Groot2Publisher publisher(tree, 1667);

    // Inject shared context into the blackboard
    tree.rootBlackboard()->set("robot_context", ctx);

    // Seed callbacks before first tick
    rclcpp::spin_some(node);

    // --- Mission Loop -------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "=== Starting Pre-Qualification Mission ===");

    constexpr auto TICK_PERIOD = std::chrono::milliseconds(100);
    BT::NodeStatus status      = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        update_ctx_params(); // Refresh parameters every tick for live tuning
        status = tree.tickOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(TICK_PERIOD);
    }

    ctx->stopMotion();

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "=== Mission COMPLETE ===");
    } else {
        RCLCPP_WARN(node->get_logger(), "=== Mission FAILED ===");
    }

    rclcpp::shutdown();
    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}