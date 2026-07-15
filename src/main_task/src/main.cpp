#include "main_task/bt_nodes.h"
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("main_task_node");

    // 1. Create the Robot Context and map the topics (Publishers & Subscribers)
    auto ctx = std::make_shared<RobotContext>();
    ctx->node = node;

    // Load parameters or use defaults
    node->declare_parameter("base_surge_speed", 0.1);
    node->declare_parameter("base_yaw_speed", 0.1);
    node->declare_parameter("gate_conf_thresh", 0.6);
    node->declare_parameter("pole_conf_thresh", 0.3);
    node->declare_parameter("depth_tolerance", 0.15);
    node->declare_parameter("pole_approach_threshold", 1.0);
    node->declare_parameter("gate_staystill_time", 3.0);
    node->declare_parameter("align_obj_threshold", 0.04);
    node->declare_parameter("align_yaw_threshold", 0.10);
    node->declare_parameter("camera_alpha", 0.25);
    node->declare_parameter("align_confirm_frames", 5);
    node->declare_parameter("gate_lost_timeout_frames", 30);

    ctx->base_surge_speed = node->get_parameter("base_surge_speed").as_double();
    ctx->base_yaw_speed = node->get_parameter("base_yaw_speed").as_double();
    ctx->gate_conf_thresh = node->get_parameter("gate_conf_thresh").as_double();
    ctx->pole_conf_thresh = node->get_parameter("pole_conf_thresh").as_double();
    ctx->depth_tolerance = node->get_parameter("depth_tolerance").as_double();
    ctx->pole_approach_threshold = node->get_parameter("pole_approach_threshold").as_double();
    ctx->gate_staystill_time = node->get_parameter("gate_staystill_time").as_double();
    ctx->align_obj_threshold = node->get_parameter("align_obj_threshold").as_double();
    ctx->align_yaw_threshold = node->get_parameter("align_yaw_threshold").as_double();
    ctx->camera_alpha = node->get_parameter("camera_alpha").as_double();
    ctx->align_confirm_frames = node->get_parameter("align_confirm_frames").as_int();
    ctx->gate_lost_timeout_frames = node->get_parameter("gate_lost_timeout_frames").as_int();

    // Register callback for live dynamic parameter tuning
    auto param_callback_handle = node->add_on_set_parameters_callback(
        [ctx](const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : parameters) {
                if (param.get_name() == "base_surge_speed") {
                    ctx->base_surge_speed = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: base_surge_speed -> %.2f", ctx->base_surge_speed);
                } else if (param.get_name() == "base_yaw_speed") {
                    ctx->base_yaw_speed = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: base_yaw_speed -> %.2f", ctx->base_yaw_speed);
                } else if (param.get_name() == "gate_conf_thresh") {
                    ctx->gate_conf_thresh = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: gate_conf_thresh -> %.2f", ctx->gate_conf_thresh);
                } else if (param.get_name() == "pole_conf_thresh") {
                    ctx->pole_conf_thresh = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: pole_conf_thresh -> %.2f", ctx->pole_conf_thresh);
                } else if (param.get_name() == "depth_tolerance") {
                    ctx->depth_tolerance = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: depth_tolerance -> %.2f", ctx->depth_tolerance);
                } else if (param.get_name() == "pole_approach_threshold") {
                    ctx->pole_approach_threshold = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: pole_approach_threshold -> %.2f", ctx->pole_approach_threshold);
                } else if (param.get_name() == "gate_staystill_time") {
                    ctx->gate_staystill_time = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: gate_staystill_time -> %.2f", ctx->gate_staystill_time);
                } else if (param.get_name() == "align_obj_threshold") {
                    ctx->align_obj_threshold = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: align_obj_threshold -> %.2f", ctx->align_obj_threshold);
                } else if (param.get_name() == "align_yaw_threshold") {
                    ctx->align_yaw_threshold = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: align_yaw_threshold -> %.2f", ctx->align_yaw_threshold);
                } else if (param.get_name() == "camera_alpha") {
                    ctx->camera_alpha = param.as_double();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: camera_alpha -> %.2f", ctx->camera_alpha);
                } else if (param.get_name() == "align_confirm_frames") {
                    ctx->align_confirm_frames = param.as_int();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: align_confirm_frames -> %d", ctx->align_confirm_frames);
                } else if (param.get_name() == "gate_lost_timeout_frames") {
                    ctx->gate_lost_timeout_frames = param.as_int();
                    RCLCPP_INFO(ctx->node->get_logger(), "Live Tune: gate_lost_timeout_frames -> %d", ctx->gate_lost_timeout_frames);
                }
            }
            return result;
        });

    // Actuator Publisher
    ctx->pico_pub = node->create_publisher<auv_msgs::msg::ControlCommand>("/control_cmd", 10);

    // IMU Subscription
    ctx->imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/zed2i_front/zed_node/imu/data", 10,
        [ctx](const sensor_msgs::msg::Imu::SharedPtr msg) {
            std::lock_guard<std::mutex> g(ctx->mtx);
            ctx->latest_imu = msg;
            ctx->imu_received = true;
        });

    // Altimeter Subscription
    ctx->alt_sub = node->create_subscription<std_msgs::msg::Float32>(
        "/pressure", 10,
        [ctx](const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> g(ctx->mtx);
            ctx->latest_altimeter = msg->data;
            ctx->altimeter_received = true;
        });

    // 3D YOLO/HSV Detections Subscription
    ctx->detection_sub = node->create_subscription<vision_msgs::msg::Detection3DArray>(
        "/detections_3d", 10,
        [ctx](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
            std::lock_guard<std::mutex> g(ctx->mtx);
            ctx->latest_detections = msg;
        });

    // ZED Diagnostics Subscription
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

    // ZED Image stream Subscription
    ctx->image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/zed2i_front/zed_node/rgb/color/rect/image", 10,
        [ctx](const sensor_msgs::msg::Image::SharedPtr) {
            std::lock_guard<std::mutex> g(ctx->mtx);
            ctx->image_received = true;
            ctx->last_image_t   = ctx->node->get_clock()->now().seconds();
        });

    // 2. Initialize the Behavior Tree Factory
    BT::BehaviorTreeFactory factory;
    registerAllNodes(factory);

    // Load XML files from the installation share directory
    std::string share_dir = ament_index_cpp::get_package_share_directory("main_task");

    // Register the StartGate and AlignObject subtrees first
    factory.registerBehaviorTreeFromFile(share_dir + "/start_gate.xml");
    factory.registerBehaviorTreeFromFile(share_dir + "/alignobject.xml");
    factory.registerBehaviorTreeFromFile(share_dir + "/test_align.xml");
    factory.registerBehaviorTreeFromFile(share_dir + "/test_surge.xml");

    // Build the Main Tree (which calls subtrees)
    auto tree = factory.createTreeFromFile(share_dir + "/main_task.xml");

    // Inject our Robot Context into the tree blackboard
    tree.rootBlackboard()->set("robot_context", ctx);

    // Inject into all subtree blackboards programmatically to ensure it is visible everywhere
    for (auto& subtree : tree.subtrees) {
        if (subtree && subtree->blackboard) {
            subtree->blackboard->set("robot_context", ctx);
        }
    }

    // Start the Groot2 Publisher
    BT::Groot2Publisher publisher(tree);

    // 3. Ticking Loop (10Hz)
    RCLCPP_INFO(node->get_logger(), "=== Starting Main Task BT Mission ===");
    constexpr auto TICK_PERIOD = std::chrono::milliseconds(100);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(TICK_PERIOD);
    }

    // Stop robot motion when BT finishes
    ctx->stopMotion();

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "=== MISSION COMPLETED SUCCESSFULLY ===");
    } else {
        RCLCPP_WARN(node->get_logger(), "=== MISSION FAILURE ===");
    }

    rclcpp::shutdown();
    return 0;
}