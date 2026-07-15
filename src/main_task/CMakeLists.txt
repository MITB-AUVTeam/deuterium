cmake_minimum_required(VERSION 3.16)
project(main_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS2 / ament dependencies ─────────────────────────────────────────────────
find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(vision_msgs      REQUIRED)
find_package(auv_msgs         REQUIRED)
find_package(tf2              REQUIRED)
find_package(ament_index_cpp  REQUIRED)
find_package(diagnostic_msgs   REQUIRED)
find_package(rclcpp_action    REQUIRED)

# ── BehaviorTree.CPP ──────────────────────────────────────────────────────────
find_package(behaviortree_cpp REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(main_task_node
    src/main.cpp
    src/bt_nodes.cpp
)

target_include_directories(main_task_node PRIVATE
    include
)

ament_target_dependencies(main_task_node
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    vision_msgs
    auv_msgs
    tf2
    ament_index_cpp
    diagnostic_msgs
    rclcpp_action
)

target_link_libraries(main_task_node
    behaviortree_cpp::behaviortree_cpp
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS main_task_node
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES 
    main_task.xml
    start_gate.xml
    src/alignobject.xml
    test_align.xml
    test_surge.xml
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
