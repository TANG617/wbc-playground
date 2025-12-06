cmake_minimum_required(VERSION 3.10)

project(wbc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译 ruckig 库
add_subdirectory(third_party/ruckig)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(genie_msgs REQUIRED) 
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED) 
find_package(rosidl_default_generators REQUIRED)

find_library(PLACO_SYSTEM_LIB 
    NAMES liblibplaco libplaco placo
    PATHS /usr/local/lib /usr/lib
    NO_DEFAULT_PATH
)

set(PLACO_LIB_PATH ${PLACO_SYSTEM_LIB})
if(NOT EXISTS /usr/local/include/placo)
    message(FATAL_ERROR "placo headers not found at /usr/local/include/placo")
endif()

add_library(placo SHARED IMPORTED)

set_target_properties(placo PROPERTIES
    IMPORTED_LOCATION ${PLACO_LIB_PATH}
    IMPORTED_NO_SONAME ON
)

# 配置 placo
target_include_directories(placo INTERFACE
    /usr/local/include
)
# 添加 pinocchio 头文件
if(EXISTS /opt/ros/humble/include/pinocchio)
    target_include_directories(placo INTERFACE
        /opt/ros/humble/include
    )
elseif(EXISTS /opt/openrobots/include)
    target_include_directories(placo INTERFACE
        /opt/openrobots/include
    )
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WBCCommand.msg"
  "msg/WBCState.msg"
  "msg/TaskCommand.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

add_executable(fk_node 
    src/fk_node.cpp
    src/robot_kinematics_tool.cpp
)

add_executable(arm_control_node 
    src/ArmControl_node.cpp
    src/jointLimitManager.cpp   
    src/tinyxml2.cpp
)

target_include_directories(arm_control_node PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(arm_control_node
    rclcpp
    sensor_msgs
    genie_msgs
)

target_link_libraries(arm_control_node ruckig)

set_target_properties(arm_control_node PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS arm_control_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ruckig
    DESTINATION lib
)


target_include_directories(fk_node PRIVATE include)

ament_target_dependencies(fk_node
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    ament_index_cpp
    std_msgs 
    genie_msgs
)

target_link_libraries(fk_node
  yaml-cpp
  placo 
)

rosidl_target_interfaces(fk_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

set_target_properties(fk_node PROPERTIES
    INSTALL_RPATH "/usr/local/lib" 
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS fk_node
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(ik_node
  src/ik_node.cpp
  src/robot_kinematics_tool.cpp
  src/ik_solve_logger.cpp
)

target_include_directories(ik_node PRIVATE include)

ament_target_dependencies(ik_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(ik_node
  yaml-cpp
  placo
)

set_target_properties(ik_node PROPERTIES
    INSTALL_RPATH "/usr/local/lib" 
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

rosidl_target_interfaces(ik_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ik_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

ament_package()
