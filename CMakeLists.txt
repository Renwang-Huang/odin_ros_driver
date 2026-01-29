cmake_minimum_required(VERSION 3.8)
project(odin_ros_driver)

# ROS2 only
add_compile_definitions(ROS2)

# Platform detection
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(ARCH STREQUAL "x86_64")
    set(TARGET_PLATFORM "x86")
    message(STATUS "Detected x86_64 architecture")
elseif(ARCH MATCHES "arm|aarch64")
    set(TARGET_PLATFORM "arm")
    message(STATUS "Detected ARM architecture: ${ARCH}")
else()
    message(WARNING "Unsupported architecture: ${ARCH}. Using default settings")
    set(TARGET_PLATFORM "unknown")
endif()

# Set library path
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
message(STATUS "Library directory: ${LIB_DIR}")

# Set library name based on platform
if(TARGET_PLATFORM STREQUAL "arm")
    set(LYD_HOST_API_LIB_NAME "lydHostApi_arm")
else()
    set(LYD_HOST_API_LIB_NAME "lydHostApi_amd")
endif()

# Find precompiled lydHostApi library
find_library(LYD_HOST_API_LIB
    NAMES
        ${LYD_HOST_API_LIB_NAME}
        lib${LYD_HOST_API_LIB_NAME}.a
        lib${LYD_HOST_API_LIB_NAME}.so
    PATHS ${LIB_DIR}
    NO_DEFAULT_PATH
)

if(LYD_HOST_API_LIB)
    message(STATUS "Found lydHostApi library: ${LYD_HOST_API_LIB}")
else()
    file(GLOB LIB_FILES "${LIB_DIR}/lib${LYD_HOST_API_LIB_NAME}.*")
    if(LIB_FILES)
        message(STATUS "Found library files: ${LIB_FILES}")
        set(LYD_HOST_API_LIB ${LIB_FILES})
    else()
        message(FATAL_ERROR "Could not find precompiled lydHostApi library in ${LIB_DIR}")
    endif()
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIR}
    ${LIBUSB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Shared library list
set(COMMON_LIBS
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    pthread
    rt
    ${CMAKE_DL_LIBS}
    ${LYD_HOST_API_LIB}
)

# ===== host_sdk_sample (main driver node) =====
add_executable(host_sdk_sample
    src/host_sdk_sample.cpp
    src/yaml_parser.cpp
    src/rawCloudRender.cpp
    src/camera_pose_visualization.cpp
)
target_link_libraries(host_sdk_sample
    ${COMMON_LIBS}
    yaml-cpp
    usb-1.0
)
ament_target_dependencies(host_sdk_sample
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    cv_bridge
    image_transport
    tf2
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
)

# ===== pointcloud_depth_converter =====
add_library(pointcloud_depth_converter src/pointcloud_depth_converter.cpp)
target_link_libraries(pointcloud_depth_converter
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

# ===== depth_image_ros2_node =====
add_library(depth_image_ros2_node_lib src/depth_image_node.cpp)
target_link_libraries(depth_image_ros2_node_lib
    pointcloud_depth_converter
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
ament_target_dependencies(depth_image_ros2_node_lib
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    pcl_conversions
    message_filters
)

# ===== pcd2depth_ros2_node =====
add_executable(pcd2depth_ros2_node src/pcd_to_depth.cpp)
target_link_libraries(pcd2depth_ros2_node
    depth_image_ros2_node_lib
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp
)
ament_target_dependencies(pcd2depth_ros2_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    pcl_conversions
    message_filters
)

# ARM platform specific link options
if(TARGET_PLATFORM STREQUAL "arm")
    set_target_properties(host_sdk_sample PROPERTIES
        LINK_FLAGS "-Wl,--no-as-needed -Wl,--rpath=${LIB_DIR}"
    )
    message(STATUS "Adding ARM-specific link options and RPATH")
endif()

# ===== Install =====
install(TARGETS
    host_sdk_sample
    pcd2depth_ros2_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch_ROS2")
    install(DIRECTORY launch_ROS2/
        DESTINATION share/${PROJECT_NAME}/launch
    )
    message(STATUS "Installing launch_ROS2 to share/${PROJECT_NAME}/launch")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    cv_bridge
    image_transport
    pcl_conversions
    message_filters
    tf2
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
)

ament_package()

# Summary
message(STATUS "=======================================")
message(STATUS "Project: ${PROJECT_NAME} (ROS2)")
message(STATUS "Target platform: ${TARGET_PLATFORM}")
message(STATUS "lydHostApi: ${LYD_HOST_API_LIB}")
message(STATUS "=======================================")
