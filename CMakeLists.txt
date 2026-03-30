cmake_minimum_required(VERSION 3.8)
project(krabi_caisse_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(cv_bridge        REQUIRED)
find_package(OpenCV           REQUIRED)
find_package(krabi_msgs       REQUIRED)

include_directories(include)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(caisse_detector_node
  src/caisse_detector_node.cpp
)

target_include_directories(caisse_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(caisse_detector_node PUBLIC cxx_std_17)

ament_target_dependencies(caisse_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  krabi_msgs
)



# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS caisse_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
