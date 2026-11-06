
cmake_minimum_required(VERSION 3.8)

project(retreival_robot)

# ==========================================
# Compiler warnings
# ==========================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# Dependencies
# ==========================================

find_package(ament_cmake REQUIRED)

# ==========================================
# Install Python executable nodes
# ==========================================

install(
  PROGRAMS

    # Cylinder perception
    cylinder_perception_pkg/cylinder_perception/cylinder_detector.py

    # Wheel odometry
    odometry/simple_odometry.py

    # Occupancy map publisher
    navigation/map_publisher.py

    # EKF landmark localization
    navigation/landmark_localizer.py

    # A* motion planner
    navigation/motion_planner.py

    # Path-following navigation
    navigation/navigator.py

    # Automatic navigation goal assignment
    navigation/startup_goal.py

  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# Install package resources
# ==========================================

install(
  DIRECTORY

    # ROS 2 launch files
    launch

    # Robot description and meshes
    urdf
    meshes

    # Controller configuration
    config

    # Gazebo environments
    worlds

    # Perception source files
    cylinder_perception_pkg

    # Odometry source files
    odometry
    

    # Navigation source files
    navigation

    # Landmark and occupancy maps
    maps

    # World/map generation scripts
    scripts

  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# Testing
# ==========================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

# ==========================================
# Ament package
# ==========================================

ament_package()
