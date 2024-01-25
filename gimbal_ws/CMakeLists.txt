cmake_minimum_required(VERSION 3.5)
project(raisin_rf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# NODE
set(JOY_EXEC_NAME joy_node)

add_executable(${JOY_EXEC_NAME}
    src/${PROJECT_NAME}/main.cpp
    src/${PROJECT_NAME}/joy.cpp
)

target_include_directories(${JOY_EXEC_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${JOY_EXEC_NAME}
    rclcpp
    sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${JOY_EXEC_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/rf_joy_up.sh
              scripts/rf_joy_down.sh
              scripts/remote_rf_joy_up.sh
              scripts/remote_rf_joy_down.sh
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
        DESTINATION bin)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()