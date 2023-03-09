cmake_minimum_required(VERSION 3.8)
project(gazebo_set_joint_positions_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)


target_include_directories(
        ${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} rclcpp gazebo_dev gazebo_ros std_msgs sensor_msgs)
ament_export_dependencies(gazebo_dev)

ament_export_targets(SetJointPositions HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


install(DIRECTORY include DESTINATION include)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT SetJointPositions
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include
)

#
# Add ros tests
#
enable_testing()
find_file(NOSETESTS_EXECUTABLE nosetests)
add_test(nosetests "${NOSETESTS_EXECUTABLE}" -v --with-xunit)

install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/test)

find_package(launch_testing_ament_cmake)
add_launch_test(test/test_plugin.py)

ament_package()

