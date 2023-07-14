#!/usr/bin/env python3
import os
import unittest
import time
import launch
import launch.actions
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
import launch_testing
import launch_testing.actions
from ament_index_python import get_package_share_directory
from std_msgs.msg import Header
import subprocess
import rclpy
import rclpy.clock
import rclpy.time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
import pytest
from rclpy.parameter import Parameter

from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from sensor_msgs.msg import JointState

from time import sleep


@pytest.mark.launch_test
def generate_test_description():
    world_file_name = os.path.join(get_package_share_directory('gazebo_set_joint_positions_plugin'),
                                   'test', 'test.world')
    urdf_file_name = os.path.join(get_package_share_directory('gazebo_set_joint_positions_plugin'),
                                  'test', 'test.urdf')

    print('robot  urdf_file_name : {}'.format(urdf_file_name))
    print('world world_file_name : {}'.format(world_file_name))

    # Gazebo is launched using this command because it's the only option which GazeboRosFactory is activated in ci/cd
    launch_gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    return launch.LaunchDescription(
        [
            launch_gazebo,

            # Launch robot_state_publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True, 'robot_description': Command(
                    ['xacro ', urdf_file_name])}],
            ),
            Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'test_robot',
                            '-topic', '/robot_description'],
                 output='screen'),

            launch_testing.actions.ReadyToTest(),
        ]
    )


# There is a bug where the gzserver is not killed after the tests run.
# https://github.com/ros2/launch/issues/545
# The issue should be fixed by https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1376
# This is a workaround to kill the gzserver after the tests run.
# Remove this once gazebo updates the apt package to the latest version.
@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_kill_sim(self):
        subprocess.run(["pkill", "gzserver"])
        subprocess.run(["pkill", "gzclient"])


class TestSetJointsPositionsPlugin(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.__test_joint_name = 'set_link_joint'

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def joint_state_callback(self, msg):
        self.__joint_state = msg

    def setUp(self):

        def spin_srv(executor):
            try:
                executor.spin()
            except rclpy.executors.ExternalShutdownException:
                pass

        self.node = rclpy.create_node('test_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        srv_executor = MultiThreadedExecutor()
        srv_executor.add_node(self.node)
        srv_thread = Thread(target=spin_srv, args=(srv_executor,), daemon=True)
        srv_thread.start()

        self.clock = rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.ROS_TIME)
        self.log = self.node.get_logger()

        # Client for checking entity status in gazebo
        self.log.info("Creating subscriber for joint state...")
        self.__joint_state = None
        self.__joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        #  publisher with latching QoS
        qos_profile = QoSProfile(
            depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.joint_publisher = self.node.create_publisher(
            JointState, '/ur_driver/joint_states', qos_profile)

        self.robot_name = 'test_robot'
        self.world_frame = 'world'

        sleep(5)  # Give time to Gazebo client/server to bring up

    def tearDown(self):
        self.node.destroy_node()

    def test_set(self):
        while (self.__joint_state is None):
            time.sleep(1)
        self.set_and_test_position(1.0)
        self.set_and_test_position(3.0)
        self.set_and_test_position(-1.0)
        self.set_and_test_position(-3.0)

    def set_and_test_position(self, test_position):
        # Set test position
        self.set_position(test_position)
        time.sleep(2)  # Small sleep to wait for the set to take effect

        # Get entity joint position
        self.assertAlmostEqual(
            self.__joint_state.position[0], test_position)

    def set_position(self, position):
        test_state = JointState()
        test_state.header = Header()
        test_state.header.stamp = self.node.get_clock().now().to_msg()
        test_state.header.frame_id = 'world'
        test_state.name = [self.__test_joint_name]
        test_state.position = [position]
        test_state.velocity = []
        test_state.effort = []
        self.joint_publisher.publish(test_state)
