#!/usr/bin/env python3
import os
import unittest

import launch
import launch.actions

import launch_testing
import launch_testing.actions
from ament_index_python import get_package_share_directory

import subprocess
import rclpy
import rclpy.clock
import rclpy.time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
import pytest
from rclpy.parameter import Parameter

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist

from time import sleep


@pytest.mark.launch_test
def generate_test_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file_name = os.path.join(get_package_share_directory('gazebo_planar_move_plugin'),
                                   'test', 'test.world')
    urdf_file_name = os.path.join(get_package_share_directory('gazebo_planar_move_plugin'),
                                  'test', 'test.urdf')

    print('robot  urdf_file_name : {}'.format(urdf_file_name))
    print('world world_file_name : {}'.format(world_file_name))

    return launch.LaunchDescription(
        [
            # Launch GAZEBO
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch',
                                 'gzserver.launch.py')
                ),
                launch_arguments={
                    'world': world_file_name, 'gui': '0'}.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch',
                                 'gzclient.launch.py')
                ),
                launch_arguments={'gui': '0'}.items(),
            ),

            # Launch robot_state_publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[urdf_file_name]
            ),

            # Spawn robot in Gazebo
            Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'test_robot', '-file', urdf_file_name],
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


class TestPlanarMovePlugin(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.__get_joint_state_srv = rospy.ServiceProxy(
            name='/gazebo/get_joint_properties',
            service_class=GetJointProperties
        )
        if not self.entity_state_client.wait_for_service(timeout_sec=10.0):
            raise Exception("Entity state service not available, waiting again...")

        cls.__joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=100, latch=True)

        cls.__test_joint_name = 'set_link_joint'

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.clock = rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.ROS_TIME)
        self.log = self.node.get_logger()

        # Client for checking entity status in gazebo
        self.entity_state_client = self.node.create_client(
            GetEntityState, '/gazebo/get_entity_state')
        while not self.entity_state_client.wait_for_service(timeout_sec=1.0):
            self.log.info(
                'Entity state service not available, waiting again...')

        # Twist publisher with latching QoS
        qos_profile = QoSProfile(
            depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.twist_publisher = self.node.create_publisher(
            Twist, 'cmd_vel', qos_profile)

        self.robot_name = 'test_robot'
        self.world_frame = 'world'

        sleep(5)  # Give time to Gazebo client/server to bring up

    def tearDown(self):
        self.node.destroy_node()

    def test_set(self):
        rospy.sleep(1)
        self.set_and_test_position(1.0)
        self.set_and_test_position(3.0)
        self.set_and_test_position(-1.0)
        self.set_and_test_position(-3.0)

    def set_and_test_position(self, test_position):
        # Set test position
        self.set_position(test_position)

        rospy.sleep(1)  # Small sleep to wait for the set to take effect

        # Get joint position
        get_joint_response = self.__get_joint_state_srv.call(
            GetJointPropertiesRequest(joint_name=self.__test_joint_name))

        rospy.loginfo('Joint pose retrieved as value: ''{}'' expected ''{}'''.format(get_joint_response.position[0],
                                                                                     test_position))
        assert isinstance(get_joint_response, GetJointPropertiesResponse)
        self.assertTrue(get_joint_response.success)
        self.assertAlmostEqual(get_joint_response.position[0], test_position)

    def set_position(self, position):
        test_state = JointState()
        test_state.header = Header()
        test_state.header.stamp = rospy.Time.now()
        test_state.header.frame_id = 'world'
        test_state.name = [self.__test_joint_name]
        test_state.position = [position]
        test_state.velocity = []
        test_state.effort = []
        self.__joint_state_pub.publish(test_state)
