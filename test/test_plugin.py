#!/usr/bin/env python

import unittest
import rospy
import rostest
from std_msgs.msg import Header
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, GetJointPropertiesResponse
from sensor_msgs.msg import JointState


class TestSetJointPositionsPlugin(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.__get_joint_state_srv = rospy.ServiceProxy(
            name='/gazebo/get_joint_properties',
            service_class=GetJointProperties
        )
        cls.__get_joint_state_srv.wait_for_service(timeout=30)

        cls.__joint_state_pub = rospy.Publisher("/ur_driver/joint_states", JointState, queue_size=100, latch=True)

        cls.__test_joint_name = 'set_link_joint'

    def test_set(self):
        self.set_and_test_position(1.0)
        self.set_and_test_position(3.0)
        self.set_and_test_position(-1.0)
        self.set_and_test_position(-3.0)

    def set_and_test_position(self, test_position):
        # Set test position
        self.set_position(test_position)

        rospy.sleep(0.5)  # Small sleep to wait for the set to take effect
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


if __name__ == '__main__':
    rospy.init_node('test_plugin')

    rostest.rosrun('set_joint_positions', 'test_set_joint_positions', TestSetJointPositionsPlugin)
