# Gazebo Set Joint Positions Plugin

[![pipeline status](https://git.web.boeing.com/robotics/ros/gazebo_set_joint_positions_plugin/badges/master/pipeline.svg)](https://git.web.boeing.com/robotics/ros/gazebo_set_joint_positions_plugin/commits/master)
[![coverage](https://git.web.boeing.com/robotics/ros/gazebo_set_joint_positions_plugin/badges/master/coverage.svg)](https://git.web.boeing.com/robotics/ros/gazebo_set_joint_positions_plugin/commits/master)

## Motivation

All multi jointed robot drivers in ROS will publish their current joint positions on a _JointState_ topic.
In order to simulate these robots in Gazebo, a way to set a Gazebo _Model_ joint state to the drivers joint state is required.

## Goals

- Listen for a _JointState_ on default topic /joint_states, set the Gazebo _Model_ to match.

## Requirements

- The JointState must be published periodically, not latched.
- The plugin must be attached to a _Model_, not to the _World._

## Definitions

| Definition | Description                                                                                                              |
| ---------- | ------------------------------------------------------------------------------------------------------------------------ |
| Model       | An instanced SDF in Gazebo. _Models_ consist of _Links_ connected by _Joints_.                                                                                                               |
| Link        | A sub component of a _Model_. This plugin creates static 'attachment' _Joints_ between _Links_. _Links_ can contain geometry elements or attached plugins.                                                                                                               |
| Joint       | A connection between _Links_ in Gazebo.

## Design

### Assumptions

- The _number_ of joints in the published _JointState_ message must match the number of Joints in the associated _Model_.
- The _name_ of the joints in the _JointState_ message must match the name of a joint in the _Model_.

### Limitations

- All joints being set must be _revolute_ type.
- If the published JointState for any Joint is outside the upper or lower joint limits (as defined by the _Model_ SDF), the joint limits will be enforced and the joint shall be set to the limit value.

## Requirements Evaluation

| Requirement | Met? | Comments |
| ------------| ------- | ---------- |
| Model has its joint positions set in line with the /ur_driver/joint_state topic. | Yes | None |
| Joints do not move outside of upper or lower limit range. | Yes | None |

## Related Components

| Name                | Link                                                                       |
| ------------------- | -------------------------------------------------------------------------- |
| gazebo_ros_pkgs | https://git.web.boeing.com/robotics/ros-thirdparty/gazebo_ros_pkgs |
| universal_robot | https://git.web.boeing.com/robotics/ros/universal_robot |
| gazebo_no_physics_plugin | https://git.web.boeing.com/robotics/ros/gazebo_no_physics_plugin |