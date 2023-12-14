# Gazebo Set Joint Positions Plugin

| Distro | CI Status |
| ------ | --------- |
| Noetic | [![CI](https://github.com/Boeing/gazebo_set_joint_positions_plugin/actions/workflows/main.yml/badge.svg?branch=humble)](https://github.com/Boeing/gazebo_set_joint_positions_plugin/actions/workflows/main.yml) |
| Humble | [![CI](https://github.com/Boeing/gazebo_set_joint_positions_plugin/actions/workflows/main.yml/badge.svg?branch=noetic)](https://github.com/Boeing/gazebo_set_joint_positions_plugin/actions/workflows/main.yml) |

## Motivation

Multi joint robot drivers publish their current joint positions on a _JointState_ topic.
In order to simulate these robots in Gazebo, a way to set a Gazebo _Model_ joint state to the drivers joint state is required.
This plugin sets a RobotModel's joint values to the latest published values on a speicifed JointState topic from ROS. This topic is typically published by a robot driver (ie. Universal Robotics driver) or by ROS2 Control. By using this plugin you are not required to use the gazebo_ros_control_plugin (or ROS control at all).

## Goals

- Listen for a _JointState_ on the default topic /joint_states, set the Gazebo _Model_ to match the joint states.


## Definitions

| Definition | Description                                                                                                              |
| ---------- | ------------------------------------------------------------------------------------------------------------------------ |
| Model       | An instanced SDF in Gazebo. _Models_ consist of _Links_ connected by _Joints_.                                                                                                               |
| Link        | A sub component of a _Model_. This plugin creates static 'attachment' _Joints_ between _Links_. _Links_ can contain geometry elements or attached plugins.                                                                                                               |
| Joint       | A connection between _Links_ in Gazebo.


### Assumptions

- The _number_ of joints in the published _JointState_ message must match the number of Joints in the associated _Model_.
- The _name_ of the joints in the _JointState_ message must match the name of a joint in the _Model_.

### Limitations

- All joints being set must be _revolute_ type.
- If the published JointState for any Joint is outside the upper or lower joint limits (as defined by the _Model_ SDF), the joint limits will be enforced and the joint shall be set to the limit value.

# Authors
The Boeing Company

        Boeing Reasearch and Technology Advanced Production Systems Team

# License
The package is released under the Apache 2.0 License

# Contributing

Any contribution that you make to this repository will
be under the Apache-2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0)

To contribute, issue a PR and @brta-jc (jason.cochrane@boeing.com)
