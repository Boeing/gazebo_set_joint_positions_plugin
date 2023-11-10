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

Copyright 2023 The Boeing Company

Licensed under the Apache License, Version 2.0 (the "License") with the following modification;
you may not use this file except in compliance with the Apache License and the following modification to it:

(Appended as Section 10)

By accepting this software, recipient agrees that the representations, warranties, obligations, and liabilities of The Boeing Company set forth in this software, if any, are exclusive and in substitution for all other all other representations, warranties, obligations, and liabilities of The Boeing Company.
Recipient hereby waives, releases, and renounces all other rights, remedies, and claims (including tortious claims for loss of or damage to property) of recipient against The Boeing Company with respect to this software.
The Boeing Company hereby disclaims all implied warranties, including but not limited to, all implied warranties of merchantability, fitness, course of dealing, and usage of trade.
The Boeing Company shall have no liability, whether arising in contract (including warranty), tort (whether or not arising from the negligence of The Boeing Company), or otherwise, for any loss of use, revenue, or profit, or for any other unspecified direct, indirect, incidental, or consequential damages for anything delivered or otherwise provided by The Boeing Company under this software.

You may obtain a copy of the original, unmodified License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
echo 

# Contributing

Any contribution that you make to this repository will
be under the Modified Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0):

```
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
```

To contribute, issue a PR and @brta-jc (jason.cochrane@boeing.com)
