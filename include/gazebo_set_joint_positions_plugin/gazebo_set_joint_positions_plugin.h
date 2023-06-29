#ifndef GAZEBO_SET_JOINT_POSITIONS_PLUGIN_H
#define GAZEBO_SET_JOINT_POSITIONS_PLUGIN_H

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace gazebo
{

class SetJointPositions : public ModelPlugin
{
  public:
    SetJointPositions();
    virtual ~SetJointPositions();

  protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void UpdateChild();

  private:
    void jointStateCallback(const sensor_msgs::msg::JointState msg);

    physics::ModelPtr model_;
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

    std::mutex lock_;
    sensor_msgs::msg::JointState joint_state_;

    std::string topic_name_;
    std::string robot_namespace_;

    bool update_needed_;

    event::ConnectionPtr update_connection_;

    std::vector<physics::JointPtr> joints_list_;
    std::vector<physics::LinkPtr> links_list_;
};
}  // namespace gazebo

#endif
