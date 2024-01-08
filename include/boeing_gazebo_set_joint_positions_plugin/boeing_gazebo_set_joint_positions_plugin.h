#ifndef GAZEBO_SET_JOINT_POSITIONS_PLUGIN_H
#define GAZEBO_SET_JOINT_POSITIONS_PLUGIN_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

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
    void jointStateCallback(const sensor_msgs::JointState msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    std::mutex lock_;
    sensor_msgs::JointState joint_state_;

    std::string topic_name_;
    std::string robot_namespace_;

    event::ConnectionPtr update_connection_;

    std::vector<physics::JointPtr> joints_list_;
    std::vector<physics::LinkPtr> links_list_;
};
}  // namespace gazebo

#endif
