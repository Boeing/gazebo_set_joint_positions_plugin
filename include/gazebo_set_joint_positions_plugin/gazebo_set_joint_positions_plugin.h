// Copyright 2018 Boeing
#ifndef GAZEBO_SET_JOINT_POSITIONS_PLUGIN_H
#define GAZEBO_SET_JOINT_POSITIONS_PLUGIN_H

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>

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

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    std::thread callback_queue_thread_;
    void queueThread();

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;

    std::vector<physics::JointPtr> joints_list_;
    std::vector<physics::LinkPtr> links_list_;
};

}  // namespace gazebo

#endif  // GAZEBO_SET_JOINT_POSITIONS_PLUGIN_H
