// Copyright 2018 Boeing
#include <algorithm>
#include <string>

#include <set_joint_positions/set_joint_positions.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(SetJointPositions)

SetJointPositions::SetJointPositions()
{
}

SetJointPositions::~SetJointPositions()
{
    event::Events::DisconnectWorldUpdateBegin(update_connection_);

    // Custom Callback Queue
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
    joints_list_.clear();
    sub_.shutdown();

    delete rosnode_;
}

void SetJointPositions::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    ROS_INFO("Initialising SetJointPositions Plugin");

    // load parameters
    robot_namespace_ = "";

    if (_sdf->HasElement("robotNamespace"))
        robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    if (!_sdf->HasElement("topicName"))
    {
        ROS_FATAL("force plugin missing <topicName>, cannot proceed");
        return;
    }
    else
    {
        topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
    }

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    ROS_INFO_STREAM("topicName:" << topic_name_);

    rosnode_ = new ros::NodeHandle(robot_namespace_);

    sub_ = rosnode_->subscribe(topic_name_, 1, &SetJointPositions::jointStateCallback, this, ros::TransportHints().tcpNoDelay());

    // Custom Callback Queue
    callback_queue_thread_ = std::thread(&SetJointPositions::queueThread, this);

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every simulation iteration
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SetJointPositions::UpdateChild, this));

    joints_list_ = _model->GetJoints();
}

void SetJointPositions::jointStateCallback(const sensor_msgs::JointState msg)
{
    std::lock_guard<std::mutex> lock(lock_);

    for (std::size_t i=0; i < msg.name.size(); ++i)
    {
        const std::string& name = msg.name.at(i);

        auto it = std::find_if(joints_list_.begin(),
                               joints_list_.end(),
                               [name](const physics::JointPtr& jt)
        {
            return jt->GetName() == name;
        });  // NOLINT
        if (it == joints_list_.end())
        {
            ROS_WARN_STREAM("Could not find JointState message joint " << name << " in gazebo joint models");
        }
        else
        {
            (*it)->SetPosition(0, msg.position[i]);
        }
    }
}

void SetJointPositions::UpdateChild()
{
    std::lock_guard<std::mutex> lock(lock_);

    // Maybe update position every frame?
}

void SetJointPositions::queueThread()
{
    const double timeout = 0.01;
    while (rosnode_->ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

}  // namespace gazebo
