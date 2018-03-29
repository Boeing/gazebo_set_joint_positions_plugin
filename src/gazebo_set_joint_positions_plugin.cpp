// Copyright 2018 Boeing
#include <gazebo_set_joint_positions_plugin/gazebo_set_joint_positions_plugin.h>
#include <algorithm>
#include <string>
#include <vector>


namespace gazebo
{


SetJointPositions::SetJointPositions()
{
}

SetJointPositions::~SetJointPositions()
{
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

    sub_ = rosnode_->subscribe(topic_name_, 1, &SetJointPositions::jointStateCallback, this,
                               ros::TransportHints().tcpNoDelay());

    // Custom Callback Queue
    callback_queue_thread_ = std::thread(&SetJointPositions::queueThread, this);

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every simulation iteration
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SetJointPositions::UpdateChild, this));

    joints_list_ = _model->GetJoints();
    links_list_ = _model->GetLinks();

    for (physics::LinkPtr link : links_list_)
    {
        link->SetEnabled(false);
    }
}

void SetJointPositions::jointStateCallback(const sensor_msgs::JointState msg)
{
    std::lock_guard<std::mutex> lock(lock_);
    joint_state_ = msg;
}

void SetJointPositions::UpdateChild()
{
    std::lock_guard<std::mutex> lock(lock_);

    for (std::size_t i = 0; i < joint_state_.name.size(); ++i)
    {
        const std::string &name = joint_state_.name.at(i);

        auto it = std::find_if(joints_list_.begin(),
                               joints_list_.end(),
                               [name](const physics::JointPtr &jt)
                               {
                                   return jt->GetName() == name;
                               }); // NOLINT

        if (it == joints_list_.end())
        {
            ROS_WARN_STREAM_THROTTLE(1, "Could not find JointState message joint " << name
                                                                                   << " in gazebo joint models");
        }
        else
        {
            double position = joint_state_.position[i];
#if GAZEBO_MAJOR_VERSION >= 8
            double upper_limit = (*it)->UpperLimit(0);
            double lower_limit = (*it)->LowerLimit(0);
            double old_angle = (*it)->Position(0);
#else
            double upper_limit = (*it)->GetUpperLimit(0).Radian();
            double lower_limit = (*it)->GetLowerLimit(0).Radian();
            gazebo::math::Angle old_angle = (*it)->GetAngle(0);
#endif
            // Bounds checks are required, if outside bounds Gazebo will not update the joint!
            if (position > upper_limit)
            {
                ROS_WARN_STREAM_THROTTLE(1, "Joint " << (*it)->GetName() << " is above upper limit " << position
                                                     << " > " << upper_limit);
                position = upper_limit;
            }
            else if (position < lower_limit)
            {
                ROS_WARN_STREAM_THROTTLE(1, "Joint " << (*it)->GetName() << " is below lower limit " << position
                                                     << " < " << lower_limit);
                position = lower_limit;
            }

            ROS_DEBUG_STREAM(
                "Updating joint " << (*it)->GetName() << " from " << old_angle << " to " << position);

            (*it)->SetPosition(0, position);

            // Now also check if a mimic joint exists, and set it if it does
            using Iter = std::vector<physics::JointPtr>::const_iterator;
            for (Iter it_mimic = joints_list_.begin(); it_mimic != joints_list_.end(); ++it_mimic)
            {
                bool set_mimic = false;
                if ((*it_mimic)->GetName() == name + "_mimic")
                {
                    set_mimic = true;
                }
                else if ((*it_mimic)->GetName() == name + "_mimic_inverted")
                {
                    set_mimic = true;
                    position = -position;
                }

                if (set_mimic)
                {
#if GAZEBO_MAJOR_VERSION >= 8
                    double upper_limit = (*it_mimic)->UpperLimit(0);
                    double lower_limit = (*it_mimic)->LowerLimit(0);
                    double old_angle = (*it_mimic)->Position(0);
#else
                    double upper_limit = (*it_mimic)->GetUpperLimit(0).Radian();
                    double lower_limit = (*it_mimic)->GetLowerLimit(0).Radian();
                    gazebo::math::Angle old_angle = (*it_mimic)->GetAngle(0);
#endif
                    // Bounds checks are required, if outside bounds Gazebo will not update the joint!
                    if (position > upper_limit)
                    {
                        ROS_WARN_STREAM_THROTTLE(1, "Joint " << (*it_mimic)->GetName() << " is above upper limit "
                                                             << position << " > "
                                                             << upper_limit);
                        position = upper_limit;
                    }
                    else if (position < lower_limit)
                    {
                        ROS_WARN_STREAM_THROTTLE(1, "Joint " << (*it_mimic)->GetName() << " is below lower limit "
                                                             << position << " < "
                                                             << lower_limit);
                        position = lower_limit;
                    }

                    ROS_DEBUG_STREAM(
                        "Updating joint " << (*it_mimic)->GetName() << " from " << old_angle
                                          << " to " << position);
                    (*it_mimic)->SetPosition(0, position);
                }
            }
        }
    }
    // Hack disables physics, required after call to any physics related function call
    for (physics::LinkPtr link : links_list_)
    {
        link->SetEnabled(false);
    }
}

void SetJointPositions::queueThread()
{
    const double timeout = 0.01;
    while (rosnode_->ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

GZ_REGISTER_MODEL_PLUGIN(SetJointPositions)
}  // namespace gazebo
