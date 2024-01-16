#include <gazebo_set_joint_positions_plugin/gazebo_set_joint_positions_plugin.h>

#include <algorithm>
#include <string>
#include <vector>

#include "gazebo_ros/node.hpp"

namespace gazebo
{

namespace
{

template <typename TYPE>
void loadParam(sdf::ElementPtr sdf, TYPE& value, const TYPE& default_value, const std::string& param_name)
{
    if (!sdf->HasElement(param_name))
    {
        value = default_value;
    }
    else
    {
        value = sdf->GetElement(param_name)->Get<TYPE>();
    }
}
}  // namespace

SetJointPositions::SetJointPositions() : update_needed_(false)
{
}

SetJointPositions::~SetJointPositions()
{
}

// cppcheck-suppress unusedFunction
void SetJointPositions::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    model_ = _model;

    loadParam(_sdf, robot_namespace_, std::string("/"), std::string("robot_namespace"));
    loadParam(_sdf, topic_name_, std::string("/joint_states"), std::string("topic_name"));

    nh_ = gazebo_ros::Node::Get(_sdf);
    sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
        topic_name_, 1, std::bind(&SetJointPositions::jointStateCallback, this, std::placeholders::_1));
    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every simulation iteration
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SetJointPositions::UpdateChild, this));

    joints_list_ = _model->GetJoints();
    links_list_ = _model->GetLinks();

    for (physics::LinkPtr link : links_list_)
    {
        link->SetEnabled(false);
    }

    RCLCPP_INFO(nh_->get_logger(), "Loaded SetJointPositions gazebo plugin. Watching topic: %s", topic_name_.c_str());
}

void SetJointPositions::jointStateCallback(const sensor_msgs::msg::JointState msg)
{
    {  // Start lock
        std::lock_guard<std::mutex> lock(lock_);
        joint_state_ = msg;
    }  // End lock
    update_needed_ = true;
}

void SetJointPositions::UpdateChild()
{
    if (update_needed_)  // Only update if there is a change. Any updates should still be done on the world update event
    {
        sensor_msgs::msg::JointState last_joint_state;  // Local copy of callback value
        {                                               // Lock scope, keep the locks tight for speed
            std::lock_guard<std::mutex> lock(lock_);
            last_joint_state = joint_state_;
        }

        if (last_joint_state.header.stamp == rclcpp::Time(0))
            return;

        if (last_joint_state.position.empty())
            return;

        for (std::size_t i = 0; i < last_joint_state.name.size(); ++i)
        {
            const std::string& name = last_joint_state.name.at(i);

            auto it = std::find_if(joints_list_.begin(), joints_list_.end(),
                                   [name](const physics::JointPtr& jt) { return jt->GetName() == name; });  // NOLINT

            if (it == joints_list_.end())
            {
                RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                            "Could not find JointState message joint " << name
                                                                                       << " in gazebo joint models");
            }
            else
            {
                double position = last_joint_state.position[i];
                double upper_limit = (*it)->UpperLimit(0);
                double lower_limit = (*it)->LowerLimit(0);

                // Bounds checks are required, if outside bounds Gazebo will not update the joint!
                if (position > upper_limit)
                {
                    RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                                "Joint " << (*it)->GetName() << " is above upper limit " << position
                                                         << " > " << upper_limit);
                    position = upper_limit;
                }
                else if (position < lower_limit)
                {
                    RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                                "Joint " << (*it)->GetName() << " is below lower limit " << position
                                                         << " < " << lower_limit);
                    position = lower_limit;
                }

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
                        upper_limit = (*it_mimic)->UpperLimit(0);
                        lower_limit = (*it_mimic)->LowerLimit(0);
                        double old_angle = (*it_mimic)->Position(0);

                        // Bounds checks are required, if outside bounds Gazebo will not update the joint!
                        if (position > upper_limit)
                        {
                            RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                                        "Joint " << (*it_mimic)->GetName() << " is above upper limit "
                                                                 << position << " > " << upper_limit);
                            position = upper_limit;
                        }
                        else if (position < lower_limit)
                        {
                            RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                                        "Joint " << (*it_mimic)->GetName() << " is below lower limit "
                                                                 << position << " < " << lower_limit);
                            position = lower_limit;
                        }

                        RCLCPP_DEBUG_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                                     "Updating joint " << (*it_mimic)->GetName() << " from "
                                                                       << old_angle << " to " << position);
                        (*it_mimic)->SetPosition(0, position);
                    }
                }
            }
        }
        update_needed_ = false;  // Flag we are done
    }
}

GZ_REGISTER_MODEL_PLUGIN(SetJointPositions)
}  // namespace gazebo
