#include <gazebo_set_joint_positions_plugin/gazebo_set_joint_positions_plugin.h>

#include <algorithm>
#include <string>
#include <vector>

namespace gazebo
{

    namespace
    {

        template <typename TYPE>
        void loadParam(sdf::ElementPtr sdf, TYPE &value, const TYPE &default_value, const std::string &param_name)
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
    } // namespace

    SetJointPositions::SetJointPositions()
    {
    }

    SetJointPositions::~SetJointPositions()
    {
    }

    // cppcheck-suppress unusedFunction
    void SetJointPositions::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        loadParam(_sdf, robot_namespace_, std::string("/"), std::string("robot_namespace"));
        loadParam(_sdf, topic_name_, std::string("/joint_states"), std::string("topic_name"));

        nh_ = std::make_shared<rclcpp::Node>("gazebo_set_joint_positions_plugin", robot_namespace_);

        sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(topic_name_, 1,
                                                                      std::bind(&SetJointPositions::jointStateCallback, this, std::placeholders::_1));
        //    sub_ =
        //    nh_.subscribe(topic_name_, 1, &SetJointPositions::jointStateCallback, this, ros::TransportHints().tcpNoDelay());

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

    void SetJointPositions::jointStateCallback(const sensor_msgs::msg::JointState msg)
    {
        std::lock_guard<std::mutex> lock(lock_);
        joint_state_ = msg;
    }

    void SetJointPositions::UpdateChild()
    {
        auto logger_ = nh_->get_logger();
        std::lock_guard<std::mutex> lock(lock_);

        if (joint_state_.header.stamp == rclcpp::Time(0))
            return;

        if (joint_state_.position.empty())
            return;

        for (std::size_t i = 0; i < joint_state_.name.size(); ++i)
        {
            const std::string &name = joint_state_.name.at(i);

            auto it = std::find_if(joints_list_.begin(), joints_list_.end(),
                                   [name](const physics::JointPtr &jt)
                                   { return jt->GetName() == name; }); // NOLINT

            if (it == joints_list_.end())
            {
                RCLCPP_WARN_STREAM_THROTTLE(logger_, *nh_->get_clock(), 1000,
                                            "Could not find JointState message joint " << name << " in gazebo joint models");
            }
            else
            {
                double position = joint_state_.position[i];
#if GAZEBO_MAJOR_VERSION >= 8
                double upper_limit = (*it)->UpperLimit(0);
                double lower_limit = (*it)->LowerLimit(0);
#else
                double upper_limit = (*it)->GetUpperLimit(0).Radian();
                double lower_limit = (*it)->GetLowerLimit(0).Radian();
#endif
                // Bounds checks are required, if outside bounds Gazebo will not update the joint!
                if (position > upper_limit)
                {
                    RCLCPP_WARN_STREAM_THROTTLE(logger_, *nh_->get_clock(), 1000, "Joint " << (*it)->GetName() << " is above upper limit " << position << " > " << upper_limit);
                    position = upper_limit;
                }
                else if (position < lower_limit)
                {
                    RCLCPP_WARN_STREAM_THROTTLE(logger_, *nh_->get_clock(), 1000, "Joint " << (*it)->GetName() << " is below lower limit " << position << " < " << lower_limit);
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
#if GAZEBO_MAJOR_VERSION >= 8
                        upper_limit = (*it_mimic)->UpperLimit(0);
                        lower_limit = (*it_mimic)->LowerLimit(0);
                        double old_angle = (*it_mimic)->Position(0);
#else
                        upper_limit = (*it_mimic)->GetUpperLimit(0).Radian();
                        lower_limit = (*it_mimic)->GetLowerLimit(0).Radian();
                        gazebo::math::Angle old_angle = (*it_mimic)->GetAngle(0);
#endif
                        // Bounds checks are required, if outside bounds Gazebo will not update the joint!
                        if (position > upper_limit)
                        {
                            RCLCPP_WARN_STREAM_THROTTLE(logger_, *nh_->get_clock(), 1000, "Joint " << (*it_mimic)->GetName() << " is above upper limit " << position << " > " << upper_limit);
                            position = upper_limit;
                        }
                        else if (position < lower_limit)
                        {
                            RCLCPP_WARN_STREAM_THROTTLE(logger_, *nh_->get_clock(), 1000, "Joint " << (*it_mimic)->GetName() << " is below lower limit " << position << " < " << lower_limit);
                            position = lower_limit;
                        }

                        RCLCPP_DEBUG_STREAM(logger_, "Updating joint " << (*it_mimic)->GetName() << " from " << old_angle << " to "
                                                                       << position);
                        (*it_mimic)->SetPosition(0, position);
                    }
                }
            }
        }
        // Hack disables physics, required after call to any physics related function call
        for (physics::LinkPtr link : links_list_)
        {
            link->SetEnabled(false);
            link->OnPoseChange();
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(SetJointPositions)
} // namespace gazebo
