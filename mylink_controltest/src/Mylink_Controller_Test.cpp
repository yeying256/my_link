#include "Mylink_Controller_Test.h"
#include <pluginlib/class_list_macros.h>

namespace mylink_controller_test_ns
{
    bool Mylink_Controller_Test::init(hardware_interface::EffortJointInterface *robot,
                            ros::NodeHandle &n)
    {
    std::string joint_name;
    if (!n.getParam("joint_name", joint_name))
    {
        ROS_ERROR("No joint given in namespace: '%s')",
                n.getNamespace().c_str());
        return false;
    }
    
    joint_ = robot->getHandle(joint_name);
    sub_command_ = n.subscribe<std_msgs::Float64>("command", 
                                    1, &Mylink_Controller_Test::setCommandCB, 
                                    this);
    return true;
    }

    void Mylink_Controller_Test::starting()
    {
        init_pos_ = joint_.getPosition();
    }


    void Mylink_Controller_Test::update(const ros::Time& time, const ros::Duration& period)
    {
    static ros::Time time_from_start(0);
    time_from_start+=period;

    command_struct_ = *(command_.readFromRT());
    double command_position = command_struct_.position_;
    double current_pos = joint_.getPosition();
    double commanded_effort = 10000 * (command_position-current_pos);
    joint_.setCommand(commanded_effort);
    }

    void Mylink_Controller_Test::stopping()
    {}

    void Mylink_Controller_Test::setCommandCB(const std_msgs::Float64ConstPtr& msg)
    {
    setCommand(msg->data);
    }
    void Mylink_Controller_Test::setCommand(double pos_command)
    {
    command_struct_.position_ = pos_command;
    command_.writeFromNonRT(command_struct_);
    }


}
PLUGINLIB_EXPORT_CLASS(mylink_controller_test_ns::Mylink_Controller_Test, 
                         controller_interface::ControllerBase);