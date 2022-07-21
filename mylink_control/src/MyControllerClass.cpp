#include "MyControllerClass.h"
#include <pluginlib/class_list_macros.h>

namespace mylink_control{

    bool MyControllerClass::init(hardware_interface::EffortJointInterface *robot,
                                ros::NodeHandle &n)
    {
        std::string joint_name;
        if(!n.getParam("joint_name",joint_name))
        {ROS_ERROR("No joint given in namespace: '%s'",n.getNamespace().c_str());
        ROS_INFO("fuck????");
        return false;
        }else
        {
            
            

        }


        
        ROS_INFO("fuck!!!!!");
        joint_=robot->getHandle(joint_name);
        sub_command_ = n.subscribe<std_msgs::Float64>("command",
        1,
        &MyControllerClass::setCommandCB,this);
        return true;
    }

    void MyControllerClass::starting()
    {
        init_pos_=joint_.getPosition();
    }

    void MyControllerClass::update(const ros::Time& time,const ros::Duration& period)
    {
        command_struct_ = *(command_.readFromRT());
        double command_position = command_struct_.position_;
        double desired_pos = init_pos_ + 0.5 * sin(ros::Time::now().toSec());
        double current_pos = joint_.getPosition();

        double commanded_effort = 1000 * (command_position - current_pos);
        joint_.setCommand(commanded_effort);
    }
    void MyControllerClass::stopping()
    {}

    void MyControllerClass::setCommandCB(const std_msgs::Float64ConstPtr& msg)
    {
        setCommand(msg->data);
    }
    void MyControllerClass::setCommand(double pos_command)
    {
        command_struct_.position_ = pos_command;
        command_struct_.has_velocity_ = false;
        command_.writeFromNonRT(command_struct_);
    }
    void MyControllerClass::setCommand(double pos_command,double vel_command)
    {
        command_struct_.position_=pos_command;
        command_struct_.velocity_=vel_command;
        command_struct_.has_velocity_=true;
    }

}
PLUGINLIB_EXPORT_CLASS(mylink_control::MyControllerClass, 
                         controller_interface::ControllerBase);