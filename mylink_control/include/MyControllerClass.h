#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>


namespace mylink_control{

    class MyControllerClass: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    private:
        /* data */
        hardware_interface::JointHandle joint_;
        double init_pos_;
        ros::Subscriber sub_command_;
    public:
    struct Commands
    {
        double position_;//上一次的命令
        double velocity_;
        bool has_velocity_;
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
        virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        virtual void starting();
        virtual void update(const ros::Time& time, const ros::Duration& period);
        virtual void stopping();
        void setCommandCB(const std_msgs::Float64ConstPtr& msg);
        void setCommand(double pos_target);
        void setCommand(double pos_target, double vel_target);
    
        // MyControllerClass();
        // ~MyControllerClass();
    };
    

    
}