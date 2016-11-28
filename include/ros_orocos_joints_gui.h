#ifndef ROS_OROCOS_JOINTS_GUI_H
#define ROS_OROCOS_JOINTS_GUI_H

// RTT header files. Might missing some or some be unused
#include <rtt/RTT.hpp>
#include <string>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>


class ros_orocos_joints_gui_server: public RTT::TaskContext {
public:
    ros_orocos_joints_gui_server(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    //void updateHook();
    //void stopHook();
    //void cleanupHook();

private:
    bool attachToRobot(const std::string& robot_name);
    sensor_msgs::JointState _joint_state_msg;
    std::string _robot_name;
    std::vector<std::string> _joint_list;

    std::map<std::string, std::vector<std::string> > _map_kin_chains_joints;

    RTT::InputPort<sensor_msgs::JointState> _joint_state_port;
    RTT::OutputPort<sensor_msgs::JointState> _joint_zero_port;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> > > _kinematic_chains_feedback_ports;
    std::map<std::string, rstrt::robot::JointState> _kinematic_chains_joint_state_map;

    std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > > _kinematic_chains_output_ports;
    std::map<std::string, rstrt::kinematics::JointAngles> _kinematic_chains_desired_joint_state_map;
};

#endif
