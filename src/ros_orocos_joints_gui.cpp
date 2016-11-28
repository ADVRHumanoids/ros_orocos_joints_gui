#include <ros_orocos_joints_gui.h>
#include <rtt/plugin/PluginLoader.hpp>
#include <ros/ros.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rtt_rostopic.h>



ros_orocos_joints_gui_server::ros_orocos_joints_gui_server(std::string const & name):
    RTT::TaskContext(name)
{
    this->setActivity(new RTT::Activity(1, 0.01));

    this->addOperation("attachToRobot", &ros_orocos_joints_gui_server::attachToRobot,
                this, RTT::ClientThread);
}

bool ros_orocos_joints_gui_server::configureHook()
{
    this->addPort(_joint_state_desired_port).doc("Joint State desired from ROS gui");

    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::vector<std::string> joints = it->second;
        for(unsigned int i = 0; i < joints.size(); ++i)
            _joint_list.push_back(joints[i]);
    }

    _joint_state_msg.name = _joint_list;
    for(unsigned int i = 0; i < _joint_list.size(); ++i)
    {
        _joint_state_msg.position.push_back(0.0);
        _joint_state_msg.effort.push_back(0.0);
        _joint_state_msg.velocity.push_back(0.0);
    }
    RTT::log(RTT::Info)<<"joint_state_msg has been initialized"<<RTT::endlog();

    return true;
}

bool ros_orocos_joints_gui_server::startHook()
{
    _joint_state_desired_port.createStream(rtt_roscomm::topic("joint_states_desired"));

    return true;
}

bool ros_orocos_joints_gui_server::attachToRobot(const std::string &robot_name)
{
    _robot_name = robot_name;
    RTT::log(RTT::Info)<<"Robot name: "<<_robot_name<<RTT::endlog();

    RTT::TaskContext* task_ptr = this->getPeer(robot_name);
    if(!task_ptr){
        RTT::log(RTT::Error)<<"Can not getPeer("<<robot_name<<")"<<RTT::endlog();
        return false;}

    RTT::log(RTT::Info)<<"Found Peer "<<robot_name<<RTT::endlog();

    RTT::OperationCaller<std::map<std::string, std::vector<std::string> >(void) > getKinematicChainsAndJoints
        = task_ptr->getOperation("getKinematicChainsAndJoints");

    _map_kin_chains_joints = getKinematicChainsAndJoints();

    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string kin_chain_name = it->first;
        std::vector<std::string> joint_names = it->second;

        _kinematic_chains_feedback_ports[kin_chain_name] =
            boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> >(
                        new RTT::InputPort<rstrt::robot::JointState>(
                            kin_chain_name+"_"+"JointFeedback"));
        this->addPort(*(_kinematic_chains_feedback_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointFeedback port");

        _kinematic_chains_feedback_ports.at(kin_chain_name)->connectTo(
                    task_ptr->ports()->getPort(kin_chain_name+"_"+"JointFeedback"));


        _kinematic_chains_output_ports[kin_chain_name] =
                boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> >(
                            new RTT::OutputPort<rstrt::kinematics::JointAngles>(
                                kin_chain_name+"_"+"JointPositionCtrl"));
        this->addPort(*(_kinematic_chains_output_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointPositionCtrl port");
        _kinematic_chains_output_ports.at(kin_chain_name)->connectTo(
                    task_ptr->ports()->getPort(kin_chain_name+"_"+"JointPositionCtrl"));


        rstrt::robot::JointState tmp(joint_names.size());
        _kinematic_chains_joint_state_map[kin_chain_name] = tmp;
        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data"<<RTT::endlog();
    }

    return true;
}

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(ros_orocos_joints_gui_server)
