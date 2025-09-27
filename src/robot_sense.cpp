#include <bt_hhcm/robot/robot_sense.h>
#include <yaml-cpp/yaml.h>

namespace tree {

RobotSense::RobotSense(std::string name, const BT::NodeConfiguration &config):
    BT::SyncActionNode(name, config), _p(*this)
{
    if(!getInput("robot", _robot))
    {
        throw BT::RuntimeError("RobotSense: missing required input [robot]");
    }
}

BT::NodeStatus RobotSense::tick()
{
    // read state and return success
    
    if(!_robot->sense())
    {
        _p.cerr() << "RobotSense: failed to read state\n";
        return BT::NodeStatus::RUNNING;
    }

    setOutput<Eigen::VectorXd>("qref", _robot->getPositionReferenceFeedback());
    setOutput<Eigen::VectorXd>("q", _robot->getJointPosition());
    setOutput<Eigen::VectorXd>("tau", _robot->getJointEffort());

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList RobotSense::providedPorts()
{
    return {
        BT::InputPort<XBot::RobotInterface::Ptr>("robot"),
        BT::OutputPort<Eigen::VectorXd>("qref"),
        BT::OutputPort<Eigen::VectorXd>("q"),
        BT::OutputPort<Eigen::VectorXd>("tau")
    };
}

} // namespace tree