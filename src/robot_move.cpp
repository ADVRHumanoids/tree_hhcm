#include <tree_hhcm/robot/robot_move.h>
#include <yaml-cpp/yaml.h>

namespace tree {

RobotMove::RobotMove(std::string name, const BT::NodeConfiguration &config):
    BT::SyncActionNode(name, config), _p(*this)
{
    if(!getInput("robot", _robot))
    {
        throw BT::RuntimeError("RobotMove: missing required input [robot]");
    }

    bool enable_pos = getInput<bool>("enable_pos").value_or(true);
    
    if(enable_pos)
    {
        _p.cout() << "RobotMove: setting control mode to POSITION\n";
        _robot->setControlMode(XBot::ControlMode::Position());
    }

}

BT::NodeStatus RobotMove::tick()
{
    if(getInput("qref", _q))
    {
        _robot->setPositionReference(_q);
    }

    XBot::JointNameMap qref_map;
    if(getInput("qref_map", qref_map))
    {
        _robot->mapToQ(qref_map, _q);
        _robot->setPositionReference(_q);
    }

    // very complex logic...
    _robot->move();
    
    // robotmove is always running
    // use it inside a <Parallel> node with some other node that returns SUCCESS/FAILURE
    // to stop the execution
    return BT::NodeStatus::RUNNING;
}

BT::PortsList RobotMove::providedPorts()
{
    return {
        BT::InputPort<bool>("enable_pos"),
        BT::InputPort<XBot::RobotInterface::Ptr>("robot"),
        BT::InputPort<Eigen::VectorXd>("qref"),
        BT::InputPort<XBot::JointNameMap>("qref_map")
    };
}

} // namespace tree