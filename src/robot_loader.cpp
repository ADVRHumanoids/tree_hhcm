#include <bt_hhcm/robot/robot_loader.h>

#include <xbot2_interface/ros2/config_from_param.hpp>

tree::RobotLoader::RobotLoader(std::string name, const BT::NodeConfiguration &config):
    BT::SyncActionNode(name, config), _p(*this)
{
    rclcpp::Node::SharedPtr node;
    if(!getInput("node", node))
    {
        throw BT::RuntimeError("NodeProvider: missing required input [node]");
    }

    if(!node)
    {
        throw BT::RuntimeError("NodeProvider: input [node] is null");
    }

    std::string prefix;
    getInput("prefix", prefix);
    _p.cout() << "Loading parameters from prefix: " << prefix << std::endl;

    auto cfg = XBot::ConfigOptionsFromParams(node, prefix);
    _p.cout() << "Set output 'config' with type XBot::ConfigOptions" << std::endl;
    setOutput<XBot::ConfigOptions>("config", cfg);

    _robot = XBot::RobotInterface::getRobot(cfg);
    if(!_robot)
    {
        throw BT::RuntimeError("RobotLoader: error creating RobotInterface");
    }

    _p.cout() << "Set output 'robot' with type XBot::RobotInterface::Ptr" << std::endl;
    setOutput<XBot::RobotInterface::Ptr>("robot", _robot);

}

BT::NodeStatus tree::RobotLoader::tick()
{
    return BT::NodeStatus::SUCCESS;
}


BT::PortsList tree::RobotLoader::providedPorts()
{
    return {
        BT::InputPort<rclcpp::Node::SharedPtr>("node"),
        BT::InputPort<std::string>("prefix"),
        BT::OutputPort<XBot::ConfigOptions>("config"),
        BT::OutputPort<XBot::RobotInterface::Ptr>("robot")
    };
}

