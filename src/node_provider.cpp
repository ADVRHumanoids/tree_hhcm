#include <tree_hhcm/ros/node_provider.h>

namespace tree {

NodeProvider::NodeProvider(std::string name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config), _p(*this)
{
    
}

BT::NodeStatus tree::NodeProvider::tick()
{
    std::string node_name = "tree_hhcm_node";
    if(!getInput("node_name", node_name))
    {
        throw BT::RuntimeError("NodeProvider: missing required input [node_name]");
    }

    _node = rclcpp::Node::make_shared(node_name);

    _p.cout() << "Set output 'node' with type rclcpp::Node::SharedPtr" << std::endl;
    setOutput<rclcpp::Node::SharedPtr>("node", _node);


    return BT::NodeStatus::SUCCESS;
}


BT::PortsList tree::NodeProvider::providedPorts()
{
    return {
        BT::InputPort<std::string>("node_name"),
        BT::OutputPort<rclcpp::Node::SharedPtr>("node")
    };
}

}
