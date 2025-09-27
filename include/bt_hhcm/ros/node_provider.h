#ifndef _BT_HHCM_NODEPROVIDER_H
#define _BT_HHCM_NODEPROVIDER_H

#include <bt_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// ros
#include <rclcpp/rclcpp.hpp>


namespace tree {

class NodeProvider : public BT::SyncActionNode
{
public:

    NodeProvider(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:

    rclcpp::Node::SharedPtr _node;
    Printer _p;

};

}


#endif // ROBOTLOADER_H
