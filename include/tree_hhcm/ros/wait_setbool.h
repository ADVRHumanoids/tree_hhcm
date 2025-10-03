#ifndef WAIT_SETBOOL_H
#define WAIT_SETBOOL_H

#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tree_hhcm/common/common.h>

namespace tree
{
class WaitSetBool : public BT::StatefulActionNode
{
public:
    WaitSetBool(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _srv;
    std::optional<bool> _value;
    Printer _p;
    double _time, _print_time = 0.0;
};
}

#endif // WAIT_SETBOOL_H