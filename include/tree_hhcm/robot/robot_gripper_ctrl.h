#ifndef tree_hhcm_ROBOT_GRIPPER_CTRL_H
#define tree_hhcm_ROBOT_GRIPPER_CTRL_H

#include <tree_hhcm/common/common.h>
#include <xbot2_interface/robotinterface2.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class RobotGripperCtrl : public BT::StatefulActionNode
{

public:

    RobotGripperCtrl(std::string name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    static BT::PortsList providedPorts();

private:

    XBot::RobotInterface::Ptr _robot;
    XBot::Gripper::Ptr _gripper;
    Printer _p;
    std::optional<bool> _closure_success;
};

}

#endif // tree_hhcm_ROBOT_MOVE_H