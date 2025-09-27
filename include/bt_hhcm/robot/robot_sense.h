#ifndef BT_HHCM_ROBOT_MOVE_H
#define BT_HHCM_ROBOT_MOVE_H

#include <bt_hhcm/common/common.h>
#include <xbot2_interface/robotinterface2.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class RobotSense : public BT::SyncActionNode
{

public:

    RobotSense(std::string name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:

    XBot::RobotInterface::Ptr _robot;
    Eigen::VectorXd _q, _v;
    Printer _p;
};

}

#endif // BT_HHCM_ROBOT_MOVE_H