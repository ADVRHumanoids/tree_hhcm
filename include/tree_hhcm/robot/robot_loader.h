#ifndef _tree_hhcm_ROBOTLOADER_H
#define _tree_hhcm_ROBOTLOADER_H

#include <tree_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// robotinterface
#include <xbot2_interface/robotinterface2.h>


namespace tree {

class RobotLoader : public BT::SyncActionNode
{
public:

    RobotLoader(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:

    XBot::ConfigOptions parse_from_inputs();
    XBot::ConfigOptions parse_from_ros2();

    XBot::RobotInterface::Ptr _robot;
    Printer _p;

};

}


#endif // ROBOTLOADER_H
