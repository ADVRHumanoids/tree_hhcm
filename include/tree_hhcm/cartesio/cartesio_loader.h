#ifndef _tree_hhcm_CARTESIOLOADER_H
#define _tree_hhcm_CARTESIOLOADER_H

#include <tree_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// ros
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/ros/RosServerClass.h>


namespace tree {
class CartesioLoader : public BT::SyncActionNode
{
public:

    CartesioLoader(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:

    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::Cartesian::RosServerClass::Ptr _ros_server;
    Printer _p;

};
}


#endif // CARTESIOLOADER_H
