#ifndef _BT_HHCM_CARTESIOLOADER_H
#define _BT_HHCM_CARTESIOLOADER_H

#include <bt_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// ros
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Cartesian.h>


namespace tree {
class CartesioLoader : public BT::SyncActionNode
{
public:

    CartesioLoader(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:

    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    Printer _p;

};
}


#endif // CARTESIOLOADER_H
