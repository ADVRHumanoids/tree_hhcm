#ifndef _BT_HHCM_CARTESIOSOLVE_H
#define _BT_HHCM_CARTESIOSOLVE_H

#include <bt_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// cartesio
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Cartesian.h>


namespace tree {
class CartesioSolve : public BT::StatefulActionNode
{
public:

    CartesioSolve(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    static BT::PortsList providedPorts();

private:

    double _time, _dt;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::ModelInterface::Ptr _model;
    Eigen::VectorXd _q, _v, _qnext;
    Printer _p;

};
}


#endif // CARTESIOLOADER_H
