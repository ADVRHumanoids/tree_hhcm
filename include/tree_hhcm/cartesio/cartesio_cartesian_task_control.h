#ifndef _BTHHCM_CARTESIOTASKCONTROL_H
#define _BTHHCM_CARTESIOTASKCONTROL_H

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// cartesio
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Cartesian.h>

// utils
#include <tree_hhcm/common/common.h>

namespace tree {

class CartesioTaskControl : public BT::StatefulActionNode {

public:

    typedef std::function<void(Eigen::Vector6d)> GoalCallback;

    CartesioTaskControl(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:

    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::Cartesian::CartesianTask::Ptr _task;
    Eigen::Vector6d _vref;
    Eigen::Affine3d _Tref;
    bool _velocity_ctrl = false;
    Printer _p;
    double _time = 0, _trj_time = 0;
    double _goal_velocity_threshold = 0.01; // [rad/s]
    GoalCallback _goal_cb;

};
}



#endif // CARTESIOTASKCONTROL_H
