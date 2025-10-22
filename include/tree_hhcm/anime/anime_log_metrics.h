#ifndef tree_hhcm_ANIMELOGMETRICS_H
#define tree_hhcm_ANIMELOGMETRICS_H

#include <tree_hhcm/common/common.h>
#include <matlogger2/matlogger2.h>
#include <xbot2_interface/robotinterface2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Cartesian.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class AnimeLogMetrics : public BT::StatefulActionNode
{

public:

    AnimeLogMetrics(std::string name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override {}

    static BT::PortsList providedPorts();

private:

    XBot::ModelInterface::Ptr _model;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::MatLogger2::Ptr _logger;

    Printer _p;

#ifdef CARTESIO_HAS_ROS_MARKER_PUBLISHER
    std::unique_ptr<XBot::Cartesian::Utils::RobotMarkerPublisher> _rmpub;
#endif
};

}

#endif // tree_hhcm_ROBOT_MOVE_H