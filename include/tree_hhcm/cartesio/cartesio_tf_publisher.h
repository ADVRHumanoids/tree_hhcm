#ifndef _tree_hhcm_CARTESIO_TFPUB_H
#define _tree_hhcm_CARTESIO_TFPUB_H

#include <tree_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// cartesio
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/ros/RosServerClass.h>

#if __has_include(<cartesian_interface/ros/utils/RobotMarkerPublisher.h>)
    #include <cartesian_interface/ros/utils/RobotMarkerPublisher.h>
    #include <xbot2_interface/collision.h>
    #define CARTESIO_HAS_ROS_MARKER_PUBLISHER
#endif

namespace tree {

class CartesioTfPublisher : public BT::StatefulActionNode
{
public:

    CartesioTfPublisher(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override {}

    static BT::PortsList providedPorts();

private:

    rclcpp::Node::SharedPtr _node;
    std::string _tf_prefix;
    std::unique_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;
    XBot::ModelInterface::Ptr _model;
    Printer _p;

#ifdef CARTESIO_HAS_ROS_MARKER_PUBLISHER
    XBot::Collision::CollisionModel::UniquePtr _cm;
    std::unique_ptr<XBot::Cartesian::Utils::RobotMarkerPublisher> _rmpub;
    XBot::Collision::CollisionModel::LinkPairVector _coll_pairs;
#endif

};
}


#endif // CARTESIOLOADER_H
