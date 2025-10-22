#ifndef tree_hhcm_ROBOT_COLLISIONCHECK_H
#define tree_hhcm_ROBOT_COLLISIONCHECK_H

#include <tree_hhcm/common/common.h>
#include <matlogger2/matlogger2.h>
#include <xbot2_interface/robotinterface2.h>
#include <xbot2_interface/collision.h>
#if __has_include(<cartesian_interface/ros/utils/RobotMarkerPublisher.h>)
    #include <cartesian_interface/ros/utils/RobotMarkerPublisher.h>
    #define CARTESIO_HAS_ROS_MARKER_PUBLISHER
#endif

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class RobotCollisionCheck : public BT::StatefulActionNode
{

public:

    RobotCollisionCheck(std::string name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override {}

    static BT::PortsList providedPorts();

private:

    rclcpp::Node::SharedPtr _node;
    XBot::ModelInterface::Ptr _model;
    XBot::Collision::CollisionModel::UniquePtr _cm;
    XBot::Collision::CollisionModel::LinkPairVector _coll_pairs;
    Eigen::VectorXd _q, _v;
    Printer _p;

#ifdef CARTESIO_HAS_ROS_MARKER_PUBLISHER
    std::unique_ptr<XBot::Cartesian::Utils::RobotMarkerPublisher> _rmpub;
#endif
};

}

#endif // tree_hhcm_ROBOT_MOVE_H