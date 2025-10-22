#include <tree_hhcm/robot/robot_collision_check.h>

using namespace tree;

tree::RobotCollisionCheck::RobotCollisionCheck(std::string name, const BT::NodeConfiguration &config):
    BT::StatefulActionNode(name, config),
    _p(*this)
{
}

BT::NodeStatus RobotCollisionCheck::onStart()
{

    if(!getInput("model", _model))
    {
        throw BT::RuntimeError("missing required input [model]");
    }

    XBot::Collision::CollisionModel::Options cm_options;
    cm_options.assume_convex_meshes = false;

    _cm = std::make_unique<XBot::Collision::CollisionModel>(_model, cm_options);
    _coll_pairs = _cm->getCollisionPairs(true);

    bool publish_collisions = false;
    getInput("publish_collisions", publish_collisions);
    if(publish_collisions)
    {
        if(!getInput("node", _node))
        {
            throw BT::RuntimeError("missing required input [node]");
        }
#ifdef CARTESIO_HAS_ROS_MARKER_PUBLISHER
        _rmpub = std::make_unique<XBot::Cartesian::Utils::RobotMarkerPublisher>(_model, "~/collision_markers", _node, Eigen::Vector4d(0.0,0,0,0.0));
#else 
        throw BT::RuntimeError("CartesioTfPublisher: collision marker publisher not available, please update cartesian_interface");
#endif
    }       

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RobotCollisionCheck::onRunning()
{
    _cm->update();
    std::vector<int> coll_ids;
    _cm->checkCollision(coll_ids, true, 0.0);

    setOutput("colliding_links", coll_ids);
    
    std::map<std::string, Eigen::Vector4d> color_map;
    
    for(auto id : coll_ids)
    {
        auto lp = _coll_pairs[id];
        color_map[lp.first] = Eigen::Vector4d(1,0,0,.8);
        color_map[lp.second] = Eigen::Vector4d(1,0,0,.8);
        _p.cout() << "collision between " << lp.first << " and " << lp.second << std::endl;
    }
    
#ifdef CARTESIO_HAS_ROS_MARKER_PUBLISHER 
    if(_rmpub)
    {
        _rmpub->publishMarkers(_node->now(), "world", "ci/world", color_map);
    }
#endif

    return BT::NodeStatus::RUNNING;
}

BT::PortsList RobotCollisionCheck::providedPorts()
{
    return {
        BT::InputPort<XBot::ModelInterface::Ptr>("model", "the robot model"),
        BT::InputPort<rclcpp::Node::SharedPtr>("node", "the ROS2 node"),
        BT::InputPort<bool>("publish_collisions", false, "Whether to publish collision markers"),
        BT::OutputPort<std::vector<int>>("colliding_links", "List of colliding links")
    };
}