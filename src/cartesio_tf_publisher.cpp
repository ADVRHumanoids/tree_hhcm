#include <tree_hhcm/cartesio/cartesio_tf_publisher.h>

using namespace tree;

tree::CartesioTfPublisher::CartesioTfPublisher(std::string name, const BT::NodeConfiguration &config):
    BT::StatefulActionNode(name, config),
    _p(*this)
{
}

BT::NodeStatus tree::CartesioTfPublisher::onStart()
{
    if(!getInput("model", _model))
    {
        throw BT::RuntimeError("missing required input [model]");
    }

    if(!getInput("node", _node))
    {
        throw BT::RuntimeError("missing required input [node]");
    }

    getInput("tf_prefix", _tf_prefix);

    _rspub = std::make_unique<XBot::Cartesian::Utils::RobotStatePublisher>(_node, _model);

    bool publish_collisions = false;
    getInput("publish_collisions", publish_collisions);
    if(publish_collisions)
    {
#ifdef CARTESIO_HAS_ROS_MARKER_PUBLISHER
        _cm = std::make_unique<XBot::Collision::CollisionModel>(_model);
        _coll_pairs = _cm->getCollisionPairs(true);
        _rmpub = std::make_unique<XBot::Cartesian::Utils::RobotMarkerPublisher>(_model, "~/collision_markers", _node, Eigen::Vector4d(0.0,0,0,0.0));
#else 
        throw BT::RuntimeError("CartesioTfPublisher: collision marker publisher not available, please update cartesian_interface");
#endif
    }       

    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus tree::CartesioTfPublisher::onRunning()
{
    _rspub->publishTransforms(_node->now(), _tf_prefix);

#ifdef CARTESIO_HAS_ROS_MARKER_PUBLISHER 
    if(_rmpub && _cm)
    {
        _cm->update();
        std::vector<int> coll_ids;
        _cm->checkCollision(coll_ids, true, 0.0);
        
        std::map<std::string, Eigen::Vector4d> color_map;

        for(auto id : coll_ids)
        {
            auto lp = _coll_pairs[id];
            color_map[lp.first] = Eigen::Vector4d(1,0,0,.8);
            color_map[lp.second] = Eigen::Vector4d(1,0,0,.8);
            _p.cout() << "collision between " << lp.first << " and " << lp.second << std::endl;
        }

        _rmpub->publishMarkers(_node->now(), "world", "ci/world", color_map);
        
    }
#endif

    return BT::NodeStatus::RUNNING;
}

BT::PortsList tree::CartesioTfPublisher::providedPorts()
{
    return {
        BT::InputPort<XBot::ModelInterface::Ptr>("model", "the robot model"),
        BT::InputPort<rclcpp::Node::SharedPtr>("node", "the ROS2 node"),
        BT::InputPort<std::string>("tf_prefix", "", "the tf prefix to be added to all published frames"),
        BT::InputPort<bool>("publish_collisions", false, "Whether to publish collision markers")
    };
}
