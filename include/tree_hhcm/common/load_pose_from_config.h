#ifndef TREE_HHCM_COMMON_LOAD_POSE_FROM_CONFIG_H
#define TREE_HHCM_COMMON_LOAD_POSE_FROM_CONFIG_H

#include <tree_hhcm/common/common.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class LoadPoseFromConfig : public BT::SyncActionNode
{
public:

    LoadPoseFromConfig(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:

    Printer _p;
    static std::map<std::string, YAML::Node> _config_cache;

};

}


#endif
