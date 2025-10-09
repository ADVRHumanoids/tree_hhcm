#ifndef TREE_HHCM_COMMON_LOAD_PARAM_FROM_CONFIG_H
#define TREE_HHCM_COMMON_LOAD_PARAM_FROM_CONFIG_H

#include <tree_hhcm/common/common.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class LoadParamFromConfig : public BT::SyncActionNode
{
public:

    LoadParamFromConfig(std::string name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:

    Printer _p;
    static std::map<std::string, YAML::Node> _config_cache;

    void store_all_params(const YAML::Node& cfg, std::string prefix="");

    void set_param_to_blackboard(const std::string& output_name, const std::string& output_type, const YAML::Node& cfg);

};

}


#endif
