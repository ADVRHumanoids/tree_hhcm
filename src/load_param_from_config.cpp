#include <tree_hhcm/common/load_param_from_config.h>
#include <tree_hhcm/common/parsers.h>

using namespace tree;

std::map<std::string, YAML::Node> tree::LoadParamFromConfig::_config_cache;

LoadParamFromConfig::LoadParamFromConfig(std::string name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
    , _p("LoadParamFromConfig")
{
}


BT::NodeStatus tree::LoadParamFromConfig::tick()
{
    // load config file
    std::string cfg_path;
    if(!getInput("config", cfg_path))
    {
        throw BT::RuntimeError("LoadPoseFromConfig: missing required input [config]");
    }

    // check if config is already loaded
    YAML::Node cfg;
    try 
    {
        cfg = YAML::Clone(_config_cache.at(cfg_path));
    }
    catch(const std::out_of_range& e)
    {
        // not found, load it
        
        // use shell to expand path
        cfg_path = Globals::instance().parse_shell(cfg_path);

        // load config as yaml
        cfg = YAML::LoadFile(cfg_path);
        
        // cache it
        _config_cache[cfg_path] = YAML::Clone(cfg);
    }

    // get input key
    std::string input_key;
    if(!getInput("input_key", input_key))
    {
        throw BT::RuntimeError("LoadParamFromConfig: missing required input [input_key]");
    }

    // parse input key
    auto input_tokens = BT::splitString(input_key, '.');

    for (auto tok : input_tokens)
    {
        cfg = cfg[tok];
    }

    // convert to required type
    std::string output_type;
    getInput("output_type", output_type);

    // get output name
    std::string output_name;
    getInput("output_name", output_name);

    // get input index (for array types)
    int input_index = -1;
    getInput("input_index", input_index);
    if(input_index >= 0)
    {
        if(!cfg.IsSequence())
        {
            throw BT::RuntimeError("LoadParamFromConfig: input_index provided but input_path is not an array");
        }
        if(input_index >= static_cast<int>(cfg.size()))
        {
            throw BT::RuntimeError("LoadParamFromConfig: input_index is out of range");
        }
        cfg = cfg[input_index];
    }

    // get load_all flag
    bool load_all = false;
    getInput("load_all", load_all);
    if(load_all)
    {
        if(!cfg.IsMap())
        {
            throw BT::RuntimeError("LoadParamFromConfig: load_all is true but input_path is not a map");
        }

        // load all entries in the map as separate blackboard entries
        store_all_params(cfg, output_name);
    }
    else
    {
        if(output_name.empty())
        {
            throw BT::RuntimeError("LoadParamFromConfig: output_name is empty");
        }

        if(output_type.empty())
        {
            throw BT::RuntimeError("LoadParamFromConfig: output_type is empty");
        }
        
        set_param_to_blackboard(output_name, output_type, cfg);
    }

   
    

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList tree::LoadParamFromConfig::providedPorts()
{
    return {
        BT::InputPort<std::string>("config", "YAML file with parameters"),
        BT::InputPort<std::string>("input_key", "Namespace inside the YAML file"),
        BT::InputPort<bool>("load_all", false, "If true, load all parameters under input_key as separate blackboard entries; output_name is treated as prefix"),
        BT::InputPort<int>("input_index", -1, "Index of the parameter to load (for array types)"),
        BT::InputPort<std::string>("output_type", "Type of parameter to load from YAML"),
        BT::InputPort<std::string>("output_name", "Blackboard key to store the loaded parameter"),
    };
}

void LoadParamFromConfig::set_param_to_blackboard(const std::string &output_name, const std::string &output_type, const YAML::Node &cfg) 
{
    _p.cout() << "Setting param " << output_name << " of type " << output_type << " value:\n"
              << cfg << "\n";   

    // store to blackboard
    if(output_type == "string")
    {
        config().blackboard->set(output_name, cfg.as<std::string>());
    }
    else if(output_type == "int")
    {
        config().blackboard->set(output_name, cfg.as<int>());
    }
    else if(output_type == "double")
    {
        config().blackboard->set(output_name, cfg.as<double>());
    }
    else if(output_type == "bool")
    {
        config().blackboard->set(output_name, cfg.as<bool>());
    }
    else if(output_type == "Eigen::Affine3d")
    {
        config().blackboard->set(output_name, cfg.as<Eigen::Affine3d>());
    }
    else if(output_type == "vector<string>")
    {
        config().blackboard->set(output_name, cfg.as<std::vector<std::string>>());
    }
    else if(output_type == "vector<int>")
    {
        config().blackboard->set(output_name, cfg.as<std::vector<int>>());
    }
    else if(output_type == "vector<double>")
    {
        config().blackboard->set(output_name, cfg.as<std::vector<double>>());
    }
    else if(output_type == "vector<bool>")
    {
        config().blackboard->set(output_name, cfg.as<std::vector<bool>>());
    }
    else if(output_type == "vector<Eigen::Affine3d>")
    {
        config().blackboard->set(output_name, cfg.as<std::vector<Eigen::Affine3d>>());
    }
    else if(output_type == "map<string,string>")
    {
        config().blackboard->set(output_name, cfg.as<std::map<std::string,std::string>>());
    }
    else if(output_type == "map<string,int>")
    {
        config().blackboard->set(output_name, cfg.as<std::map<std::string,int>>());
    }
    else if(output_type == "map<string,double>")
    {
        config().blackboard->set(output_name, cfg.as<std::map<std::string,double>>());
    }
    else if(output_type == "map<string,bool>")
    {
        config().blackboard->set(output_name, cfg.as<std::map<std::string,bool>>());
    }
    else
    {
        throw BT::RuntimeError("LoadParamFromConfig: unsupported input type [" + output_type + "]");
    }
}


void tree::LoadParamFromConfig::store_all_params(const YAML::Node &cfg, std::string prefix)
{
    for(auto item : cfg)
    {
        std::string key = item.first.as<std::string>();
        YAML::Node value = item.second;

        std::string prefix_dot = prefix.empty() ? "" : prefix + ".";

        if(value.IsMap())
        {
            store_all_params(value, prefix_dot + key);
            continue;
        }

        std::string value_type = detect_yaml_type(value);
        
        if(value_type.empty())
        {
            throw BT::RuntimeError("LoadParamFromConfig: cannot detect type of parameter " + key);
        }

        std::string full_key = prefix_dot + key;

        set_param_to_blackboard(full_key, value_type, value);
    }
}