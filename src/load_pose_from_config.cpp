#include <tree_hhcm/common/load_pose_from_config.h>
#include <xbot2_interface/xbotinterface2.h>

std::map<std::string, YAML::Node> tree::LoadPoseFromConfig::_config_cache;

using namespace tree;

LoadPoseFromConfig::LoadPoseFromConfig(std::string name, const BT::NodeConfiguration &config) : 
    BT::SyncActionNode(name, config), _p(*this)
{
}

BT::NodeStatus LoadPoseFromConfig::tick()
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
        cfg = _config_cache.at(cfg_path);
    }
    catch(const std::out_of_range& e)
    {
        // not found, load it
        
        // use shell to expand path
        cfg_path = Globals::instance().parse_shell(cfg_path);

        // load config as yaml
        cfg = YAML::LoadFile(cfg_path);
        
        // cache it
        _config_cache[cfg_path] = cfg;
    }


    // get frame expr
    std::string frame_expr;
    if (!getInput("frame", frame_expr))
    {
        throw BT::RuntimeError("LoadPoseFromConfig: missing required input [frame]");
    }

    // get index
    int idx = getInput<int>("idx").value_or(0);

    // parsing logic
    auto frame_tokens = BT::splitString(frame_expr, '.');

    for (auto tok : frame_tokens)
    {
        cfg = cfg[tok];
    }

    // get parent
    std::string parent = cfg["parent"].as<std::string>("");
    _p.cout() << "LoadPoseFromConfig: loading pose for frame " << frame_expr << " with parent " << parent << "\n";
    Eigen::Affine3d w_T_parent = Eigen::Affine3d::Identity();

    // if parent not world, get its pose from model
    if (parent != "")
    {
        XBot::ModelInterface::Ptr model;
        if (!getInput("model", model))
        {
            throw BT::RuntimeError("LoadPoseFromConfig: missing required input [model]");
        }

        w_T_parent = model->getPose(parent);
    }

    // resulting config must be either a sequence or a map (single pose)
    cfg = cfg["offset"];
    int num_poses = 1;

    if (cfg.IsSequence())
    {
        if (idx < 0 || idx >= cfg.size())
        {
            throw BT::RuntimeError("LoadPoseFromConfig: index out of range");
        }
        num_poses = cfg.size();
        cfg = cfg[idx];
    }
    else if (cfg.IsMap())
    {
        // single pose, ok
    }
    else
    {
        throw BT::RuntimeError("LoadPoseFromConfig: invalid frame expression");
    }


    // parse pose as affine3d
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d::Map(cfg["pos"].as<std::vector<double>>().data());
    pose.linear() = Eigen::Quaterniond(Eigen::Vector4d::Map(cfg["rot"].as<std::vector<double>>().data())).normalized().toRotationMatrix();

    // get offset
    Eigen::Affine3d offset = Eigen::Affine3d::Identity();
    getInput("offset", offset);

    // set output
    setOutput("pose", offset * w_T_parent * pose);

    setOutput("num_poses", num_poses);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList LoadPoseFromConfig::providedPorts()
{
    return {
        BT::InputPort<XBot::ModelInterface::Ptr>("model"),
        BT::InputPort<std::string>("config"),
        BT::InputPort<std::string>("frame"),
        BT::InputPort<Eigen::Affine3d>("offset"),
        BT::InputPort<int>("idx", 0, "index of the pose to load, starting from 0"),
        BT::OutputPort<Eigen::Affine3d>("pose"),
        BT::OutputPort<int>("num_poses")
    };
}
