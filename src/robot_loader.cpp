#include <tree_hhcm/robot/robot_loader.h>

#include <xbot2_interface/ros2/config_from_param.hpp>

tree::RobotLoader::RobotLoader(std::string name, const BT::NodeConfiguration &config):
    BT::SyncActionNode(name, config), _p(*this)
{
    XBot::ConfigOptions cfg;

    if(getInput<std::string>("urdf").has_value())
    {
        cfg = parse_from_inputs();
    }
    else
    {
        cfg = parse_from_ros2();
    }
    
    
    _p.cout() << "Set output 'config' with type XBot::ConfigOptions" << std::endl;
    setOutput<XBot::ConfigOptions>("config", cfg);

    if(getInput<bool>("load_robot").value_or(true))
    {
        _robot = XBot::RobotInterface::getRobot(cfg);

        if(!_robot)
        {
            throw BT::RuntimeError("RobotLoader: error creating RobotInterface");
        }

        _p.cout() << "Set output 'robot' with type XBot::RobotInterface::Ptr" << std::endl;

        setOutput<XBot::RobotInterface::Ptr>("robot", _robot);

        Eigen::VectorXd q_start = _robot->getPositionReferenceFeedback();
        setOutput<Eigen::VectorXd>("q_start", q_start);

        std::vector<std::string> disabled_joints;
        if(getInput("disabled_joints", disabled_joints))
        {
            XBot::CtrlModeMap ctrl;
            for(const auto& jname : disabled_joints)
            {
                if(!_robot->hasJoint(jname))
                {
                    throw BT::RuntimeError("RobotLoader: robot has no joint named " + jname);
                }
                ctrl[jname] = XBot::ControlMode::None();
                _p.cout() << "disabling joint " << jname << "\n";
            }
            _robot->setControlMode(ctrl);
        }
    }
    else 
    {
        _p.cout() << "Skipping robot loading" << std::endl;
    }

}

XBot::ConfigOptions tree::RobotLoader::parse_from_inputs()
{
    // load from given urdf and srdf, if provided
    XBot::ConfigOptions cfg;
    std::string urdf_cmd;
    std::string srdf_cmd;
    
    if(getInput("urdf", urdf_cmd))
    {
        std::string urdf = Globals::instance().check_output(urdf_cmd);
        cfg.set_urdf(urdf);
    }
    
    if(getInput("srdf", srdf_cmd))
    {
        std::string srdf = Globals::instance().check_output(srdf_cmd);
        cfg.set_srdf(srdf);
    }

    return cfg;
}

XBot::ConfigOptions tree::RobotLoader::parse_from_ros2()
{
    // load from ros2 otherwise
    std::string prefix;
    getInput("prefix", prefix);

    rclcpp::Node::SharedPtr node;
    if(!getInput("node", node))
    {
        throw BT::RuntimeError("NodeProvider: missing required input [node]");
    }

    if(!node)
    {
        throw BT::RuntimeError("NodeProvider: input [node] is null");
    }

    _p.cout() << "Loading parameters from prefix: " << prefix << std::endl;

    return XBot::ConfigOptionsFromParams(node, prefix);
}

BT::NodeStatus tree::RobotLoader::tick()
{
    return BT::NodeStatus::SUCCESS;
}


BT::PortsList tree::RobotLoader::providedPorts()
{
    return {
        BT::InputPort<rclcpp::Node::SharedPtr>("node"),
        BT::InputPort<std::string>("prefix"),
        BT::InputPort<std::string>("urdf"),
        BT::InputPort<std::string>("srdf"),
        BT::InputPort<bool>("load_robot"),
        BT::InputPort<std::vector<std::string>>("disabled_joints"),
        BT::OutputPort<XBot::ConfigOptions>("config"),
        BT::OutputPort<XBot::RobotInterface::Ptr>("robot"),
        BT::OutputPort<Eigen::VectorXd>("q_start")
    };
}

