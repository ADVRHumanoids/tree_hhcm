#include <bt_hhcm/cartesio/cartesio_loader.h>


tree::CartesioLoader::CartesioLoader(std::string name, const BT::NodeConfiguration &config):
    BT::SyncActionNode(name, config),
    _p(*this)
{
    // get robot model config
    XBot::ConfigOptions model_cfg;
    if(!getInput("model_config", model_cfg))
    {
        throw BT::RuntimeError("CartesioLoader: missing required input [model_config]");
    }

    // load robot model
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(model_cfg);

    // configure cartesio
    std::string cartesio_log_path = "/tmp";
    getInput("cartesio_log_path", cartesio_log_path);

    auto params = std::make_shared<XBot::Cartesian::Parameters>(Globals::instance().tree_dt,
                                                                cartesio_log_path);

    auto ctx = std::make_shared<XBot::Cartesian::Context>(params, model);

    std::string ikpb_file = Globals::instance().tree_dirname + getInput<std::string>("cartesio_config_path").value();

    _p.cout() << "loading ik problem from file " << ikpb_file << "\n";

    auto ikpb_yaml = YAML::LoadFile(ikpb_file);

    XBot::Cartesian::ProblemDescription ikpb(ikpb_yaml, ctx);

    _ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot", ikpb, ctx);

    if(_ci)
    {
        _p.cout() << "CartesIO configuration succeeded \n";
    }

    setOutput<XBot::Cartesian::CartesianInterfaceImpl::Ptr>("cartesio", _ci);
}

BT::NodeStatus tree::CartesioLoader::tick()
{
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList tree::CartesioLoader::providedPorts()
{
    return {
        BT::InputPort<XBot::ConfigOptions>("model_config"),
        BT::InputPort<std::string>("cartesio_config_path"),
        BT::InputPort<std::string>("cartesio_log_path"),
        BT::OutputPort<XBot::Cartesian::CartesianInterfaceImpl::Ptr>("cartesio")
    };
}

