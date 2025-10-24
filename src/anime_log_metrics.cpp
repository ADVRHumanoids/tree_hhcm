#include <tree_hhcm/anime/anime_log_metrics.h>
#include <fstream>

using namespace tree;

tree::AnimeLogMetrics::AnimeLogMetrics(std::string name, const BT::NodeConfiguration &config):
    BT::StatefulActionNode(name, config),
    _p(*this)
{
}

BT::NodeStatus tree::AnimeLogMetrics::onStart()
{
    std::string log_file_path;
    if(!getInput("log_file_path", log_file_path))
    {
        throw BT::RuntimeError("AnimeLogMetrics: missing required input [log_file_path]");
    }

    XBot::MatLogger2::Options logger_options;
    logger_options.default_buffer_size = 1e6;
    _logger = XBot::MatLogger2::MakeLogger(log_file_path, logger_options);

    // get model
    if(!getInput("model", _model))
    {
        throw BT::RuntimeError("AnimeLogMetrics: missing required input [model]");
    }

    // get cartesian interface
    if(!getInput("cartesian_interface", _ci))
    {
        throw BT::RuntimeError("AnimeLogMetrics: missing required input [cartesian_interface]");
    }

    // save urdf and srdf to file
    auto urdf = _model->getUrdfString();
    auto srdf = _model->getSrdfString();
    _logger->save("robot_urdf", urdf);
    _logger->save("robot_srdf", srdf);

    std::ofstream urdf_file(log_file_path + "_robot.urdf");
    urdf_file << urdf;
    urdf_file.close();

    std::ofstream srdf_file(log_file_path + "_robot.srdf");
    srdf_file << srdf;
    srdf_file.close();

    // reset time
    _time = 0.0;

    return BT::NodeStatus::RUNNING;
}


BT::NodeStatus tree::AnimeLogMetrics::onRunning()
{
    // log time 
    _logger->add("time", _time);
    _time += Globals::instance().tree_dt;

    // log model
    auto [qmin, qmax] = _model->getJointLimits();
    auto vmax = _model->getVelocityLimits();
    auto taumax = _model->getEffortLimits();
    auto q =  _model->getJointPosition();
    auto v = _model->getJointVelocity();
    auto tau = _model->getJointEffort();
    Eigen::VectorXd q_normalized = (q - qmin).cwiseQuotient(qmax - qmin);
    Eigen::VectorXd v_normalized = v.cwiseQuotient(vmax);
    Eigen::VectorXd tau_normalized = tau.cwiseQuotient(taumax);

    auto J = _model->getJacobian("gripper_A");
    double manipulability = std::sqrt((J * J.transpose()).determinant());

    _logger->add("q", q_normalized);
    _logger->add("v", v_normalized);
    _logger->add("tau", tau_normalized);
    _logger->add("manipulability", manipulability);

    // log cartesian interface metrics
    auto task_names = _ci->getTaskList();
    for(auto tname : task_names)
    {
        auto task = _ci->getTask(tname);
        
        Eigen::VectorXd e;
        task->getTaskError(e);
        if(e.size() > 0)
        {
            _logger->add("error_" + tname, e);
        }

        _logger->add("active_" + tname, 1-static_cast<int>(task->getActivationState()));
    }

    // log collision metrics
    std::vector<int> colliding_links;
    getInput("colliding_links", colliding_links);
    _logger->add("num_collisions", colliding_links.size());

    return BT::NodeStatus::RUNNING;
}
BT::PortsList tree::AnimeLogMetrics::providedPorts()
{
    return {
        BT::InputPort<std::string>("log_file_path", "Path to the MAT log file"),
        BT::InputPort<XBot::ModelInterface::Ptr>("model", "XBot2 ModelInterface pointer"),
        BT::InputPort<XBot::Cartesian::CartesianInterfaceImpl::Ptr>("cartesian_interface", "XBot2 CartesianInterfaceImpl pointer"),
        BT::InputPort<std::vector<int>>("colliding_links", std::vector<int>{}, "List of link indices involved in collisions to log")
    };
}