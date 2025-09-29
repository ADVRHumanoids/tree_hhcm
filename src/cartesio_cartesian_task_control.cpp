#include <tree_hhcm/cartesio/cartesio_cartesian_task_control.h>
#include <tree_hhcm/common/config_value.h>

tree::CartesioTaskControl::CartesioTaskControl(std::string name, const BT::NodeConfiguration &config) : BT::StatefulActionNode(name, config),
                                                                                                        _p(*this)
{
}

BT::NodeStatus tree::CartesioTaskControl::onStart()
{
    _ci = getInput<XBot::Cartesian::CartesianInterfaceImpl::Ptr>("cartesio").value();

    if (_ci)
    {
        _p.cout() << "got ci \n";
    }
    else
    {
        _p.cerr() << "got nullptr ci \n";
        return BT::NodeStatus::FAILURE;
    }

    // configure cartesio task
    std::string task_name;
    getInput("task_name", task_name);

    if (task_name.empty())
    {
        _p.cerr() << "missing required input [task_name] \n";
        return BT::NodeStatus::FAILURE;
    }

    auto task = _ci->getTask(task_name);
    _task = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(task);

    _p.setHeader(name() + " (" + task->getName() + ")");

    if (!_task)
    {
        _p.cerr() << "task type invalid (" << typeid(*task).name() << ") \n";
        return BT::NodeStatus::FAILURE;
    }

    // set task active/inactive
    bool active = getInput<bool>("active").value_or(true);
    task->setActivationState(active ? XBot::Cartesian::ActivationState::Enabled : XBot::Cartesian::ActivationState::Disabled);

    if (!active)
    {
        _p.cout() << "task set inactive, returning SUCCESS \n";
        return BT::NodeStatus::SUCCESS;
    }

    // get velocity
    if (getInput<Eigen::Vector6d>("velocity").has_value())
    {
        _vref = getInput<Eigen::Vector6d>("velocity").value();

        if (getInput<bool>("local").value_or(false))
        {
            Eigen::Affine3d T0;
            _task->getCurrentPose(T0);

            _vref.head<3>() = T0.linear() * _vref.head<3>();
            _vref.tail<3>() = T0.linear() * _vref.tail<3>();
        }

        _p.cout() << "got velocity reference " << _vref.transpose() << ", setting velocity ctrl \n";
        _task->setControlMode(XBot::Cartesian::ControlType::Velocity);
        _velocity_ctrl = true;
    }

    // get position
    if (getInput<Eigen::Affine3d>("pose").has_value())
    {
        if (getInput<Eigen::Vector6d>("velocity").has_value())
        {
            throw std::invalid_argument("cannot set both velocity and pose target");
        }

        getInput("goal_velocity_threshold", _goal_velocity_threshold);

        _Tref = getInput<Eigen::Affine3d>("pose").value();

        _p.cout() << "got pose reference:\n"
                  << _Tref.matrix() << "\n";
        _velocity_ctrl = false;

        ConfigValue<double> trj_time_cfg;
        if (!getInput("trj_time", trj_time_cfg))
        {
            throw std::invalid_argument("missing required input [trj_time]");
        }
        _trj_time = trj_time_cfg.value();

        if (_trj_time <= 0)
        {
            throw std::invalid_argument("invalid trj time <= 0");
        }

        _task->setControlMode(XBot::Cartesian::ControlType::Position);

        _task->setPoseTarget(_Tref, _trj_time);
    }

    _time = 0;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus tree::CartesioTaskControl::onRunning()
{
    if (_task && _velocity_ctrl)
    {
        _task->setVelocityReference(_vref);
    }

    if (_task && !_velocity_ctrl && _task->getTaskState() == XBot::Cartesian::State::Online)
    {
        if ((_ci->getModel()->getJointVelocity().array() < _goal_velocity_threshold).all())
        {
            _p.cout() << "task reached target with zero velocity \n";
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    if (_task && _velocity_ctrl && _trj_time > 0 && _time >= _trj_time)
    {
        _p.cout() << "trajectory finished \n";
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void tree::CartesioTaskControl::onHalted()
{
}

BT::PortsList tree::CartesioTaskControl::providedPorts()
{
    return {
        BT::InputPort<XBot::Cartesian::CartesianInterfaceImpl::Ptr>("cartesio"),
        BT::InputPort<std::string>("task_name"),
        BT::InputPort<bool>("local"),
        BT::InputPort<bool>("active", true, "set task active/inactive"),
        BT::InputPort<Eigen::Affine3d>("pose"),
        BT::InputPort<ConfigValue<double>>("trj_time"),
        BT::InputPort<Eigen::Vector6d>("velocity"),
        BT::InputPort<double>("goal_velocity_threshold"),
    };
}
