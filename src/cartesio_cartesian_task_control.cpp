#include <bt_hhcm/cartesio/cartesio_cartesian_task_control.h>

tree::CartesioTaskControl::CartesioTaskControl(std::string name, const BT::NodeConfiguration &config):
    BT::StatefulActionNode(name, config),
    _p(*this)
{

}

BT::NodeStatus tree::CartesioTaskControl::onStart()
{
    _ci = getInput<XBot::Cartesian::CartesianInterfaceImpl::Ptr>("cartesio_ptr").value();

    if(_ci)
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

    if(task_name.empty())
    {
        _p.cerr() << "missing required input [task_name] \n";
        return BT::NodeStatus::FAILURE;
    }


    auto task = _ci->getTask(task_name);
    _task = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(task);

    _p.setHeader(name() + " (" + task->getName() + ")");

    if(!_task)
    {
        _p.cerr() << "task type invalid (" << typeid(*task).name() << ") \n";
        return BT::NodeStatus::FAILURE;
    }

    // get velocity
    if(getInput<std::string>("velocity").has_value())
    {
        auto vref_vec = YAML::Load(getInput<std::string>("velocity").value()).as<std::vector<double>>();

        if(vref_vec.size() != 6)
        {
            throw std::invalid_argument("invalid velocity size != 6");
        }

        _vref = Eigen::Vector6d::Map(vref_vec.data());

        if(getInput<bool>("local").value_or(false))
        {
            Eigen::Affine3d T0;
            _task->getCurrentPose(T0);

            _vref.head<3>() = T0.linear()*_vref.head<3>();
            _vref.tail<3>() = T0.linear()*_vref.tail<3>();
        }

        _p.cout() << "got velocity reference " << _vref.transpose() << ", setting velocity ctrl \n";
        _task->setControlMode(XBot::Cartesian::ControlType::Velocity);
        _velocity_ctrl = true;
    }

    // get position
    if(getInput<std::string>("pose").has_value())
    {
        if(getInput<std::string>("velocity").has_value())
        {
            throw std::invalid_argument("cannot set both velocity and pose target");
        }

        auto pref_vec = YAML::Load(getInput<std::string>("pose").value()).as<std::vector<double>>();

        if(pref_vec.size() != 7)
        {
            throw std::invalid_argument("invalid pose size != 7");
        }

        Eigen::Affine3d T0;
        _task->getCurrentPose(T0);

        _Tref.translation() = T0.translation() + Eigen::Vector3d::Map(pref_vec.data());
        _Tref.linear() = T0.linear() * Eigen::Quaterniond(pref_vec.data() + 3).normalized().toRotationMatrix();
        _p.cout() << "got pose reference:\n" << _Tref.matrix() << "\n";
        _velocity_ctrl = false;

        if(_trj_time <= 0)
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
    if(_task && _velocity_ctrl)
    {
        _task->setVelocityReference(_vref);
    }

    if(_task && !_velocity_ctrl && _task->getTaskState() == XBot::Cartesian::State::Online)
    {
        _p.cout() << "pose trajectory finished \n";
        return BT::NodeStatus::SUCCESS;
    }

    if(_task && _velocity_ctrl && _trj_time > 0 && _time >= _trj_time)
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
        BT::InputPort<XBot::Cartesian::CartesianInterfaceImpl::Ptr>("cartesio_ptr"),
        BT::InputPort<std::string>("task_name"),
        BT::InputPort<bool>("local"),
        BT::InputPort<std::string>("pose"),
        BT::InputPort<double>("trj_time"),
        BT::InputPort<std::string>("velocity"),
    };
}
