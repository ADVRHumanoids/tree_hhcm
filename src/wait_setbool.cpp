#include <tree_hhcm/ros/wait_setbool.h>

tree::WaitSetBool::WaitSetBool(const std::string &name, const BT::NodeConfiguration &config):
    BT::StatefulActionNode(name, config), _p(*this)
{
    
}

BT::NodeStatus tree::WaitSetBool::onStart()
{
    _time = 0.0;
    _print_time = 0.0;

    std::string srv_name;
    if(!getInput("service_name", srv_name))
    {
        throw BT::RuntimeError("WaitSetBool: missing required input [service_name]");
    }

    if(!getInput("node", _node))
    {
        throw BT::RuntimeError("WaitSetBool: missing required input [node]");
    }

    _srv = _node->create_service<std_srvs::srv::SetBool>(srv_name,
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            response->success = true;
            _value = request->data;
            return true;
        });

    _value.reset();
    _p.cout() << "waiting for service call on [" << _srv->get_service_name() << "]\n";

    bool blocking = false;
    getInput("blocking", blocking);

    if(blocking)
    {
        auto node_status = BT::NodeStatus::RUNNING;
        
        while(node_status == BT::NodeStatus::RUNNING)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(Globals::instance().tree_dt));
            node_status = onRunning();
        }

        return node_status;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus tree::WaitSetBool::onRunning()
{
    rclcpp::spin_some(_node);

    if(_time >= _print_time)
    {
        _p.cout() << "waiting for service call on [" << _srv->get_service_name() << "]\n";
        _print_time += 1.0;
    }

    _time += Globals::instance().tree_dt;

    if(_value.has_value())
    {
        _p.cout() << "received value: " << *_value << "\n";
        return *_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void tree::WaitSetBool::onHalted()
{
}

BT::PortsList tree::WaitSetBool::providedPorts()
{
    return {
        BT::InputPort<std::string>("service_name"),
        BT::InputPort<rclcpp::Node::SharedPtr>("node"),
    };
}
