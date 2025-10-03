#include <tree_hhcm/robot/robot_gripper_ctrl.h>
#include <yaml-cpp/yaml.h>

namespace tree {

RobotGripperCtrl::RobotGripperCtrl(std::string name, const BT::NodeConfiguration &config):
    BT::StatefulActionNode(name, config), _p(*this)
{
    
}

BT::NodeStatus RobotGripperCtrl::onStart()
{
    // get robot and gripper handle
    if(!getInput("robot", _robot))
    {
        throw BT::RuntimeError("RobotGripperCtrl: missing required input [robot]");
    }

    std::string name;
    if(!getInput("gripper_name", name))
    {
        throw BT::RuntimeError("RobotGripperCtrl: missing required input [gripper_name]");
    }

    _gripper = _robot->getGripper(name);

    if(!_gripper)
    {
        _p.cerr() << "gripper not found \n";
        return BT::NodeStatus::FAILURE;
    }

    // wait for valid gripper data
    auto ts = _gripper->getTimestamp();

    while(_gripper->getTimestamp() == ts)
    {
        _p.cout() << "waiting for valid gripper data..\n";
        _robot->sense();
        usleep(1e5);
    }

    
    // define grasping callback that will be called on action completion
    // (or timeout)
    _closure_success.reset();

    auto cb = [this](bool timeout)
    {
        if(timeout)
        {
            _p.cout() << "gripper action timed out (closure = " << _gripper->getClosure() << ")\n";
        }
        else
        {
            _p.cout() << "gripper action completed (closure = " << _gripper->getClosure() << ")\n";
        }

        _closure_success = !timeout;
    };

    // set commands
    double timeout_sec = 5.0;
    getInput("timeout", timeout_sec);

    double closure_ref;
    if(getInput("closure_ref", closure_ref))
    {
        _p.cout() << "setting closure reference to " << closure_ref << "\n";
        _gripper->setClosureReference(closure_ref, cb, timeout_sec);
    }

    double effort_ref;
    if(getInput("effort_ref", effort_ref))
    {
        _p.cout() << "setting effort reference to " << effort_ref << "\n";
        _gripper->setEffortReference(effort_ref, cb, timeout_sec);
    }
    
    _robot->move();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RobotGripperCtrl::onRunning()
{
    // run until action completes (callback sets _closure_success)

    _robot->sense();  // callback is called from here
    
    if(_closure_success.has_value())
    {
        return *_closure_success ? BT::NodeStatus::SUCCESS :
                                   BT::NodeStatus::FAILURE;
    }
    
    _robot->move();
    
    return BT::NodeStatus::RUNNING;
}

void RobotGripperCtrl::onHalted()
{
}

BT::PortsList RobotGripperCtrl::providedPorts()
{
    return {
        BT::InputPort<XBot::RobotInterface::Ptr>("robot"),
        BT::InputPort<std::string>("gripper_name"),
        BT::InputPort<double>("timeout"),
        BT::InputPort<double>("closure_ref"),
        BT::InputPort<double>("effort_ref"),
    };
}

} // namespace tree