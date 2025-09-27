#include <bt_hhcm/cartesio/cartesio_solve.h>

namespace tree {

CartesioSolve::CartesioSolve(std::string name, const BT::NodeConfiguration &config):
    BT::StatefulActionNode(name, config), _p(*this)
{
    if(!getInput("cartesio", _ci))
    {
        throw BT::RuntimeError("CartesioSolve: missing required input [cartesio]");
    }

    if(!_ci)
    {
        throw BT::RuntimeError("CartesioSolve: input [cartesio] is null");
    }

    _model = _ci->getModel();

    _dt = Globals::instance().tree_dt;
    _time = 0.0;

}

BT::NodeStatus CartesioSolve::onStart()
{
    _p.cout() << "CartesioSolve: starting\n";

    Eigen::VectorXd q0;
    getInput("q0", q0);

    if(q0.size() > 0)
    {
        _p.cout() << "CartesioSolve: setting initial position from input [q0] = " << q0.transpose().format(2) << "\n";
        _model->setJointPosition(q0);
        _model->update();
    }

    // reset ci to model state
    _time = 0.0;
    _ci->reset(_time);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CartesioSolve::onRunning()
{

    bool ok = _ci->update(_time, _dt);

    if(!ok)
    {
        _p.cerr() << "CartesioSolve: error in cartesio solve\n";
        return BT::NodeStatus::FAILURE;
    }

    _model->getJointPosition(_q);
    _model->getJointVelocity(_v);
    _model->sum(_q, _v * _dt, _qnext);
    _model->setJointPosition(_qnext);
    _model->update();

    return BT::NodeStatus::RUNNING;
}

BT::PortsList CartesioSolve::providedPorts()
{
    return {
        BT::InputPort<XBot::Cartesian::CartesianInterfaceImpl::Ptr>("cartesio")
    };
}

} // namespace tree