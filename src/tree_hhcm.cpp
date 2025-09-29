#include <tree_hhcm/tree_hhcm.h>

#include <tree_hhcm/common/load_pose_from_config.h>

#include <tree_hhcm/ros/node_provider.h>

#include <tree_hhcm/robot/robot_loader.h>
#include <tree_hhcm/robot/robot_move.h>
#include <tree_hhcm/robot/robot_sense.h>

#include <tree_hhcm/cartesio/cartesio_cartesian_task_control.h>
#include <tree_hhcm/cartesio/cartesio_loader.h>
#include <tree_hhcm/cartesio/cartesio_solve.h>

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<tree::LoadPoseFromConfig>("LoadPoseFromConfig");
    factory.registerNodeType<tree::NodeProvider>("NodeProvider");
    factory.registerNodeType<tree::RobotLoader>("RobotLoader");
    factory.registerNodeType<tree::RobotSense>("RobotSense");
    factory.registerNodeType<tree::RobotMove>("RobotMove");
    factory.registerNodeType<tree::CartesioLoader>("CartesioLoader");
    factory.registerNodeType<tree::CartesioSolve>("CartesioSolve");
    factory.registerNodeType<tree::CartesioTaskControl>("CartesioTaskControl");
}

BT::Blackboard::Ptr tree::ConfigValueBase::root_tree_blackboard = nullptr;