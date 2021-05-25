#include "RvizController.h"

RvizController::RvizController()
    :   _robot_model_loader("robot_description")
    ,   _kinematic_model(_robot_model_loader.getModel())
    ,   _planning_scene(_kinematic_model)
    ,   _visual_tools("world")
    ,   _kinematic_state(_kinematic_model)
{
    _cmd_vel_sub = _nh.subscribe("cmd_vel", 1000, &RvizController::cmdVelCallback, this);
    _kinematic_state.setToDefaultValues();
    _end_effector_pose = _kinematic_state.getGlobalLinkTransform("end_link");
}

void RvizController::cmdVelCallback(const geometry_msgs::Twist &msg_vel)
{
    ROS_INFO("Velocity callback")
    _current_velocity.x = msg_vel.linear.x;
    _current_velocity.y = msg_vel.linear.y;
    _current_velocity.z = msg_vel.linear.z;
}

void RvizController::spin()
{
    ROS_INFO_NAMED(RVIZ_CONTROLLER_LOG_NAME, "Controller node started");
    auto jmg = _kinematic_state.getJointModelGroup("manipulator");
    while(ros::ok())
    {
        _end_effector_pose.translation().x() += _current_velocity.x;
        _end_effector_pose.translation().y() += _current_velocity.y;
        _end_effector_pose.translation().z() += _current_velocity.z;
        _kinematic_state.setFromIK(jmg, _end_effector_pose);
        _kinematic_state.update(true);
        _visual_tools.publishRobotState(_kinematic_state, rvt::colors::BLUE);
        ros::spinOnce();
        _rate.sleep();
    }
}
