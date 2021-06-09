#include "RvizController.h"

RvizController::RvizController()
    :   _robot_model_loader("robot_description")
    ,   _kinematic_model(_robot_model_loader.getModel())
    ,   _planning_scene(_kinematic_model)
    ,   _visual_tools("world")
    ,   _kinematic_state(_kinematic_model)
{
    _cmd_vel_sub = _nh.subscribe("cmd_vel", 1000, &RvizController::cmdVelCallback, this);
    _cmd_pose_sub = _nh.subscribe("joy", 1000, &RvizController::cmdPoseCallback, this);
    _kinematic_state.setToDefaultValues();
    _end_effector_pose = _kinematic_state.getGlobalLinkTransform("end_link");
}

void RvizController::cmdVelCallback(const geometry_msgs::Twist &msg_vel)
{
    _current_velocity.setX(msg_vel.linear.x);
    _current_velocity.setY(msg_vel.linear.y);
    _current_velocity.setZ(msg_vel.linear.z);
}

void RvizController::cmdPoseCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    // auto diff = std::chrono::system_clock::now() - _time_stamp;
    ROS_INFO("Rotate in local frame: %u", _rotate_in_local_frame);
    if (joy_msg->buttons[1])
    {
        _rotate_in_local_frame = _rotate_in_local_frame ? false : true;
        // _time_stamp = std::chrono::system_clock::now();
    }

    if(joy_msg->buttons[2])
    {   
        Eigen::Matrix3d roll;
        if (_rotate_in_local_frame)
        {
            roll = Eigen::AngleAxisd( joy_msg->buttons[7] ? -STATIC_POSE_DELTA : STATIC_POSE_DELTA, Eigen::Vector3d::UnitX());
            _end_effector_pose.rotate(roll);
        }
        else
        {
            roll = Eigen::AngleAxisd( joy_msg->buttons[7] ? -STATIC_POSE_DELTA : STATIC_POSE_DELTA, 
            _end_effector_pose.inverse().rotation() * Eigen::Vector3d::UnitX() + _end_effector_pose.translation());
            _end_effector_pose = roll * _end_effector_pose;
        }
    }
    if(joy_msg->buttons[3])
    {
        Eigen::Matrix3d pitch;
        if (_rotate_in_local_frame)
        {
            pitch = Eigen::AngleAxisd( joy_msg->buttons[7] ? -STATIC_POSE_DELTA : STATIC_POSE_DELTA, Eigen::Vector3d::UnitY());
            _end_effector_pose.rotate(pitch);
        }
        else
        {
            pitch = Eigen::AngleAxisd( joy_msg->buttons[7] ? -STATIC_POSE_DELTA : STATIC_POSE_DELTA, 
            _end_effector_pose.inverse().rotation() * Eigen::Vector3d::UnitY());
             _end_effector_pose = pitch * _end_effector_pose;
        }
    }
    if(joy_msg->buttons[0])
    {   
       Eigen::Matrix3d yaw;
        if (_rotate_in_local_frame)
        {
            yaw = Eigen::AngleAxisd( joy_msg->buttons[7] ? -STATIC_POSE_DELTA : STATIC_POSE_DELTA, Eigen::Vector3d::UnitZ());
            _end_effector_pose.rotate(yaw);
        }
        else
        {
            yaw = Eigen::AngleAxisd( joy_msg->buttons[7] ? -STATIC_POSE_DELTA : STATIC_POSE_DELTA, 
            _end_effector_pose.inverse().rotation() * Eigen::Vector3d::UnitZ());
            _end_effector_pose = yaw * _end_effector_pose;
        }
    }
}

void RvizController::spin()
{
    ROS_INFO_NAMED(RVIZ_CONTROLLER_LOG_NAME, "Controller node started");
    auto jmg = _kinematic_state.getJointModelGroup("manipulator");
    while(ros::ok())
    {   
        _end_effector_pose.translation().x() += _current_velocity.x();
        _end_effector_pose.translation().y() += _current_velocity.y();
        _end_effector_pose.translation().z() += _current_velocity.z();
        _kinematic_state.setFromIK(jmg, _end_effector_pose);
        _kinematic_state.update(true);
        _visual_tools.publishRobotState(_kinematic_state);
        ros::spinOnce();
        _rate.sleep();
    }
}
