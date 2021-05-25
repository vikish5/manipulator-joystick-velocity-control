#include <moveit_visual_tools/moveit_visual_tools.h>

//project
#include "ros/ros.h"

#include <cmath>
#include <chrono>
#include <thread>

#define RVIZ_CONTROLLER_LOG_NAME "RvizController"

namespace rvt = rviz_visual_tools;

using LinearVelocity = tf::Vector3;

class RvizController
{
    ros::NodeHandle _nh;
    ros::Subscriber _cmd_vel_sub;
    ros::Rate _rate = ros::Rate(20);

    Eigen::Affine3d _end_effector_pose;
    LinearVelocity _current_velocity = tf::Vector3(0, 0, 0);

    robot_model_loader::RobotModelLoader _robot_model_loader;
    const moveit::core::RobotModelPtr _kinematic_model;
    planning_scene::PlanningScene _planning_scene;
    moveit_visual_tools::MoveItVisualTools _visual_tools;
    robot_state::RobotState _kinematic_state;

public:
    RvizController();

    ~RvizController() = default;

    void cmdVelCallback(const geometry_msgs::Twist &msg_vel);

    void spin();
};