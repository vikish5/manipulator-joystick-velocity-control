#include <moveit_visual_tools/moveit_visual_tools.h>

//project
#include <Eigen/Eigen>
#include <tf/tf.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <cmath>
#include <chrono>
#include <thread>

#define RVIZ_CONTROLLER_LOG_NAME "RvizController"
#define STATIC_POSE_DELTA 0.001

namespace rvt = rviz_visual_tools;

using LinearVelocity = tf::Vector3;

class RvizController
{
    ros::NodeHandle _nh;
    ros::Subscriber _cmd_vel_sub;
    ros::Subscriber _cmd_pose_sub;
    ros::Rate _rate = ros::Rate(20);

    Eigen::Affine3d _end_effector_pose;
    LinearVelocity _current_velocity = tf::Vector3(0, 0, 0);

    robot_model_loader::RobotModelLoader _robot_model_loader;
    const moveit::core::RobotModelPtr _kinematic_model;
    planning_scene::PlanningScene _planning_scene;
    moveit_visual_tools::MoveItVisualTools _visual_tools;
    robot_state::RobotState _kinematic_state;
    // std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> _time_stamp = std::chrono::system_clock::now();
    bool _rotate_in_local_frame;

public:
    RvizController();

    ~RvizController() = default;

    void cmdVelCallback(const geometry_msgs::Twist &msg_vel);

    void cmdPoseCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    void spin();
};