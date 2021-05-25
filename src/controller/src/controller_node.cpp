#include "RvizController.h"

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "controller_node");
    RvizController rviz_controller;
    rviz_controller.spin();
    return 0;
}