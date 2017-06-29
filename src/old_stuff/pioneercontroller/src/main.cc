#include <pioneercontroller2/pioneercontroller.h>

#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv)
{
    std::cout << "PID: " << getpid() << std::endl;

    ros::init(argc, argv, "borg_pioneer");
    PioneerController controller;
    controller.run();
}
