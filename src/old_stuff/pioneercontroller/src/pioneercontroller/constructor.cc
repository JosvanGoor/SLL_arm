#include "pioneercontroller.ih"
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

PioneerController::PioneerController()
:
    d_odometry_txt_topic(d_nh.advertise<std_msgs::String>("pioneer_odometry", 100)),
    d_odometry_topic(d_nh.advertise<nav_msgs::Odometry>("odom", 100)),
    d_status_topic(d_nh.advertise<std_msgs::String>("pioneer_status", 100, true)),
    d_message_topic(d_nh.subscribe<std_msgs::String>("pioneercontroller", 100, &PioneerController::messageReceived, this)),
    d_twist_topic(d_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &PioneerController::twistMessageReceived, this))
{}
