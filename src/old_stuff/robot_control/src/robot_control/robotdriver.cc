#include "robotdriver.ih"

RobotDriver::RobotDriver(ros::NodeHandle &nh)
:
last_command(0),
loop_rate(50),
nh_(nh)
{
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    client_writer = nh_.serviceClient<borg_pioneer::MemorySrv>("memory");
    client_reader = nh_.serviceClient<borg_pioneer::MemoryReadSrv>("memory_read");
};
