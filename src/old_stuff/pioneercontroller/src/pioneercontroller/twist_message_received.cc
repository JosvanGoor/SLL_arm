#include "pioneercontroller.ih"
#include <geometry_msgs/Twist.h>
#include <pioneercontroller2/commandparser.h>

void PioneerController::twistMessageReceived(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (!connected())
    {
        ROS_WARN("Received Twist message, but there is no connection to Pioneer is available");
        return;
    }
    else
        ROS_DEBUG("Twist message arrived");

    CommandParser CP;
    pairActionpairDouble cmd = CP.parseTwistCommand(msg);
    d_mc.handleCommand(cmd);
}
