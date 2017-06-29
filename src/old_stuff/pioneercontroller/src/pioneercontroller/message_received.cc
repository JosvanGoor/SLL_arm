#include "pioneercontroller.ih"
#include <pioneercontroller2/commandparser.h>

void PioneerController::messageReceived(const std_msgs::String::ConstPtr &msg)
{
    if (!connected())
    {
        ROS_WARN("Received String command, but there is no connection to Pioneer is available");
        return;
    }
    else
        ROS_DEBUG("I heard: [%s]", msg->data.c_str());

    CommandParser CP;
    pairActionpairInt cmd = CP.parseCommand(msg->data);

    d_mc.handleCommand(cmd);
}
