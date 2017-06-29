#include "pioneercontroller.ih"

void PioneerController::publishStatus()
{
    if (d_status_topic.getNumSubscribers() == 0)
        return;

    char msg[255];
    d_mc.getStatusString(msg);
    std_msgs::String status_msg;
    status_msg.data = std::string(msg);
    d_status_topic.publish(status_msg);
}
