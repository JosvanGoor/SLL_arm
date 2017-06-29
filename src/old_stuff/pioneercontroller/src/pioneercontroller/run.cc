#include "pioneercontroller.ih"

void PioneerController::run()
{
    ros::Rate loop_rate(30);
    d_mc.start();
    while (ros::ok())
    {
        publishOdometry();
        publishStatus();

        ros::spinOnce();
        loop_rate.sleep();
    }
    d_mc.stop();
}
