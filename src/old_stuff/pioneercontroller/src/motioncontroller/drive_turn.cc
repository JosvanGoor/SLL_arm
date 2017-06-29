#include <pioneercontroller2/motioncontroller.h>
#include <iostream>

using namespace std;

void MotionController::driveTurn(int angle)
{
    d_lock.lock();
    if (not connected())
    {
        ROS_ERROR("Not connected to robot, so not sending command");
        d_lock.unlock();
        return;
    }

    d_robot->lock();
    ROS_INFO("Setting delta heading to %d degrees", angle);
    d_robot->setDeltaHeading(angle);
    d_robot->unlock();
    d_lock.unlock();
}
