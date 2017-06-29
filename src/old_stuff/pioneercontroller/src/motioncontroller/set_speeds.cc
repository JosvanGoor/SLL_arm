#include <pioneercontroller2/motioncontroller.h>
#include <iostream>

using namespace std;

void MotionController::setSpeeds(int leftSpeed, int rightSpeed)
{
    d_lock.lock();
    if (not connected())
    {
        ROS_ERROR("Not connected to robot, so not sending command");
        d_lock.unlock();
        return;
    }

    d_robot->lock();
    ROS_INFO("Set left/right speeds to %d/%d mm/s", leftSpeed, rightSpeed);
    d_robot->setVel2(leftSpeed, rightSpeed);
    d_robot->unlock();
    d_lock.unlock();
}
