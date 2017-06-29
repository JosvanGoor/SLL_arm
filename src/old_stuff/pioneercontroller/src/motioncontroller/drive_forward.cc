#include <pioneercontroller2/motioncontroller.h>
#include <iostream>

using namespace std;

void MotionController::driveForward(int distance)
{
    d_lock.lock();
    if (not connected())
    {
        ROS_ERROR("Not connected to robot, so not sending command");
        d_lock.unlock();
        return;
    }

    d_robot->lock();
    ROS_INFO("Sending command to move %d mm at speed 200", distance);
    d_robot->setTransVelMax(200);
    d_robot->move(distance);
    d_robot->unlock();
    d_lock.unlock();
}
