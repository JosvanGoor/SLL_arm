#include <pioneercontroller2/motioncontroller.h>
#include <iostream>

using namespace std;

void MotionController::disconnectRobot()
{
    d_lock.lock();

    if (d_robot == 0)
    {
        ROS_ERROR("No robot connection available. Cannot disconnect");
        d_lock.unlock();
        return;
    }

    if (d_connected)
        ROS_ERROR("Disconnecting from robot");
    else
        ROS_ERROR("Removing disconnected robot object");

    if (not d_robot->isRunning())
    {
        // For some reason, eveon if waitForRunExit says it has exited,
        // it can still be running. Give it half a second to terminate
        // before deleting it, to avoid segfaults...
        Aria::shutdown();
        usleep(500000);
        delete d_robot;
        d_robot = 0;
        ROS_INFO("Deleted robot");
        d_connected = false;

        // Store last odometry as new transform
        d_tf_x = d_last_odometry.x;
        d_tf_y = d_last_odometry.y;
        d_tf_theta = d_last_odometry.theta * TO_RAD;
        ROS_INFO("Updated transform");
    }
    else
    {
        // Clear up robot
        d_robot->lock();
        ROS_INFO("Requesting robot stop");
        d_robot->stopRunning(true);
        ROS_INFO("called stop running");
        d_robot->unlock();
        ROS_INFO("unlocked robot");

    }

    d_lock.unlock();
}
