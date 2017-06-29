#include <pioneercontroller2/motioncontroller.h>
#include <iostream>

#include <ros/ros.h>

using namespace std;

bool MotionController::start()
{
    if (d_running)
        return true;

    bool thread = false;
    cout << "Trying to start thread \n";
    while (not thread)
        try
        {
            d_thread = std::thread(&MotionController::runner, this);
            thread = true;
            d_running = true;
        }
        catch (...)
        {
            ROS_ERROR("Starting thread failed");
        }
    cout << "Thread started\n";

    return true;
}
