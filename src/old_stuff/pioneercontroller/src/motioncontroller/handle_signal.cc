#include <pioneercontroller2/motioncontroller.h>

void MotionController::handleSignal(int a)
{
    if (s_instance != 0)
    {
        s_instance->d_connected = false;
        switch (a)
        {
            case SIGHUP:
            case SIGPIPE:
                // Nothing to do, just disconnect
                break;
            case SIGINT:
            case SIGTERM:
            case SIGKILL:
            case SIGABRT:
            case SIGQUIT:
                // Now, we want to actually stop the robot
                s_instance->stop();
                ros::shutdown();
                break;
        }
    }
}
