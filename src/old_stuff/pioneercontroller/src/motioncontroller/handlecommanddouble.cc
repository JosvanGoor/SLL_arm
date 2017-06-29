#include <pioneercontroller2/motioncontroller.h>

#include <ctime>
#include <unistd.h>
#include <iostream>

using namespace std;

void MotionController::handleCommand(pairActionpairDouble command)
{
    switch(command.first)
    {
	    case TWIST:
		    setCmdVel(command.second.first, command.second.second);
		    d_do_timeout = true;
		    break;
        default:   //or any other value
          timespec current_time;
          clock_gettime(CLOCK_REALTIME, &current_time);
          float elapsed_time = tdif(d_received_time, current_time);

        if ((elapsed_time > d_max_idle_time) && d_do_timeout)
           setSpeeds(0, 0); // stop when we haven't received a command for too long
    }
}
