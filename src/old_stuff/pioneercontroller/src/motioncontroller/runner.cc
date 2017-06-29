#include <pioneercontroller2/motioncontroller.h>
#include <iostream>
#include <ctime>

#include <ros/ros.h>

using namespace std;

void shutdown_handler(int a)
{
    cout << "[MotionController] Caught signal, quitting" << endl << flush;
    signal(SIGINT, shutdown_handler);
    signal(SIGTERM, shutdown_handler);
    signal(SIGKILL, shutdown_handler);
    signal(SIGHUP, shutdown_handler);
    signal(SIGABRT, shutdown_handler);
    signal(SIGQUIT, shutdown_handler);
    signal(SIGPIPE, shutdown_handler);

    MotionController::handleSignal(a);
}

void MotionController::runner()
{
    cout << "[MotionController] Robot control thread started\n";

    // Catch signal handlers
    signal(SIGINT, shutdown_handler);
    signal(SIGTERM, shutdown_handler);
    signal(SIGKILL, shutdown_handler);
    signal(SIGHUP, shutdown_handler);
    signal(SIGABRT, shutdown_handler);
    signal(SIGQUIT, shutdown_handler);
    signal(SIGPIPE, shutdown_handler);

    ros::Rate loop_rate(10);
    while (ros::ok() && d_running)
    {
    	d_lock.lock();
        if (d_robot != 0 && not connected())
        {
            disconnectRobot();
        }
        else if (d_robot != 0)
        {
            ArDeviceConnection *conn = d_robot->getDeviceConnection();
            if (conn == 0 || conn->getStatus() != ArDeviceConnection::STATUS_OPEN)
            {
                if (conn == 0)
                    ROS_ERROR("No available ArDeviceConnection. Restarting robot connection");
                else
                    ROS_ERROR("ArDeviceConnection does not have an open connection. Status: %d. Restarting robot connection", conn->getStatus());
                d_connected = false;
                //disconnectRobot();
            }
        }
        d_lock.unlock();

        if (d_robot == 0)
            connectRobot();

        //double const maxAcceleration = 5;
        //d_robot->setTransDecel(maxAcceleration);
        //d_robot->setTransAccel(maxAcceleration);
        if (connected())
        {
            timespec current_time;
            clock_gettime(CLOCK_REALTIME, &current_time);
            float elapsed_time = tdif(d_received_time, current_time);

            if ((elapsed_time > d_max_idle_time) and false)
            {
                // stop when we haven't received a command for too long
            	double leftWheelSpeed = d_robot->getLeftVel();
            	double rightWheelSpeed = d_robot->getRightVel();
            	while (d_robot->getLeftVel() > 50)
            	{
            		leftWheelSpeed -= 10;
            		rightWheelSpeed -= 10;
            		setSpeeds(leftWheelSpeed, rightWheelSpeed);
            	}
		if (leftWheelSpeed >= 10 || rightWheelSpeed >= 10)
            		setSpeeds(0 , 0);
                d_error = MotionController::NO_ERROR;
            }
        }

        loop_rate.sleep();
    }
    cout << "Disconnecting robot\n";
    disconnectRobot();
    cout << "[MotionController] Robot control thread ended\n";
}
