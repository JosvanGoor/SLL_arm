#include <pioneercontroller2/motioncontroller.h>
#include <iostream>
#include <fstream>

#include <stdlib.h>  
#include <stdio.h>

#include <ros/ros.h>

using namespace std;

bool MotionController::connectRobot()
{
    static timespec last_attempt;

    if (d_connected)
        return true;

    timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);
    if (tdif(current_time, last_attempt) < (d_connect_timeout / 1000.0))
        return false;

    last_attempt = current_time;

    if (d_error == NO_ERROR)
        d_error = INITIALIZING;
    
    Aria::init(Aria::SIGHANDLE_NONE);
    ArArgumentBuilder *args;
    args = new ArArgumentBuilder();
    args->add("-rp"); //pass robot's serial port to Aria
    char str_buf[255];
    sprintf(str_buf, "%s/brain/dev/pioneer", getenv("BORG"));
    args->add(str_buf);
    args->add("-rb");
    args->add("9600");
    //args->add("38400");

    ArSimpleConnector con(args);
    
    d_tmp_robot = new ArRobot("borg", true, false, true, false);
    d_tmp_robot->addConnectCB(&d_connected_cb, ArListPos::FIRST);
    d_tmp_robot->addFailedConnectCB(&d_conn_fail_cb, ArListPos::FIRST);
    d_tmp_robot->addDisconnectNormallyCB(&d_disconnected_cb, ArListPos::FIRST);
    d_tmp_robot->addDisconnectOnErrorCB(&d_disconnected_cb, ArListPos::FIRST);
    d_tmp_robot->setConnectionTimeoutTime(1000);
    //cout << d_tmp_robot->getRotAccel() << '\n';
    //d_tmp_robot->setRotAccel(50);
    //d_tmp_robot->setRotDecel(50);
    //d_tmp_robot->setTransAccel(50);
    //d_tmp_robot->setTransDecel(50);
    //d_tmp_robot->setRotVel(100);
    cout << "[MotionController] Connecting to robot\n";
    if (!con.connectRobot(d_tmp_robot))
    {
        ROS_WARN("Could not connect to the robot.");
        d_lock.lock();
        d_error = CONNECTION_FAILED;
        d_lock.unlock();
        delete d_tmp_robot;
        d_tmp_robot = 0;
        return false;
    }

    d_lock.lock();
  
    ROS_INFO("Established connection to robot");
    d_error = NO_ERROR;
    d_connected = true;
    d_robot = d_tmp_robot;
    d_tmp_robot = 0;
  
    // Run the robot processing cycle in its own thread. Note that after
    // starting this thread, we must lock and unlock the ArRobot object before
    // and after accessing it.
    d_robot->runAsync(true);
    
    clock_gettime(CLOCK_REALTIME, &d_received_time); //initialize d_receivedTime (time we last received a non-empty message)
    d_lock.unlock();
    return true;
}
