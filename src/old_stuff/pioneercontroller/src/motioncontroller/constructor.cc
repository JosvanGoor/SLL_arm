#include <pioneercontroller2/motioncontroller.h>
#include <iostream>


MotionController::MotionController()
:
    d_robot(0),
    d_connect_timeout(1000),
    d_max_idle_time(100),
    d_do_timeout(true),
    d_error(ERROR_STATE::NO_ERROR),
    d_connected(false),
    d_running(false),
    d_tf_x(0),
    d_tf_y(0),
    d_tf_theta(0),  
    d_connected_cb(this, &MotionController::on_connected),  
    d_conn_fail_cb(this, &MotionController::on_connFail),
    d_disconnected_cb(this, &MotionController::on_disconnected)
{
    // Do not let Aria handle signals, as the main program will take care of that
    d_last_odometry.x = d_last_odometry.y = d_last_odometry.theta = 0;
    s_instance = this;
}
