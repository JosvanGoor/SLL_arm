#ifndef INCL_MOTIONCONTROLLER_H
#define INCL_MOTIONCONTROLLER_H

#include <thread>
#include <mutex>
#include <string>
#include <strings.h> //for Aria
#include <cstring>
#include <climits> //for Aria
#include <cstdlib> //for Aria
#include <vector>
#include <Aria.h>
#include "types.h"
#include <ros/ros.h>

class MotionController {
    public:
        enum ERROR_STATE
        {
            NO_ERROR,
            CONNECTION_LOST,
            CONNECTION_FAILED,
            INITIALIZING,
            DISCONNECTED
        };

        struct Odometry
        {
            float x;
            float y;
            float theta;
            float voltage;
            double time; //Warning the current time is big, it does not fit in a float
            float trans_vel;
            float rot_vel;
        };

    private:
        // robot pointer
        ArRobot *d_robot;
        ArRobot *d_tmp_robot;
        timespec d_received_time;
        size_t d_connect_timeout;
        double d_max_idle_time;
        bool d_do_timeout; //whether the robot should time out if not receives commands for d_maxIdleTime
        ERROR_STATE d_error;
        bool d_connected;
        bool d_running;
        std::recursive_mutex d_lock;
        std::thread d_thread;

        Odometry d_last_odometry;
        float d_tf_x;
        float d_tf_y;
        float d_tf_theta;

        static MotionController *s_instance;

    protected:
        // the functor callbacks
        ArFunctorC<MotionController> d_connected_cb;
        ArFunctorC<MotionController> d_conn_fail_cb;
        ArFunctorC<MotionController> d_disconnected_cb;
    
    public:
        MotionController();
        ~MotionController();

        static void handleSignal(int a);

        bool connectRobot(); // Connect to the robot
        void disconnectRobot(); // Disconnect from the robot

        bool connected() const; // Returns whether or not there is an active connectiona to the robot

        void driveForward(int); //move in millimeters
        void driveTurn(int);    //turn in degrees (??)
        void setSpeeds(int, int); //wheel speeds in mm/s
        void setOptions(int, int); //Options to control velocity and acceleration of rotation and translation movement.
	void setCmdVel(double, double); //controls robot speed using lateral, and rotational speed

        // to be called if the connection was made
        void on_connected(void);
        // to call if the connection failed
        void on_connFail(void);
        // to be called if the connection was lost
        void on_disconnected(void);
        //handle a command
        void handleCommand(pairActionpairInt);
	void handleCommand(pairActionpairDouble);
        ERROR_STATE error() const;
        void resetError();

        /**
        * Returns a string representation of the X, Y postion, angle and battery voltage level.
        * WARNING: Make sure the provided message buffer is at least 255 Bytes long.
        */
        void getOdometry(char *msg, Odometry &odo);
        void getStatusString(char *msg);

        bool start();   // Starts the robot control thread
        void stop();    // Stops the robot control thread
        void runner();   // The robot control thread

        static float tdif(timespec t1, timespec t2);
};

inline MotionController::ERROR_STATE MotionController::error() const
{
    return d_error;
}

inline void MotionController::resetError()
{
    d_error = NO_ERROR;
}

inline bool MotionController::connected() const
{
    return d_robot != 0 && d_connected;
}

#endif
