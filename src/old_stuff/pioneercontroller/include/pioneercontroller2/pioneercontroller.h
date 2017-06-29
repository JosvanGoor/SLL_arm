#ifndef __INCLUDED_PIONEERCONTROLLER_H_
#define __INCLUDED_PIONEERCONTROLLER_H_

#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "motioncontroller.h"

class PioneerController
{
    private:
        ros::NodeHandle d_nh;
        ros::Publisher d_odometry_txt_topic;
        ros::Publisher d_odometry_topic;
        ros::Publisher d_status_topic;
        ros::Subscriber d_message_topic;
        ros::Subscriber d_twist_topic;
        MotionController d_mc;
        tf::TransformBroadcaster d_odom_broadcaster;
    
    public:
        PioneerController();
        void run();

        void twistMessageReceived(const geometry_msgs::Twist::ConstPtr &msg);
        void messageReceived(const std_msgs::String::ConstPtr &msg);

        void publishOdometry();
        void publishStatus();
        void publishEmergency();

        void connect();
        bool connected() const;
        void disconnect();

        static double pythonTime();
};

inline double PioneerController::pythonTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + (tv.tv_usec / 1000000.0);
}

inline bool PioneerController::connected() const
{
    return d_mc.connected();
}


#endif

