#include "pioneercontroller.ih"
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

// Linear standard deviation is +- 10mm (really? needs testing!)
#define LINEAR_STDDEV (0.01)
#define LINEAR_VARIANCE (LINEAR_STDDEV * LINEAR_STDDEV)
// Angular standard deviation is +- 5 degrees (really? needs testing!)
#define ANGULAR_STDDEV (5.0)
#define ANGULAR_VARIANCE (ANGULAR_STDDEV * ANGULAR_STDDEV * TO_RAD * TO_RAD)

/** publishOdometry will publish the odometry in two ways: using a string with
  * a text format that the body controller in the brain understands, or with a
  * nav_msgs/Odometry format that specifies exact values for the position and
  * the velocity. Also, the transform from /odom to /base_link is always published.
  *
  * Neither of these topics will be published when there are no subscribers. However,
  * the transform between /odom to /base_link will always be published when there is
  * a connection to the robot available.
  */
void PioneerController::publishOdometry()
{
    if (!connected())
        return;

    // Obtain odometry information from the robot
    char msg[255];
    MotionController::Odometry odo;
    d_mc.getOdometry(msg, odo);

    // Publish the string message of the odometry
    if (d_odometry_txt_topic.getNumSubscribers() > 0)
    {
        std_msgs::String odo_msg;
        odo_msg.data = std::string(msg);
        d_odometry_txt_topic.publish(odo_msg);
    }

    // For some reason, setEuler claims to accept parameters in ZYX format,
    // however, the orientation of the robot is essentially a rotation
    // around the Z axis, but it still needs to be the third argument
    // to result in a valid transformation. Nature of this weirdness is unknown,
    // if you can enlighten me, let me know ;-)
    tf::Quaternion orientation;
    orientation.setEuler(0, 0, odo.theta * TO_RAD);
    tf::Vector3 position(odo.x / 1000.0, odo.y / 1000.0, 0);

    // Convert the time in seconds to the ros::Time format
    //ros::Time stamp(odo.time);
    ros::Time stamp = ros::Time::now();

    // Publish the transform from /odom to /base_link
    d_odom_broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(orientation, position), 
        stamp,
        "/odom", 
        "/base_footprint"
    ));

    // If noone is subscribed to the Odometry topic, we're done
    if (d_odometry_topic.getNumSubscribers() == 0)
        return;

    // Prepare the pose: current position of the robot
    geometry_msgs::Pose pose;
    // *sigh*, why are no sensible constructors generated...
    pose.position.x = position.getX();
    pose.position.y = position.getY();
    pose.position.z = position.getZ();
    pose.orientation.x = orientation.getX();
    pose.orientation.y = orientation.getY();
    pose.orientation.z = orientation.getZ();
    pose.orientation.w = orientation.getW();

    // Prepare the twist: current velocity of the robot
    geometry_msgs::Twist twist;

    // Lateral velocity is always along the X-axis in the base_link frame.
    // However, the Twist message wants this relative to the odom frame, so we
    // need to transform it.
    tf::Vector3 linear(odo.trans_vel / 1000.0, 0, 0);
    linear = quatRotate(orientation, linear);
    tf::Vector3 angular(0, 0, odo.rot_vel);

    twist.linear.x = linear.getX();
    twist.linear.y = linear.getY();
    twist.linear.z = linear.getZ();
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = odo.rot_vel * TO_RAD;

    // Publish the actual odometry message
    nav_msgs::Odometry odometry;

    // Overwrite the objects in the Odometry message with the
    // values just prepared
    odometry.pose.pose = pose;
    odometry.twist.twist = twist;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";

    // Set variance of the parts. This definitely needs more fine-tuning after
    // a thorough analysis of errors in the odometry. Currently, the following
    // set of assumptions is used:
    //
    // - The robot cannot turn along the X and Y axes, so their (co)variance is 0
    // - The robot cannot move along the Z axis, so its (co)variance is 0
    // - Position and linear velocity along X and Y axes have equal errors, as set in the
    //   LINEAR_VARIANCE define above. 
    // - Rotation and angular velocity along Z axis have equal errors, as set in the
    //   ANGULAR_VARIANCE define above.
    // - All errors are independent, so only the diagonal of the covariance
    //   matrix is filled
    for (size_t i = 0; i < 6; ++i)
    {
        size_t idx = i * 6 + i;
        if (i == 0 || i == 1) // X and Y coordinates may differ, Z is fixed
        {
            odometry.pose.covariance[idx] = LINEAR_VARIANCE;
            odometry.twist.covariance[idx] = LINEAR_VARIANCE;
        }
        else if (i == 5) // Z angle may differ, X and Y angles are fixed
        {
            odometry.pose.covariance[idx] = ANGULAR_VARIANCE;
            odometry.twist.covariance[idx] = ANGULAR_VARIANCE;
        }
    }

    d_odometry_topic.publish(odometry);
}
