#include <micoArm.h>

float MicoArm::toDegrees(float x)
{
	return x * 180.0 / M_PI;
}

bool MicoArm::checkDistance(float current, float goal)
{
	float offSet = 5.0;

	return fabs(fabs(current) - fabs(goal)) < offSet;
}

float MicoArm::velocity(float acceleration, float time)
{
	return 0.5 * acceleration * time;
}

bool MicoArm::isZeroVelocity(trajectory_msgs::JointTrajectoryPoint point)
{
	return (toDegrees(point.velocities[0]) == 0.0 and toDegrees(point.velocities[1]) == 0.0 and toDegrees(point.velocities[2]) == 0.0 and
			toDegrees(point.velocities[3]) == 0.0 and toDegrees(point.velocities[4]) == 0.0 and toDegrees(point.velocities[5]) == 0.0);
}

bool MicoArm::isAlmostZeroVelocity(TrajectoryPoint point)
{
	float zeroPoint = 0.05;
	if (point.Position.Actuators.Actuator1 >= zeroPoint and point.Position.Actuators.Actuator1 <= zeroPoint and
			point.Position.Actuators.Actuator2 >= zeroPoint and point.Position.Actuators.Actuator2 <= zeroPoint and
			point.Position.Actuators.Actuator3 >= zeroPoint and point.Position.Actuators.Actuator3 <= zeroPoint and
			point.Position.Actuators.Actuator4 >= zeroPoint and point.Position.Actuators.Actuator4 <= zeroPoint and
			point.Position.Actuators.Actuator5 >= zeroPoint and point.Position.Actuators.Actuator5 <= zeroPoint and
			point.Position.Actuators.Actuator6 >= zeroPoint and point.Position.Actuators.Actuator6 <= zeroPoint)
		return true;

	return false;
}

void MicoArm::createMicoTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
	// first point is zero velocity, need acceleration and total time information
	trajectory_msgs::JointTrajectoryPoint point = goal->trajectory.points[0];
	float acceleration1 = point.accelerations[0];
	float acceleration2 = point.accelerations[1];
	float acceleration3 = point.accelerations[2];
	float acceleration4 = point.accelerations[3];
	float acceleration5 = point.accelerations[4];
	float acceleration6 = point.accelerations[5];

	float t1 = point.time_from_start.toSec();

	// first calculate the average velocity with the acceleration
	// all between points just use the given velocity
	// last point can just be zero (I think)

}

void MicoArm::actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
	ROS_INFO_STREAM("Trajectory received");

	TrajectoryPoint waypoint;
	waypoint.InitStruct();
	waypoint.Position.Type = ANGULAR_VELOCITY;

	AngularPosition currentPosition;

	float prevTime = 0;

	ROS_INFO_STREAM("Nr of waypoints: " << goal->trajectory.points.size());

	for (size_t t = 1; t < goal->trajectory.points.size() - 1; ++t)
	{
		trajectory_msgs::JointTrajectoryPoint point = goal->trajectory.points[t];
		trajectory_msgs::JointTrajectoryPoint goalPoint;

		if (t != goal->trajectory.points.size() - 1)
			goalPoint = goal->trajectory.points[t + 1];

		ROS_INFO_STREAM(point);
		ROS_INFO_STREAM(goalPoint);

		// first calculate the average velocity with the acceleration
		// all between points just use the given velocity
		// last point can just be zero (I think)

		if (isZeroVelocity(point) and t != goal->trajectory.points.size() - 1)
		{
			waypoint.Position.Actuators.Actuator1 = velocity(toDegrees(point.accelerations[0]), goalPoint.time_from_start.toSec() - prevTime);
			waypoint.Position.Actuators.Actuator2 = velocity(toDegrees(point.accelerations[1]), goalPoint.time_from_start.toSec() - prevTime);
			waypoint.Position.Actuators.Actuator3 = velocity(toDegrees(point.accelerations[2]), goalPoint.time_from_start.toSec() - prevTime);
			waypoint.Position.Actuators.Actuator4 = velocity(toDegrees(point.accelerations[3]), goalPoint.time_from_start.toSec() - prevTime);
			waypoint.Position.Actuators.Actuator5 = velocity(toDegrees(point.accelerations[4]), goalPoint.time_from_start.toSec() - prevTime);
			waypoint.Position.Actuators.Actuator6 = velocity(toDegrees(point.accelerations[5]), goalPoint.time_from_start.toSec() - prevTime);
		}
		else
		{
			// current waypoint velocities
			waypoint.Position.Actuators.Actuator1 = toDegrees(point.velocities[0]);
			waypoint.Position.Actuators.Actuator2 = toDegrees(point.velocities[1]);
			waypoint.Position.Actuators.Actuator3 = toDegrees(point.velocities[2]);
			waypoint.Position.Actuators.Actuator4 = toDegrees(point.velocities[3]);
			waypoint.Position.Actuators.Actuator5 = toDegrees(point.velocities[4]);
			waypoint.Position.Actuators.Actuator6 = toDegrees(point.velocities[5]);

			ROS_INFO_STREAM(waypoint.Position.Actuators.Actuator1 << ", " << waypoint.Position.Actuators.Actuator2);
		}

		// current goal positions
		AngularPosition goalPosition;
		goalPosition.Actuators.Actuator1 = toDegrees(goalPoint.positions[0]);
		goalPosition.Actuators.Actuator2 = toDegrees(goalPoint.positions[1]);
		goalPosition.Actuators.Actuator3 = toDegrees(goalPoint.positions[2]);
		goalPosition.Actuators.Actuator4 = toDegrees(goalPoint.positions[3]);
		goalPosition.Actuators.Actuator5 = toDegrees(goalPoint.positions[4]);
		goalPosition.Actuators.Actuator6 = toDegrees(goalPoint.positions[5]);

		while(true)
		{
			// the current joint positions
			getRealJointPositions(currentPosition);

			if (checkDistance(currentPosition.Actuators.Actuator1, goalPosition.Actuators.Actuator1) &&
					checkDistance(currentPosition.Actuators.Actuator2, goalPosition.Actuators.Actuator2) &&
					checkDistance(currentPosition.Actuators.Actuator3, goalPosition.Actuators.Actuator3) &&
					checkDistance(currentPosition.Actuators.Actuator4, goalPosition.Actuators.Actuator4) &&
					checkDistance(currentPosition.Actuators.Actuator5, goalPosition.Actuators.Actuator5) &&
					checkDistance(currentPosition.Actuators.Actuator6, goalPosition.Actuators.Actuator6))
				break;

			ROS_INFO_STREAM(waypoint.Position.Actuators.Actuator1 << ", " << waypoint.Position.Actuators.Actuator2);
			sendAdvanceTrajectory(waypoint);
			usleep(5000);
		}

		prevTime = goalPoint.time_from_start.toSec();
	}

	waypoint.Position.Actuators.Actuator1 = 0.0;
	waypoint.Position.Actuators.Actuator2 = 0.0;
	waypoint.Position.Actuators.Actuator3 = 0.0;
	waypoint.Position.Actuators.Actuator4 = 0.0;
	waypoint.Position.Actuators.Actuator5 = 0.0;
	waypoint.Position.Actuators.Actuator6 = 0.0;
	sendAdvanceTrajectory(waypoint);

	micoTrajectoryServer.setSucceeded();
}
