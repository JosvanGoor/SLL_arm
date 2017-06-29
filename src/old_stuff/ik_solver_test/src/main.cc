#include <ros/ros.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <math.h>
#include <tf/transform_datatypes.h>

float toRad(float deg)
{
    return deg * (M_PI / 180.0f);
}

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az)
{
	Eigen::Affine3d rx =
	  Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d ry =
	  Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
	Eigen::Affine3d rz =
	  Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
	return rz * ry * rx;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "mico_kinematics");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();

	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("mico_arm");
//	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("eef");

	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

	for(std::size_t i = 0; i < joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

//	kinematic_state->setToRandomPositions(joint_model_group);
	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("mico_link_hand");

	const Eigen::Affine3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform("mico_link_hand");

	ros::NodeHandle nh;
	ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>( "tutorial_robot_state", 1 );
	moveit_msgs::DisplayRobotState msg;


	const double RADIUS = 0.1;
	ros::Rate loop_rate(10);
	float xx = -0.0;
	float yy = 0.0;
	float zz = 0.0;
//	for (double angle=0; angle<=2*M_PI && ros::ok(); angle+=2*M_PI/20)

	for (size_t idx = 0; idx < 15; ++idx)
	{
		xx -= 0.00;
		yy += 0.00;
		zz += 0.0;

		Eigen::Affine3d r = create_rotation_matrix(toRad(90.0), toRad(0.0), toRad(90.0)); // RYP pitch and yaw are turned around, don't ask...

		/*
		tf::Quaternion qt;
		float yaw,pitch,roll;
		yaw = toRad(0.0);
		pitch = toRad(0.0);
		roll = toRad(90.0);
		qt.setEuler(yaw, pitch, roll);
		float w = qt.w();
		float qx = qt.x();
		float qy = qt.y();
		float qz = qt.z();
		Eigen::Quaternion<double> q(w, qx, qy, qz);
		*/

		Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(-end_effector_default_pose(0,3), -end_effector_default_pose(1,3), -end_effector_default_pose(2,3))));

		ROS_INFO_STREAM("X: " << xx << ", Y: " << yy << ", Z: " << zz);
		/* calculate a position for the end effector */
	//	Eigen::Affine3d end_effector_pose = Eigen::Translation3d(xx, yy, zz) * end_effector_default_pose;

		Eigen::Affine3d end_effector_pose = t * end_effector_default_pose;

		//end_effector_pose = q.toRotationMatrix() * end_effector_pose;
		end_effector_pose = r * end_effector_pose;

		Eigen::Affine3d newPos(Eigen::Translation3d(Eigen::Vector3d(0.0, -0.2, 0.2)));

		end_effector_pose = newPos * end_effector_pose;

		ROS_INFO_STREAM("End effector position:\n" << end_effector_pose.translation());
		/* use IK to get joint angles satisfyuing the calculated position */

		bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);

		if (!found_ik)
		{
			ROS_INFO_STREAM("Could not solve IK for pose\n" << end_effector_pose.translation());
			continue;
		}

		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for(std::size_t i = 0; i < joint_names.size(); ++i)
		{
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}

		/* get a robot state message describing the pose in kinematic_state */
		moveit_msgs::DisplayRobotState msg;
		robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
		/* send the message to the RobotState display */
		robot_state_publisher.publish( msg );
		/* let ROS send the message, then wait a while */
		ros::spinOnce();
		loop_rate.sleep();
	}

/*
	ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
	ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

	Eigen::Affine3d position;

	double yaw,pitch,roll;
	yaw = toRad(0); // alpha
	pitch = toRad(0); // beta
	roll = toRad(90);	// theta

	float x,y,z;
	x = 0.0;
	y = -0.0;
	z = 0.6;

	position.matrix() <<
		 cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll), x,
		 sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll), y,
			-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), z,
			0, 0, 0, 1;
//	ROS_INFO_STREAM("Test: \n" << position.matrix());

	//end_effector_state.translation.x = 0.9999f;
	//ROS_INFO_STREAM("Test Translation: " << end_effector_state.translation());

	// bool found_ik = kinematic_state->setFromIK(joint_model_group, position, 10, 0.5);
	 bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.5);

	 // Now, we can print out the IK solution (if found):
	 if (found_ik)
	 {
		 ROS_INFO_STREAM("Found IK solution");
		 kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		 for(std::size_t i=0; i < joint_names.size(); ++i)
		 {
			 ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		 }
	 }
	 else
	 {
		 ROS_INFO("Did not find IK solution");
	 }

	 */

	 ros::shutdown();
}
