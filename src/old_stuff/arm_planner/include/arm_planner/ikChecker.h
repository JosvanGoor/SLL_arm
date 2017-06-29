#ifndef _H_IKCHECKER
#define _H_IKCHECKER

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <math.h>
#include <tf/transform_datatypes.h>

class IKChecker
{
	Eigen::Affine3d d_end_effector_default_pose;
	robot_model_loader::RobotModelLoader robot_model_loader;
	robot_model::RobotModelPtr kinematic_model;
	robot_state::RobotStatePtr kinematic_state;
	robot_state::JointModelGroup* joint_model_group;
	Eigen::Affine3d end_effector_default_pose;

	public:
		IKChecker();
		bool check(float x, float y, float z, float thetax = 90.0, float thetaY = 0.0, float thetaZ = 0.0);

	private:
		Eigen::Affine3d create_rotation_matrix(double ax, double az, double ay);
};

#endif
