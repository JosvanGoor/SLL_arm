#include <arm_planner/ikChecker.h>

IKChecker::IKChecker() :
	robot_model_loader("robot_description")
{
	kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state_temp(new robot_state::RobotState(kinematic_model));
	kinematic_state = kinematic_state_temp;
	kinematic_state->setToDefaultValues();

	joint_model_group = kinematic_model->getJointModelGroup("mico_arm");

	end_effector_default_pose = kinematic_state->getGlobalLinkTransform("mico_link_hand");

}

Eigen::Affine3d IKChecker::create_rotation_matrix(double ax, double az, double ay)
{
	Eigen::Affine3d rx =
	  Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d ry =
	  Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
	Eigen::Affine3d rz =
	  Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
	return rz * ry * rx;
}

float toRad(float deg)
{
    return deg * (M_PI / 180.0f);
}

bool IKChecker::check(float x, float y, float z, float thetaX, float thetaY, float thetaZ)
{
//	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//	kinematic_state->setToDefaultValues();
//	robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("mico_arm");

//	const Eigen::Affine3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform("mico_link_hand");

	Eigen::Affine3d r = create_rotation_matrix(toRad(thetaX), toRad(thetaY), toRad(thetaZ));
	Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(-end_effector_default_pose(0,3), -end_effector_default_pose(1,3), -end_effector_default_pose(2,3))));
	Eigen::Affine3d end_effector_pose = t * end_effector_default_pose;
	end_effector_pose = r * end_effector_pose;
	Eigen::Affine3d newPos(Eigen::Translation3d(Eigen::Vector3d(x, y, z)));
	end_effector_pose = newPos * end_effector_pose;

	return kinematic_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);

}
