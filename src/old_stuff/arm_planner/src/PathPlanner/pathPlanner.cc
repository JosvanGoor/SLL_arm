#include <arm_planner/pathPlanner.h>

PathPlanner::PathPlanner(ros::NodeHandle &n) :
	as(n, "arm_planning", boost::bind(&PathPlanner::execute, this, _1), false),
	pathClient("grasping_path", true),
	d_grid(new PCLPointCloud)
{
	createGrid();
	kdtree.setInputCloud(d_grid);
	pathClient.waitForServer();
	ROS_INFO_STREAM("found server");
	as.start();
}

void PathPlanner::execute(const arm_planner::goalGoalConstPtr &goal)
{
	float x = goal->x;
	float y = goal->y;
	float z = goal->z;

	if (goal->manObjTest)
	{
		ROS_INFO_STREAM("MAN AND OBJ TEST");
		if (makePathTest(x, y, z)) // construct a path for the man and object test
		{
			ROS_INFO_STREAM("Successful path");
			pathClient.waitForResult(ros::Duration(60.0));

			if (pathClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				as.setSucceeded();
			}
			else
				as.setAborted();
		}
		else
		{
			ROS_INFO_STREAM("Failed path");
			as.setAborted();
		}
		ROS_INFO_STREAM("Goal x: " << x << ", y:" << y << ", z:" << z);

	}
	else
	{
		if (makePath(x, y, z)) // construct a path
		{
			ROS_INFO_STREAM("Successful path");
			pathClient.waitForResult(ros::Duration(60.0));

			if (pathClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				as.setSucceeded();
			}
			else
				as.setAborted();
		}
		else
		{
			ROS_INFO_STREAM("Failed path");
			as.setAborted();
		}
		ROS_INFO_STREAM("Goal x: " << x << ", y:" << y << ", z:" << z);
	}

}

arm_planner::Position PathPlanner::createPoint(float x, float y, float z, float thetaX, float thetaY, float thetaZ)
{
	arm_planner::Position mPoint;
	mPoint.x = x;
	mPoint.y = y;
	mPoint.z = z;

	mPoint.thetaX = thetaX;
	mPoint.thetaY = thetaY;
	mPoint.thetaZ = thetaZ;

	return mPoint;
}

arm_planner::Position createMidPoint(arm_planner::Position sPoint, arm_planner::Position ePoint)
{
	arm_planner::Position iPoint;
	iPoint.x = (sPoint.x + ePoint.x) / 2;
	iPoint.y = (sPoint.y + ePoint.y) / 2;
	iPoint.z = (sPoint.z + ePoint.z) / 2;

	/*
	float toClose = 0.35f;
	if (ePoint.approach == false and sqrt(pow(iPoint.x, 2) + pow(iPoint.y, 2)) < toClose)
	{
		ROS_INFO_STREAM("To close");
		float r = sqrt(pow(iPoint.x, 2) + pow(iPoint.y, 2));
		float mul = toClose / r;
		iPoint.x *= mul;
		iPoint.y *= mul;
	}
	*/
	if (sPoint.approach == true and sPoint.openFingers == true)
	{
		iPoint.thetaX = sPoint.thetaX;
		iPoint.thetaY = sPoint.thetaY;
		iPoint.thetaZ = sPoint.thetaZ;
		iPoint.openFingers = sPoint.openFingers;
		iPoint.approach = sPoint.approach;
	}
	else
	{
		iPoint.thetaX = (sPoint.thetaX + ePoint.thetaX) / 2;
		iPoint.thetaY = (sPoint.thetaY + ePoint.thetaY) / 2;
		iPoint.thetaZ = (sPoint.thetaZ + ePoint.thetaZ) / 2;
		iPoint.openFingers = ePoint.openFingers;
		iPoint.approach = ePoint.approach;
	}

	return iPoint;
}

void PathPlanner::addInbetweens(vector<arm_planner::Position> &vPoints)
{
	vector<arm_planner::Position> tPoints;

	// add a mid point for each two points.
	for (size_t idx = 0; idx < vPoints.size() - 1; ++idx)
	{
		arm_planner::Position iPoint;

		arm_planner::Position bPoint = vPoints.at(idx);
		arm_planner::Position ePoint = vPoints.at(idx + 1);
		iPoint = createMidPoint(bPoint, ePoint);
		tPoints.push_back(iPoint);
	}

	for (size_t idx = 0; idx < tPoints.size(); ++idx)
	{
		vector<arm_planner::Position>::iterator it;
		it = vPoints.begin();
		vPoints.insert(it + idx * 2 + 1, tPoints.at(idx));
	}
}


void addLeftRotation(vector<arm_planner::Position> &vPoints)
{
	vector<arm_planner::Position> tPoints;

	arm_planner::Position sPoint;
	sPoint.x = 0.30;
	sPoint.y = -0.10;
	sPoint.z = 0.35;
	sPoint.thetaX = 90.0f;
	sPoint.thetaY = 90.0f;
	sPoint.thetaZ = 0.0f;
	sPoint.approach = false;

	tPoints.push_back(sPoint);

	sPoint.thetaZ = 90.0f;
	tPoints.push_back(sPoint);

	sPoint.thetaX= 180.0f;
	sPoint.thetaY = 0.0f;
	sPoint.thetaZ = 0.0f;
	tPoints.push_back(sPoint);

	sPoint.thetaX = 90.0f;
	sPoint.thetaY = 0.0f;
	sPoint.thetaZ = 0.0f;
	tPoints.push_back(sPoint);

	vPoints.insert(vPoints.begin(), tPoints.begin(), tPoints.end());

	// TEMP, SHOULD GO TO OTHER POSITION
//	vPoints.insert(vPoints.end(), tPoints.rbegin(), tPoints.rend());
}

void moveToRightSide(vector<arm_planner::Position> &vPoints)
{
	vector<arm_planner::Position> tPoints;
	arm_planner::Position sPoint;

	sPoint.x = 0.30;
	sPoint.y = -0.10;
	sPoint.z = 0.35;
	sPoint.thetaX= 90.0f;
	sPoint.thetaY = 90.0f;
	sPoint.thetaZ = 0.0f;
	sPoint.approach = false;

	tPoints.push_back(sPoint);

	sPoint.x = 0.30;
	sPoint.y = -0.10;
	sPoint.z = 0.35;
	sPoint.thetaX = 90.0f;
	sPoint.thetaY = 90.0f;
	sPoint.thetaZ = 0.0f;

	tPoints.push_back(sPoint);

	sPoint.thetaZ = 90.0f;
	tPoints.push_back(sPoint);

	sPoint.x = 0.20;
	sPoint.thetaX= 180.0f;
	sPoint.thetaY = 0.0f;
	sPoint.thetaZ = 0.0f;
	tPoints.push_back(sPoint);

	sPoint.x = 0.10;
	sPoint.y = -0.20;
	tPoints.push_back(sPoint);

	sPoint.x = -0.10;
	sPoint.y = -0.20;
	tPoints.push_back(sPoint);

	sPoint.x = -0.30;
	sPoint.y = -0.10;
	sPoint.z = 0.35;
	tPoints.push_back(sPoint);

	sPoint.x = -0.30;
	sPoint.y = -0.10;
	sPoint.z = 0.35;
	sPoint.thetaX= 90.0f;
	sPoint.thetaY = 0.0f;
	sPoint.thetaZ = 0.0f;
	tPoints.push_back(sPoint);

	vPoints.insert(vPoints.begin(), tPoints.begin(), tPoints.end());

}

arm_planner::Position PathPlanner::getStartPosition()
{
	arm_planner::Position startPoint;
	startPoint.x = 0.0f;
	startPoint.y = 0.1f;
	startPoint.z = 0.20f;
	startPoint.thetaX = 180.0f;
	startPoint.thetaY = 15.0f;
	startPoint.thetaZ = 0.0f;

	return startPoint;
}

void PathPlanner::moveToLeft(vector<arm_planner::Position> &vPath)
{
	arm_planner::Position point;
	point.x = 0.1f;
	point.y = 0.15f;
	point.z = 0.25f;
	point.thetaX = 180.0f;
	point.thetaY = 20.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = 0.2f;
	point.y = -0.1f;
	point.z = 0.25f;
	point.thetaX = 90.0f;
	point.thetaY = 0.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);
}

void PathPlanner::moveToRight(vector<arm_planner::Position> &vPath)
{
	arm_planner::Position point;
	point.x = -0.1f;
	point.y = 0.15f;
	point.z = 0.25f;
	point.thetaX = 180.0f;
	point.thetaY = 0.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = -0.2f;
	point.y = -0.1f;
	point.z = 0.25f;
	point.thetaX = 90.0f;
	point.thetaY = 0.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);
}

void PathPlanner::moveNavLeft(vector<arm_planner::Position> &vPath)
{
	arm_planner::Position point;
	point.x = 0.35f;
	point.y = -0.05f;
	point.z = 0.40f;
	point.thetaX = 90.0f;
	point.thetaY = 20.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = 0.37f;
	point.y = 0.15f;
	point.z = 0.40f;
	point.thetaX = 90.0f;
	point.thetaY = 90.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = 0.1f;
	point.y = 0.20f;
	point.z = 0.12f;
	point.thetaX = 90.0f;
	point.thetaY = 90.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

}

void PathPlanner::moveNavRight(vector<arm_planner::Position> &vPath)
{
	arm_planner::Position point;
	point.x = -0.35f;
	point.y = -0.05f;
	point.z = 0.40f;
	point.thetaX = 90.0f;
	point.thetaY = -20.0f;
	point.thetaZ = 0.0f;
	vPath.push_back(point);

	point.x = -0.37f;
	point.y = 0.15f;
	point.z = 0.40f;
	point.thetaX = 90.0f;
	point.thetaY = -90.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = -0.1f;
	point.y = 0.20f;
	point.z = 0.12f;
	point.thetaX = 90.0f;
	point.thetaY = 270.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	/*
	arm_planner::Position point;
	point.x = -0.35f;
	point.y = -0.05f;
	point.z = 0.40f;
	point.thetaX = 90.0f;
	point.thetaY = -20.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = -0.37f;
	point.y = 0.15f;
	point.z = 0.40f;
	point.thetaX = 90.0f;
	point.thetaY = -90.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = -0.25f;
	point.y = 0.15f;
	point.z = 0.40f;
	point.thetaX = 90.0f;
	point.thetaY = -180.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = -0.15f;
	point.y = 0.20f;
	point.z = 0.20f;
	point.thetaX = 90.0f;
	point.thetaY = -225.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = 0.0f;
	point.y = 0.20f;
	point.z = 0.20f;
	point.thetaX = 90.0f;
	point.thetaY = -270.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);

	point.x = 0.1f;
	point.y = 0.20f;
	point.z = 0.12f;
	point.thetaX = 90.0f;
	point.thetaY = -270.0f;
	point.thetaZ = 0.0f;

	vPath.push_back(point);
	*/
}

bool PathPlanner::makePathTest(float x, float y, float z)
{
	vector<float> vOrientation = {20.0f, -20.0f}; // needs -std=c++0x flaggy thing in CMake file

	if (x > 0.0f)
	{
		vOrientation.at(0) = -20.0f;
		vOrientation.at(1) = 20.0f;
	}

	int orientationIndex = -1;

	vector<arm_planner::Position> vPoints;


	vPoints.push_back(getStartPosition());

	if (x > 0.0f)
		moveToLeft(vPoints);
	else
		moveToRight(vPoints);


	// check orientations for final grasping position
	for (size_t idx = 0; idx < vOrientation.size(); ++idx)
		if (d_ikChecker.check(x, y, z, 90.0f, vOrientation.at(idx)))
		{
			ROS_INFO_STREAM("Angle: " << vOrientation.at(idx));
			orientationIndex = idx;
			break;
		}

	if (orientationIndex == -1)
		return false; // could not find (simple) valid grasping goal

	arm_planner::Position sPoint;
	sPoint.x = 0.20;
	sPoint.y = -0.10;
	sPoint.thetaX= 90.0f;
	sPoint.thetaY = 90.0f;
	sPoint.thetaZ = 0.0f;
	sPoint.approach = false;

	if (x > 0.0f) // the object is on the left side
		sPoint.thetaY = 0.0f;
	else
	{
		sPoint.x = -0.20;
		sPoint.y = -0.10;
		sPoint.thetaX= 90.0f;
		sPoint.thetaY = 0.0f;
		sPoint.thetaZ = 0.0f;
	}

	if (z < 0.1f)
		sPoint.y = -0.4f;

	sPoint.z = z;

	vPoints.push_back(sPoint);

	float rDistance = 0.10;  // distance in front of object, before approach
	float xOffset = rDistance * sin(vOrientation.at(orientationIndex) * M_PI/180);
	float yOffset = rDistance * cos(vOrientation.at(orientationIndex) * M_PI/180);

	arm_planner::Position gPoint;
	gPoint.x = x - xOffset;
	gPoint.y = y + yOffset;
	gPoint.z = z;
	gPoint.thetaX = 90.0f;
	gPoint.thetaY = vOrientation.at(orientationIndex);
	gPoint.thetaZ = 0.0f;
	gPoint.approach = false;

	vPoints.push_back(gPoint);

//	if (!findPath(vPoints))
//		return false;

	gPoint.x = x - xOffset;
	gPoint.y = y + yOffset;
	gPoint.approach = true;
	gPoint.openFingers = true;
	vPoints.push_back(gPoint);

	gPoint.x = x;
	gPoint.y = y;
	vPoints.push_back(gPoint);

	gPoint.openFingers = false;
	vPoints.push_back(gPoint);

	gPoint.z += 0.02f; // move 2 cm up
	vPoints.push_back(gPoint);

	gPoint.x = x - xOffset;
	gPoint.y = y + yOffset;
	vPoints.push_back(gPoint);

	vPoints.push_back(sPoint);

	arm_planner::Position point;

	if (x > 0.0f)
	{
		point.x = 0.35f;
		point.y = -0.05f;
		point.z = 0.40f;
		point.thetaX = 90.0f;
		point.thetaY = 20.0f;
		point.thetaZ = 0.0f;
	}
	else
	{
		point.x = -0.35f;
		point.y = -0.05f;
		point.z = 0.40f;
		point.thetaX = 90.0f;
		point.thetaY = -20.0f;
		point.thetaZ = 0.0f;
	}

	vPoints.push_back(point);

	arm_planner::pathGoal path;
	path.waypoints = vPoints;

	if (x > 0.0f)
	{
		path.left = true;
		path.right = false;
	}
	else
	{
		path.left = false;
		path.right = true;
	}

	pathClient.sendGoal(path);

	return true;
}

bool PathPlanner::makePath(float x, float y, float z)
{
//	vector<float> vOrientation = {0.0f, -20.0f, 20.0f}; // needs -std=c++0x flaggy thing in CMake file
	vector<float> vOrientation = {20.0f, -20.0f}; // needs -std=c++0x flaggy thing in CMake file

	if (x > 0.0f)
	{
		vOrientation.at(0) = -20.0f;
		vOrientation.at(1) = 20.0f;
	}

	int orientationIndex = -1;

	vector<arm_planner::Position> vPoints;

	vPoints.push_back(getStartPosition());

	if (x > 0.0f)
		moveToLeft(vPoints);
	else
		moveToRight(vPoints);

	// check orientations for final grasping position
	for (size_t idx = 0; idx < vOrientation.size(); ++idx)
		if (d_ikChecker.check(x, y, z, 90.0f, vOrientation.at(idx)))
		{
			ROS_INFO_STREAM("Angle: " << vOrientation.at(idx));
			orientationIndex = idx;
			break;
		}

	if (orientationIndex == -1)
		return false; // could not find (simple) valid grasping goal

	arm_planner::Position sPoint;
	sPoint.x = 0.20;
	sPoint.y = -0.10;
	sPoint.thetaX= 90.0f;
	sPoint.thetaY = 90.0f;
	sPoint.thetaZ = 0.0f;
	sPoint.approach = false;

	if (x > 0.0f) // the object is on the left side
		sPoint.thetaY = 0.0f;
	else
	{
		sPoint.x = -0.20;
		sPoint.y = -0.10;
		sPoint.thetaX= 90.0f;
		sPoint.thetaY = 0.0f;
		sPoint.thetaZ = 0.0f;
	}

	if (z < 0.1f)
		sPoint.y = -0.4f;

	sPoint.z = z;

	vPoints.push_back(sPoint);

	float rDistance = 0.10;  // distance in front of object, before approach
	float xOffset = rDistance * sin(vOrientation.at(orientationIndex) * M_PI/180);
	float yOffset = rDistance * cos(vOrientation.at(orientationIndex) * M_PI/180);

	arm_planner::Position gPoint;
	gPoint.x = x - xOffset;
	gPoint.y = y + yOffset;
	gPoint.z = z;
	gPoint.thetaX = 90.0f;
	gPoint.thetaY = vOrientation.at(orientationIndex);
	gPoint.thetaZ = 0.0f;
	gPoint.approach = false;

	vPoints.push_back(gPoint);

//	if (!findPath(vPoints))
//		return false;

	gPoint.x = x - xOffset;
	gPoint.y = y + yOffset;
	gPoint.approach = true;
	gPoint.openFingers = true;
	vPoints.push_back(gPoint);

	gPoint.x = x;
	gPoint.y = y;
	vPoints.push_back(gPoint);

	gPoint.openFingers = false;
	vPoints.push_back(gPoint);

	gPoint.z += 0.02f; // move 2 cm up
	vPoints.push_back(gPoint);

	gPoint.x = x - xOffset;
	gPoint.y = y + yOffset;
	vPoints.push_back(gPoint);

	vPoints.push_back(sPoint);

	if (x > 0.0f)
		moveNavLeft(vPoints);
	else
		moveNavRight(vPoints);

	arm_planner::pathGoal path;
	path.waypoints = vPoints;

	if (x > 0.0f)
	{
		path.left = true;
		path.right = false;
	}
	else
	{
		path.left = false;
		path.right = true;
	}

	pathClient.sendGoal(path);

	return true;
}

/*
bool PathPlanner::makePath(float x, float y, float z)
{
	vector<float> vOrientation = {0.0f, -45.0f, 45.0f}; // needs -std=c++0x flaggy thing in CMake file
//	vector<float> vOrientation = {0.0f};
	int orientationIndex = -1;

	vector<arm_planner::Position> vPoints;

	// check orientations for final grasping position
	for (size_t idx = 0; idx < vOrientation.size(); ++idx)
		if (d_ikChecker.check(x, y, z, 90.0f, vOrientation.at(idx)))
		{
			orientationIndex = idx;
			break;
		}

	if (orientationIndex == -1)
		return false; // could not find (simple) valid grasping goal

	arm_planner::Position sPoint;
	sPoint.x = 0.30;
	sPoint.y = -0.10;
	sPoint.z = 0.35;
	sPoint.thetaX= 90.0f;
	sPoint.thetaY = 90.0f;
	sPoint.thetaZ = 0.0f;

	if (x > 0.0f) // point is to the left side of the arm
	{
		sPoint.thetaY = 0.0f;
	}
	else if ( x < 0.0f)
	{
		sPoint.x = -0.30;
		sPoint.thetaX = 90.0f;
		sPoint.thetaY = -90.0f;
	}

	sPoint.approach = false;
	vPoints.push_back(sPoint);

	sPoint.z = z; // go to object height
	vPoints.push_back(sPoint);

	addInbetweens(vPoints);
	addInbetweens(vPoints);


	if (x > 0.0f) // add a weird rotation, this helps for better approaching on the left side
	{
		addLeftRotation(vPoints);
	}

	for (size_t idp = 0; idp < vPoints.size(); ++idp)
	{
		arm_planner::Position tPoint = vPoints.at(idp);

		if (!d_ikChecker.check(tPoint.x, tPoint.y, tPoint.z, tPoint.thetaX, tPoint.thetaY, tPoint.thetaZ))
		{
			ROS_INFO_STREAM("Fail at:" << idp << "/" << vPoints.size());
			ROS_INFO_STREAM(tPoint.x << ", " << tPoint.y  << ", " << tPoint.z  << ", " << tPoint.thetaX  << ", " << tPoint.thetaY  << ", " << tPoint.thetaZ);
			ROS_INFO_STREAM(tPoint.approach);

			return false;
		}
	}


	ROS_INFO_STREAM("Nr. of waypoints: " << vPoints.size());
	ROS_INFO_STREAM("Angle: " << vOrientation.at(orientationIndex));
	arm_planner::pathGoal path;
	path.waypoints = vPoints;

	pathClient.sendGoal(path);
	return true;
}
*/



/*
bool PathPlanner::makePath(float x, float y, float z)
{
	vector<float> vOrientation = {0.0f, -45.0f, 45.0f}; // needs -std=c++0x flaggy thing in CMake file
//	vector<float> vOrientation = {0.0f};
	int orientationIndex = -1;

	vector<arm_planner::Position> vPoints;

	// check orientations for final grasping position
	for (size_t idx = 0; idx < vOrientation.size(); ++idx)
		if (d_ikChecker.check(x, y, z, 90.0f, vOrientation.at(idx)))
		{
			orientationIndex = idx;
			break;
		}

	if (orientationIndex == -1)
		return false; // could not find (simple) valid grasping goal

	// add a mid-waypoint between starting position and grasping position
	arm_planner::Position sPoint;
/*
	sPoint.x = 0.35;
	sPoint.y = -0.10;
	sPoint.z = 0.40;
	sPoint.thetaX = 90.0f;
	sPoint.thetaY = 90.0f;
	sPoint.thetaZ = 0.0f;
 */
/*
	sPoint.x = 0.30;
	sPoint.y = -0.10;
	sPoint.z = 0.35;
	sPoint.thetaX= 90.0f;
	sPoint.thetaY = 90.0f;
	sPoint.thetaZ = 0.0f;
	//vPoints.push_back(sPoint);
//

	if (x > 0.0f) // point is to the left side of the arm
	{
		sPoint.thetaY = 0.0f;
	}

	sPoint.approach = false;
	vPoints.push_back(sPoint);

	float rDistance = 0.10;  // distance in front of object, before approach
	float xOffset = rDistance * sin(vOrientation.at(orientationIndex) * M_PI/180);
	float yOffset = rDistance * cos(vOrientation.at(orientationIndex) * M_PI/180);


//	if (d_ikChecker.check(midPoint.x, midPoint.y, midPoint.z, midPoint.thetaX, midPoint.thetaY, midPoint.thetaZ))
//		vPoints.push_back(midPoint);
	//else
	//	return false;


	arm_planner::Position mPoint = createPoint(x - xOffset, y + yOffset, z, 90.0f, vOrientation.at(orientationIndex));
	vPoints.push_back(mPoint);

	// open the fingers
	arm_planner::Position gPoint = vPoints.at(vPoints.size()-1);
	gPoint.approach = true;
	gPoint.openFingers = true;
	vPoints.push_back(gPoint);

	mPoint = createPoint(x, y, z, 90.0f, vOrientation.at(orientationIndex));
	mPoint.approach = true;
	mPoint.openFingers = true;
	vPoints.push_back(mPoint);

	// close fingers
	gPoint = vPoints.at(vPoints.size() - 1);
	gPoint.approach = true;
	gPoint.openFingers = false;
	vPoints.push_back(gPoint);

	float moveUp = 0.02f;

	mPoint = createPoint(x, y, z + moveUp, 90.0f, vOrientation.at(orientationIndex));
	mPoint.approach = true;
	mPoint.openFingers = false;
	vPoints.push_back(mPoint);

	// move back
	mPoint = createPoint(x - xOffset, y + yOffset, z + moveUp, 90.0f, vOrientation.at(orientationIndex));
	mPoint.approach = true;
	mPoint.openFingers = false;
	vPoints.push_back(mPoint);

	vPoints.push_back(sPoint);
//	midPoint.approach = false;

//	if (d_ikChecker.check(midPoint.x, midPoint.y, midPoint.z, midPoint.thetaX, midPoint.thetaY, midPoint.thetaZ))
//		vPoints.push_back(midPoint);

	addInbetweens(vPoints);
	addInbetweens(vPoints);
//	addInbetweens(vPoints);
//	addInbetweens(vPoints);

	if (x > 0.0f) // add a weird rotation, this helps for better approaching on the left side
	{
		addLeftRotation(vPoints);
	}
	for (size_t idp = 0; idp < vPoints.size(); ++idp)
	{
		arm_planner::Position tPoint = vPoints.at(idp);

		if (!d_ikChecker.check(tPoint.x, tPoint.y, tPoint.z, tPoint.thetaX, tPoint.thetaY, tPoint.thetaZ))
		{
			ROS_INFO_STREAM("Fail at:" << idp << "/" << vPoints.size());
			ROS_INFO_STREAM(tPoint.x << ", " << tPoint.y  << ", " << tPoint.z  << ", " << tPoint.thetaX  << ", " << tPoint.thetaY  << ", " << tPoint.thetaZ);
			ROS_INFO_STREAM(tPoint.approach);

			return false;
		}

	}


	ROS_INFO_STREAM("Nr. of waypoints: " << vPoints.size());
	ROS_INFO_STREAM("Angle: " << vOrientation.at(orientationIndex));
	arm_planner::pathGoal path;
	path.waypoints = vPoints;

	pathClient.sendGoal(path);
	return true;
}
*/

