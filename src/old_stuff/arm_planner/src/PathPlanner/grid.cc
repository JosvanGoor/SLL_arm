#include <arm_planner/pathPlanner.h>

void PathPlanner::createGrid()
{
	float wayPointRadius = 0.05;

	for (float idx = -0.40; idx <= 0.40; idx += wayPointRadius)
	{
		for (float idy = 0.05; idy >= -0.60; idy -= wayPointRadius)
		{
			for (float idz = -0.20; idz <= 0.70; idz += wayPointRadius)
			{
				Point wayPoint;
				wayPoint.x = idx;
				wayPoint.y = idy;
				wayPoint.z = idz;

				if (sqrt(pow(idx,2) + pow(idy,2) + pow(idz,2)) > 0.3)
					d_grid->push_back(wayPoint);
			}
		}
	}

	ROS_INFO_STREAM("Size: " << d_grid->points.size());
	counter = 0;
}

vector<int> PathPlanner::checkPointsInRange(Point search)
{
	//float radius = 0.06f; // radius of point to search
	float radius = 0.08f;
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	kdtree.radiusSearch(search, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	return pointIdxRadiusSearch;
}

float PathPlanner::getDistance(arm_planner::Position sPoint, arm_planner::Position gPoint)
{
	return sqrt(pow(sPoint.x - gPoint.x, 2) + pow(sPoint.y - gPoint.y, 2) + pow(sPoint.z - gPoint.z, 2));
}

void PathPlanner::addPoint(vector< vector<arm_planner::Position> > &vPath, vector<arm_planner::Position> vWaypoints, arm_planner::Position nPoint, const arm_planner::Position &gPoint, float prevDistance)
{
	++counter;
	Point pcPoint;
	pcPoint.x = nPoint.x;
	pcPoint.y = nPoint.y;
	pcPoint.z = nPoint.z;

	vector<int> vIndex = checkPointsInRange(pcPoint);

	for (size_t idx = 0; idx < vIndex.size(); ++idx) // check the points in the radius, and pick the one that leads to a closer path
	{
		arm_planner::Position tPoint;
		tPoint.x = d_grid->points.at(vIndex.at(idx)).x;
		tPoint.y = d_grid->points.at(vIndex.at(idx)).y;
		tPoint.z = d_grid->points.at(vIndex.at(idx)).z;

		float currentDistance = getDistance(gPoint, tPoint);

		if (currentDistance < prevDistance and currentDistance > 0.001f) // and d_ikChecker.check(tPoint.x, tPoint.y, tPoint.z, tPoint.thetaX, tPoint.thetaY, tPoint.thetaZ))
		{
			vWaypoints.push_back(tPoint);
			prevDistance = currentDistance;
			if (currentDistance < 0.06f)
				vPath.push_back(vWaypoints);
			else
				addPoint(vPath, vWaypoints, tPoint, gPoint, prevDistance);
		}
	}
}

bool PathPlanner::findPath(vector<arm_planner::Position> &vPoints)
{
	arm_planner::Position sPoint;
	sPoint = vPoints.at(3);

	arm_planner::Position gPoint;
	gPoint = vPoints.at(vPoints.size() - 1);

	bool foundPath = false;

	vector <arm_planner::Position> vPath;

	while (!foundPath)
	{
		arm_planner::Position nPoint;

		if (vPath.size() == 0)
		{
			vPath.push_back(sPoint); // add the starting point
			nPoint = sPoint;
		}
		else
			nPoint = vPath.at(vPath.size()- 1);

		Point pcPoint;
		pcPoint.x = nPoint.x;
		pcPoint.y = nPoint.y;
		pcPoint.z = nPoint.z;

		vector<int> vIndex = checkPointsInRange(pcPoint);

		float prevDistance = 99.0f;
		size_t lIndex = 0;

		for (size_t idx = 0; idx < vIndex.size(); ++idx) // check the points in the radius, and pick the one that leads to a closer path
		{
			arm_planner::Position tPoint;
			tPoint.x = d_grid->points.at(vIndex.at(idx)).x;
			tPoint.y = d_grid->points.at(vIndex.at(idx)).y;
			tPoint.z = d_grid->points.at(vIndex.at(idx)).z;

			float currentDistance = getDistance(gPoint, tPoint);

			if (currentDistance < prevDistance and currentDistance > 0.001f) // and d_ikChecker.check(tPoint.x, tPoint.y, tPoint.z, tPoint.thetaX, tPoint.thetaY, tPoint.thetaZ))
			{
				prevDistance = currentDistance;
				lIndex = idx;
			}
		}

		if (vIndex.size() == 0)
		{
			return false;
		}

		arm_planner::Position tPoint;
		tPoint.x = d_grid->points.at(vIndex.at(lIndex)).x;
		tPoint.y = d_grid->points.at(vIndex.at(lIndex)).y;
		tPoint.z = d_grid->points.at(vIndex.at(lIndex)).z;
		vPath.push_back(tPoint);

		if (getDistance(gPoint, tPoint) <= 0.08f)
			foundPath = true;
	}

	float rotation = sPoint.thetaY - gPoint.thetaY;
	float stepRotation = rotation / vPath.size();

	float step = 0.0f;

	for (size_t idx = 0; idx < vPath.size(); ++idx)
	{
		vPath.at(idx).thetaX = 90.0f;
		vPath.at(idx).thetaZ = 0.0f;

		step += stepRotation;
		vPath.at(idx).thetaY = sPoint.thetaY - step;
	}

	for (size_t idx = 0; idx < vPoints.size(); ++idx)
	{
		arm_planner::Position tPoint = vPoints.at(idx);
		if (!d_ikChecker.check(tPoint.x, tPoint.y, tPoint.z, tPoint.thetaX, tPoint.thetaY, tPoint.thetaZ))
		{
			ROS_INFO_STREAM("Failed Point: " << idx << ": "<< tPoint.x << ", " << tPoint.y << ", " <<  tPoint.z << ", " << tPoint.thetaX << ", " << tPoint.thetaY << ", " << tPoint.thetaZ);
			return false;
		}
	}

	vPoints.insert(vPoints.end() - 1, vPath.begin(), vPath.end());

	addInbetweens(vPoints);

	return true;
}

/*
bool PathPlanner::findPath(vector<arm_planner::Position> &vPoints, vector<arm_planner::Position> &vPathReturn)
{
	arm_planner::Position sPoint;
	sPoint = vPoints.at(0);

	arm_planner::Position gPoint;
	gPoint = vPoints.at(1); // there should only be 2 points in this vector

	float goalDistance = getDistance(sPoint, gPoint);

	ROS_INFO_STREAM("Distance: " << goalDistance);

	vector<arm_planner::Position> vWaypoints;

	bool foundGoal = false;

	vector< vector<arm_planner::Position> > vPath;

	addPoint(vPath, vWaypoints, sPoint, gPoint);

	ROS_INFO_STREAM("Counter: "  << counter);
	ROS_INFO_STREAM("Paths: "  << vPath.size());

	size_t indexPath = 0;
	vector< vector<arm_planner::Position> > vCorrectPaths;

	volatile bool flag = false;

	size_t counter1 = 0;
	size_t maxCounter = 100;

	#pragma omp parallel for shared(flag)
	for (size_t idp = 0; idp < vPath.size(); ++idp)
	{
		 if(flag) continue;

		vector<arm_planner::Position> vCurrentPath = vPath.at(idp);
	//	ROS_INFO_STREAM("Path size: " << vCurrentPath .size());

		float rotation = sPoint.thetaY - gPoint.thetaY;
		float stepRotation = rotation / vCurrentPath .size();

		bool correct = false;

		float step = 0.0f;

		for (size_t idx = 0; idx < vCurrentPath .size(); ++idx)
		{
			step += stepRotation;
			vCurrentPath.at(idx).thetaX = 90.0f;
			vCurrentPath.at(idx).thetaY = sPoint.thetaY - step;
			vCurrentPath.at(idx).thetaZ = 0.0f;
			correct = d_ikChecker.check(vCurrentPath .at(idx).x, vCurrentPath .at(idx).y, vCurrentPath .at(idx).z, vCurrentPath .at(idx).thetaX, vCurrentPath .at(idx).thetaY);
	//		ROS_INFO_STREAM("x: " << vCurrentPath .at(idx).x << ", y: " << vCurrentPath .at(idx).y << ", z: " << vCurrentPath .at(idx).z << ", thetaX: " << vCurrentPath .at(idx).thetaX << ", thetaY: " << vCurrentPath .at(idx).thetaY);

			if (!correct)
				break;
		}

	//	indexPath = idp;

		if (correct = true)
		{
			bool stillCorrect = true;
			addInbetweens(vCurrentPath);
			for (size_t idx = 0; idx < vCurrentPath .size(); ++idx)
			{
				if (!d_ikChecker.check(vCurrentPath .at(idx).x, vCurrentPath .at(idx).y, vCurrentPath .at(idx).z, vCurrentPath .at(idx).thetaX, vCurrentPath .at(idx).thetaY))
					stillCorrect = false;

				if (!stillCorrect)
					break;
			}

			if (stillCorrect)
			{
				#pragma omp critical
				{
					vCorrectPaths.push_back(vCurrentPath);
					++counter1;

					if (counter1 == maxCounter)
						flag = true;
				}
			}
		}
	}

	size_t lIndex = 0;
	size_t lSize = 2000000;

	ROS_INFO_STREAM("Size: " << vCorrectPaths.size());
	for (size_t idx = 0; idx < vCorrectPaths.size(); ++idx)
	{
		if (vCorrectPaths.size() < lSize)
		{
			lIndex = idx;
			lSize = vCorrectPaths.size();
		}
	}

	vPathReturn = vCorrectPaths.at(lIndex);

	return true;
}
*/
/*
bool PathPlanner::findPath(vector<arm_planner::Position> &vPoints)
{
	arm_planner::Position sPoint;
	sPoint = vPoints.at(0);

	arm_planner::Position gPoint;
	gPoint = vPoints.at(1); // there should only be 2 points in this vector

	float goalDistance = getDistance(sPoint, gPoint);

	ROS_INFO_STREAM("Distance: " << goalDistance);

	vector<arm_planner::Position> vWaypoints;

	bool foundGoal = false;

	while (!foundGoal)
	{
		arm_planner::Position wPoint;

		if (vWaypoints.size() == 0)
			wPoint = sPoint;  // the the starting position as the first waypoint, starting point is not in the waypoint list
		else
			wPoint = vWaypoints.at(vWaypoints.size() - 1); // take the last waypoint

		Point pcPoint;
		pcPoint.x = wPoint.x;
		pcPoint.y = wPoint.y;
		pcPoint.z = wPoint.z;

		vector<int> vIndex = checkPointsInRange(pcPoint);

		size_t lowIndex = 0;
		float lowestDistance = 99.0f;

		for (size_t idx = 0; idx < vIndex.size(); ++idx) // check the points in the radius, and pick the one that leads to a closer path
		{
			arm_planner::Position tPoint;
			tPoint.x = d_grid->points.at(vIndex.at(idx)).x;
			tPoint.y = d_grid->points.at(vIndex.at(idx)).y;
			tPoint.z = d_grid->points.at(vIndex.at(idx)).z;
			tPoint.thetaX = 90.0f;
			tPoint.thetaY = 90.0f;
			tPoint.thetaZ = 0.0f;

			float currentDistance = getDistance(gPoint, tPoint);

			if (currentDistance < lowestDistance and currentDistance > 0.001f) // and d_ikChecker.check(tPoint.x, tPoint.y, tPoint.z, tPoint.thetaX, tPoint.thetaY, tPoint.thetaZ))
			{
				lowIndex = idx;
				lowestDistance = currentDistance;
			}
		}

		if (lowestDistance == 99.0f)
			return false;

		// the best point
		Point ntPoint; // PCL point;
		ntPoint = d_grid->points.at(vIndex.at(lowIndex));

		arm_planner::Position nPoint; // new point
		nPoint.x = ntPoint.x;
		nPoint.y = ntPoint.y;
		nPoint.z = ntPoint.z;

		vWaypoints.push_back(nPoint);

		ROS_INFO_STREAM("points: " << nPoint.x << " , " << nPoint.y << ", " << nPoint.z);
		ROS_INFO_STREAM("Distance: " << getDistance(nPoint, gPoint));

		if (getDistance(nPoint, gPoint) < 0.06f)
			foundGoal = true;
	}


	return true;
}
*/
