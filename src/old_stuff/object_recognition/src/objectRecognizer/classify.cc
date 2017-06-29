#include "objectRecognizer.ih"

/*
 * Compare all the objects that were seen in an image to the classes in the database and store the objects that were recognized
 * TODO: write recognized objects to memory
 */

 void ObjectRecognizer::classify()
{
	cout << "run has " << d_inputDescriptors.size() << " objects" << endl;
	
	//to store the objects that are recognized, and with what distance. This should be written to memory rather than be stored.
	vector<string> objects;
	vector<double> objectScores;
	
	//For all the clusters in the last image...
	for (size_t jdx = 0; jdx != d_inputDescriptors.size(); ++jdx)
	{
		string bestClass = ""; //init the closest match to nothing
		double bestDistToClass = d_threshold; //init the minimal distance to the class to the maximum

		//Compare the object to each class in the database
		for (db_it it = d_shotDatabase.begin(); it != d_shotDatabase.end(); ++it)
		{
			// If the object's size falls outside of the class' boundary, continue to the next class
			if (d_dimDatabase[it->first][0] > (double)d_inputSizes[jdx][0]  || d_dimDatabase[it->first][1] < (double)d_inputSizes[jdx][0] ||
				d_dimDatabase[it->first][2] > (double)d_inputSizes[jdx][1]  || d_dimDatabase[it->first][3] < (double)d_inputSizes[jdx][1])
				//continue;
			{}

			double bestDistToInstance = d_threshold; //init minimal distance to class instance to the maximum

			double allMinDists[d_inputDescriptors[jdx]->size()];
			for (size_t idx = 0; idx != d_inputDescriptors[jdx]->size(); ++idx)
				allMinDists[idx] = 1.0;

			//For all instances of the class...
			for (v_it it2 = it->second.begin(); it2 != it->second.end(); ++it2)
			{
				//Create a search tree and set the current instance as its cloud
				search::KdTree<descriptorType>::Ptr objTree (new search::KdTree<descriptorType>);
				objTree->setInputCloud(*it2);

//				double distSum = 0.0; //initialize cumulative distance for the instance

				//For all points in the object cluster...
				for (size_t idx = 0; idx != d_inputDescriptors[jdx]->size(); ++idx)
				{
					//Find the point of the class instance that has the least (euclidian) distance to the point 
					vector<int> bearNecessities(1);
					vector<float> dist(1);
					int greatSuccess = objTree->nearestKSearch(d_inputDescriptors[jdx]->at(idx), 1, bearNecessities, dist);

					if (greatSuccess)
						if (dist[0] < allMinDists[idx])
							allMinDists[idx] = dist[0];

/*
					//if we found a nearest match add the distance to the total distance
					//in case of failure add the maximum distance
					if (greatSuccess)
						distSum += dist[0];
					else
						distSum += 1.0; //maximum distance for SHOT descriptors is 1
*/				}

				//compute the average distance, rather than the total distance
//				distSum /= d_inputDescriptors[jdx]->size();

				//check whether the current instance is a better match than the previously best match. If so, set it.
//				if (distSum < bestDistToInstance)
//					bestDistToInstance = distSum;
			}
			double avgDist = 0.0;
			for (size_t idx = 0; idx != d_inputDescriptors[jdx]->size(); ++idx)
				avgDist += allMinDists[idx];

			bestDistToInstance = avgDist / d_inputDescriptors[jdx]->size();

			//check whether the current class is a better match than the previous one. If so, set it.
			if (bestDistToInstance < bestDistToClass)
			{
				bestDistToClass = bestDistToInstance;
				bestClass = it->first;
			}
		}

		//If the best matching class performs well enough, store it.
		if (bestDistToClass < d_threshold)
		{
			borg_pioneer::MemorySrv srv;
			srv.request.timestamp = ros::Time::now();
			srv.request.name = "recognized_object";
			char jsonmsg[255];
			sprintf(jsonmsg, "{\"name\": \"%s\", \"x\": %f, \"y\": %f, \"z\": %f, \"h\": %f, \"baseframe\":\"mico_base_link\"}", bestClass.c_str(), d_inputPositions[jdx][0],d_inputPositions[jdx][1], d_inputPositions[jdx][2], d_inputPositions[jdx][3]);
			srv.request.json = std::string(jsonmsg);
			d_clientWriter.call(srv);

			objects.push_back(bestClass);
			objectScores.push_back(bestDistToClass);
		}
	}

	/* Used to save the results for experiments. Assumes that we can recognize only 1 object, and returns the best matching object.
	double best = 1.0;
	string bestClass = "nothing";
	for (size_t idx = 0; idx != objectScores.size(); ++idx)
	{
		if (objectScores[idx] < best)
		{
			best = objectScores[idx];
			bestClass = objects[idx];
		}
	}

	++results[bestClass];
	*/

	// print some feedback on what objects are recognized
	cout << "\033[1;31mI think I see ";
	switch(objects.size())
	{
		case 0: 
			cout << "no objects I should recognize :(" << endl;
			break;
		case 1:
			cout << "a " << objects[0] << "!" << endl;
			break;
		default:
			for(size_t idx = 0; idx < objects.size()-2; ++idx)
				cout << "a " << objects[idx] << ", ";
			cout << "a " << objects[objects.size()-2] << " and a " << objects[objects.size()-1] << "!" << endl; 
	}

	cout << "\033[0m";

	for(size_t idx = 0; idx != objects.size(); ++ idx)
		cout << objects[idx] << ": average descriptor distance is " << roundf(objectScores[idx] * 1000) / 1000 << endl;
}
