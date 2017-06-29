#include "objectDetection.h"

void ObjectDetection::writeToMemory(vector<DetectedObject> objects)
{	
	stringstream jsonmsg;

	jsonmsg << "[";

	for (size_t idx = 0; idx < objects.size(); ++idx)
	{
		cout << "Object name: " << objects.at(idx).objectName << ", (" << objects.at(idx).x << ", " 
			<< objects.at(idx).y << ", " << objects.at(idx).z << ")\n";	

		jsonmsg << "{\"objectName\" : \"" << objects.at(idx).objectName << "\", " <<
					 "\"x\" : \"" << objects.at(idx).x << "\", " << 
					 "\"y\" : \"" << objects.at(idx).y << "\", " <<
					 "\"z\" : \"" << objects.at(idx).z << "\"}";
		
		if (idx != objects.size() - 1)
			jsonmsg << ",";
	}

	jsonmsg << "]";

	borg_pioneer::MemorySrv srv;	
	srv.request.name = "detected_objects";
	srv.request.timestamp = ros::Time::now();
	srv.request.json = jsonmsg.str();
	client_writer.call(srv);

	// sync operation, to tell its is ok to read.
	srv.request.name = "ready_to_read_objects";
	srv.request.timestamp = ros::Time::now();
	jsonmsg.clear();
	jsonmsg.str("");

	jsonmsg << "{\"ready\":\"True\"}";
	srv.request.json = jsonmsg.str();
	client_writer.call(srv);
//	jsonmsg.str("");
}