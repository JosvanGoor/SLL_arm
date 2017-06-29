#include "colorblob.ih"

void Colorblob::writeToMemory(vector<cv::Rect>& obs, string name)
{
	boost::property_tree::ptree pt_observations;
	int index = 1;	
	
	// Sort observations by size
	sortObservations(obs);

	for (vector<cv::Rect>::iterator it = obs.begin(); it != obs.end(); ++it)
	{
		//Create tree for obs
		boost::property_tree::ptree pt_obs;
		pt_obs.put("x", it->x);
		pt_obs.put("y", it->y);
		pt_obs.put("width", it->width);
		pt_obs.put("height", it->height);
		
		//Convert tree to JSON buffer
		ostringstream pt_obs_buffer;
		boost::property_tree::write_json(pt_obs_buffer, pt_obs, false);
		
		//Add obs to observations
		string index_str = boost::lexical_cast<string>(index);
		pt_observations.put(index_str, pt_obs_buffer.str());

		++index;
	}
	
	//Convert final tree to Python dict
	ostringstream pt_observations_buffer;
	boost::property_tree::write_json(pt_observations_buffer, pt_observations, false);
	string observations_dict = pt_observations_buffer.str();

	//Create service
	alice_msgs::MemorySrv srv;
	srv.request.name = name;
	srv.request.timestamp = ros::Time::now();
	srv.request.json = observations_dict;

	//ROS_INFO(observations_dict.c_str());

	d_memory_client.call(srv);
	//	ROS_INFO("Memory error!!");
}
