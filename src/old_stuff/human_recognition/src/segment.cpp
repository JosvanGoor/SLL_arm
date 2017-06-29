#include <human_recognition/segment.h>

// Variables to keep track of data streams
PCLPointCloudPtr camera_cloud;

// For publishing PointCloud data. We don't want to create a new publisher every time
ros::Publisher personCloud_pub;

// Memory service client
ros::ServiceClient memory_client;
ros::ServiceClient memory_client_write;

// If you declare a listener in the callback function, then it loses the history of the tf data.
// It therefore starts nagging about "Can't find frame_id 'base_link'" and stuff. So a global listener is required.
// The listener can't be initialized here, as ros::init has to be called first.
tf::TransformListener* listener;

SegmentHumans::SegmentHumans(bool publish_cloud)
{
    d_base_frame = "/base_link";
    d_human_width = 0.8;
    d_human_depth = 1.2;
    d_human_height = 2.5;
    leg_height = 1.0;
    voxel_size = 0.01;
    display_histogram = true;
    mode = FILTERING;
    image_counter = 0;
    max_images = 8;
    gap_limit = 2;
    image_width = 200;
    frame_iterator = 0;
    n_frames = 50;
    frame_time = 0;
    d_publish_cloud = publish_cloud;
    image_ratio = d_human_width / d_human_height;
}

SegmentHumans::~SegmentHumans()
{
    foreach (Human* human, d_humans)
    {
	    delete human;
    }
    delete image_array_filt;
    delete image_array_clust;
};

void SegmentHumans::initImageArrays()
{
    image_height = (int)(image_width / image_ratio);    
    
    // Initialise arrays
    image_array_filt = new rgb[image_width*image_height];
    image_array_clust = new rgb[image_width*image_height];
}

void SegmentHumans::readHumanDetectionsFromMemory(std::string message)
{
	// Clear humans
	foreach (Human* human, d_humans)
	{
		delete human;	// DIE!!!!
	}
	d_humans.clear();

	// Check if there is any data
	if (message.size() == 0 || strcmp(message.c_str(), "null") == 0)
	{
		// No data
		return;
	}

	//Get rid of unwanted info
	message = message.substr(1, message.size() - 2);		// Remove outer brackets
	size_t first_occurrence_opening_bracket = message.find_first_of('{');
	size_t last_occurrence_closing_bracket = message.find_last_of('}');
	message = message.substr(first_occurrence_opening_bracket, last_occurrence_closing_bracket - first_occurrence_opening_bracket + 1);

	// Separate responses, message can contain multiple responses
	std::vector<std::string> responses;
	boost::char_separator<char> sep("}");
	boost::tokenizer< boost::char_separator<char> > tokens(message, sep);
	int count = 0;
	for (boost::tokenizer<boost::char_separator<char> >::iterator it = tokens.begin(); it != tokens.end(); ++it)
	{
		std::string single_response = *it;
		single_response += '}';	// Add the closing bracket that was first removed by splitting the response
		if (count != 0) single_response = single_response.substr(2, single_response.size() - 2);	// Remove seperating comma between single responses
		responses.push_back(single_response);
		++count;
	}

	// Decode data
	foreach (std::string response, responses)
	{
		ptree pt;
		std::stringstream ss; 
		ss << response;
		boost::property_tree::read_json(ss, pt);

		// Check timestamp
		double stamp = pt.get<double>("time");
		double now = ros::Time::now().toSec();
		if ((now - stamp) > 0.2)    // was 0.2
		{
			// This observation was too long ago -> skip
			continue;
		}

		// Get human info and save
		Human* human = new Human();	// I have the power of God!
		human->x = pt.get<double>("x_actual");
		human->y = pt.get<double>("y_actual");
		human->z = pt.get<double>("z_actual");
		human->x_pix = pt.get<double>("x");
		human->y_pix = pt.get<double>("depth");
        human->max_channels = {0.0, 0.0, 0.0};
        human->min_channels = {256.0, 256.0, 256.0};
		d_humans.push_back(human);	// Sorry human ...
	}
}

void SegmentHumans::readFromMemory()
{
	// Create request
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "HumanDetector";
	srv.request.params = "";

	// Make the call
	memory_client.call(srv);

	// Read message
	readHumanDetectionsFromMemory(srv.response.json);
    
    //ROS_INFO_STREAM("MEMORY: " << srv.response.json);
}


void SegmentHumans::writeToMemory()
{
	// Create JSON property dict, it will have the following form:
	// {
	// 		1: {'x': x_1, 'y': y_1, ...},	//Person 1
	//		2: {'x': x_2, 'y': y_2, ...}	//Person 2, etc.
	// }
	boost::property_tree::ptree pt_prop_dict;
    int person_nr = 1;
	foreach (Human* human, d_humans)
	{
		// Create person_dict
		boost::property_tree::ptree pt_person_dict;
		pt_person_dict.put("height", human->height);
		pt_person_dict.put("x", human->x);
		pt_person_dict.put("y", human->y);
		pt_person_dict.put("z", human->z);
		pt_person_dict.put("legs_black", human->legs_black);
		pt_person_dict.put("legs_white", human->legs_white);
		pt_person_dict.put("legs_grey", human->legs_grey);
		pt_person_dict.put("legs_red", human->legs_red);
		pt_person_dict.put("legs_green", human->legs_green);
		pt_person_dict.put("legs_blue", human->legs_blue);
		pt_person_dict.put("legs_yellow", human->legs_yellow);
		pt_person_dict.put("upper_black", human->upper_black);
		pt_person_dict.put("upper_white", human->upper_white);
		pt_person_dict.put("upper_grey", human->upper_grey);
		pt_person_dict.put("upper_red", human->upper_red);
		pt_person_dict.put("upper_green", human->upper_green);
		pt_person_dict.put("upper_blue", human->upper_blue);
		pt_person_dict.put("upper_yellow", human->upper_yellow);
		pt_person_dict.put("x_pix", static_cast<int>(human->x_pix));
		pt_person_dict.put("y_pix", static_cast<int>(human->y_pix));
		//ROS_INFO("----------legs:\n black: %f\n white: %f\n grey: %f\n red: %f\n green: %f\n blue: %f", human->legs_black, human->legs_white, human->legs_grey, human->legs_red, human->legs_green, human->legs_blue);
		std::ostringstream buf_person_dict;
		boost::property_tree::write_json(buf_person_dict, pt_person_dict, false);

		// Add person dict to property dict
		std::string person_nr_str = boost::lexical_cast<std::string>(person_nr);
		pt_prop_dict.put(person_nr_str, buf_person_dict.str());
		++person_nr;
	}
	std::ostringstream buf_prop_dict;
	boost::property_tree::write_json(buf_prop_dict, pt_prop_dict, false);
	std::string prop_dict = buf_prop_dict.str();

	// Output findings to memory
	borg_pioneer::MemorySrv srv;
	srv.request.name = "humans_found";
	srv.request.timestamp = ros::Time::now();
	srv.request.json = prop_dict;

	//ROS_INFO(prop_dict.c_str());

	// Make the call
	if (!memory_client_write.call(srv))
        ROS_WARN("Could not write to memory, damnit");

}

bool SegmentHumans::availabilityChecks(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
	// Read memory
	readFromMemory();

	// Check if there are humans detected
	if (d_humans.size() == 0)
	{
		//ROS_INFO("No humans detected");
		return false;
	}

	// Check if transform is available
	try
	{
		tf::StampedTransform transform;
		listener->lookupTransform(d_base_frame, cloudMsg->header.frame_id, cloudMsg->header.stamp,transform);
	}
	catch (tf::LookupException &ex)
	{
		// No transform available
		ROS_INFO("No transform available");
		return false;
	}
	return true;
}

PCLPointCloudPtr SegmentHumans::getCloud(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
	// Transform PointCloud to base frame
	sensor_msgs::PointCloud2 cloudMsg_transformed;
	pcl_ros::transformPointCloud(d_base_frame, *cloudMsg, cloudMsg_transformed, *listener);

	// Convert PointCloud2 to PCL::PointCloud
	PCLPointCloudPtr pclCloud (new PCLPointCloud);
	pcl::fromROSMsg(cloudMsg_transformed, *pclCloud);
	
	
	
	return pclCloud;
}

void SegmentHumans::processCam(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
    //ROS_INFO("Received data from cam: 1");
    if(availabilityChecks(cloudMsg))
    { 
        camera_cloud = getCloud(cloudMsg);
        
        double iterate_time = ros::Time::now().toSec();
                
        findHumans(camera_cloud);

        // Calculate frames per second
        frame_time += (ros::Time::now().toSec() - iterate_time);
        frame_iterator++;
        if(frame_iterator == n_frames - 1)
        {
            double fps = n_frames / frame_time;
            frame_iterator = 0;
            frame_time = 0;
            ROS_INFO("----> FPS: \t%.2f", fps);
            
            // debug: h distribution:
            std::stringstream test;
            int h0_25 = 0;
            int h25_50 = 0;
            int h50_75 = 0;
            int h75_100 = 0;
            int average = 0;
            int h_min = h_values[0];
            int h_max = h_values[0];
            
            for(unsigned int i=0; i < h_values.size(); i++)
            {
                if(h_values[i] < h_min)
                { h_min = h_values[i]; }
                if(h_values[i] > h_max)
                { h_max = h_values[i]; }
                
                if(h_values[i] < 90)
                { h0_25++; }
                else if(h_values[i] < 180)
                { h25_50++; }
                else if(h_values[i] < 270)
                { h50_75++; }
                else
                { h75_100++; }
                
                average += h_values[i];
            }
            average = average / h_values.size();
            h0_25 = h0_25 * 100 / h_values.size();
            h25_50 = h25_50 * 100 / h_values.size();
            h50_75 = h50_75 * 100 / h_values.size();
            h75_100 = h75_100 * 100 / h_values.size();
            
            test << "\n0-25: \t\t" << h0_25 << "%\n"
                 << "25-50: \t\t" << h25_50 << "%\n"
                 << "50-75: \t\t" << h50_75 << "%\n"
                 << "75-100: \t" << h75_100 << "%\n"
                 << "average: \t" << average << "\n"
                 << "min: \t\t" << h_min << ", max: \t" << h_max;
                 
            ROS_INFO(test.str().c_str());
            
            h_values.clear();
        }
    }
}

void SegmentHumans::findHumans(PCLPointCloudPtr pclCloud)
{
	// Read memory
	readFromMemory();

	// Discard floor using voxel grid filter
	PCLPointCloudPtr pclCloud_filtered (new PCLPointCloud);
	pcl::VoxelGrid<Point> voxelgrid_filter;
	voxelgrid_filter.setInputCloud(pclCloud);
	voxelgrid_filter.setLeafSize(voxel_size, voxel_size, voxel_size);  // Leaf size: was 0.01 m
	voxelgrid_filter.setFilterFieldName("z");
	voxelgrid_filter.setFilterLimits(0.1, d_human_height);			// Everything < 0.1m is considered floor
	voxelgrid_filter.filter(*pclCloud_filtered);

	// Create new PointCloud that contains the segmented persons
	// cloud for method 1
	PCLPointCloudPtr pclCloud_segmented_filt (new PCLPointCloud);
	pclCloud_segmented_filt->header.frame_id = d_base_frame;
	pclCloud_segmented_filt->header.stamp = ros::Time::now().toSec();
	pclCloud_segmented_filt->height = 1;
	pclCloud_segmented_filt->is_dense = true;
	// cloud for method 2
	PCLPointCloudPtr pclCloud_segmented_clust (new PCLPointCloud);
	pclCloud_segmented_clust->header.frame_id = d_base_frame;
	pclCloud_segmented_clust->header.stamp = ros::Time::now().toSec();
	pclCloud_segmented_clust->height = 1;
	pclCloud_segmented_clust->is_dense = true;
	
	/*std::stringstream printer;
	printer << "bla";
	ROS_INFO(printer.str().c_str());*/

	foreach (Human* human, d_humans)
	{
        rgb black;
        black.r = 0; black.g = 0; black.b = 0;
	    // Fill image arrays by default with black
        for(int i=0; i < image_width; i++)
        {
            for(int j=0; j < image_height; j++)
            {
                image_array_filt[j*image_width + i] = black;
                image_array_clust[j*image_width + i] = black;
            }
        }	
	
		// Clear points belonging to humans
		human->points.clear();

		// Determine boundaries
		double x_min = human->x - d_human_depth / 2;
		double x_max = human->x + d_human_depth / 2;
		double y_min = human->y - d_human_width / 2;
		double y_max = human->y + d_human_width / 2;
		double z_min = human->z - d_human_height / 2;
		double z_max = human->z + d_human_height / 2;

		//ROS_INFO_STREAM("Y: " << human->y << ", Y_MIN: " << y_min << ", Y_MAX: " << y_max);

		// Use a pass through filter to filter depth (filter x)
		PCLPointCloudPtr pclCloud_filtered_x (new PCLPointCloud);
		pcl::PassThrough<Point> passthrough_filter_x;
		passthrough_filter_x.setInputCloud(pclCloud_filtered);
		passthrough_filter_x.setFilterFieldName("x");
		passthrough_filter_x.setFilterLimits(x_min, x_max);
		passthrough_filter_x.filter(*pclCloud_filtered_x);

		// Use a pass through filter to filter width (filter y)
		PCLPointCloudPtr pclCloud_filtered_y (new PCLPointCloud);
		pcl::PassThrough<Point> passthrough_filter_y;
		passthrough_filter_y.setInputCloud(pclCloud_filtered_x);
		passthrough_filter_y.setFilterFieldName("y");
		passthrough_filter_y.setFilterLimits(y_min, y_max);
		passthrough_filter_y.filter(*pclCloud_filtered_y);

		int n_points = pclCloud_filtered_y->points.size();
		/*std::stringstream printer;
		printer << "points: " << n_points;
		ROS_INFO(printer.str().c_str());*/
		if (pclCloud_filtered_y->points.size() < 50)
		{
			ROS_INFO("Not enough points belonging to the human");
			continue;
		}
		
		if(mode == FILTERING || mode == PICTURES)
		{
		    double current_time = ros::Time::now().toSec();
		  
		    // Use a pass through filter to filter height (filter z)
		    /*PCLPointCloudPtr pclCloud_filtered_z (new PCLPointCloud);
		    pcl::PassThrough<Point> passthrough_filter_z;
		    passthrough_filter_z.setInputCloud(pclCloud_filtered_y);
		    passthrough_filter_z.setFilterFieldName("z");
		    passthrough_filter_z.setFilterLimits(0.1, d_human_height);
		    passthrough_filter_z.filter(*pclCloud_filtered_z);*/
		    
		    human->height = 0;
		    double tmp_height = 0;
            // Set the height of each human
            // Also set the extreme color values
		    for(int i=0; i < pclCloud_filtered_y->points.size(); i++)
		    {
		        pclCloud_segmented_filt->points.push_back(pclCloud_filtered_y->points[i]);
		        human->points.push_back(pclCloud_filtered_y->points[i]);
		        tmp_height = pclCloud_filtered_y->points[i].z;
		        if (tmp_height > human->height)
                { human->height = tmp_height; }
                float r = (float)pclCloud_filtered_y->points[i].r;
                float g = (float)pclCloud_filtered_y->points[i].g;
                float b = (float)pclCloud_filtered_y->points[i].b;
                // Max values
                if(r > human->max_channels.r)
                { human->max_channels.r = r; }
                if(g > human->max_channels.g)
                { human->max_channels.g = g; }
                if(b > human->max_channels.b)
                { human->max_channels.b = b; }
                // Min values
                if(r < human->min_channels.r)
                { human->min_channels.r = r; }
                if(g < human->min_channels.g)
                { human->min_channels.g = g; }
                if(b < human->min_channels.b)
                { human->min_channels.b = b; }
		    }
		  
		    // Store colours into image array
            /*for(int i=0; i < pclCloud_segmented_filt->points.size(); i++)
            {
                std::stringstream t;
                //ROS_INFO("y_min: %f\t y_max: %f",y_min,y_max);
                // normalised position
                float x_norm = (pclCloud_segmented_filt->points[i].y - y_min) / (y_max - y_min);
                float y_norm = (pclCloud_segmented_filt->points[i].z - z_min) / (z_max - z_min);
                t << "point y: " << pclCloud_segmented_filt->points[i].y;
                //ROS_INFO(t.str().c_str());
                //ROS_INFO("normalised y (x): %f", x_norm);
                
                t.str("");
                t << "x_norm: " << x_norm << " y_norm: "  << y_norm;
                //ROS_INFO(t.str().c_str());
                
                // array position
                int x_target = (int)(x_norm * image_width);
                int y_target = (int)(y_norm * image_height);
                t.str("");
                t << "x_target: " << x_target << " y_target: " << y_target;
                //ROS_INFO(t.str().c_str());
                
                
                t.str("");
                t << "array position: " << y_target*image_width + x_target;
                //ROS_INFO(t.str().c_str());
                if(x_target < image_width && x_target >= 0 && y_target < image_height && y_target >= 0) // to avoid out-of-bounds errors
                {
                    image_array_filt[y_target*image_width + x_target].r = (float)pclCloud_segmented_filt->points[i].r / 255.0;
                    image_array_filt[y_target*image_width + x_target].g = (float)pclCloud_segmented_filt->points[i].g / 255.0;
                    image_array_filt[y_target*image_width + x_target].b = (float)pclCloud_segmented_filt->points[i].b / 255.0;
                }
            }
            //ROS_INFO("done writing array");
            
            // Filter out objects next to the main human
            // Human position in array position:
            int human_x = (int)( (human->y - y_min) / (y_max - y_min) * image_width );
            
            float x_max_filter, x_min_filter;   // X-start and X-end of human in meters
            int black_counter = 0;
            // Iterating towards the right
            for(int i=human_x; i < image_width; i++) // width
            {
                bool empty_column = true;
                for(int j=0; j < image_height; j++) // height
                {
                    if(image_array_filt[j*image_width + i].r != 0 || image_array_filt[j*image_width + i].g != 0 || image_array_filt[j*image_width + i].b != 0)
                    {
                        // Array position is not black
                        empty_column = false;
                        break; // No need to look through the rest of the column
                    }                    
                }
                if(empty_column)
                { black_counter++; }
                
                if(black_counter >= gap_limit)
                {
                    //ROS_INFO("i right: %d", i);
                    //ROS_INFO("y_min: %f\t y_max: %f",y_min,y_max);
                    // Space between human and other object is wide enough
                    x_max_filter = ((float)i / image_width) * (y_max - y_min) + y_min;
                    break;
                }
            }
            
            black_counter = 0;
            // Iterating towards the left
            for(int i=human_x; i >= 0; i--) // width
            {
                bool empty_column = true;
                for(int j=0; j < image_height; j++) // height
                {
                    if(image_array_filt[j*image_width + i].r != 0 || image_array_filt[j*image_width + i].g != 0 || image_array_filt[j*image_width + i].b != 0)
                    {
                        // Array position is not black
                        empty_column = false;
                        break; // No need to look through the rest of the column
                    }                    
                }
                if(empty_column)
                { black_counter++; }
                
                if(black_counter >= gap_limit)
                {
                    //ROS_INFO("i left: %d", i);
                    //ROS_INFO("y_min: %f\t y_max: %f",y_min,y_max);
                    // Space between human and other object is wide enough
                    x_min_filter = ((float)i / image_width) * (y_max - y_min) + y_min;
                    break;
                }
            }
            //ROS_INFO("x_min: %f\t x_max: %f",x_min_filter, x_max_filter);
            
		    // Filter the width using the previously determined boundaries
		    PCLPointCloudPtr pclCloud_filtered_objects (new PCLPointCloud);
		    pcl::PassThrough<Point> passthrough_filter_y;
		    passthrough_filter_y.setInputCloud(pclCloud_segmented_filt);
		    passthrough_filter_y.setFilterFieldName("y");
		    passthrough_filter_y.setFilterLimits(x_min_filter, x_max_filter);
		    passthrough_filter_y.filter(*pclCloud_filtered_objects);           
		    
		    pclCloud_segmented_filt->points.clear(); // Reset point cloud
		    for(int i=0; i < pclCloud_filtered_objects->points.size(); i++)
		    {		    
		        pclCloud_segmented_filt->points.push_back(pclCloud_filtered_objects->points[i]);
		        human->points.push_back(pclCloud_filtered_objects->points[i]);
		    }*/
		    
		    // Extract features
            getFeatures(pclCloud_segmented_filt, human);
            // Draw color histograms
            drawHistograms(display_histogram);
		    //ROS_INFO("Done filtering and finding features.");
		    
		    /*if(frame_iterator < 20)
		    {
		        frame_times[frame_iterator] = ros::Time::now().toSec() - current_time;
		        frame_iterator++;
		    }
		    else
		    {
		        double total_time = 0;
		        for(int i=0; i < 20; i++)
		        {
		            total_time += frame_times[i];
		        }
		        double average_time = total_time / 20.0;
		        //ROS_INFO("--> Average filter time: %f", average_time);
		    }*/
		}
		if(mode == CLUSTERING || mode == PICTURES)
		{
		    double current_time = ros::Time::now().toSec();
		    
		    // We want to only select the biggest cluster
		    // First, extract clusters
		    std::vector<pcl::PointIndices> cluster_indices;     // Cluster information is stored here
		    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);  // Creating the KdTree object for the search method of the extraction
		    tree->setInputCloud (pclCloud_filtered_y);
		    pcl::EuclideanClusterExtraction<Point> ec;
		    ec.setClusterTolerance(0.01);       // was 0.04
		    ec.setMinClusterSize(1000);         // Minimal points that must belong to a cluster // was 1000
		    ec.setMaxClusterSize(2500000);      // Maximal points that must belong to a cluster
		    ec.setSearchMethod(tree);
		    ec.setInputCloud(pclCloud_filtered_y);
		    ec.extract(cluster_indices);

		    // Check if there are any clusters that are big enough
		    if (cluster_indices.size() == 0)
		    {
			    ROS_INFO("Did not find any cluster belonging to the human");
			    continue;
		    }

		    // Determine biggest cluster
		    size_t biggest_cluster = 0;
		    size_t biggest_cluster_size = cluster_indices.front().indices.size();
		    if (cluster_indices.size() > 1)
		    {
			    size_t current_cluster = 1;
			    foreach (pcl::PointIndices indices, cluster_indices)
			    {
				    size_t cluster_size = indices.indices.size();
				    if (cluster_size > biggest_cluster_size)
				    {
					    biggest_cluster_size = cluster_size;
					    biggest_cluster = current_cluster;
				    }
				    ++current_cluster;
			    }
		    }

		    // Copy points of biggest cluster to PointCloud and human
		    pcl::PointIndices point_indices = cluster_indices.at(biggest_cluster);
		    foreach (int index, point_indices.indices)
		    {
			    Point p = pclCloud_filtered_y->points[index];
			    pclCloud_segmented_clust->points.push_back(p);
			    human->points.push_back(p);
		    }
		    
		    // Extract features
            getFeatures(pclCloud_segmented_clust, human);
            // Draw color histograms
            drawHistograms(display_histogram);
		    //ROS_INFO("Done clustering and finding features.");

		    /*if(frame_iterator < 20)
		    {
		        frame_times[frame_iterator] = ros::Time::now().toSec() - current_time;
		        frame_iterator++;
		    }
		    else
		    {
		        double total_time = 0;
		        for(int i=0; i < 20; i++)
		        {
		            total_time += frame_times[i];
		        }
		        double average_time = total_time / 20.0;
		        //ROS_INFO("--> Average cluster time: %f", average_time);
		    }*/
		    
	        // Store colours into image array
            for(int i=0; i < pclCloud_segmented_clust->points.size(); i++)
            {
                // normalised position
                float x_norm = (pclCloud_segmented_clust->points[i].y - y_min) / (y_max - y_min);
                float y_norm = (pclCloud_segmented_clust->points[i].z - z_min) / (z_max - z_min);
                
                // array position
                int x_target = (int)(x_norm * image_width);
                int y_target = (int)(y_norm * image_height);
                
                if(x_target < image_width && x_target >= 0 && y_target < image_height && y_target >= 0) // to avoid out-of-bounds errors
                {
                    image_array_clust[y_target*image_width + x_target].r = (float)pclCloud_segmented_clust->points[i].r / 255.0;
                    image_array_clust[y_target*image_width + x_target].g = (float)pclCloud_segmented_clust->points[i].g / 255.0;
                    image_array_clust[y_target*image_width + x_target].b = (float)pclCloud_segmented_clust->points[i].b / 255.0;
                }
            }
            //ROS_INFO("done writing array");
		}
		
	    if(mode == PICTURES)
	    {
	        if(image_counter < max_images)
	        {
	            // Save cloud to png file
	            std::stringstream name;
	            name << "human_filt_" << image_counter << ".bmp";
	            saveBMP(name.str().c_str(),image_width,image_height,72,image_array_filt);
	            name.str("");
	            name << "human_clust_" << image_counter << ".bmp";
	            saveBMP(name.str().c_str(),image_width,image_height,72,image_array_clust);
	            
		        image_counter++;
	        }
	        else
	        { ROS_INFO("Gathered enough images."); }
	    }
	}

	// Check which humans to delete
	std::vector<Human*> existing_humans;
	foreach (Human* human, d_humans)
	{
		if (human->points.size() == 0)
		{
			delete human;
		}
		else
		{
			existing_humans.push_back(human);
		}
	}
	d_humans = existing_humans;

	// Publish PointCloud
	pclCloud_segmented_filt->width = pclCloud_segmented_filt->points.size();
	if (d_publish_cloud)
	{
	    personCloud_pub.publish(*pclCloud_segmented_filt);
	    /*else
	    { personCloud_pub.publish(*pclCloud_segmented_clust); }*/
	}
    
    // Write human features to memory
    writeToMemory();

	//ROS_INFO_STREAM("Humans detected: " << d_humans.size());
}

void SegmentHumans::drawHistograms(bool display)
{
    if(!display || d_humans.size() == 0)
    { return; }

    Human *drawing_human = d_humans[0]; // Which of the detected humans should be drawn
    
    // Select the human with the most points for drawing
    int max_points = 0;
	foreach(Human* human, d_humans)
	{
		if(human->points.size() > max_points)
		{
			max_points = human->points.size();
			drawing_human = human;
		}
	}
    
	// Set size
	int hist_height = 330;
	int hist_width = 648;
	int y_axes = 50;
	int n_bins = 7;
	int bin_bot = 299;
	int bin_top = 31;
	int bin_width = int((hist_width-y_axes) / (n_bins*2 + 1));
	int bin_max_height = hist_height - (hist_height - bin_bot) - bin_top;
	
	// Set colors					
	int hist_colors[][3] = {{255,0,10},{0,100,204},{0,204,0},{255, 255, 40},	//red, blue, green, yellow
							{0,0,0},{255,255,255},{184,184,184}};		        //black, white, grey
														
	int oth_colors[][3] = {{204,230,255},{224,224,224},{200,220,250}};	//lightblue, lightgrey, darkgrey

	// Create image
	cv::Mat histImg = cv::Mat::zeros(hist_height, hist_width, CV_8UC3);
	
    // Reset all
    cv::rectangle(histImg,cv::Point(0,0),cv::Point(hist_width,hist_height),CV_RGB(oth_colors[0][0],oth_colors[0][1],oth_colors[0][2]),-1);

    // Set backgrounds
    cv::rectangle(histImg,cv::Point(y_axes,30),cv::Point(hist_width,hist_height-30),CV_RGB(oth_colors[0][0],oth_colors[0][1],oth_colors[0][2]),-1);
    cv::rectangle(histImg,cv::Point(0,0),cv::Point(y_axes-1,hist_height-30),CV_RGB(255,255,255),-1);
    cv::rectangle(histImg,cv::Point(0,hist_height-30),cv::Point(hist_width,hist_height),CV_RGB(255,255,255),-1);
    cv::rectangle(histImg,cv::Point(0,0),cv::Point(hist_width,(bin_top-1)),CV_RGB(oth_colors[1][0],oth_colors[1][1],oth_colors[1][2]),-1);

    // Create bins for top point cloud features
    for ( int c = 0; c < n_bins; c++ ) 
    {
        float value = getColourData(drawing_human, UPPER, c);
        /*std::stringstream item;
        item << "colour value: " << value;
        ROS_INFO(item.str().c_str());*/
        int height = bin_bot - (value * bin_max_height);
	    cv::rectangle(histImg,cv::Point(y_axes+(c*bin_width),height),cv::Point((y_axes+(c*bin_width)+bin_width-1),bin_bot),CV_RGB(hist_colors[c][0],hist_colors[c][1],hist_colors[c][2]),-1);
	    cv::rectangle(histImg,cv::Point(y_axes+(c*bin_width),height),cv::Point((y_axes+(c*bin_width)+bin_width-1),height),CV_RGB(oth_colors[2][0],oth_colors[2][1],oth_colors[2][2]),-1);
    }

    // Create bins for bottom point cloud features
    for ( int c = 0; c < n_bins; c++ ) 
    {
        double value = getColourData(drawing_human, LEGS, c);
        int height = bin_bot - (value * bin_max_height);    
	    cv::rectangle(histImg,cv::Point(y_axes+((n_bins+1)*bin_width)+(c*bin_width),height),cv::Point((y_axes+((n_bins+1)*bin_width)+(c*bin_width)+bin_width-1),bin_bot),CV_RGB(hist_colors[c][0],hist_colors[c][1],hist_colors[c][2]),-1);
	    cv::rectangle(histImg,cv::Point(y_axes+((n_bins+1)*bin_width)+(c*bin_width),height),cv::Point((y_axes+((n_bins+1)*bin_width)+(c*bin_width)+bin_width-1),height),CV_RGB(oth_colors[2][0],oth_colors[2][1],oth_colors[2][2]),-1);
    }

    // Place lines
    cv::rectangle(histImg,cv::Point(y_axes,hist_height-30),cv::Point(hist_width,hist_height-30),cv::Scalar(0,0,0),-1);
    cv::rectangle(histImg,cv::Point(y_axes-1,30),cv::Point(y_axes-1,hist_height-30),cv::Scalar(0,0,0),-1);
    cv::rectangle(histImg,cv::Point(0,30),cv::Point(hist_width,30),CV_RGB(oth_colors[2][0],oth_colors[2][1],oth_colors[2][2]),-1);

    // Place labels
    cv::putText(histImg,"Top", cv::Point(y_axes,320), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
    cv::putText(histImg,"Bottom", cv::Point(y_axes+(bin_width*7),320), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
    cv::putText(histImg,"Ylab", cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0.8,0.8,0.8));

    // Active/Inactive
    cv::rectangle(histImg,cv::Point(10,10),cv::Point(20,20),cv::Scalar(0,255,0),-1);
    cv::putText(histImg,"Active", cv::Point(30,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,100));

    // Height information
    int height_cm = drawing_human->height * 100;
    std::stringstream stringHeight;
    stringHeight << "Height: " << height_cm << " cm";

    cv::putText(histImg, stringHeight.str(), cv::Point(hist_width-125,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,100));

    // Show image
    cv::imshow("Color Histograms", histImg);
    cv::waitKey(20);
}

void SegmentHumans::getFeatures(PCLPointCloudPtr cloud, Human* human)
{
    // Get human height
    /*double height = 0;
    double min_val = 200000000;
    double tmp_height = 0;
    for(int i=0; i < cloud->points.size(); i++)
    {
        tmp_height = cloud->points[i].z;
        if ( tmp_height > height) 
        {
            height = tmp_height;
        }
        //if(cloud->points[i].z < min_val)
        //{
        //    min_val = cloud->points[i].z;
        //}
    }
    human->height = height;*/
    /*std::stringstream t;
    t << "Max height: " << height;
    ROS_INFO(t.str().c_str());
    t.str("");
    t << "Min height: " << min_val;
    ROS_INFO(t.str().c_str());
    t.str("");
    t << "Num points : " << cloud->points.size();
    ROS_INFO(t.str().c_str());*/
    
    // Use a pass through filter (filter legs)
    /*PCLPointCloudPtr pclCloud_filtered_legs (new PCLPointCloud);
    pcl::PassThrough<Point> passthrough_filter_legs;
    passthrough_filter_legs.setInputCloud(cloud);
    passthrough_filter_legs.setFilterFieldName("z");
    passthrough_filter_legs.setFilterLimits(0.1, leg_height);
    passthrough_filter_legs.filter(*pclCloud_filtered_legs);

    int total_r = 0;
    int total_g = 0;
    int total_b = 0;
    int total_y = 0;
    int total_wh = 0; 
    int total_gr = 0;
    int total_bl = 0;
    for(int i=0; i < pclCloud_filtered_legs->points.size(); i++)
    {
        int r = pclCloud_filtered_legs->points[i].r;
        int g = pclCloud_filtered_legs->points[i].g;
        int b = pclCloud_filtered_legs->points[i].b;
        
        float red = r / 255.0;
        float green = g / 255.0;
        float blue = b / 255.0;
        
        // Convert to HSV
        float h, s, v;
        
        float max_color = max3(red, green, blue);
        float min_color = min3(red, green, blue);
        //std::stringstream extreme;
        //extreme << "max: " << max_color << " min: " << min_color;
        //ROS_INFO(extreme.str().c_str());
        if(fabs(red - max_color) < THRESHOLD)
        {
            // Red is highest
            h = ((green - blue) / (max_color - min_color)) * 60;
        }
        else if(fabs(green - max_color) < THRESHOLD)
        {
            // Green is highest
            h = (2 + (blue - red) / (max_color - min_color)) * 60;
        }
        else
        {
            // Blue is highest
            h = (4 + (red - green) / (max_color - min_color)) * 60;
        }
        s = (max_color - min_color) / max_color;
        v = max_color;

        //std::stringstream color;
        //color << "h: " << h << " s: " << s << " v: " << v;
        //ROS_INFO(color.str().c_str());
            
        
        if(v < 0.05)     // Black
        { total_bl++; }
        else if(v > 0.8) // White
        { total_wh++; }
        
        else if(s < 0.20) // Grey
        { total_gr++; }
        
        else if(h < 30 || h >= 330) // Red
        { total_r++; }
        else if(h < 90) // Yellow
        { total_y++; }
        else if(h < 150) // Green
        { total_g++; } 
        else if(h < 270) // Blue
        { total_b++; } 
        
        
        //if ( r > g && r > b && ( r - g > 20 )  && ( r - b > 20 )) 
        //if ( r > g && r > b && ( (abs(r - g) + abs(r - b) ) > 20) )
        //{
        //    total_r++;
        //}
        //else if ( g > r && g > b && ( g - r > 20 )  && ( g - b > 20 )) 
        //else if ( g > r && g > b && ( (abs(g-r) + abs(g-b) ) > 20 )) 
        //{
        //    total_g++;
        //}
        //else if ( b > r && b > g && ( b - r > 20 )  && ( b - g > 20 )) 
        //else if ( b > r && b > g && ( (abs(b-r) + abs(b-g) ) > 20) ) 
        //{
        //    total_b++;
        //}
        //else if ( ( r + b + g ) < 165 ) 
        //{
        //    total_bl++;
        //}
        //else if ( ( r + b + g ) >= 600 )
        //{
        //    total_wh++;
        //}
        //else 
        //{
        //    total_gr++;
        //}
    }
    int n_points;
    if(pclCloud_filtered_legs->points.size() == 0)
    { n_points = 1; }
    else
    { n_points = pclCloud_filtered_legs->points.size(); }
    human->legs_red = (double)total_r / n_points;
    human->legs_green = (double)total_g / n_points;
    human->legs_blue = (double)total_b / n_points;
    human->upper_yellow = (double)total_y / n_points;
    human->legs_black = (double)total_bl / n_points;
    human->legs_white = (double)total_wh / n_points;
    human->legs_grey = (double)total_gr / n_points;
    
    // Use a pass through filter (filter upper body)
    PCLPointCloudPtr pclCloud_filtered_upper (new PCLPointCloud);
    pcl::PassThrough<Point> passthrough_filter_upper;
    passthrough_filter_upper.setInputCloud(cloud);
    passthrough_filter_upper.setFilterFieldName("z");
    passthrough_filter_upper.setFilterLimits(leg_height, d_human_height);
    passthrough_filter_upper.filter(*pclCloud_filtered_upper);

    total_r = 0;
    total_g = 0;
    total_b = 0;
    total_y = 0;
    total_wh = 0;
    total_gr = 0;
    total_bl = 0;
    for(int i=0; i < pclCloud_filtered_upper->points.size(); i++)
    {
        int r = pclCloud_filtered_upper->points[i].r;
        int g = pclCloud_filtered_upper->points[i].g;
        int b = pclCloud_filtered_upper->points[i].b;
        
        float red = r / 255.0;
        float green = g / 255.0;
        float blue = b / 255.0;
        
        // Convert to HSV
        float h, s, v;
        
        float max_color = max3(red, green, blue);
        float min_color = min3(red, green, blue);
        if(fabs(red - max_color) < THRESHOLD)
        {
            // Red is highest
            h = ((green - blue) / (max_color - min_color)) * 60;
        }
        else if(fabs(green - max_color) < THRESHOLD)
        {
            // Green is highest
            h = (2 + (blue - red) / (max_color - min_color)) * 60;
        }
        else
        {
            // Blue is highest
            h = (4 + (red - green) / (max_color - min_color)) * 60;
        }
        s = (max_color - min_color) / max_color;
        v = max_color;
            
        
        if(v < 0.05)     // Black
        { total_bl++; }
        else if(v > 0.8) // White
        { total_wh++; }
        
        else if(s < 0.20) // Grey
        { total_gr++; }
        
        else if(h < 30 || h >= 330) // Red
        { total_r++; }
        else if(h < 90) // Yellow
        { total_y++; }
        else if(h < 150) // Green
        { total_g++; } 
        else if(h < 270) // Blue
        { total_b++; } 
        
        //if ( r > g && r > b && ( r - g > 30 )  && ( r - b > 30 )) 
        //{
        //    total_r++;
        //}
        //else if ( g > r && g > b && ( g - r > 30 )  && ( g - b > 30 )) 
        //{
        //    total_g++;
        //}
        //else if ( b > r && b > g && ( b - r > 30 )  && ( b - g > 30 )) 
        //{
        //    total_b++;
        //}
        //else if ( ( r + b + g ) < 150 ) 
        //{
        //    total_bl++;
        //}
        //else if ( ( r + b + g ) >= 750 )
        //{
        //    total_wh++;
        //}
        //else 
        //{
        //    total_gr++;
        //}
    }
    n_points;
    if(pclCloud_filtered_upper->points.size() == 0)
    { n_points = 1; }
    else
    { n_points = pclCloud_filtered_upper->points.size(); }
    human->upper_red = (double)total_r / n_points;
    human->upper_green = (double)total_g / n_points;
    human->upper_blue = (double)total_b / n_points;
    human->upper_yellow = (double)total_y / n_points;
    human->upper_black = (double)total_bl / n_points;
    human->upper_white = (double)total_wh / n_points;
    human->upper_grey = (double)total_gr / n_points;*/

    int legs_total_r = 0;
    int legs_total_g = 0;
    int legs_total_b = 0;
    int legs_total_y = 0;
    int legs_total_wh = 0; 
    int legs_total_gr = 0;
    int legs_total_bl = 0;
    int upper_total_r = 0;
    int upper_total_g = 0;
    int upper_total_b = 0;
    int upper_total_y = 0;
    int upper_total_wh = 0; 
    int upper_total_gr = 0;
    int upper_total_bl = 0;
    int legs_n_points = 0;
    int upper_n_points = 0;
    
    /*std::stringstream printer;
    printer << "min_red: " << human->min_channels.r << "\t max_red: " << human->max_channels.r << "\n";
    ROS_INFO(printer.str().c_str());*/
    
    for(int i=0; i < cloud->points.size(); i++)
    {
        float r = cloud->points[i].r;
        float g = cloud->points[i].g;
        float b = cloud->points[i].b;
        
        // Scale colors to full range of [0,255]
        /*float range_r = (human->max_channels.r - human->min_channels.r);
        float range_g = (human->max_channels.g - human->min_channels.g);
        float range_b = (human->max_channels.b - human->min_channels.b);
        range_r = range_r > 0.0 ? range_r : 0.0001;
        range_g = range_g > 0.0 ? range_g : 0.0001;
        range_b = range_b > 0.0 ? range_b : 0.0001;        
        r = (r / range_r) * 255.0;
        g = (g / range_g) * 255.0;
        b = (b / range_b) * 255.0;*/

        // Put colors back into point cloud
        /*cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;*/
        
        float red = r / 255.0;
        float green = g / 255.0;
        float blue = b / 255.0;
        
        float h, s, v;
        h = 0.0; // In case a precision error occurs and H gets evaluated when the color should be dependent only on S or V.
        s = 1.0;
        
        float max_color = max3(red, green, blue);
        float min_color = min3(red, green, blue);
        /*std::stringstream extreme;
        extreme << "max: " << max_color << " min: " << min_color << std::endl;
        ROS_INFO(extreme.str().c_str());*/
        // Convert to HSV
        if(fabs(max_color - min_color) >= THRESHOLD) // Avoids dividing by 0
        {
            if(fabs(red - max_color) < THRESHOLD)
            {
                // Red is highest
                h = ((green - blue) / (max_color - min_color)) * 60;
            }
            else if(fabs(green - max_color) < THRESHOLD)
            {
                // Green is highest
                h = (2 + (blue - red) / (max_color - min_color)) * 60;
            }
            else
            {
                // Blue is highest
                h = (4 + (red - green) / (max_color - min_color)) * 60;
            }
        }
        if(max_color > 0)
        { s = (max_color - min_color) / max_color; }
        v = max_color;
        
        h = h >= 0.0 ? h : h + 360;

        if(v >= V_BLACK && v <= V_WHITE && s >= S_GREY)
        { h_values.push_back(h); } // debug

        //std::stringstream color;
        //color << "h: " << h << " s: " << s << " v: " << v;
        //ROS_INFO(color.str().c_str());
            
        if (cloud->points[i].z < leg_height)
        {
            /*float addition = 1.0;*/
	        if(v < V_BLACK)     // Black
	        { legs_total_bl++; }
	        else if(v > V_WHITE) // White
	        { legs_total_wh++; }
	        else if(s < S_GREY) // Grey
	        { legs_total_gr++; }
            else if(h < H_RED_YELLOW)
            { legs_total_r++; }
            else if(h < H_YELLOW_GREEN)
            { legs_total_y++; }
            else if(h < H_GREEN_BLUE)
            { legs_total_g++; }
            else if(h < H_BLUE_RED)
            { legs_total_b++; }            
            else
            { legs_total_r++; }
            /*else
            {
                float red_plus = 0.5 * computeColourPortion(h, H_RED_LOW_CENTER, H_RED_LOW_CENTER, H_RED_YELLOW);
                red_plus += 0.5 * computeColourPortion(h, H_RED_HIGH_CENTER, H_BLUE_RED, H_RED_HIGH_CENTER);
                float yellow_plus = computeColourPortion(h, H_YELLOW_CENTER, H_RED_YELLOW, H_YELLOW_GREEN);
                float green_plus = computeColourPortion(h, H_GREEN_CENTER, H_YELLOW_GREEN, H_GREEN_BLUE);
                float blue_plus = computeColourPortion(h, H_BLUE_CENTER, H_GREEN_BLUE, H_BLUE_RED);
                legs_total_r += red_plus;
                legs_total_y += yellow_plus;
                legs_total_g += green_plus;
                legs_total_b += blue_plus;
                addition = red_plus + yellow_plus + green_plus + blue_plus;
            }*/
	        
	        /*else if(h < 25 || h >= 335) // Red
	        { legs_total_r++; }
	        else if(h < 90) // Yellow
	        { legs_total_y++; }
	        else if(h < 150) // Green
	        { legs_total_g++; }
	        else if(h < 270) // Blue
	        { legs_total_b++; } */

	    	++legs_n_points;
		}
		else
		{
            /*float addition = 1.0;*/
	        if(v < V_BLACK)     // Black
	        { upper_total_bl++; }
	        else if(v > V_WHITE) // White
	        { upper_total_wh++; }
	        else if(s < S_GREY) // Grey
	        { upper_total_gr++; }
            else if(h < H_RED_YELLOW)
            { upper_total_r++; }
            else if(h < H_YELLOW_GREEN)
            { upper_total_y++; }
            else if(h < H_GREEN_BLUE)
            { upper_total_g++; }
            else if(h < H_BLUE_RED)
            { upper_total_b++; }            
            else
            { upper_total_r++; }
            /*else
            {
                float red_plus = 0.5 * computeColourPortion(h, H_RED_LOW_CENTER, H_RED_LOW_CENTER, H_RED_YELLOW);
                red_plus += 0.5 * computeColourPortion(h, H_RED_HIGH_CENTER, H_BLUE_RED, H_RED_HIGH_CENTER);
                float yellow_plus = computeColourPortion(h, H_YELLOW_CENTER, H_RED_YELLOW, H_YELLOW_GREEN);
                float green_plus = computeColourPortion(h, H_GREEN_CENTER, H_YELLOW_GREEN, H_GREEN_BLUE);
                float blue_plus = computeColourPortion(h, H_BLUE_CENTER, H_GREEN_BLUE, H_BLUE_RED);
                upper_total_r += red_plus;
                upper_total_y += yellow_plus;
                upper_total_g += green_plus;
                upper_total_b += blue_plus;
                addition = red_plus + yellow_plus + green_plus + blue_plus;
            }*/
	        
	        /*else if(h < 30 || h >= 330) // Red
	        { upper_total_r++; }
	        else if(h < 90) // Yellow
	        { upper_total_y++; }
	        else if(h < 150) // Green
	        { upper_total_g++; } 
	        else if(h < 270) // Blue
	        { upper_total_b++; } */ 

	    	++upper_n_points;
		}
       
    }
    legs_n_points = legs_n_points > 0.0 ? legs_n_points : 1.0;
    upper_n_points = upper_n_points > 0.0 ? upper_n_points : 1.0;
    
    // Scale to use full range
    human->legs_red 	= legs_total_r / (float)legs_n_points;
    human->legs_green 	= legs_total_g / (float)legs_n_points;
    human->legs_blue 	= legs_total_b / (float)legs_n_points;
    human->legs_yellow 	= legs_total_y / (float)legs_n_points;
    human->legs_black 	= legs_total_bl / (float)legs_n_points;
    human->legs_white 	= legs_total_wh / (float)legs_n_points;
    human->legs_grey 	= legs_total_gr / (float)legs_n_points;

    human->upper_red 	= upper_total_r / (float)upper_n_points;
    human->upper_green 	= upper_total_g / (float)upper_n_points;
    human->upper_blue 	= upper_total_b / (float)upper_n_points;
    human->upper_yellow = upper_total_y / (float)upper_n_points;
    human->upper_black 	= upper_total_bl / (float)upper_n_points;
    human->upper_white 	= upper_total_wh / (float)upper_n_points;
    human->upper_grey 	= upper_total_gr / (float)upper_n_points;
}

float SegmentHumans::getColourData(Human *human, const int portion, const int colour)
{    
    if(portion == UPPER)
    {
        switch(colour)
        {
            case RED:
                return human->upper_red;
            case GREEN:
                return human->upper_green;
            case BLUE:
                return human->upper_blue;
            case YELLOW:
                return human->upper_yellow;
            case WHITE:
                return human->upper_white;
            case BLACK:
                return human->upper_black;
            case GREY:
                return human->upper_grey;
        }        
    }
    else if(portion == LEGS)
    {
        switch(colour)
        {
            case RED:
                return human->legs_red;
            case GREEN:
                return human->legs_green;
            case BLUE:
                return human->legs_blue;
            case YELLOW:
                return human->legs_yellow;
            case WHITE:
                return human->legs_white;
            case BLACK:
                return human->legs_black;
            case GREY:
                return human->legs_grey;
        }        
    }
    
    return 0;
}

float SegmentHumans::computeColourPortion(float h, const int center, const int lower_bound, const int upper_bound)
{
    if(upper_bound <= lower_bound)
    { ROS_INFO("shit"); }
    
    float result = fabs(h - center) / (float)(upper_bound - lower_bound);
    
    if(result > 1.0 || result < 0.0)
    { return 0.0; }

    return result;
}

void SegmentHumans::saveBMP(const char *filename, int w, int h, int dpi, rgb *data)
{
	FILE *f;
	int k = w*h;
	int s = 4*k;
	int filesize = 54 + s;
	
	double factor = 39.375;
	int m = static_cast<int>(factor);
	
	int ppm = dpi*m;
	
	unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0,0,0, 54,0,0,0};
	unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0,24,0};
	
	bmpfileheader[ 2] = (unsigned char)(filesize);
	bmpfileheader[ 3] = (unsigned char)(filesize>>8);
	bmpfileheader[ 4] = (unsigned char)(filesize>>16);
	bmpfileheader[ 5] = (unsigned char)(filesize>>24);
	
	bmpinfoheader[ 4] = (unsigned char)(w);
	bmpinfoheader[ 5] = (unsigned char)(w>>8);
	bmpinfoheader[ 6] = (unsigned char)(w>>16);
	bmpinfoheader[ 7] = (unsigned char)(w>>24);
	
	bmpinfoheader[ 8] = (unsigned char)(h);
	bmpinfoheader[ 9] = (unsigned char)(h>>8);
	bmpinfoheader[10] = (unsigned char)(h>>16);
	bmpinfoheader[11] = (unsigned char)(h>>24);
	
	bmpinfoheader[21] = (unsigned char)(s);
	bmpinfoheader[22] = (unsigned char)(s>>8);
	bmpinfoheader[23] = (unsigned char)(s>>16);
	bmpinfoheader[24] = (unsigned char)(s>>24);
	
	bmpinfoheader[25] = (unsigned char)(ppm);
	bmpinfoheader[26] = (unsigned char)(ppm>>8);
	bmpinfoheader[27] = (unsigned char)(ppm>>16);
	bmpinfoheader[28] = (unsigned char)(ppm>>24);
	
	bmpinfoheader[29] = (unsigned char)(ppm);
	bmpinfoheader[30] = (unsigned char)(ppm>>8);
	bmpinfoheader[31] = (unsigned char)(ppm>>16);
	bmpinfoheader[32] = (unsigned char)(ppm>>24);
	
	f = fopen(filename,"wb");
	
	fwrite(bmpfileheader,1,14,f);
	fwrite(bmpinfoheader,1,40,f);
	
	for (int i = 0; i < k; i++) {
		//RGBType rgb = data[i];
		
		double red = (data[i].r)*255;
		double green = (data[i].g)*255;
		double blue = (data[i].b)*255;
		
		unsigned char color[3] = {(int)floor(blue),(int)floor(green),(int)floor(red)};
		
		fwrite(color,1,3,f);
	}
	
	fclose(f);
}

float SegmentHumans::max3(float a, float b, float c)
{
    float max = a;
    if(b > max)
    { max = b; }
    if(c > max)
    { max = c; }
    
    return max;
}

float SegmentHumans::min3(float a, float b, float c)
{
    float min = a;
    if(b < min)
    { min = b; }
    if(c < min)
    { min = c; }
    
    return min;
}

int main(int argc, char** argv)
{
    // ROS initialization stuff
	ros::init(argc, argv, "segment_humans");
	ros::NodeHandle n;
	ros::Rate r(10);

	// Create listener and set global pointer
	tf::TransformListener temp_listener;
	listener = &temp_listener;

	// Create memory service client
	memory_client = n.serviceClient<borg_pioneer::MemoryReadSrv>("memory_read");
	memory_client_write = n.serviceClient<borg_pioneer::MemorySrv>("memory");

	// Get parameters
	bool publish_cloud;			ros::param::param<bool>("~publish_cloud", publish_cloud, true);
	/*std::string camera_topic3;	ros::param::param<std::string>("~camera_topic3", camera_topic3, "camera3/depth_registered/points");*/
	std::string camera_topic2;  ros::param::param<std::string>("~camera_topic2", camera_topic2, "camera2/depth_registered/points");

	// Create segmentizer/tracker
	SegmentHumans sh(publish_cloud);
	sh.initImageArrays();

	//Subscribe to point cloud data
	ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>(camera_topic2, 1, &SegmentHumans::processCam, &sh);

	// Create a ROS publisher for the output PointCloud
	personCloud_pub = n.advertise<PCLPointCloud>("human_cloud/output", 1);
	
	// Create window for color histograms
	cv::namedWindow("Color Histograms", CV_WINDOW_AUTOSIZE);
	cv::moveWindow("Color Histograms", 100, 100);

	ros::spin();

    return 0;
}
