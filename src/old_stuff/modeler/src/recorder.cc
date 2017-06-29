#include <modeler/recorder.h>

Recorder::Recorder()
{
	d_bCameraInfoReady = false; 
	d_bRetryCluster = false;
	d_fDegrees = 30.0f; // the number of rotational degrees between snapshots of the object
	d_fPrevRotation = 9000.1f; // yea, its over 9000
	d_stNrOfSegments = 0; // for counting the number of segments of the object

	d_depth_sub = d_nh.subscribe<PCLPointCloud>("/camera/depth_registered/points", 1, &Recorder::pcCallBack, this);
	d_cam_info_sub = d_nh.subscribe<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 1, &Recorder::camInfoCallback, this);

	d_cloud_pub = d_nh.advertise<PCLPointCloud>("transformation/model", 1);

	cout << "Enter object name: ";
	cin >> d_objectName;
}


void Recorder::pcCallBack(const PCLPointCloud::ConstPtr &cloudMsg)
{
	Mat image = makeImage(*cloudMsg);
	Eigen::Matrix4f transformMatrix = getTransformation(image);

	if (transformMatrix == Eigen::Matrix4f::Zero()) // if patterns are not found, we don't know the transformation
        return;

    float currentRotation = atan2(transformMatrix(0,1), transformMatrix(0,0)) * 180 / M_PI; // get the current rotation around the z-axis in degrees
    

//    if (d_bRetryCluster || (d_fPrevRotation != 9000.1f && abs(d_fPrevRotation - currentRotation) >= d_fDegrees))
//    {
    	d_fPrevRotation = currentRotation;

    	PCLPointCloudPtr cloud (new PCLPointCloud);
	    cloud->header.frame_id = "camera_rgb_optical_frame"; // needed when publishing the cloud 

	    Eigen::Matrix4f invMatrix = transformMatrix.inverse();
	    pcl::transformPointCloud (*cloudMsg, *cloud, invMatrix); // transform pointcloud to the pattern transformation

	    segmentPointcloud(cloud, transformMatrix); // segment the object out of the scene

	    PCLPointCloudPtr objectCloud(new PCLPointCloud());
	   
	    if (cluster(cloud, objectCloud, transformMatrix)) // get a single cluster
	    {
    		++d_stNrOfSegments;

	    	d_bRetryCluster = false;
	    	objectCloud->height = 1;
	    	objectCloud->width = objectCloud->size();
	    	d_vObject.push_back(*objectCloud);
	    	
	    	ROS_INFO_STREAM("number of segments: " << d_stNrOfSegments << ", " << 360.0f / d_fDegrees);

	    //	if (d_stNrOfSegments >= (360.0f / d_fDegrees))
	    //	{
	    		d_depth_sub.shutdown(); // don't need this function anymore, so don't use cpu power 
	    		
	    		for (size_t idx = 0; idx < d_vObject.size(); ++idx)
	    		{	
	    			stringstream ss;
	    			ss << d_objectName << "_" << idx << ".pcd";

	    			io::savePCDFile(ss.str(), d_vObject.at(idx)); 
	    		}

	    		ROS_INFO_STREAM("DONE!, press ctrl+c to exit!");	    	
	    		return;
	    //	}
	    
	    	objectCloud->header.frame_id = "camera_rgb_optical_frame"; // needed when publishing the cloud 
	    	d_cloud_pub.publish(objectCloud);
	    }
	    else
	    	d_bRetryCluster = true;

  //  }
  //  else if (d_fPrevRotation == 9000.1f)
  //  	d_fPrevRotation = currentRotation;	
}

bool Recorder::cluster(PCLPointCloudPtr &cloud, PCLPointCloudPtr &outCloud, Eigen::Matrix4f transformMatrix)
{
    vector<pcl::PointIndices> cluster; 
    pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint>);
    tree->setInputCloud(cloud);
    EuclideanClusterExtraction<PCLPoint> ec;    

    ec.setClusterTolerance(0.05);       // in meters
    ec.setMinClusterSize(100);       // Minimal points that must belong to a cluster
    ec.setMaxClusterSize(400000);      // Maximal points that must belong to a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster);

    if (cluster.size() == 0)
        return false;

 //   cout << "CLUSTERS: " << cluster.size() << "\n";
    float midX = transformMatrix(0,3);
    float midY = transformMatrix(1,3);

    float prevDistance = 9000.0f;
    size_t prevSize = 0;

    PCLPointCloudPtr cloudt(new PCLPointCloud());

    for (size_t idx = 0; idx < cluster.size(); ++idx)
    {
        pcl::PointIndices point_indices = cluster.at(idx);
        PCLPointCloudPtr tempCloud(new PCLPointCloud());

        foreach (int index, point_indices.indices)          
        {
            PCLPoint p = cloud->points[index];              
            tempCloud->points.push_back(p);
            cloudt->points.push_back(p);
        //    p.r = 255;
        //    p.g = 0;
        //    p.b = 0;
        //    cloudt->points.push_back(p);
        }

        if (tempCloud->size() > prevSize)
        {
            *outCloud = *tempCloud;
         //   prevDistance = distance;
            prevSize = tempCloud->size();
        }
    }

    if (cluster.size() > 1)
    {
        ROS_INFO_STREAM("More then one cluster found, not saving!");
        cloudt->header.frame_id = "camera_rgb_optical_frame"; // needed when publishing the cloud 
        d_cloud_pub.publish(cloudt);
        return false;
    }

    outCloud->header.frame_id = "camera_rgb_optical_frame"; // needed when publishing the cloud 
    d_cloud_pub.publish(outCloud);

    cout << "correct y/n: ";
    string s; 
    cin >> s;

    if (s == "y")
        return true;
    else
        return false; 
    
}

void Recorder::segmentPointcloud(PCLPointCloudPtr &cloud, Eigen::Matrix4f transformMatrix)
{
	PassThrough<PCLPoint> passthrough_filter;
	passthrough_filter.setInputCloud(cloud);
	passthrough_filter.setFilterFieldName("z");
	passthrough_filter.setFilterLimits(0, transformMatrix(2,3) - 0.00f);
//	passthrough_filter.setFilterLimits(0.4, 0.70f);
	passthrough_filter.filter(*cloud);	

	// filter Y		
	passthrough_filter.setInputCloud(cloud);
	passthrough_filter.setFilterFieldName("y");
//	passthrough_filter.setFilterLimits(-0.2 , 0.2;
	passthrough_filter.setFilterLimits(transformMatrix(1,3) - 0.2, transformMatrix(1,3) + 0.2);
	passthrough_filter.filter(*cloud);

	// filter X 
	passthrough_filter.setInputCloud(cloud);
	passthrough_filter.setFilterFieldName("x");
//	passthrough_filter.setFilterLimits(-0.20, 0.20);
	passthrough_filter.setFilterLimits(transformMatrix(0,3) - 0.20, transformMatrix(0,3) + 0.20);
	passthrough_filter.filter(*cloud);

	ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	SACSegmentation<PCLPoint> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.02);  // 0.015 

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);		
	
	ExtractIndices<PCLPoint> extract;

	extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // true means remove planes
    extract.filter(*cloud);
}

void Recorder::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &cam_info)
{
	Mat D;
	CameraInfo ci = *cam_info;
	d_mK = cv::Mat(3, 3, CV_32FC1);
	
	for (int i = 0; i < 9; i++)
		d_mK.at<float>(i / 3, i % 3) = ci.K[i];
	
	D = cv::Mat(ci.D.size(), 1, CV_32FC1);
		
	for (size_t i = 0; i < ci.D.size(); i++)
		D.at<float>(i) = ci.D[i];

	d_bCameraInfoReady = true;
	d_cam_info_sub.shutdown(); // don't need this anymore, so shut it down
}

Mat Recorder::makeImage(PCLPointCloud cloud)
{
	int width = cloud.height;
    int height = cloud.width;
    cv::Mat newImage(width, height, CV_8UC3);
    
    int count = 0;
        
    for (size_t idx = 0; idx < width; ++idx)
        for (size_t idy = 0; idy < height; ++idy)
        {
            int b = cloud.points[count].b;
            int g = cloud.points[count].g;
            int r = cloud.points[count].r;
            newImage.at<cv::Vec3b>(idx, idy)[0] = (unsigned char)b;
            newImage.at<cv::Vec3b>(idx, idy)[1] = (unsigned char)g;
            newImage.at<cv::Vec3b>(idx, idy)[2] = (unsigned char)r;                 
            ++count;    
        }
 
    return newImage;
}

Eigen::Matrix4f Recorder::getTransformation(Mat image)
{
    Mat imageGray;
    cvtColor(image, imageGray, CV_RGB2GRAY);    

    vector<Point2f> out_bw;
    vector<Point2f> out_wb;

    Size grid_size = Size(3, 5); // pattern size 

    // find black dots, white background pattern
    bool patternFoundBw = findCirclesGrid(imageGray, grid_size, out_bw, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);

    bitwise_not(imageGray, imageGray);  // reverse image colors (turn black to white, and white to black)
    
    // find white dots, black background pattern
    bool patternFoundWb = findCirclesGrid(imageGray, grid_size, out_wb, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
    
    drawChessboardCorners(image, grid_size, out_bw, patternFoundBw);
    drawChessboardCorners(image, grid_size, out_wb, patternFoundWb);
   
    float square_size = 0.02f;
    Point3f offset; // pattern offsets
	offset.z = 0.00f; 
	offset.y = -0.05f;

	vector<Point3f> ideal_pts;
	vector<Point2f> out;
	Mat rvec, tvec;

    if (patternFoundBw)
    {
	    offset.x = -0.157;    
	    
	    vector<Point3f> ideal_pts_bw = calcChessboardCorners(grid_size, square_size, offset);
	    ideal_pts.insert(ideal_pts.end(), ideal_pts_bw.begin(), ideal_pts_bw.end());
	    out.insert(out.end(), out_bw.begin(), out_bw.end());
	}

	if (patternFoundWb)
	{
    	offset.x = 0.075;

    	vector<Point3f> ideal_pts_wb = calcChessboardCorners(grid_size, square_size, offset);
    	ideal_pts.insert(ideal_pts.end(), ideal_pts_wb.begin(), ideal_pts_wb.end());
    	out.insert(out.end(), out_wb.begin(), out_wb.end());   
    }

    if (patternFoundBw || patternFoundWb)  // a pattern was found, so we have translation and rotation
    {
    	solvePnP(ideal_pts, out, d_mK, Mat(), rvec, tvec, false);  
    	draw(image, d_mK, rvec, tvec);

        Mat rmat;
        Rodrigues(rvec, rmat); // transform rotation vector to rotation matrix

        Eigen::Matrix4f Tm = Eigen::Matrix4f::Zero();
        Tm(3,3) = 1;
        
        // add the rotation matrix to the transformation matrix
        for (size_t idx = 0; idx < 3; ++idx)
            for (size_t idy = 0; idy < 3; ++idy)
                Tm (idx,idy) = rmat.at<double>(idx,idy);

        Tm(0,3) = (float)tvec.at<double>(0);
        Tm(1,3) = (float)tvec.at<double>(1);
        Tm(2,3) = (float)tvec.at<double>(2);

        imshow("image", image);
        waitKey(1);
        return Tm;
    }
    else // if pattern was not found just return a matrix full of zeros
    {
        imshow("image", image);
        waitKey(1);
        return Eigen::Matrix4f::Zero();
    }
}

vector<cv::Point3f>  Recorder::calcChessboardCorners(Size boardSize, float squareSize, Point3f offset)
{
	vector<Point3f> corners;

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float(i * squareSize), float((2 * j + i % 2) * squareSize), 0) + offset);
	
	return corners;
}

void Recorder::draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T)
{
	if (R.empty() || T.empty())
		return;
	float scale = 0.1;
	Point3f z(0, 0, scale);
	Point3f x(scale, 0, 0);
	Point3f y(0, scale, 0);
	Point3f o(0, 0, 0);
	vector<Point3f> op(4);
	op[1] = x, op[2] = y, op[3] = z, op[0] = o;
	vector<Point2f> ip;
	projectPoints(Mat(op), R, T, K, Mat(4, 1, CV_64FC1, Scalar(0)), ip);

	vector<Scalar> c(4); //colors
	c[0] = Scalar(255, 255, 255);
	c[1] = Scalar(255, 0, 0); //x
	c[2] = Scalar(0, 255, 0); //y
	c[3] = Scalar(0, 0, 255); //z
	line(drawImage, ip[0], ip[1], c[1]);
	line(drawImage, ip[0], ip[2], c[2]);
	line(drawImage, ip[0], ip[3], c[3]);
	string scaleText = boost::str(boost::format("scale %0.2f meters")%scale);
	int baseline = 0;
	Size sz = getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
	Point box_origin(10, drawImage.size().height - 10);
	rectangle(drawImage, box_origin + Point(0, 5), box_origin + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
	putText(drawImage, scaleText, box_origin, CV_FONT_HERSHEY_SIMPLEX, 1.0, c[0], 1, CV_AA, false);
	putText(drawImage, "Z", ip[3], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[3], 1, CV_AA, false);
	putText(drawImage, "Y", ip[2], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[2], 1, CV_AA, false);
	putText(drawImage, "X", ip[1], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[1], 1, CV_AA, false);

}