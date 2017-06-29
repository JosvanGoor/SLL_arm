#include <object_modeler/recorder.h>
#include <fstream>

Recorder::Recorder(ros::NodeHandle &nh)
{
    d_bCameraInfoReady = false; 
    d_bRetryCluster = false;

    nh.param("step_size", d_dStepSize, 15.0); // get the step_size param from the launch file
    ROS_INFO_STREAM("Step size is: " << d_dStepSize);

    d_fPrevRotation = 9000.1f; // yea, its over 9000
    d_stNrOfSegments = 0; // for counting the number of segments of the object
ROS_INFO_STREAM("KOMT Hier wel");
//    d_depth_sub = d_nh.subscribe<PCLPointCloud>("/xtion/depth_registered/points", 1, &Recorder::pcCallBack, this);
//    d_cam_info_sub = d_nh.subscribe<sensor_msgs::CameraInfo>("/xtion/rgb/camera_info", 1, &Recorder::camInfoCallback, this);
    d_depth_sub = d_nh.subscribe<sensor_msgs/PointCloud2>("/kinect2/sd/points", 1, &Recorder::pcCallBack, this);
    d_cam_info_sub = d_nh.subscribe<sensor_msgs::CameraInfo>("/kinect2/sd/camera_info", 1, &Recorder::camInfoCallback, this);


    d_cloud_pub = d_nh.advertise<PCLPointCloud>("transformation/model", 1);
    d_cloud_pub2 = d_nh.advertise<PCLPointCloud>("transformation/model2", 1);

    cv::namedWindow("Controls", 1);
    bottomSlider = 0;
    bottomSliderMax = 50;
    cv::createTrackbar("bottomSide", "Controls", &bottomSlider, bottomSliderMax, Recorder::callbackBottomSlider, this);
}

void Recorder::callbackBottomSlider(int, void *)
{
	ROS_INFO_STREAM(bottomSlider);
}

void Recorder::pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{

    Mat image = makeImage(*cloudMsg);  // create a rgb image from the pointcloud for viewing

    Eigen::Matrix4f transformMatrix = getTransformation(image); // find the transformation from the pattern

    if (transformMatrix == Eigen::Matrix4f::Zero()) // if patterns are not found, we don't know the transformation
        return;


    d_fPrevRotation = d_dStepSize;
    float currentRotation = atan2(transformMatrix(0,1), transformMatrix(0,0)) * 180 / M_PI; // get the current rotation around the z-axis in degrees

    if (d_bRetryCluster || (d_fPrevRotation != 9000.1f && abs(d_fPrevRotation - currentRotation) >= d_dStepSize))
    {
        d_fPrevRotation = currentRotation;

        PCLPointCloudPtr cloud (new PCLPointCloud);
        cloud->header.frame_id = "xtion_rgb_optical_frame"; // needed when publishing the cloud

        Eigen::Matrix4f tempMatrix = transformMatrix;
        tempMatrix(0,3) = 0.0f;
        tempMatrix(1,3) = 0.0f;
        tempMatrix(2,3) = 0.0f;

        Eigen::Matrix4f invMatrix = transformMatrix.inverse();
     //   Eigen::Matrix4f invMatrix = tempMatrix.inverse();
        pcl::transformPointCloud (*cloudMsg, *cloud, invMatrix); // transform pointcloud to the pattern transformation

       // d_cloud_pub2.publish(cloud);
        segmentPointcloud(cloud, transformMatrix); // segment the object out of the scene


        PCLPointCloudPtr objectCloud(new PCLPointCloud());
       
        if (cluster(cloud, objectCloud, transformMatrix)) // get a single cluster
        {
            pcl::transformPointCloud (*objectCloud, *objectCloud, transformMatrix);
            ++d_stNrOfSegments;

            d_bRetryCluster = false;
            vector<int> ind;
            removeNaNFromPointCloud(*objectCloud, *objectCloud, ind);            

            objectCloud->width = objectCloud->points.size();
            objectCloud->height = 1;
            d_vObject.push_back(*objectCloud);
            
            ROS_INFO_STREAM("number of segments: " << d_stNrOfSegments << ", " << 360.0f / d_dStepSize);

            if (d_stNrOfSegments >= (360.0f / d_dStepSize))
            {
                d_depth_sub.shutdown(); // don't need this function anymore, so don't use cpu power 
                destroyWindow("image");
                waitKey(1);
                
                vector<double> xs;
                vector<double> ys;
                vector<double> zs;

                cout << "class name: ";
                string dirname; 
                cin >> dirname;
                mkdir(dirname.c_str(),S_IRWXU);
                
                for (size_t idx = 0; idx < d_vObject.size(); ++idx) 
                {
                    stringstream fileName;
                    fileName << dirname << "/file"  << idx << ".pcd";
                    vector<double> size;
                    size = getSize(d_vObject[idx]);

                    xs.push_back(size[0]);
                    ys.push_back(size[1]);
                    zs.push_back(size[2]);


                    pcl::PointCloud<SHOT352>::Ptr write = databasialize(d_vObject[idx]);
                    io::savePCDFile(fileName.str(), *write); 
                    cout << "written that shit!" << endl;
                }   

                findDims(xs, ys, zs, dirname);

                /*
                for (size_t idx = 0; idx < d_vObject.size(); ++idx)
                {
                    stringstream fileName; 
                    fileName << "file" << idx << ".pcd";
                    PCLPointCloudPtr tempCloud(new PCLPointCloud());
                    io::loadPCDFile(fileName.str(), *tempCloud);
                    tempCloud->header.frame_id = "camera_rgb_optical_frame";
                    d_cloud_pub.publish(tempCloud);
                    sleep(1);
                }         
                */              
                ROS_INFO_STREAM("DONE!, kill program with 'ctrl+c'");
                return;
            }
        
            objectCloud->header.frame_id = "xtion_rgb_optical_frame"; // needed when publishing the cloud 
        //    d_cloud_pub.publish(objectCloud);
        }
        else
            d_bRetryCluster = true;

    }
    else if (d_fPrevRotation == 9000.1f)
        d_fPrevRotation = currentRotation;  
}

bool Recorder::cluster(PCLPointCloudPtr &cloud, PCLPointCloudPtr &outCloud, Eigen::Matrix4f transformMatrix)
{
    vector<pcl::PointIndices> cluster; 
    pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint>);
    tree->setInputCloud(cloud);
    EuclideanClusterExtraction<PCLPoint> ec;    

    ec.setClusterTolerance(0.06);       // in meters
    ec.setMinClusterSize(100);       // Minimal points that must belong to a cluster
    ec.setMaxClusterSize(30000);      // Maximal points that must belong to a cluster
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
        ROS_INFO_STREAM("More than one cluster found, not saving!");
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
 //   passthrough_filter.setFilterLimits(-1.5, 0);
    passthrough_filter.setFilterLimits(0, transformMatrix(2,3));
//  passthrough_filter.setFilterLimits(0.4, 0.70f);
    passthrough_filter.filter(*cloud);  

    ROS_INFO_STREAM(transformMatrix(2,3));

/*
    // filter Y     
    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.setFilterFieldName("y");
//  passthrough_filter.setFilterLimits(-0.2 , 0.2;
    passthrough_filter.setFilterLimits(transformMatrix(1,3) - 0.3, transformMatrix(1,3) + 0.1);
    passthrough_filter.filter(*cloud);

    // filter X 
    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.setFilterFieldName("x");
//  passthrough_filter.setFilterLimits(-0.20, 0.20);
    passthrough_filter.setFilterLimits(transformMatrix(0,3) - 0.2, transformMatrix(0,3) + 0.3);
    passthrough_filter.filter(*cloud);
*/

    d_cloud_pub2.publish(cloud);
    ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    SACSegmentation<PCLPoint> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);  // 0.015 

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);       
    
    ExtractIndices<PCLPoint> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // true means remove planes
    extract.filter(*cloud);

    //d_cloud_pub2.publish(cloud);
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
   
    float square_size = 0.03f; // 0.02f
    Point3f offset; // pattern offsets
    offset.z = 0.00f; 
    offset.y = -0.075f;  // -0.05f;

    vector<Point3f> ideal_pts;
    vector<Point2f> out;
    Mat rvec, tvec;

    if (patternFoundBw)
    {
        offset.x = -0.23f;   // -0.157
        
        vector<Point3f> ideal_pts_bw = calcChessboardCorners(grid_size, square_size, offset);
        ideal_pts.insert(ideal_pts.end(), ideal_pts_bw.begin(), ideal_pts_bw.end());
        out.insert(out.end(), out_bw.begin(), out_bw.end());
    }

    if (patternFoundWb)
    {
        offset.x = 0.11f; // 0.075;

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
    	//	corners.push_back(Point3f(float((2*j + i % 2)*squareSize),float(i*squareSize), 0));
    
    return corners;
}

void Recorder::draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T)
{
    if (R.empty() || T.empty())
        return;
    float scale = 0.065;
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

pcl::PointCloud<SHOT352>::Ptr Recorder::databasialize(PCLPointCloud inputPtr)
{
    cout << "start databasialize" << endl;
    vector<int> keypoints;
  //  cout << "made new keypoints" << endl;
    PointCloudOut sampled;
    sample(inputPtr, sampled);
    cout << "omdat Marten er blij van wordt" << endl;
    keypoints.insert(keypoints.begin(), sampled.points.begin(), sampled.points.end());
//    cout << "insertion success" << endl;
    pcl::PointCloud<Normal>::Ptr normal(new pcl::PointCloud<Normal>);
    normal = computeNormal(inputPtr);
    cout << "Eric stiekem ook" << endl;
    pcl::PointCloud<SHOT352>::Ptr SHOTOneClust(new pcl::PointCloud<SHOT352>);
    getSHOTDescriptors(inputPtr, keypoints, normal, *SHOTOneClust);
    cout << "finished databasialize" << endl;
    return SHOTOneClust;
}


void Recorder::sample(PCLPointCloud inputImg, PointCloudOut &output)
{
    PCLPointCloudPtr inputPtr;
    inputPtr = inputImg.makeShared();

    pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint>);
    tree->setInputCloud(inputPtr);

    UniformSampling<PCLPoint> uSampler;

    uSampler.setInputCloud(inputPtr);
    uSampler.setRadiusSearch(0.015f);
    uSampler.setSearchMethod(tree);

    uSampler.compute(output);
}

pcl::PointCloud<Normal>::Ptr Recorder::computeNormal(PCLPointCloud inputCloud)
{
    // Create the normal estimation class, and pass the input dataset to it
  PCLPointCloudPtr inputPtr;
  inputPtr = inputCloud.makeShared();
  NormalEstimation<PCLPoint, Normal> ne;
  ne.setInputCloud (inputPtr);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<Normal>::Ptr cloudNormals (new pcl::PointCloud<Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloudNormals);
    
  return cloudNormals;
}

void Recorder::getSHOTDescriptors(PCLPointCloud inputImg, vector<int> keypoints, pcl::PointCloud<Normal>::Ptr normals, pcl::PointCloud<SHOT352> &output)
{
    PCLPointCloudPtr inputPtr;
    inputPtr = inputImg.makeShared();
    PointIndices::Ptr keyIndices(new PointIndices);
    keyIndices->indices = keypoints;

    pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint>);
    tree->setInputCloud(inputPtr);


    pcl::SHOTEstimation<PCLPoint, Normal, SHOT352> shot;
    shot.setSearchMethod (tree); //kdtree
    shot.setIndices (keyIndices); //keypoints
    shot.setInputCloud (inputPtr); //input
    shot.setInputNormals(normals);//normals
    shot.setRadiusSearch (0.06); //support
    shot.compute (output); //descriptors
}

vector<double> Recorder::getSize(PCLPointCloud inputCld)
{
    double min_x = DBL_MAX, min_y = DBL_MAX, min_z = DBL_MAX;
    double max_x = -DBL_MAX, max_y = -DBL_MAX, max_z = -DBL_MAX;

    vector<double> of_the_jedi;

    size_t max_x_idx = 0, max_y_idx = 0, min_x_idx = 0, min_y_idx = 0;

    for (size_t idx = 0; idx != inputCld.points.size(); ++idx)
    {
        if (inputCld.points[idx].x < min_x)
        {
            min_x = inputCld.points[idx].x;
            min_x_idx = idx;
        }
        if (inputCld.points[idx].x > max_x)
        {
            max_x_idx = idx;
            max_x = inputCld.points[idx].x;
        }

        if (inputCld.points[idx].y < min_y)
        {
            min_y_idx = idx;
            min_y = inputCld.points[idx].y;
        }
        if (inputCld.points[idx].y > max_y)
        {
            max_y_idx = idx;
            max_y = inputCld.points[idx].y;
        }

        if (inputCld.points[idx].z < min_z)
            min_z = inputCld.points[idx].z;
        
        if (inputCld.points[idx].z > max_z)
            max_z = inputCld.points[idx].z;

    }

    of_the_jedi.push_back(max_x - min_x);
    of_the_jedi.push_back(max_y - min_y);
    of_the_jedi.push_back(max_z - min_z);

    return of_the_jedi;
}

void Recorder::findDims(vector<double> xs, vector<double> ys, vector<double> zs, string dirname)
{
    double stdDevX;
    double stdDevY;
    double stdDevZ;

    accumulator_set<double, stats<tag::variance> > accX;
    for_each(xs.begin(), xs.end(), boost::bind<void>(boost::ref(accX), _1));
    stdDevX = sqrt(variance(accX));

    accumulator_set<double, stats<tag::variance> > accY;
    for_each(ys.begin(), ys.end(), boost::bind<void>(boost::ref(accY), _1));
    stdDevY = sqrt(variance(accY));

    accumulator_set<double, stats<tag::variance> > accZ;
    for_each(zs.begin(), zs.end(), boost::bind<void>(boost::ref(accZ), _1));
    stdDevZ = sqrt(variance(accZ));

    double min_x = *min_element(xs.begin(),xs.end());
    double max_x = *max_element(xs.begin(),xs.end());
    double min_y = *min_element(ys.begin(),ys.end());
    double max_y = *max_element(ys.begin(),ys.end());
    double min_z = *min_element(zs.begin(),zs.end());
    double max_z = *max_element(zs.begin(),zs.end());

    string fileLoc = dirname + "/size.txt";
    ofstream file(fileLoc.c_str());

    file << min_x - (2 * stdDevX) << endl;
    file << max_x + (2 * stdDevX) << endl;
    file << min_y - (2 * stdDevY) << endl;
    file << max_y + (2 * stdDevY) << endl;
    file << min_z - (2 * stdDevZ) << endl;
    file << max_z + (2 * stdDevZ) << endl;

    file.close();
}
