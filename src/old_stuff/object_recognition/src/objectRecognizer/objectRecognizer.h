#ifndef H_OBJREC
#define H_OBJREC

#include "../utils.h"
#include "../planarRemover/planarRemover.h"
#include "../clusterObjects/clusterObjects.h"

#include <pcl/keypoints/keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <borg_pioneer/MemorySrv.h>

class ObjectRecognizer
{
    typedef typename Keypoint<Point, int>::PointCloudOut PointCloudOut;
    typedef SHOT352 descriptorType;
    typedef vector<PointCloud<descriptorType>::Ptr > shotData;
    typedef map<string, shotData> shotDB;
    typedef map<string, vector<double> > dimDB;

    private:
        double d_threshold;
        bool d_publish;
        size_t numFrames;
        size_t orientation;
        string objectName;
        bool d_pause;
        double d_lastCommand;

        bool d_initRos;

        PlanarRemover d_planRemover;
        ClusterObjects d_objClusterer;

        shotDB d_shotDatabase;
        dimDB d_dimDatabase;

        shotData d_inputDescriptors;
        vector<vector<double> > d_inputSizes;
        vector<vector<double> > d_inputPositions;

	    NodeHandle d_rosHandle;
		Publisher d_pubClean;
		Publisher d_pubAll;
	    Publisher d_pubSampled;

        ServiceClient d_clientWriter;
        ServiceClient d_clientReader;

        map<string, size_t> results;
        vector<double> runtimes;
        vector<double> clustertimes;
        vector<double> cleanuptimes;
        vector<double> samplingtimes;
        vector<double> descriptortimes;
        vector<double> classifytimes;

    public:
        ObjectRecognizer();
        void rosrun(const sensor_msgs::PointCloud2ConstPtr &inputMsg);
        void run(PCLPointCloudPtr PCLimg);
        void recordExperiment(const sensor_msgs::PointCloud2ConstPtr &inputMsg);
        void runExperiment();
        tf::TransformListener *tfTransformer;
    private:
        //Transform listener for transform functions


        //Preprocessing of the input image: processPCL.cc
    	void processPCL(const sensor_msgs::PointCloud2ConstPtr &inputMsg);
    	void processPCL(PCLPointCloudPtr PCLimg);

        //filter input image
        PCLPointCloudPtr filterView(PCLPointCloudPtr input, double width, double depth);

    	//color clusters to make them pretty for publishing: colorClusters.cc
    	void colorClusters(const vector<PointIndices> cluster, PCLPointCloudPtr input, bool grey = false);
    	void colorCluster(const PointIndices cluster, PCLPointCloudPtr input, size_t color);
        void colorCluster(const PointIndices cluster, PCLPointCloudPtr input, size_t r, size_t g, size_t b);

        //for cleaning up images: cleanUpImage.cc
    	void cleanUpImage(PCLPointCloudPtr inputCloud, vector<PointIndices> clusterIndices);

        //Get PointClouds for all individual clusters: splitClusters.cc
        vector<PCLPointCloudPtr> splitClusters(PCLPointCloudPtr inputCloud, vector<PointIndices> clusterIndices);
        bool niceSize(PCLPointCloudPtr cluster);

        //Publish all clusters in one image: publishize.cc
        PCLPointCloudPtr publishize(vector<PCLPointCloudPtr> clusterPCLs, vector<vector<int> > indices);

        //For uniform sampling: sample.cc
        vector<vector<int> > getKeypoints(vector<PCLPointCloudPtr> inputCloud, vector<PointCloudOut> outputs);
    	void sample(PCLPointCloudPtr inputImg, PointCloudOut &output);

        //Compute the normals of a 3D image: computeNormals.cc
        vector<PointCloud<Normal>::Ptr> computeNormals(vector<PCLPointCloudPtr> inputClouds);
        PointCloud<Normal>::Ptr computeNormal(PCLPointCloudPtr inputCloud);

        //Estimate descriptors of clusters: estimateDescriptors.cc
        void estimateDescriptors(vector<PCLPointCloudPtr> clusterPCLs, vector<vector<int> > keypoints, vector<PointCloud<Normal>::Ptr> normals);
        void getSHOTDescriptors(PCLPointCloudPtr inputImg, vector<int> keypoints, PointCloud<Normal>::Ptr normals, PointCloud<descriptorType> &output);

        // load the database: loadDB.cc
        void loadDB(string DBlocation = "/home/borg/demo_data");
        shotData getInstances(string path, vector<double> &tmpDimensions);

        // extract the requested features from a (vector of) PCL(s): extractFeatures.cc
        void extractFeatures(vector<PCLPointCloudPtr> clusterPCLs);
        PointCloud<descriptorType>::Ptr extractFeatures(PCLPointCloudPtr cluster);

        // classify the currently viewed feature as one of the classes in the database: classify.cc
        void classify();

        // get size(s) of pointcloud(s): getSize.cc
        vector<double> getExtremes(PCLPointCloudPtr inputCld);
        void getSizes(vector<PCLPointCloudPtr> inputCld);

        //write experiment results to file
        void writeResults(string location);

        //parse message whether to pause or not
        bool jsonParser(bool &pause);
};

#endif
