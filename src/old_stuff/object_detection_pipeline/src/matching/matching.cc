#include <object_detection_pipeline/matching/matching.h>

Matching::Matching()
{
}

void Matching::loadFiles(vector<Object> &objects)
{
    d_objects = objects;
}

void Matching::match(pcl::PointCloud<DescriptorType>::Ptr &descriptors, Keypoint<Point, int>::PointCloudOut &keypoints, PCLPointCloudPtr &cloud)
{
    if (descriptors->size() == 0)
        return;

    vector<float> vDistances;

    KdTreeFLANN<DescriptorType> match_search; // for matching the descriptors


    for (size_t ido = 0; ido < d_objects.size(); ++ido) // for each object side
    {
    	search::KdTree<DescriptorType>::Ptr objTree (new search::KdTree<DescriptorType>);
		objTree->setInputCloud(d_objects.at(ido).descriptors); // database object

		double distSum = 0.0;

		size_t count = 0;
		for (size_t idx = 0; idx != descriptors->size(); ++idx)
		{
			if (pcl_isnan(descriptors->at(idx).descriptor[0]))
			{
				distSum = 999.99f;
				break;
			}


			vector<int> bearNecessities(1);
			vector<float> dist(1);
			int greatSuccess = objTree->nearestKSearch(descriptors->at(idx), 1, bearNecessities, dist);

			float threshold = 0.25;

			if (greatSuccess)
				distSum += dist[0];
			else
				distSum += 1.0; //maximum distance for SHOT descriptors is 1
		}

		distSum /= descriptors->size();


		if (distSum <= 0.3)
		{
			ROS_INFO_STREAM("Distance: " << distSum);

			for (size_t idp = 0; idp < cloud->points.size(); ++idp)
			{
				cloud->points.at(idp).g = 255;
				cloud->points.at(idp).r = 0;
				cloud->points.at(idp).b = 0;
			}
		}
    }

/*
    if (!pcl_isfinite(d_objects.->at(idd).descriptor[0])) //skipping NaNs
    	continue;
    // for each object, the database
 //   for (size_t ido = 0; ido < d_objects.size(); ++ido)
 //   {
  //  	ROS_INFO_STREAM("IDO:" << ido);
  //      match_search.setInputCloud(d_objects.at(ido).descriptors);
        match_search.setInputCloud(descriptors);
        // calculate the distance between cluster and object-side
        // for each descriptor of the objects
        float distance = 0;
        size_t count = 0;
        vector<float> tDis;

        ROS_INFO_STREAM("Descriptors: " << descriptors->size());

        for (size_t idd = 0; idd < d_objects.at()->size(); ++idd)
        {
            vector<int> neigh_indices (1);
            vector<float> neigh_sqr_dists (1);



            float sd_distance = 0.25f; // threshold for descriptor distance

            int found_neighs = match_search.nearestKSearch(descriptors->at(idd), 1, neigh_indices, neigh_sqr_dists);

            if (found_neighs)
                dist.at(idd) = neigh_sqr_dists[0];
            else
                dist.at(idd) = 1.0f;

            ++count;
        }

        for (size_t ids = 0; ids < dist.size(); ++ids)
        	distance += dist.at(ids);

        distance /= count;
        vDistances.push_back(distance);

        if (distance <= 0.5f)
        {
            ROS_INFO_STREAM("DISTANCE: " << distance);
        }
 //   }
*/

}

