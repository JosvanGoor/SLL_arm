#include "sift.h"

#include <omp.h> 

using namespace cv; 

const int LOOP_NUM = 1;
const int GOOD_PTS_MAX = 50;
const float GOOD_PORTION = 0.15f;

int64 work_begin1 = 0;
int64 work_end1 = 0;

static void workBegin()
{
	work_begin1 = getTickCount();
}

static void workEnd()
{
	work_end1 = getTickCount() - work_begin1;
}

static double getTimeNow()
{
	return work_end1 /((double)cvGetTickFrequency() * 1000.);
}


set<string> Sift::match(Mat image, bool largeObject)
{
//	double surf_time = 0.;
	
	cv::SiftFeatureDetector detector; 
    cv::SiftDescriptorExtractor extractor;      
    
    // L2 distance based matching. Brute Force Matching
    BruteForceMatcher<cv::L2<float> > matcher;
 
    // Flann-based matching
 //   FlannBasedMatcher matcher;

//	workBegin(); // start timing, temp code not needed
	
	vector<KeyPoint> keypoints;
	Mat descriptors; 
	
	detector.detect(image, keypoints);		 
	extractor.compute(image, keypoints, descriptors);

	set<string> objectsFound;
	
	if (keypoints.size() > 0)
	{
		#pragma omp parallel for
		for (size_t ido = 0; ido < objects.size(); ++ido)
		{
		//	cout << "Thread Nr: " <<  omp_get_thread_num()<< "\n";
			if (true)
		//	if (objects.at(ido).largeObject == largeObject)
			{
				vector<DMatch> matches;
				
				matcher.match(objects.at(ido).descriptors, descriptors, matches);
				 // matching result
				Mat result;
					
				double max_dist = 0; double min_dist = 100;
				
				//-- Quick calculation of max and min distances between keypoints
		//		#pragma omp parallel for
				for( int i = 0; i < objects.at(ido).descriptors.rows; i++ )
				{
					double dist = matches[i].distance;
					if (dist < min_dist) 
						min_dist = dist;
					
					if (dist > max_dist) 
						max_dist = dist;
				}
				
				
				//-- Take only "good" matches (i.e. whose distance is less than 3*min_dist )

				std::vector< DMatch > good_matches;

			//	#pragma omp parallel for ordered schedule(dynamic)
				for( int i = 0; i < objects.at(ido).descriptors.rows; i++ )
				{ 

					if( matches[i].distance < 3 * min_dist )
					{
			//			#pragma omp ordered
						good_matches.push_back(matches[i]); 
					}
				}

				//-- Localize the object
				std::vector<Point2f> obj;
				std::vector<Point2f> scene;

				// try to get rid of false positives, check for minimum of good matches
				if (good_matches.size() > objects.at(ido).descriptors.rows / 3 && good_matches.size() >= 4)
				{
			//		#pragma omp parallel for ordered schedule(dynamic)
					for( int i = 0; i < good_matches.size(); i++ )
					{
						//-- Get the keypoints from the good matches
			//			#pragma omp ordered
						obj.push_back( objects.at(ido).keypoints[ good_matches[i].queryIdx ].pt );

			//			#pragma omp ordered
						scene.push_back( keypoints[ good_matches[i].trainIdx ].pt );
					}
				
					Mat H = findHomography(obj, scene, CV_RANSAC);	  

					//-- Get the corners from the image_1 ( the object to be "detected" )
					std::vector<Point2f> obj_corners(4);
					obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( objects.at(ido).image.cols, 0 );
					obj_corners[2] = cvPoint( objects.at(ido).image.cols, objects.at(ido).image.rows ); obj_corners[3] = cvPoint( 0, objects.at(ido).image.rows );
					std::vector<Point2f> scene_corners(4);

					perspectiveTransform(obj_corners, scene_corners, H);

					// do an extra check to see of object boundary makes sense
					// take the distance from two diagonals
					float dist1 = sqrt( pow(scene_corners[0].x - scene_corners[2].x, 2) + pow(scene_corners[0].y - scene_corners[2].y, 2));	
					float dist2 = sqrt( pow(scene_corners[1].x - scene_corners[3].x, 2) + pow(scene_corners[1].y - scene_corners[3].y, 2));

					size_t upThreshold, lowThreshold;

					upThreshold = sqrt(image.cols * image.cols + image.rows * image.rows);

					lowThreshold = upThreshold / 2;
					
					// diagonal distance should be between the thresholds
					if (dist1 < upThreshold && dist1 > lowThreshold && dist2 < upThreshold && dist2 > lowThreshold)
					{

						size_t countCheck0 = 0;
						size_t countCheck1 = 0;
						size_t countCheck2 = 0;
						size_t countCheck3 = 0;

						// extra check, check if all points are seperated
						for (size_t idpt = 0; idpt < 4; ++idpt)
						{

							if (idpt != 0)
							{	
								float distPt0 = sqrt( pow(scene_corners[0].x - scene_corners[idpt].x, 2) + pow(scene_corners[0].y - scene_corners[idpt].y, 2));

								if (distPt0 > 1)
									++countCheck0;
							}

							if (idpt != 1)
							{
								float distPt1= sqrt( pow(scene_corners[1].x - scene_corners[idpt].x, 2) + pow(scene_corners[1].y - scene_corners[idpt].y, 2));

								if (distPt1 > 1)
									++countCheck1;
							}

							if (idpt != 2)
							{
								float distPt2= sqrt( pow(scene_corners[2].x - scene_corners[idpt].x, 2) + pow(scene_corners[2].y - scene_corners[idpt].y, 2));

								if (distPt2 > 1)
									++countCheck2;
							}

							if (idpt != 3)
							{
								float distPt3= sqrt( pow(scene_corners[3].x - scene_corners[idpt].x, 2) + pow(scene_corners[3].y - scene_corners[idpt].y, 2));

								if (distPt3 > 1)
									++countCheck3;
							}
						}

						if (countCheck0 == 3 && countCheck1 == 3 && countCheck2 == 3 && countCheck3 == 3)
						{
							//-- Draw lines between the corners (the mapped object in the scene - image_2 )
							line( image, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4 );
							line( image, scene_corners[1] , scene_corners[2], Scalar( 0, 255, 0), 4 );
							line( image, scene_corners[2],  scene_corners[3], Scalar( 0, 255, 0), 4 );
							line( image, scene_corners[3], scene_corners[0], Scalar( 0, 255, 0), 4 );


							objectsFound.insert(objects.at(ido).objectName);

							//cout << "Object: " << objects.at(ido).objectName << " found.\n";
						}					
					}	
				}				
			}
		}
			
		// temp code for timing only
//		workEnd();
//		surf_time = getTimeNow();
	//	std::cout << "Sift run time: " << surf_time << " ms" << std::endl<<"\n";
	}	


/*
	if (objectsFound.size() > 0)
	{
		for (set<string>::iterator idof = objectsFound.begin(); idof != objectsFound.end(); ++idof)
		{
			cout << "Object: " << *idof << " found.\n";
		}
	}
*/
	return objectsFound;

	//-- Show detected matches
//	imshow( "Good Matches & Object detection", image );
//    cvWaitKey(1);
     
}
