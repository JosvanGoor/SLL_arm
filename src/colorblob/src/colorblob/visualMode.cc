#include "colorblob.ih"

void Colorblob::visualMode()
{
	cv::Point org(0,10);

	switch (d_visual_state)
	{
	case RGB:
		cv::putText(d_cv_ptr->image, "RGB-Images", org ,1 , 1, cv::Scalar(0,255,0));
		cv::imshow("Colorblob", d_cv_ptr->image);
		break;

	case HSV:
		cv::putText(d_HSV_image, "HSV-Image", org , 1, 1, cv::Scalar(0,255,0));
		cv::imshow("Colorblob", d_HSV_image);
		break;
	
	case FILTER:
		cv::putText(d_filtered_image, "Filtered-Image", org , 1, 1, cv::Scalar(0,255,0));
		cv::imshow("Colorblob", d_filtered_image);
		break;

	case OBJECT:
		cv::putText(d_object_image, "Objects", org, 1, 1, cv::Scalar(0,255,0));
		cv::imshow("Colorblob", d_object_image);
		break;
	}

	cv::waitKey(10);
}
