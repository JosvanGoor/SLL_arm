#include "colorblob.ih"

void Colorblob::sortObservations(vector<cv::Rect>& obs)
{
	for(unsigned int i=0; i < obs.size() - 1; i++)
	{		
		unsigned int index = 0;
		while(index < obs.size() - 1)
		{
			int blob_size_curr = obs[index].width * obs[index].height;
			int blob_size_next = obs[index+1].width * obs[index+1].height;

			cv::Rect temp = obs[index];
			if(blob_size_next > blob_size_curr)
			{
				// Swap elements
				obs[index] = obs[index+1];
				obs[index+1] = temp;
			}

			index++;
		}
	}
}