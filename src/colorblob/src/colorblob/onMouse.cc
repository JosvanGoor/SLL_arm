#include "colorblob.ih"


void Colorblob::onMouse(int event, int x, int y, int, void *param)
{
	Colorblob *self = static_cast<Colorblob*>(param);
	if (event == cv::EVENT_LBUTTONUP)
		self->updateVisualState();
}
