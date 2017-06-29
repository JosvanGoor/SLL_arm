#include "colorblob.ih"

void Colorblob::updateVisualState()
{
	if (d_visual_state++ == OBJECT)
		d_visual_state = RGB;
}
