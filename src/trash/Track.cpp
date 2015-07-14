/*
 * Track.cpp
 *
 *  Created on: Nov 8, 2014
 *      Author: davy
 */

#include "Track.h"

Track::Track(double a, double b, double g)
{
	this->a = a;
	this->b = b;
	this->g = g;
	x = 0; y = 0; width = 0; height = 0;
	vx = 0; vy = 0; vwidth = 0; vheight = 0;
	ax = 0; ay = 0; awidth = 0; aheight = 0;
	initialized = 0;
	old_time = 0;
	unassoCount = 0;
}

int Track::coincide(cv::Rect *boundingBox)
{
	cv::Rect coincide = cv::Rect(x, y, width, height) & *boundingBox;
	return coincide.area();
}
void Track::predict_update()
{
     double current_time = (double)cv::getTickCount();
     double T = (current_time - old_time)/cv::getTickFrequency();

	 x = x + T*vx + 0.5*T*T*ax;
	 y = y + T*vy + 0.5*T*T*ay;
	 width = width + T*vwidth + 0.5*T*T*awidth;
	 height = height + T*vheight + 0.5*T*T*aheight;

	 vx = vx + T*ax;
	 vy = vy + T*ay;
	 vwidth = vwidth + T*awidth;
	 vheight = vheight + T*aheight;

	 old_time = current_time;
	 unassoCount++;
}
void Track::predict_update(cv::Rect *boundingBox)
{
	double current_time = (double)cv::getTickCount();
	double T = (current_time - old_time)/cv::getTickFrequency();

	x = x + T*vx + 0.5*T*T*ax;
	y = y + T*vy + 0.5*T*T*ay;
	width = width + T*vwidth + 0.5*T*T*awidth;
	height = height + T*vheight + 0.5*T*T*aheight;

	 vx = vx + T*ax;
	 vy = vy + T*ay;
	 vwidth = vwidth + T*awidth;
	 vheight = vheight + T*aheight;

	int rx = boundingBox->x - x;
	x = x + a*rx;
	vx = vx + b/T*rx;
	ax = ax + g/(T*T)*rx;
	int ry = boundingBox->y -y;
	y = y + a*ry;
	vy = vy + b/T*ry;
	ay = ay + g/(T*T)*ry;
	int rwidth = boundingBox->width -width;
	width = width + a*rwidth;
	vwidth = vwidth + b/T*rwidth;
	awidth = awidth + g/(T*T)*rwidth;
	int rheight = boundingBox->height - height;
	height = height + a*rheight;
	vheight = vheight + b/T*rheight;
	aheight = aheight + g/(T*T)*rheight;

	old_time = current_time;
	unassoCount = 0;
	initialized = 1;
}
cv::Rect Track::getBoundingBox()
{
	return cv::Rect(x, y, width, height);
}
int Track::getUnassoCount()
{
	return unassoCount;
}
Track:: ~Track()
{

}
