/*
 * Track.h
 *
 *  Created on: Nov 8, 2014
 *      Author: davy
 */

#ifndef TRACK_H_
#define TRACK_H_

#include <opencv2/opencv.hpp>


class Track {
protected:

	double a, b, g;
	int x,y, width, height;
	double vx, vy, vwidth, vheight;
	double ax, ay, awidth, aheight;
    double old_time;
    int unassoCount;

	public:
	Track(double a, double b, double g);
	void init(cv::Rect *boundingbox);
	int coincide(cv::Rect *boundingBox);
	void predict_update();
	void predict_update(cv::Rect *boundingBox);
	int getUnassoCount();
	cv::Rect getBoundingBox();
	virtual ~Track();
	int initialized;

};

#endif /* TRACK_H_ */
