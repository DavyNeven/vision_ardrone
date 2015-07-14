//
// Created by davy on 4/30/15.
//

#ifndef VISION_ARDRONE_PDCONTROLLER_H
#define VISION_ARDRONE_PDCONTROLLER_H


#include "../../../../../../opt/ros/hydro/include/ros/duration.h"

using namespace std;

class PDcontroller {

protected:
    double p_gain, d_gain;
    double u_limit, l_limit;
    double prev_error;

public:
    PDcontroller(double p_gain, double d_gain, double u_limit=1, double l_limit=-1);
    void reset();
    double computeCommand(double error, ros::Duration dt);
    void setGains(double p_gain, double d_gain);

};


#endif //VISION_ARDRONE_PDCONTROLLER_H
