//
// Created by davy on 4/30/15.
//

#include "PDcontroller.h"


PDcontroller::PDcontroller(double p_gain, double d_gain, double u_limit, double l_limit) {
    setGains(p_gain, d_gain);
    this->u_limit = u_limit;
    this->l_limit = l_limit;
}

void PDcontroller::reset() {
    prev_error = 0;
}

double PDcontroller::computeCommand(double error, ros::Duration dt) {

    double error_diff = error - prev_error;
    prev_error = error;

    if (dt == ros::Duration(0.0) || isnan(error) || isinf(error) || dt < ros::Duration(0.01) )
    {
        cout << "No time difference!" << endl;
        return 0.0;
    }


    cout << "time diff" << dt.toSec() << endl;
    cout << "error_diff" << error_diff << endl;

    double Ed = 0;
    if(dt.toSec() > 0) {
        Ed = error_diff/dt.toSec();
        cout << "diff term :" << d_gain*Ed << endl;

    }
    double command = p_gain*error + d_gain*Ed;
    //output boundaries
    if(command > u_limit) command = u_limit;
    if(command < l_limit ) command = l_limit;
    cout << "Control command : ====   " << command << "    =====" << endl;
    return command;

}

void PDcontroller::setGains(double p_gain, double d_gain) {
    this->p_gain = p_gain;
    this->d_gain = d_gain;
    reset();
}