#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

double pi=3.1415926;
double deg2rad=pi/180.0;
double rad2deg=180.0/pi;

#include "read_controls.h"
struct flight_path{
    double h=40000,v_tot,delta_h_dot,gamma,alpha;
};
flight_path str_h;
double dt;
double set_pitch=0;
double set_vel=0;
double set_alt=0;
double set_heading=0;
bool alt_override=false; // overrides altitude loop straight to pitch control
//#include <sys/stat.h>
#include "readxslx.h"
#include "derivatives.h"
#include "RBDEqns.h"
#include "rk4.h"
#include "controller.h"

// Variables to be read from controls file
double tfinal;
bool loop;
const char filename[] = "meta/C5A.xlsx";
Eigen::Matrix<double, 4, 1> Controls;
bool Autopiloted=true;

// bool fileExists(const char* path);
    
int main(int argc, char* argv[]);