#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sys/stat.h>
#include "readxslx.h"
#include "derivatives.h"
#include "RBDEqns.h"
#include "rk4.h"
#include "read_controls.h"
#include "controller.h"

// Variables to be read from controls file
double tfinal;
double dt;
bool loop;
double pi=3.1415926;
double deg2rad=pi/180.0;
double rad2deg=180.0/pi;
double set_pitch=20*deg2rad;
const char filename[] = "meta/C5A.xlsx";
Eigen::Matrix<double, 4, 1> Controls;

bool fileExists(const char* path);
    

int main(int argc, char* argv[]);