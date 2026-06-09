#include <iostream>
#include "readxslx.h"
#include "derivatives.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "rk4.h"
#include "RBDSolver.h"

const char filename[]="meta/C5A.xlsx";
float da,de,dth,dr;
Eigen::Matrix <float,4,1> Controls; // Controls<<da,de,dth,dr;
Eigen::Matrix <float,2,3> Results;
const float dt=0.01;const float tfinal=200;

int main(){

//rkupdateData(&data,dt,tfinal);
Controls<<da,de,dth,dr;
raw_data raw=readxlsx(filename); 
aircraft_data c5a=sorting(raw);
RBDsolve myobj (c5a,Controls);
Results=myobj.calcFM();
myobj.RBDequation();
//std::cout<<"Results are:"<<Results<<'\n';

int i,j;


while (true){

}
return 0;
}