#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "xplaneConnect.h"
#include "matrix.h"
#include "structs.h"
#include <chrono>
#include <iostream>
#include <thread>
extern float dt;
extern float deg22rad,rad2deg;
extern float lon,lat;
extern float lon0,lat0;
class xplanec{
  private:
    float data[3][9] = { 0 };
    double POSI[9] = { 0.0 };
    float gear[1][9];
    float controls[1][9];
    float thrust[1][9];
    float spoilers[2][9];
    XPCSocket sock;
    float send_rate=99;
  public:
    int initxpc();
    ~xplanec();
    int updateXplane(Matrix &states,flight_path &str_h, Matrix &Controls);
};
