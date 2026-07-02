#pragma once

#include <stdio.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include <math.h>

typedef struct{
  float Inertia_temp[4];
  float Vtotal,theta0,z0,m,g;
  float Xu,Zu,Mu,Xw,Zw,Mw,Zwd,Zq,Mwd,Mq,Xde,Zde,Mde,Xdth,Zdth,Mdth;
  float G,Yb,Yv,LB ,NB, LP , NP, LR,L_DR , NR, L_DA,N_DA, N_DR,Yda,Ydr;
  float Lv, Nv;
  float SD_Long_temp[16];
  float SD_Lat_dash[14];
  Eigen::Vector3f V0;
  Eigen::Vector3f mg0;
  Eigen::VectorXf time_vector;
  Eigen::Matrix<float,3,1> omega0; 
  Eigen::Matrix<float,3,1> euler0;
  Eigen::Matrix<float,3,3> Inertia;
  Eigen::Matrix<float,9,1> states0;
  Eigen::Matrix<float, 6, 7> SD;
  Eigen::Matrix<float, 6, 4> CD;
  Eigen::Matrix<float, 10, 10> T;
  Eigen::Matrix<float, 10, 1> lat_dash;
  Eigen::Matrix<float, 10, 1> Lat_dash;
  Eigen::Matrix<float, 14, 1> SD_Lat;
}aircraft_data;

inline aircraft_data sorting(raw_data raw){
  aircraft_data d;
  // States Node
  for(int i=5;i<8;i++){
    d.V0[i-5]=raw.B[i];
    d.states0[i-5]=raw.B[i];

  }
  d.Vtotal=sqrt(pow(d.V0[0],2)+pow(d.V0[1],2)+pow(d.V0[2],2));
  d.theta0=raw.B[12]; // rad 
  d.m=raw.B[52];
  d.g=raw.B[53];
  d.mg0 << -sin(d.theta0) , 0 , cos(d.theta0);
  d.mg0=d.m*d.g *d.mg0;
  d.z0=raw.B[16];
  for (int i=8;i<11;i++){
    d.omega0[i-8]=raw.B[i];
    d.states0[i-8+3]=raw.B[i];
  }
  for (int i=11;i<14;i++){
    d.euler0[i-11]=raw.B[i];
    d.states0[i-11+6]=raw.B[i];
  }
  for(int i=54;i<58;i++){   // ixx iyy izz ixz
  d.Inertia_temp[i-54] = raw.B[i]; 
  }

  d.Inertia<< d.Inertia_temp[0],      0,            d.Inertia_temp[3],
                    0,          d.Inertia_temp[1],        0,
              d.Inertia_temp[3],      0 ,            d.Inertia_temp[2];

  // Long Stability Derivatives
  d.Xu=raw.B[22];d.Zu=raw.B[23];d.Mu=raw.B[24];
  d.Xw=raw.B[25];d.Zw=raw.B[26];d.Mw=raw.B[27];
  d.Zwd=raw.B[28];d.Zq=raw.B[29];d.Mwd=raw.B[30];
  d.Mq=raw.B[31];d.Xde=raw.B[32];d.Zde=raw.B[33];
  d.Mde=raw.B[34];d.Xdth=raw.B[35];d.Zdth=raw.B[36];d.Mdth=raw.B[37];
  
  // Lat Stability Derivatives
  for(int i=38;i<52;i++){
    d.SD_Lat_dash[i-38] = raw.B[i];
  }
  // rbna m3ak fe el cleanup
  d.Yda=raw.B[46];d.Ydr=raw.B[47];
  d.G=1/(1-(pow(d.Inertia_temp[3],2)/(d.Inertia_temp[0]*d.Inertia_temp[2])));
  d.Yv = d.SD_Lat_dash[0];d.Yb = d.SD_Lat_dash[1];d.LB = d.SD_Lat_dash[2];
  d.NB = d.SD_Lat_dash[3];d.LP = d.SD_Lat_dash[4];d.NP = d.SD_Lat_dash[5];
  d.LR = d.SD_Lat_dash[6];d.NR = d.SD_Lat_dash[7];d.L_DA = d.SD_Lat_dash[10];
  d.N_DA = d.SD_Lat_dash[11];d.L_DR = d.SD_Lat_dash[12];d.N_DR = d.SD_Lat_dash[13];
  d.lat_dash << d.LB, d.NB, d.LP, d.NP, d.LR, d.NR, d.L_DA, d.N_DA, d.L_DR, d.N_DR;
  
  d.T << d.G,d.G * d.Inertia_temp[3] / d.Inertia_temp[0], 0, 0, 0, 0, 0, 0, 0, 0,
       d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G,0, 0, 0, 0, 0, 0, 0, 0,
       0,  0,d.G, d.G * d.Inertia_temp[3] / d.Inertia_temp[0], 0, 0, 0, 0, 0, 0,
       0,  0,d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G, 0, 0, 0, 0, 0, 0,
       0,  0,0, 0, d.G, d.G * d.Inertia_temp[3] / d.Inertia_temp[0], 0, 0, 0, 0,
       0,  0,0, 0, d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G, 0, 0, 0, 0,
       0,  0,0, 0, 0, 0, d.G, d.G * d.Inertia_temp[3] / d.Inertia_temp[0], 0, 0,
       0,  0,0, 0, 0, 0, d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G, 0, 0,
       0,  0,0, 0, 0, 0, 0, 0, d.G, d.G * d.Inertia_temp[3] / d.Inertia_temp[0],
       0,  0,0, 0, 0, 0, 0, 0, d.G * d.Inertia_temp[3] / d.Inertia_temp[2], d.G;
  // lat temp = T dash-1
  d.Lat_dash=d.T.lu().solve(d.lat_dash);
  d.SD_Lat << d.Yv,
              d.Yb, // 1
              d.Lat_dash[0],//LB_dash 2
              d.Lat_dash[1],//NB_dash 3
              d.Lat_dash[2],//LP_dash 4 
              d.Lat_dash[3],//NP_dash 5 
              d.Lat_dash[4],//LR_dash 6
              d.Lat_dash[5],//NR_dash 7
              d.Yda,
              d.Ydr,
              d.Lat_dash[6],//L_DA_dash 10
              d.Lat_dash[7],//N_DA_dash 11
              d.Lat_dash[8],//L_DR_dash 12 
              d.Lat_dash[9];//N_DR_dash 13

  d.Lv=d.SD_Lat[2]/d.Vtotal;d.Nv=d.SD_Lat[3]/d.Vtotal;
  d.Yv = d.SD_Lat[0];d.Yb = d.SD_Lat[1];d.LB = d.SD_Lat[2];
  d.NB = d.SD_Lat[3];d.LP = d.SD_Lat[4];d.NP = d.SD_Lat[5];
  d.LR = d.SD_Lat[6];d.NR = d.SD_Lat[7];d.L_DA = d.SD_Lat[10];
  d.N_DA = d.SD_Lat[11];d.L_DR = d.SD_Lat[12];d.N_DR = d.SD_Lat[13];

  // SD Matrix Construction
  d.SD << d.Xu, 0,    d.Xw, 0,    0, 0, 0,
        0,    d.Yv, 0,    0,    0, 0, 0,
        d.Zu, 0,    d.Zw, 0,    d.Zq, 0, d.Zwd,
        0,    d.Lv, 0,    d.LP, 0, d.LR, 0,
        d.Mu, 0,    d.Mw, 0,    d.Mq, 0, d.Mwd,
        0,    d.Nv, 0,    d.NP, 0, d.NR, 0;
  d.CD << 0, d.Xde, d.Xdth, 0,
          d.Yda, 0, 0 , d.Ydr,
          0, d.Zde, d.Zdth,0,
          d.L_DA, 0,0,d.L_DR,
          0,d.Mde ,d.Mdth , 0,
          d.N_DA, 0, 0, d.N_DR;
return d;
}