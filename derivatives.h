#include <stdio.h>

typedef struct{
  float Inertia[4];
  int N_steps; // N of time steps 
  float dt, tfinal,Vtotal,theta0,z0,m,g;
  float Xu,Zu,Mu,Xw,Zw,Mw,Zwd,Zq,Mwd,Mq,Xde,Zde,Mde,Xdth,Zdth,Mdth;
  float G,Yb,Yv,LB ,NB, LP , NP, LR,L_DR , NR, L_DA,N_DA, N_DR;
  float Lv, Nv;
  float V[3];
  float SD_Long_temp[16];
  float SD_Lat_temp[14];
  float SD[6][7];
  float time_vector[];
}aircraft_data;

aircraft_data sorting(raw_data raw){
  aircraft_data d;
  // States Node
  for(int i=5;i<8;i++){
    d.V[i-5]=raw.B[i];
  }
  d.Vtotal=sqrt(pow(d.V[0],2)+pow(d.V[1],2)+pow(d.V[2],2));
  d.theta0=raw.B[12]; // rad 
  d.z0=raw.B[16];
  //printf("%f\n",d.theta0);
  d.m=raw.B[52];
  d.g=raw.B[53];

  for(int i=54;i<58;i++){   // ixx iyy izz ixz
  d.Inertia[i-54] = raw.B[i]; 
  }
  // Long Stability Derivatives
  d.Xu=raw.B[22];d.Zu=raw.B[23];d.Mu=raw.B[24];
  d.Xw=raw.B[25];d.Zw=raw.B[26];d.Mw=raw.B[27];
  d.Zwd=raw.B[28];d.Zq=raw.B[29];d.Mwd=raw.B[30];
  d.Mq=raw.B[31];d.Xde=raw.B[32];d.Zde=raw.B[33];
  d.Mde=raw.B[34];d.Xdth=raw.B[35];d.Zdth=raw.B[36];d.Mdth=raw.B[37];
  //d.SD_Long={[SD_La]}
  
  // Lat Stability Derivatives
  for(int i=38;i<52;i++){
    d.SD_Lat_temp[i-38] = raw.B[i];
  }
  d.G=1/(1-(pow(d.Inertia[3],2)/(d.Inertia[0]*d.Inertia[2])));
  d.Yv = d.SD_Lat_temp[0];
  d.Yb = d.SD_Lat_temp[1];
  d.LB = d.SD_Lat_temp[2];
  d.NB = d.SD_Lat_temp[3];
  d.LP = d.SD_Lat_temp[4];
  d.NP = d.SD_Lat_temp[5];
  d.LR = d.SD_Lat_temp[6];
  d.NR = d.SD_Lat_temp[7];
  d.L_DA = d.SD_Lat_temp[10];
  d.N_DA = d.SD_Lat_temp[11];
  d.L_DR = d.SD_Lat_temp[12];
  d.N_DR = d.SD_Lat_temp[13];

  d.Lv=d.LB/d.Vtotal;
  d.Nv=d.NB/d.Vtotal;

  // SD Matrix Construction
  d.SD[0][0] = d.Xu; d.SD[0][1] = 0;    d.SD[0][2] = d.Xw; d.SD[0][3] = 0;
  d.SD[0][4] = 0;    d.SD[0][5] = 0;    d.SD[0][6] = 0;

  d.SD[1][0] = 0;    d.SD[1][1] = d.Yv; d.SD[1][2] = 0;    d.SD[1][3] = 0;
  d.SD[1][4] = 0;    d.SD[1][5] = 0;    d.SD[1][6] = 0;

  d.SD[2][0] = d.Zu; d.SD[2][1] = 0;    d.SD[2][2] = d.Zw; d.SD[2][3] = 0;
  d.SD[2][4] = d.Zq; d.SD[2][5] = 0;    d.SD[2][6] = d.Zwd;

  d.SD[3][0] = 0;    d.SD[3][1] = d.Lv; d.SD[3][2] = 0;    d.SD[3][3] = d.LP;
  d.SD[3][4] = 0;    d.SD[3][5] = d.LR; d.SD[3][6] = 0;

  d.SD[4][0] = d.Mu; d.SD[4][1] = 0;    d.SD[4][2] = d.Mw; d.SD[4][3] = 0;
  d.SD[4][4] = d.Mq; d.SD[4][5] = 0;    d.SD[4][6] = d.Mwd;

  d.SD[5][0] = 0;    d.SD[5][1] = d.Nv; d.SD[5][2] = 0;    d.SD[5][3] = d.NP;
  d.SD[5][4] = 0;    d.SD[5][5] = d.NR; d.SD[5][6] = 0;
return d;
}