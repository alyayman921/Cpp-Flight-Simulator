#include "xplane_integration.hpp"

std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

int xplanec::initxpc() {
  const char *IP = "127.0.0.1";
  sock = openUDP(IP);
  float tVal[1];
  int tSize = 1;

  gear[0][0]=67;
  thrust[0][0]=35;
  controls[0][0]=11;

  for(int i=1;i<9;i++){
    gear[0][i]=0;
  }
  for(int i=1;i<9;i++){
    spoilers[0][i]=0;
    spoilers[1][i]=0;
  }
  if (getDREF(sock, "sim/test/test_float", tVal, &tSize) < 0) {
    // this is a failure
    return 1;
    } else {
    getPOSI(sock, POSI, 0);
    lat0 = POSI[0]; // Lat
    lon0 = POSI[1]; // Lon
    pauseSim(sock, 0); // Sending 0 to unpause
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 9; j++) {
        // Initialize data values to -998 to not overwrite values.
        data[i][j] = -998;
      }
    }
  }
  if(lat0==0.0 && lon0==0.0){
   return 1;
  }else{
    std::cout<<"\nLatitude from X plane : "<<lat0<<'\n';
    std::cout<<"Longitude from X plane : "<<lon0<<'\n';
    return 0;
  }
}

xplanec::~xplanec() {
  pauseSim(sock,1);
  closeUDP(sock);
}
int xplanec::updateXplane(Matrix &states, flight_path &str_h, Matrix &Controls) {
  // Real Time Sync
  t2 = std::chrono::steady_clock::now(); // To hold the Simulation time to real time
  t3 = std::chrono::steady_clock::now(); // To Send data at 30 messagese/s
  while (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t0).count() <
         dt * 1e9) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(50));
    t2 = std::chrono::steady_clock::now();
  }
  t0 = t2;
  data[0][0] = 3; // Velocity
  data[0][1] = states.data[0][0]*0.308;
  data[0][2] = states.data[1][0]*0.308;
  data[0][3] = states.data[2][0]*0.308;

  data[1][0] = 16; // PQR
  // data[1][1] = 0;
  // data[1][2] = 0;
  // data[1][3] = 0;
  data[1][1] = states.data[3][0]*rad2deg;;
  data[1][2] = states.data[4][0]*rad2deg;;
  data[1][3] = states.data[5][0]*rad2deg;;

  data[2][0] = 17; // Euler
  data[2][1] = states.data[7][0]*rad2deg;
  data[2][2] = states.data[6][0]*rad2deg;
  data[2][3] = states.data[8][0]*rad2deg;

  // Control Surfaces? doesn't work either way for some reason, but keep it degrees
  // controls[0][1] = -Controls.data[0][0];
  // controls[0][2] = Controls.data[1][0];
  //   thrust[0][1] = Controls.data[2][0];
  // controls[0][3] = Controls.data[3][0];
  controls[0][1] = -Controls.data[0][0]*rad2deg;
  controls[0][2] = Controls.data[1][0]*rad2deg;
    thrust[0][1] = Controls.data[2][0];
  controls[0][3] = Controls.data[3][0]*rad2deg;
  // std::cout<<"aileron " <<controls[0][1]<<'\n';
  // update position in sim
  POSI[0] = lat;     // Lat
  POSI[1] = lon;     // Lon
  POSI[2] = str_h.h/10; // Alt
  while (std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t1).count() < 1/send_rate *1e9) { // 20 ms for 50 messages/sec
    std::this_thread::sleep_for(std::chrono::nanoseconds(50));
    t3 = std::chrono::steady_clock::now();
  }
  sendPOSI(sock, POSI, 3, 0);
  sendDATA(sock, data, 3);
  sendDATA(sock, gear,1);
  sendDATA(sock, thrust,1);
  sendDATA(sock, controls,1);
  sendDATA(sock, spoilers,2);
  return 0;
}
