#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>
#include <stdlib.h>
#include <math.h>

struct rkdata;
class RBDsolve;

class rk4{
    private:
        int i;
        float dt;
        float tfinal;
        int N_steps;
        float *time_v;

    public:
        rk4(struct rkdata *data, float dt, float tfinal) {

            this->dt = dt;
            this->tfinal = tfinal;
            this->N_steps = (int)(tfinal /dt);
            this->time_v = (float*)malloc(N_steps * sizeof(float));

            for (int i = 0; i < data->N_steps; i++) {
                data->time_v[i] = i * dt;
            }
        }


        Eigen::Vector3f rk4_solver(RBDsolve *obj, Eigen::Vector3f(RBDsolve::*function)() ){      
            float t = 0;
            Eigen::Vector3f y;
            Eigen::Vector3f k1, k2, k3, k4;
            y = y0;
            for (t;t<tfinal,t+dt){
                function();
            }
        }


        void free_results() {
        }

};
