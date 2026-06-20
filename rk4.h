#pragma once

#include <cstdlib>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "RBDEqns.h"

class rk4{
    private:
        int i;
        float dt;
        float tfinal;
        int N_steps;
        float *time_v;
        Eigen::Matrix<float,9,1> *state_history;

    public:
        rk4(float dt, float tfinal) {
            this->dt = dt;
            this->tfinal = tfinal;
            this->N_steps = (int)(tfinal / dt);
            this->time_v = (float*)malloc(N_steps * sizeof(float));
            this->state_history = (Eigen::Matrix<float,9,1>*)malloc((N_steps+1) * sizeof(Eigen::Matrix<float,9,1>));

            for (int i = 0; i < N_steps; i++) {
                time_v[i] = i * dt;
            }
        }

        // Classic 4th-order Runge-Kutta integration of states_dot = RBDobj.Equations(states).
        // Equations() is a pure function of its input vector (it unpacks v/omega/euler
        // internally), so no setState() call is needed before each stage evaluation.
        Eigen::Matrix<float,9,1>* rk4_solver(RBDsolve &RBDobj, Eigen::Matrix<float,9,1> states0){
            Eigen::Matrix<float,9,1> y = states0;
            Eigen::Matrix<float,9,1> k1, k2, k3, k4;

            // Store initial state
            state_history[0] = states0;

            for (int step = 0; step < N_steps; step++) {
                // k1 = f(t, y)
                k1 = RBDobj.Verify(y);

                // k2 = f(t + dt/2, y + dt*k1/2)
                k2 = RBDobj.Verify(y + 0.5f * dt * k1);

                // k3 = f(t + dt/2, y + dt*k2/2)
                k3 = RBDobj.Verify(y + 0.5f * dt * k2);

                // k4 = f(t + dt, y + dt*k3)
                k4 = RBDobj.Verify(y + dt * k3);

                // y_{n+1} = y_n + dt/6*(k1 + 2k2 + 2k3 + k4)
                y = y + dt * (k1 + 2.0f * k2 + 2.0f * k3 + k4) / 6.0f;

                // Store state
                state_history[step + 1] = y;
            }

            return state_history;
        }

        void free_results() {
            free(time_v);
            free(state_history);
        }
};