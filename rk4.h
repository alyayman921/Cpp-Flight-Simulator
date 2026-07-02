#pragma once

#include <cstdlib>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "RBDEqns.h"
#include "logger.h"  // Include the logger

class rk4{
    private:
        int i;
        float dt;
        float tfinal;
        int N_steps;
        float *time_v;
        Eigen::Matrix<float,9,1> *state_history;
        
        // Loggers
        DataLogger stateLogger;
        DataLogger forceLogger;
        DataLogger accelLogger;
        bool loggersInitialized;

    public:
        rk4(float dt, float tfinal) {
            this->dt = dt;
            this->tfinal = tfinal;
            this->N_steps = (int)(tfinal / dt);
            this->time_v = (float*)malloc(N_steps * sizeof(float));
            this->state_history = (Eigen::Matrix<float,9,1>*)malloc((N_steps+1) * sizeof(Eigen::Matrix<float,9,1>));
            this->loggersInitialized = false;

            for (int i = 0; i < N_steps; i++) {
                time_v[i] = i * dt;
            }
        }
        
        ~rk4() {
            // Loggers will be closed automatically by their destructors
        }
        
        void initializeLoggers() {
            if (loggersInitialized) return;
            
            // Initialize state logger
            stateLogger.init("states");
            std::vector<std::string> stateHeaders = {
                "time", "vx", "vy", "vz", "p", "q", "r", "phi", "theta", "psi"
            };
            stateLogger.writeHeader(stateHeaders);
            
            // Initialize force logger
            forceLogger.init("forces");
            std::vector<std::string> forceHeaders = {
                "time", 
                "F_aero_x", "F_aero_y", "F_aero_z",
                "F_grav_x", "F_grav_y", "F_grav_z",
                "F_total_x", "F_total_y", "F_total_z"
            };
            forceLogger.writeHeader(forceHeaders);
            
            // Initialize aerodynamic acceleration logger
            accelLogger.init("aerodynamic_accel");
            std::vector<std::string> accelHeaders = {
                "time", "ax", "ay", "az", "alpha", "beta", "gamma"
            };
            accelLogger.writeHeader(accelHeaders);
            
            loggersInitialized = true;
            std::cout << "All loggers initialized successfully!" << std::endl;
        }

        Eigen::Matrix<float,9,1>* rk4_solver(RBDsolve &RBDobj, Eigen::Matrix<float,9,1> states0){
            Eigen::Matrix<float,9,1> y = states0;
            Eigen::Matrix<float,9,1> k1, k2, k3, k4;

            // Initialize loggers
            initializeLoggers();

            // Store initial state
            state_history[0] = states0;
            
            // Log initial state (t=0)
            RBDobj.Equations(y, 0.0f);
            stateLogger.logStates(0.0f, y);
            forceLogger.logForces(0.0f, RBDobj.getAeroForces(), RBDobj.getGravForces(), RBDobj.getTotalForces());
            accelLogger.logWithTime(0.0f, RBDobj.getAerodynamicAccel());

            for (int step = 0; step < N_steps; step++) {
                float t = step * dt;
                float t_half = t + dt/2.0f;
                float t_full = t + dt;

                // k1 = f(t, y)
                k1 = RBDobj.Equations(y, t);

                // k2 = f(t + dt/2, y + dt*k1/2)
                k2 = RBDobj.Equations(y + 0.5f * dt * k1, t_half);

                // k3 = f(t + dt/2, y + dt*k2/2)
                k3 = RBDobj.Equations(y + 0.5f * dt * k2, t_half);

                // k4 = f(t + dt, y + dt*k3)
                k4 = RBDobj.Equations(y + dt * k3, t_full);

                // y_{n+1} = y_n + dt/6*(k1 + 2k2 + 2k3 + k4)
                y = y + dt * (k1 + 2.0f * k2 + 2.0f * k3 + k4) / 6.0f;

                // Update w_dot_state
                RBDobj.Equations(y, t + dt);
                RBDobj.updatewdot(RBDobj.getWDotAccel());

                // Store state
                state_history[step + 1] = y;
                
                // Log data at this timestep
                float current_time = (step + 1) * dt;
                stateLogger.logStates(current_time, y);
                forceLogger.logForces(current_time, 
                                     RBDobj.getAeroForces(), 
                                     RBDobj.getGravForces(), 
                                     RBDobj.getTotalForces());
                accelLogger.logWithTime(current_time, RBDobj.getAerodynamicAccel());
            }

            return state_history;
        }

        void free_results() {
            free(time_v);
            free(state_history);
            // Loggers will close automatically in their destructor
            // But you can explicitly close them if needed
            stateLogger.close();
            forceLogger.close();
            accelLogger.close();
        }
};