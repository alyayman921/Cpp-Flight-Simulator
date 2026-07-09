#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>
#include "derivatives.h" 

class RBDsolve{
    private:
        int i;
        double m, w_dot_state, g, cos_theta;
        Eigen::Matrix<double,3,1> v, v_e, v_dot;
        Eigen::Matrix<double,3,1> omega, delta_omega_dot;
        Eigen::Matrix<double,3,1> euler, euler_dot;
        Eigen::Matrix<double,3,3> J, R, I;
        Eigen::Matrix<double,6,7> SD;
        Eigen::Matrix<double,7,1> delta_states;
        Eigen::Matrix<double,6,4> CD;
        Eigen::Matrix<double,4,1> Controls;
        Eigen::Matrix<double,6,1> Aerodynamic_accel;
        Eigen::Matrix<double,3,1> delta_v, delta_omega, delta_F, delta_M;
        Eigen::Matrix<double,9,1> states_dot;
        Eigen::Matrix<double,9,1> states0;
        aircraft_data ac;

    public:
        Eigen::Matrix<double,3,1> F_aero;
        Eigen::Matrix<double,3,1> F_grav;
        Eigen::Matrix<double,3,1> F_b;
        Eigen::Matrix<double,3,1> F_g0;
        Eigen::Matrix<double,3,1> F_g;
        
        // Getters for logging
        Eigen::Matrix<double,6,1> getAerodynamicAccel() const { return Aerodynamic_accel; }
        Eigen::Matrix<double,3,1> getAeroForces() const { return F_aero; }
        Eigen::Matrix<double,3,1> getGravForces() const { return F_grav; }
        Eigen::Matrix<double,3,1> getTotalForces() const { return F_b; }
        double getWDotAccel() const { return w_dot_state; }
        double getVDotZ() const { return v_dot(2); }
        double getCurrentTime() const { return current_time; }
        
        RBDsolve(aircraft_data ac, Eigen::Matrix<double,4,1> Controls);
        Eigen::Matrix<double,3,1> delta(Eigen::Matrix<double,3,1> state, Eigen::Matrix<double,3,1> state0);
        void eulerToRotationMatrix(const Eigen::Matrix<double,3,1>& euler);
        Eigen::Matrix<double,9,1> Equations(Eigen::Matrix<double,9,1> states, double time = 0.0f);
        Eigen::Matrix<double,9,1> Verify(Eigen::Matrix<double,9,1> states);
        void setState(const Eigen::Matrix<double,9,1>& states);
        void updatewdot(double a);
        
    private:
        double current_time;  // Store current time for logging
};