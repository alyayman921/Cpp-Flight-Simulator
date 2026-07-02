#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>
#include "derivatives.h" 

class RBDsolve{
    private:
        int i;
        float m, w_dot_state, g, cos_theta;
        Eigen::Matrix<float,3,1> v, v_e, v_dot;
        Eigen::Matrix<float,3,1> omega, delta_omega_dot;
        Eigen::Matrix<float,3,1> euler, euler_dot;
        Eigen::Matrix<float,3,3> J, R, I;
        Eigen::Matrix<float,6,7> SD;
        Eigen::Matrix<float,7,1> delta_states;
        Eigen::Matrix<float,6,4> CD;
        Eigen::Matrix<float,4,1> Controls;
        Eigen::Matrix<float,6,1> Aerodynamic_accel;
        Eigen::Matrix<float,3,1> delta_v, delta_omega, delta_F, delta_M;
        Eigen::Matrix<float,10,1> states_dot;
        Eigen::Matrix<float,10,1> states0;
        aircraft_data ac;

    public:
        Eigen::Vector3f F_aero;
        Eigen::Vector3f F_grav;
        Eigen::Vector3f F_total;
        Eigen::Vector3f F_g0;
        Eigen::Vector3f F_g;
        
        // Getters for logging
        Eigen::Matrix<float,6,1> getAerodynamicAccel() const { return Aerodynamic_accel; }
        Eigen::Vector3f getAeroForces() const { return F_aero; }
        Eigen::Vector3f getGravForces() const { return F_grav; }
        Eigen::Vector3f getTotalForces() const { return F_total; }
        float getWDotAccel() const { return w_dot_state; }
        float getCurrentTime() const { return current_time; }
        
        RBDsolve(aircraft_data ac, Eigen::Matrix<float,4,1> Controls);
        Eigen::Vector3f delta(Eigen::Vector3f state, Eigen::Vector3f state0);
        void eulerToRotationMatrix(const Eigen::Vector3f& euler);
        Eigen::Matrix<float,10,1> Equations(Eigen::Matrix<float,10,1> states, float time = 0.0f);
        Eigen::Matrix<float,10,1> Verify(Eigen::Matrix<float,10,1> states);
        void setState(const Eigen::Matrix<float,10,1>& states);
        void updatewdot(float a);
        
    private:
        float current_time;  // Store current time for logging
};