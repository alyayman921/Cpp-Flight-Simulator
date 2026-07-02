#include "readxslx.h"
#include "derivatives.h"
#include "RBDEqns.h"

RBDsolve::RBDsolve(aircraft_data ac, Eigen::Matrix<float,4,1> Controls){
    this->ac = ac; this->g = ac.g; this->m = ac.m; this->I = ac.Inertia;
    this->F_g0 = ac.mg0; this->v = ac.V0;
    this->omega = ac.omega0; this->euler = ac.euler0;
    this->SD = ac.SD; this->CD = ac.CD;
    this->w_dot_state = 0;
    this->Controls = Controls;
    this->current_time = 0.0f;
    
    // Initialize force vectors
    F_aero.setZero();
    F_grav.setZero();
    F_total.setZero();
}

Eigen::Vector3f RBDsolve::delta(Eigen::Vector3f state, Eigen::Vector3f state0){
    return state - state0;
}

void RBDsolve::eulerToRotationMatrix(const Eigen::Vector3f& euler) {
    double phi = euler(0); double c_phi = std::cos(phi); double s_phi = std::sin(phi);
    double theta = euler(1); double c_theta = std::cos(theta); double s_theta = std::sin(theta);
    double psi = euler(2); double c_psi = std::cos(psi); double s_psi = std::sin(psi);

    R << c_psi * c_theta,
         c_psi * s_theta * s_phi - s_psi * c_phi,
         c_psi * s_theta * c_phi + s_psi * s_phi,

         s_psi * c_theta,
         s_psi * s_theta * s_phi + c_psi * c_phi,
         s_psi * s_theta * c_phi - c_psi * s_phi,

        -s_theta,
         c_theta * s_phi,
         c_theta * c_phi;
}

Eigen::Matrix<float,9,1> RBDsolve::Verify(Eigen::Matrix<float,9,1> states){
    float y1 = states(0);
    float y2 = states(1);
    float t  = states(2);

    Eigen::Matrix<float,9,1> states_dot2;
    states_dot2.setZero();

    states_dot2(0) = std::sin(t) + std::cos(y1) + std::cos(y2);
    states_dot2(1) = std::cos(t) + std::sin(y2) + std::cos(y1);
    states_dot2(2) = 1.0f;

    return states_dot2;
}

Eigen::Matrix<float,9,1> RBDsolve::Equations(Eigen::Matrix<float,9,1> states, float time){
    v = states.segment<3>(0);
    omega = states.segment<3>(3);
    euler = states.segment<3>(6);
    current_time = time;

    // calculate forces and moments
    delta_v = delta(v, ac.V0);
    delta_omega = delta(omega, ac.omega0);
    // Build delta_states
    delta_states(0) = delta_v(0);
    delta_states(1) = delta_v(1);
    delta_states(2) = delta_v(2);
    delta_states(3) = delta_omega(0);
    delta_states(4) = delta_omega(1);
    delta_states(5) = delta_omega(2);
    delta_states(6) = w_dot_state;

    Aerodynamic_accel = SD * delta_states + CD * Controls;

    // Compute aerodynamic forces
    for (i = 0; i < 3; i++){
        delta_F[i] = Aerodynamic_accel[i] * m;
        F_aero[i] = delta_F[i];  // Store aerodynamic forces
    }
    
    // Compute moments
    for (i = 3; i < 6; i++){
        delta_M[i-3] = Aerodynamic_accel[i] * I.coeff(i-3, i-3);
    }

    // Compute gravitational forces
    F_grav(0) = -m * g * sin(euler[1]) - F_g0[0];
    F_grav(1) =  m * g * cos(euler[1]) * sin(euler[0]) - F_g0[1];
    F_grav(2) =  m * g * cos(euler[1]) * cos(euler[0]) - F_g0[2];

    // Total forces
    F_total = delta_F + F_grav;

    // linear newton
    v_dot = F_total / m - omega.cross(v);

    // Angular Newton
    delta_omega_dot = delta_M - omega.cross(I * omega);
    delta_omega_dot = I.ldlt().solve(delta_omega_dot);

    // Euler Kinematics
    cos_theta = std::cos(euler[1]);
    if (std::abs(cos_theta) < 0.001f) cos_theta = 0.001f;

    J << 1, sin(euler[0])*tan(euler[1]), cos(euler[0])*tan(euler[1]),
         0, cos(euler[0]),               -sin(euler[0]),
         0, sin(euler[0])/cos_theta,      cos(euler[0])/cos_theta;
    euler_dot = J * omega;

    // Build states_dot (9x1)
    states_dot(0) = v_dot(0);
    states_dot(1) = v_dot(1);
    states_dot(2) = v_dot(2);
    states_dot(3) = delta_omega_dot(0);
    states_dot(4) = delta_omega_dot(1);
    states_dot(5) = delta_omega_dot(2);
    states_dot(6) = euler_dot(0);
    states_dot(7) = euler_dot(1);
    states_dot(8) = euler_dot(2);

    return states_dot;
}

void RBDsolve::updatewdot(float a){
    w_dot_state = a;
}