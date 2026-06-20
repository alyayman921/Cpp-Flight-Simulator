#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>
#include "derivatives.h" 

class RBDsolve{
	private:
		int i;
		float m,w_dot_state,g, cos_theta;
		Eigen::Matrix<float,3,1>v;Eigen::Matrix<float,3,1>v_e;Eigen::Matrix<float,3,1>v_dot;
		Eigen::Matrix<float,3,1>omega;Eigen::Matrix<float,3,1>delta_omega_dot;
		Eigen::Matrix<float,3,1>euler;Eigen::Matrix<float,3,1>euler_dot;
		Eigen::Matrix<float,3,3>J;Eigen::Matrix<float,3,3>R;
		Eigen::Matrix<float,3,3>I;
		Eigen::Matrix<float,6,7>SD;
		Eigen::Matrix<float,7,1>delta_states;
		Eigen::Matrix<float,6,4>CD;
		Eigen::Matrix<float,4,1>Controls;
		Eigen::Matrix<float,6,1>Aerodynamic_accel;
		Eigen::Matrix<float,3,1>delta_v;
		Eigen::Matrix<float,3,1>delta_omega;
		Eigen::Matrix<float,3,1>delta_F;
		Eigen::Matrix<float,3,1>delta_M;	
		Eigen::Matrix<float,9,1> states_dot;
		Eigen::Matrix<float,9,1> states0;
		aircraft_data ac;

	public:
		Eigen::Vector3f F_g0;
		Eigen::Vector3f F_g;
		RBDsolve(aircraft_data ac, Eigen::Matrix<float,4,1>Controls);

		Eigen::Vector3f delta(Eigen::Vector3f state,Eigen::Vector3f state0);

		void eulerToRotationMatrix(const Eigen::Vector3f& euler);

		Eigen::Matrix<float,9,1> Equations(Eigen::Matrix<float,9,1> states);
		Eigen::Matrix<float,9,1> Verify(Eigen::Matrix<float,9,1> states);


		void setState(const Eigen::Matrix<float,9,1>& states);
};