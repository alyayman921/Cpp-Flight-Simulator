#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>

class RBDsolve{
	private:
		int i;
		float m,w_dot_state,g;
		Eigen::Vector3f F_g0;
		Eigen::Vector3f F_g;
		Eigen::Matrix<float,3,1>v;Eigen::Matrix<float,3,1>v_e;Eigen::Matrix<float,3,1>v_dot;
		Eigen::Matrix<float,3,1>omega;Eigen::Matrix<float,3,1>omega_dot;
		Eigen::Matrix<float,3,1>euler;Eigen::Matrix<float,3,1>euler_dot;
		Eigen::Matrix<float,3,3>J;Eigen::Matrix<float,3,3>R;
		Eigen::Matrix<float,3,3>I;
		Eigen::Matrix<float,6,7>SD;
		Eigen::Matrix<float,7,1>delta_states;
		Eigen::Matrix<float,6,4>CD;
		Eigen::Matrix<float,4,1>Controls;
		Eigen::Matrix<float,6,1>Aerodynamic_accel;
		Eigen::Matrix<float,2,3>results;
		Eigen::Matrix<float,3,1>delta_v;
		Eigen::Matrix<float,3,1>delta_omega;
		Eigen::Matrix<float,3,1>delta_F;
		Eigen::Matrix<float,3,1>delta_M;	
		aircraft_data ac;

	public:
		RBDsolve(aircraft_data ac, Eigen::Matrix<float,4,1>Controls){
			this->ac=ac;this->g=ac.g;this->m=ac.m;this->I=ac.Inertia;
			this->F_g0=ac.mg0;this->v=ac.V0;
		    this->omega=ac.omega0;this->euler=ac.euler0;
			this->SD=ac.SD;this->CD=ac.CD;
			this->w_dot_state=0;
			this->Controls=Controls;
		}

		Eigen::Matrix<float,2,3> calcFM(){ 
			// calculate forces and moments
			delta_v=v-ac.V0;
			delta_omega=omega-ac.omega0;
			w_dot_state=delta_F[2];
			delta_states<<delta_v,delta_omega,w_dot_state;
			Aerodynamic_accel=SD*delta_states+CD*Controls;
			
			for (i=0;i<3;i++){
				delta_F[i]=Aerodynamic_accel[i]*m;
			}
			for (i=3;i<6;i++){
				delta_M[i-3]=Aerodynamic_accel[i]*I.coeff(i-3,i-3);
			}
            F_g << -m * g * sin(euler[1]) + F_g0[0],
		            m * g * cos(euler[1]) * sin(euler[0]) + F_g0[1], 
		            m * g * cos(euler[1]) * cos(euler[0]) + F_g0[2];
		    delta_F=delta_F+F_g;
			std::cout<<"FG - FG0 : \n"<<F_g<<'\n';
			results<<delta_F[0],delta_F[1],delta_F[2],
			delta_M[0],delta_M[1],delta_M[2];
			return results;
		}
		Eigen::Matrix3d eulerToRotationMatrix(const Eigen::Vector3f& euler) {
		    double phi = euler(0);
		    double theta = euler(1);
		    double psi = euler(2);
		    double c_phi = std::cos(phi);double s_phi = std::sin(phi);
		    double c_theta = std::cos(theta);double s_theta = std::sin(theta);
		    double c_psi = std::cos(psi);double s_psi = std::sin(psi);
		    
		    Eigen::Matrix3d R;
		    R<< c_psi * c_theta, 
		        c_psi * s_theta * s_phi - s_psi * c_phi,
		        c_psi * s_theta * c_phi + s_psi * s_phi,
		         
		        s_psi * c_theta,
		        s_psi * s_theta * s_phi + c_psi * c_phi,
		        s_psi * s_theta * c_phi - c_psi * s_phi,
		         
		       -s_theta,
		        c_theta * s_phi,
		        c_theta * c_phi;
		    return R;
}

		Eigen::Vector3f Linear_Newton(){
			v_dot=delta_F/m - delta_omega.cross(delta_v);
			return v_dot;
		}
		Eigen::Vector3f Angular_Newton(){
			omega_dot=(delta_M - omega.cross(I*omega));
			omega_dot=I.ldlt().solve(omega_dot);
			return omega_dot;
		}
		Eigen::Vector3f Euler_Kinematics(){
            J << 1, sin(euler[0])*tan(euler[1]), cos(euler[0])*tan(euler[1]),
                 0, cos(euler[0]),				-sin(euler[0]),
                 0, sin(euler[0])/cos(euler[1]), cos(euler[0])/cos(euler[1]);
            euler_dot=J*omega;
			return euler_dot;
		}
		Eigen::Vector3f v_e(){
            R=eulerToRotationMatrix(euler);
            v_e=R*v;
			return v_e;
		}
};