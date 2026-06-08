#include "Eigen/Core"
#include "Eigen/Dense"
class RBDsolve{
	private:
		int i;
		float m,w_dot_state,g;
		Eigen::Vector3f F_g0;
		Eigen::Vector3f F_g;
		Eigen::Matrix<float,3,1>v;
		Eigen::Matrix<float,3,1>omega;
		Eigen::Matrix<float,3,1>euler;
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

			delta_states<<delta_v,delta_omega,w_dot_state;
			Aerodynamic_accel=SD*delta_states+CD*Controls;

            //w_dot_state = (F_out(3) / obj.m) - p*v + q*u;
			
			for (i=0;i<3;i++){
				delta_F[i]=Aerodynamic_accel[i]*m;
			}
			for (i=3;i<6;i++){
				delta_M[i-3]=Aerodynamic_accel[i]*I.coeff(i-3,i-3);
			}
            F_g << -m*g * sin(euler[1])+F_g0[0],
		            m*g * cos(euler[1]) * sin(euler[0])+ F_g0[1], 
		            m*g * cos(euler[1]) * cos(euler[0])+ F_g0[2];

			std::cout<<F_g<<'\n';
			results<<delta_F[0]+F_g[0],delta_F[1]+F_g[1],delta_F[2]+F_g[2],
			delta_M[0],delta_M[1],delta_M[2];
			return results;
		}
};