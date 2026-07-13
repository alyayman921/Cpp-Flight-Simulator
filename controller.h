#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

float de_max=  25 * 3.1415 / 180;
float de_min= -25 * 3.1415 / 180;

class controller{
	private:
		bool Autopiloted;
		Eigen::Matrix<double,4,1>* Controls;
		float da,de,dth,dr;
		Eigen::Matrix<double, 9, 1>* results;
		float y;
		double dt=0.01;
		float theta_cmd;
		int current_step;
		double diffed;
		double y_pitch=0.0,y1_pitch=0.0,yd_pitch=0.0; 
		double set_pitch=20*3.1415/180;
	public:
		controller(Eigen::Matrix<double,4,1>* Controls,double dt, double set_pitch,bool Autopiloted=true){
			this->Controls=Controls;
			this->Autopiloted=Autopiloted;
			this->dt=dt;
			this->set_pitch=set_pitch;
			*Controls={0,0,0,0};
		}

		void rk4_pointers(Eigen::Matrix<double, 9, 1>* results){
			this->results=results;
		}

		void pitch_controller(int step){
			/*
			  307.35 (1-0.9828z-1)
			  --------------------
			        (1+z-1)
			*/
			if (Autopiloted){
				if (step<1){
					yd_pitch=(set_pitch-results[step][7])*(1.9948); // y dot from drawing, maybe try delta theta instead of theta
					de=0;
				}else{
					y_pitch += yd_pitch*dt;
					yd_pitch = (set_pitch-results[step][7])*(1.9948); // y dot prev, used in next step, where ynow=yprev+yd*dt

				de= y_pitch - ((results[step][7]*1.734+results[step][4])*1.5236);
				de=-de;
				}
				if (de>de_max){de=de_max;}
				if (de<de_min){de=de_min;}
				*Controls={da,de,dth,dr};
				if (step%100==0){
					//std::cout<<"step "<<step<<std::endl;
					//std::cout<<*Controls<<"\n";
					//std::cout<<"pitch angle "<<results[step][7]*180/3.1415<<std::endl;
					//std::cout<<"delta elevator ="<<de<<std::endl;
				}
			}
		}

		double diffrentiate(int current_step,int index){
			if (current_step==1){
				return 0.00;
			}else{
				// (K_n - K_n-1) / dt
			   diffed= (results[current_step][index]-results[current_step-1][index])/dt;
			    return diffed;
			}
		}

		double servo(){
			/*
 						output       0.04762  + 0.04762 z-1
 			G_Servo= ---------- = --------------------------- | ts = 0.01
      					input	       1 - 0.9048z-1
			*/
			return 0.0;
		}

};