#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
extern double deg2rad;
extern double rad2deg; 
float de_max=  25 * deg2rad;
float de_min= -25 * deg2rad;


class controller{
	private:
		
		struct servo_states{
			double dservo=0.0; // servo angle
			double dservo_derivated=0.0; // servo angle rate 
		};

		bool Autopiloted;
		float da,de,dth,dr;
		Eigen::Matrix<double,4,1>* Controls;
		Eigen::Matrix<double, 9, 1>* results;
		float y;
		double dt=0.01;
		int current_step;
		//double diffed; // not used

		// Pitch Controller
		double y_pitch=0.0,yd_pitch=0.0; 
		double set_pitch=20*3.1415/180;
		servo_states pitch_servo;

		// Velocity Controller
		double y_vel=0.0,yd_vel=0.0;
		double set_vel = 800;
		servo_states throttle_valve;
		servo_states engine_lag;

	public:
		controller(Eigen::Matrix<double,4,1>* Controls,double dt, double set_pitch, double set_vel,bool Autopiloted=true){
			this->Controls=Controls;
			this->Autopiloted=Autopiloted;
			this->dt=dt;
			this->set_pitch=set_pitch;
			this->set_vel=set_vel;

			if (Autopiloted){
				*Controls={0,0,0,0};
			}
		}

		void rk4_pointers(Eigen::Matrix<double, 9, 1>* results){
			this->results=results; // take the pointer to results vector, point to it here too
		}

		double servo(servo_states &s, float input, float t=10){
			/*
 						output       0.04762  + 0.04762 z-1 	    			10
 			G_Servo= ---------- = --------------------------- ts  0.01 =    ---------- 
      					input	       1 - 0.9048z-1			   			  s + 10

      		GServoEngine =        0.1  
  							---------------
  							    s + 0.1 
			*/
			if(current_step<1){
				s.dservo_derivated=t*(input-s.dservo);
			}else{
				s.dservo_derivated=t*(input-s.dservo);
				s.dservo+=s.dservo_derivated*dt;
			}
			return s.dservo;
		}


		void pitch_controller(int step){
			/*
			  307.35 (1-0.9828z-1)
			  --------------------
			        (1+z-1)
			*/
			if (Autopiloted){
				this->current_step=step;

				if (step<1){
					yd_pitch=(set_pitch-results[step][7])*(1.9948); // y dot from drawing, maybe try delta theta instead of theta
					de=0;
					}else{

					y_pitch += yd_pitch*dt;
					yd_pitch = (set_pitch-results[step][7])*(1.9948); // y dot prev, used in next step, where ynow=yprev+yd*dt
					de= y_pitch - ((results[step][7]*1.734+results[step][4])*1.5236);
					de=-de; // this is the input to servo tf, from the simulink model 
					de=servo(pitch_servo, de);
					if (de>de_max){de=de_max;}
					if (de<de_min){de=de_min;}
					*Controls={da,de,dth,dr};
				}
			}
		}
		void velocity_controller(int step){
			/*
			    1549.1 (s+0.008082)
  				------------------- = 1549.1 + 125.2/s
           				s
			*/
			if (Autopiloted){
				if (step<1){
					yd_vel=(set_vel-results[step][0])*(125.2); // y dot from drawing
					dth=0;
					}else{
					y_vel += yd_vel*dt; // Integrator
					y=y_vel+1549.1*(set_vel-results[step][0]);
					yd_vel = (set_vel-results[step][0])*(125.2); // y dot prev, used in next step, where ynow=yprev+yd*dt
					
					dth=servo(throttle_valve, y);
					dth=servo(engine_lag, dth,0.1);
					
					if (dth>167509){dth=167509;}
					if (dth<-40800){dth=-40800;}
					*Controls={da,de,dth,dr};
				}
			}
		}


		/*
		double diffrentiate(int current_step,int index){
			if (current_step==1){
				return 0.00;
			}else{
				// (K_n - K_n-1) / dt
			   diffed= (results[current_step][index]-results[current_step-1][index])/dt;
			    return diffed;
			}
		}
		*/

};