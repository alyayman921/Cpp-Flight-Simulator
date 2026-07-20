#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "tf.hpp"
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
		struct prev_store{
			double state=0.0; 
			double state_prev=0.0;
		};

		bool Autopiloted;
		bool alt_override;
		float da,de,dth,dr;
		Eigen::Matrix<double,4,1>* Controls;
		Eigen::Matrix<double, 9, 1>* results;
		float y;
		double dt=0.01;
		int current_step;
		int step;

		// Pitch Controller
		double y_pitch=0.0,yd_pitch=0.0; 
		double set_pitch=0.0;
		servo_states pitch_servo;
		transferFunction pitch_tf;

		// Velocity Controller
		double y_vel=0.0,yd_vel=0.0;
		double set_vel = 800;
		servo_states throttle_valve;
		servo_states engine_lag;
		flight_path *str_h;

		// Alt Controller
		double diffed;
		double y_alt=0.0,yd_alt=0.0; 
		prev_store r_alt;
		double rd_alt;

	public:
		controller(Eigen::Matrix<double,4,1>* Controls,flight_path *str_h ,
			double dt,int &step, double set_pitch, double set_vel,double set_alt,
			double set_heading,bool alt_override,bool Autopiloted=true){
			this->Controls=Controls;
			this->Autopiloted=Autopiloted;
			this->alt_override=alt_override;
			this->dt=dt;
			this->set_pitch=set_pitch;
			this->set_vel=set_vel;
			this->str_h=str_h;
			this->step=step;

			
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
		double diffrentiate(prev_store &t){
			if (current_step<1){
				return 0.00;
			}else{
				// (K_n - K_n-1) / dt
			   diffed= (t.state-t.state_prev)/dt;
			   t.state_prev=t.state; // update t.state when implementing
			    return diffed;
			}
		}
		void pitch_controller(){
			/*
			  307.35 (1-0.9828z-1)
			  --------------------
			        (1+z-1)
			*/
			if (Autopiloted&&!step){
				*Controls={0,0,0,0};
				double num=1.9948;
				double den[2]={0.0 ,1.0};
				transferFunction pitch_tf(1 , 2 , num , den ,step ,dt);

			}
			if (Autopiloted){
				y_pitch=pitch_tf.solve((set_pitch-results[step][7]));
				de=-(y_pitch - ((results[step][7]*1.734+results[step][4])*1.5236));
				de=servo(pitch_servo, de);
				if (de>de_max){de=de_max;}
				if (de<de_min){de=de_min;}
				*Controls={da,de,dth,dr};
					//std::cout<<"Delta Elevator = "<<de<<"\n";
				}
			}
		
		void altitude_controller(int step){
			/*
			    0.011498 (s+0.3709)
  				------------------- oh no
        				(s+10)
			*/
			if (Autopiloted&&!alt_override){
				if (step==1){
					//yd_alt=-10*y_alt+0.0115*(0+0.371*(str_h.h));
					r_alt.state=set_alt-str_h->h;
					rd_alt=diffrentiate(r_alt);
					}else if(step>1){
					//std::cout<<"Delta H needed" <<r_alt.state<<"\n";
					r_alt.state=set_alt-str_h->h;
					rd_alt=diffrentiate(r_alt);
					yd_alt=-10*y_alt+0.0115*(rd_alt+0.371*(r_alt.state));
					y_alt+=yd_alt*dt;
					set_pitch=y_alt;
					//std::cout<<"Delta Elevator needed = " <<set_pitch<<"\n";
					//std::cout<<" "<< "Current h_dot" <<str_h->delta_h_dot<<std::endl;
					//std::cout<<"current delta H = "<<r_alt.state<<"\n";
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
					//std::cout<<"current Thrust"<<dth<<"\n";

				}
			}
		}
// -------------------------------------- LATERAL CONTROLLERS ---------------------------------------
		
		void roll_controller(int step){
			/*
			     PI_Roll 
			  0.14722 (s+18)
			  -------------- 
			        s

				  PD_Roll
			  121.15 (s+1.3)
			  --------------
			      (s+15)
 
			*/
		}
		void yaw_controller(int step){
			/*
			   damper
			  0.88711 s
			  ---------
			   (s+0.1)
			*/
		}
		

};