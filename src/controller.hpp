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
		bool Autopiloted;
		bool alt_override;
		float da,de,dth,dr;
		Eigen::Matrix<double,4,1>* Controls;
		Eigen::Matrix<double, 9, 1>* results;
		float y,delta_theta;
		double dt=0.01;
		int current_step;
		int* step;
		flight_path *str_h;

		// Pitch Controller
		double y_pitch=0.0,yd_pitch=0.0; 
		double set_pitch=0.0;
		transferFunction pitch_tf;
		transferFunction pitch_servo;

		// Velocity Controller
		double y_vel=0.0,yd_vel=0.0;
		double set_vel = 800;
		transferFunction vel_tf;
		transferFunction throttle_valve;
		transferFunction engine_lag;

		// Alt Controller
		double y_alt=0.0,yd_alt=0.0;
		transferFunction alt_tf;
		// Heading Controller 
		double set_heading=0.0;

	public:
		controller(Eigen::Matrix<double,4,1>* Controls,flight_path *str_h ,
			double dt,int *step, double set_pitch, double set_vel,double set_alt,
			double set_heading,bool alt_override,bool Autopiloted=true){
			this->Controls=Controls;
			this->Autopiloted=Autopiloted;
			this->alt_override=alt_override;
			this->dt=dt;
			this->set_pitch=set_pitch;
			this->set_vel=set_vel;
			this->set_heading=set_heading;
			this->str_h=str_h;
			this->step=step;
			
			double servo_num=10; double servo_den[2]={10 ,1.0};
			double pitch_num=1.9948; double pitch_den[2]={0.0 ,1.0};
			this->pitch_tf=transferFunction(1 , 2 , pitch_num , pitch_den ,step ,dt);
			this->pitch_servo=transferFunction(1 , 2, servo_num, servo_den , step ,dt);

			double vel_num[2]={125.2,1549.1}; double vel_den[2]={0.0 ,1.0};
			double engine_num=0.1; double engine_den[2]={0.1 ,1.0};
			this->vel_tf=transferFunction(1 , 2, vel_num, vel_den , step ,dt);

			double alt_num[2]={0.011498*0.3709 , 0.011498}; double alt_den[2]={10 ,1.0};
			this->alt_tf=transferFunction(1 , 2, alt_num, alt_den , step ,dt);
		}
		void rk4_pointers(Eigen::Matrix<double, 9, 1>* results){
			this->results=results; // store a pointer to results vector
		}

		void pitch_controller(){

			if (Autopiloted&&!*step){
				*Controls={0,0,0,0};
			}
			if (Autopiloted){
				delta_theta=results[*step][7]-results[0][7];
				y_pitch=pitch_tf.solve((set_pitch-delta_theta));
				de=-(y_pitch - ((delta_theta*1.734+results[*step][4])*1.5236));
				de=pitch_servo.solve(de);
				if (de>de_max){de=de_max;}
				if (de<de_min){de=de_min;}
				*Controls={da,de,dth,dr};
				}
			}
		
		void altitude_controller(){
			/*
			    0.011498 (s+0.3709)
  				------------------- oh no
        				(s+10)
			*/
			if (Autopiloted&&!alt_override){
					y_alt=alt_tf.solve((set_alt-str_h->h));
					set_pitch=y_alt;
				}
			}

		void velocity_controller(){
			/*
			    1549.1 (s+0.008082)
  				------------------- = 1549.1 + 125.2/s
           				s
			*/
			if (Autopiloted){
				y_vel=vel_tf.solve((set_vel-results[*step][0]));
				dth=throttle_valve.solve(y_vel);
				dth=engine_lag.solve(dth);
				if (dth>167509){dth=167509;}
				if (dth<-40800){dth=-40800;}
				*Controls={da,de,dth,dr};
				//std::cout<<"current Thrust"<<dth<<"\n";
			}
		}
// -------------------------------------- LATERAL CONTROLLERS ---------------------------------------
		
		void roll_controller(){
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
		void yaw_controller(){
			/*
			   damper
			  0.88711 s
			  ---------
			   (s+0.1)
			*/
		}
		

};