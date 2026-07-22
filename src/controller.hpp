#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "tf.hpp"
extern double deg2rad;
extern double rad2deg; 

class controller{
	private:
		bool Autopiloted;
		float da,de,dth,dr;
		Eigen::Matrix<double,4,1>* Controls;
		Eigen::Matrix<double, 9, 1>* results;
		float y,delta_theta;
		double dt=0.01;
		int current_step;
		int* step;
		aircraft_data *ac;
		flight_path *str_h;
		autopilot_inputs *commands;

		// Pitch Controller
		double y_pitch=0.0,yd_pitch=0.0; 
		float de_max=  25 * deg2rad;
		float de_min= -25 * deg2rad;
		transferFunction pitch_tf;
		transferFunction pitch_servo;

		// Velocity Controller
		double y_vel=0.0,yd_vel=0.0;
		float dth_max= 167509;
		float dth_min= -40800;
		transferFunction vel_tf;
		transferFunction throttle_valve;
		transferFunction engine_lag;

		// Alt Controller
		double y_alt=0.0,yd_alt=0.0;
		transferFunction alt_tf;

		// Roll Controller
		float da_max=     25 * deg2rad;
		float da_min=    -25 * deg2rad;
		transferFunction PI_Roll;
		transferFunction PD_Roll;
		transferFunction roll_servo;

		// Heading Controller 
		double coordinated_roll=0.0;
		transferFunction yaw_damper;
		transferFunction yaw_servo;


	public:
		controller(Eigen::Matrix<double,4,1>* Controls,aircraft_data *ac ,flight_path *str_h ,
			double dt,int *step,autopilot_inputs *commands ,bool Autopiloted=true){
			this->Controls=Controls;
			this->Autopiloted=Autopiloted;
			this->dt=dt;
			this->str_h=str_h;
			this->step=step;
			this->ac=ac;
			this->commands=commands;
			// Pitch
			double servo_num=10; double servo_den[2]={10 ,1.0};
			double pitch_num=1.9948; double pitch_den[2]={0.0 ,1.0};
			this->pitch_tf=transferFunction(1 , 2 , pitch_num , pitch_den ,step ,dt);
			this->pitch_servo=transferFunction(1 , 2, servo_num, servo_den , step ,dt);
			// velocity
			double vel_num[2]={125.2,1549.1}; double vel_den[2]={0.0 ,1.0};
			double engine_num=0.1; double engine_den[2]={0.1 ,1.0};
			this->vel_tf=transferFunction(1 , 2, vel_num, vel_den , step ,dt);
			// altitude
			double alt_num[2]={0.011498*0.3709 , 0.011498}; double alt_den[2]={10 ,1.0};
			this->alt_tf=transferFunction(1 , 2, alt_num, alt_den , step ,dt);
			// roll
			double pir_num[2]={0.14722*18,0.14722};double pir_den[2]={0,1};
			this->PI_Roll=transferFunction(2 , 2, pir_num, pir_den, step, dt);
			double pdr_num[2]={121.15*1.3,121.15};double pdr_den[2]={15,1};
			this->PD_Roll=transferFunction(2 , 2, pdr_num, pdr_den, step, dt);
			this->roll_servo=transferFunction(1 , 2, servo_num, servo_den, step, dt);
			// yaw
			double yaw_num[2]={0,0.88711};double yaw_den[2]={0.1,1};
			this->yaw_damper=transferFunction(2 , 2, yaw_num, yaw_den, step, dt);
			this->yaw_servo=transferFunction(1 , 2, servo_num, servo_den, step, dt);

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
				y_pitch=pitch_tf.solve(((commands->set_pitch)-delta_theta));
				de=-(y_pitch - ((delta_theta*1.734+results[*step][4])*1.5236));
				de=pitch_servo.solve(de);
				if (de>de_max){de=de_max;}
				if (de<de_min){de=de_min;}
				*Controls={da,de,dth,dr};
				}
			}
		
		void altitude_controller(){
				//for a large value of altitude set
				// you have to feed the controller multiple gradual steps
				// Damn you, Non LTI Model
			if (Autopiloted&&!(commands->alt_override)){
				y_alt=alt_tf.solve(((commands->set_alt)-str_h->h));
					commands->set_pitch=y_alt;
			}

		void velocity_controller(){
			if (Autopiloted){
				y_vel=vel_tf.solve(((commands->set_vel)-results[*step][0]));
				dth=throttle_valve.solve(y_vel);
				dth=engine_lag.solve(dth);
				if (dth>dth_max){dth=dth_max;}
				if (dth<dth_min){dth=dth_min;}
				*Controls={da,de,dth,dr};
			}
		}
// -------------------------------------- LATERAL CONTROLLERS ---------------------------------------
		
		void roll_controller(){
			da=PI_Roll.solve(((commands->set_roll)-results[*step][6]))-PD_Roll.solve(results[*step][6]);
			da=roll_servo.solve(da);
			*Controls={da,de,dth,dr};
		}
		void yaw_controller(){
			if(!(commands->head_override)){
				coordinated_roll=((commands->set_heading)-results[*step][8])*(results[0][0])/ac->g/10;
				if (coordinated_roll>25){coordinated_roll=25;}
				if (coordinated_roll<-25){coordinated_roll=-25;}
				commands->set_roll=coordinated_roll;
				dr=yaw_damper.solve(results[*step][5]);
				dr=yaw_servo.solve(dr);
				*Controls={da,de,dth,dr};
			}

		}
		

};