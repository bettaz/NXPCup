#include "nxpcup_race.h"
#include <math.h>
#include <time.h>
const uint8_t frame_width = 78;
const uint8_t frame_height = 51;
const float PI = 3.14159265359f;
const float window_center = (frame_width / 2);
const float linear_velocity = 0.06f;
clock_t interval_start=0;
clock_t interval_end;
float error_zero;
float integral_term=0.0f;
float kp=1.0;
float ki=0.0;
float kd=0.0;
float ka=0.8;
float bias=0;
float windup=0;
float actual_angle = 0.0f;
float last_angle = 0.0f;
float hundredeighty = 180.0f;

float getSpeed(float angle_rad){
	int degrees = 0;
	if(angle_rad>0.0f){
		degrees = (int)(angle_rad*180.0f/PI);
	}
	else{
		degrees = (int)((0.0f-angle_rad)*180.0f/PI);
	}
	float output = 0.06f;
	switch (degrees/10)
	{
		case 0:
		case 1:
			output = 0.18f;
			break;
		case 2:
		case 3:
			output = 0.12f;
			break;
		default:
			output = 0.08f;
			break;
	}
	return output;
}

float calculate_controller(float error){
	// Initializes clock for first run
	if(interval_start==0){
		interval_start=clock()-1;
		//PX4_INFO("new clock");
	}
	// Gets actual time & calculates interval
	interval_end=clock();
	float interval_duration = (float)(interval_end-interval_start)/CLOCKS_PER_SEC;
	interval_start=interval_end;

	//Controller core
	integral_term+=error*interval_duration;
	float derivative_term=(error-error_zero)/interval_duration;
	float output = kp*error+kd*derivative_term+ki*integral_term;//-bias;
	// Anti windup check
	/*if(output>1.0472f||output<1.0472f){
		if(output>0){
			windup = output-1.0472f;
			output = 1.0472f;
		}
		else{
			windup = 1.0472f + output;
			output = 0 - 1.0472f;
		}
	}
	else{
		windup=0.0f;
	}*/
	//PX4_INFO("error %8.4f\t integral_term %8.4f\t derivative_term %8.4f\t output %8.4f\t interval %8.4f\t",(double)error,(double)integral_term,(double)derivative_term,(double)output,(double)interval_duration);
	// Set obsolescence
	error_zero=error;
	return output;
}

Vector orientPixyVector(Vector v){
	v.m_y0=frame_height-v.m_y0;
	v.m_y1=frame_height-v.m_y1;
	if(v.m_y0>v.m_y1){
		float tmp_x,tmp_y;
		tmp_x=v.m_x0;
		tmp_y=v.m_y0;
		v.m_x0=v.m_x1;
		v.m_y0=v.m_y1;
		v.m_x1=tmp_x;
		v.m_y1=tmp_y;
	}
	return v;
}

float getVectorModule(Vector v)
{
	return sqrt(((v.m_x1 - v.m_x0) ^ 2) + ((v.m_y1 - v.m_y0) ^ 2));
}

float getVectorAngle(Vector v)
{
	float_t output = (float)atan((v.m_y1 - v.m_y0)/(v.m_x1 - v.m_x0));
	double radians;
	if (output >0.0f)
	{
		radians = output - PI/2.0f;
	}
	else{
		radians = output + PI/2.0f;
	}

	PX4_INFO("atan = %f°",radians);
	return (float)radians;
}

float getWeightedAverage(Vector v1, Vector v2)
{
	/*float mod1 = getVectorModule(v1);
	float mod2 = getVectorModule(v2);*/
	float v_angle1 = getVectorAngle(v1);
	float v_angle2 = getVectorAngle(v2);
	return (v_angle1+v_angle2)/2.0f;//(v_angle1*mod1+v_angle2*mod2)/(mod1+mod2);
}

roverControl raceTrack(Pixy2 &pixy, float yaw)
{
	roverControl control{};
	pixy.line.getAllFeatures(LINE_VECTOR, true);
	if (pixy.line.numVectors == 0) {
		actual_angle = 0.0f;
		PX4_INFO("no vecs");
	} else {
		if (pixy.line.numVectors == 1) {
			Vector single = orientPixyVector(pixy.line.vectors[0]);
			actual_angle = 0.0f - getVectorAngle(single);  //radian
			// - calculate_controller(yaw + actual_angle , yaw);
			/*float_t angle = actual_angle*hundredeighty/PI;
			PX4_INFO("Single %f°\t x0 %d\t y0 %d",(double)angle, single.m_x0, single.m_y0);
			PX4_INFO("x1 %d\t y1 %d\t", single.m_x1,single.m_y1);
			PX4_INFO("Yaw %f", (double)control.orientation);*/

		} else {
			// Find average of both top X values
			Vector vecs[2];
			for (int i=0; i<2; i++ ){
				vecs[i]=orientPixyVector(pixy.line.vectors[i]);
			}
			actual_angle = getWeightedAverage(vecs[0], vecs[1]);
			/*PX4_INFO("Vector 0  %8.4f°\t x0 %d\t y0 %d",(double)getVectorAngle(vecs[0]), vecs[0].m_x0, vecs[0].m_y0);
			PX4_INFO(" x1 %d\t y1 %d",vecs[0].m_x1,vecs[0].m_y1);
			PX4_INFO("Vector 1  %8.4g°\t x0 %d\t y0 %d",(double)getVectorAngle(vecs[1]), vecs[1].m_x0, vecs[1].m_y0);
			PX4_INFO(" x1 %d\t y1 %d",vecs[1].m_x1,vecs[1].m_y1);
			PX4_INFO("Avg: %f° orient: %f",(double)(actual_angle*180.0f/3.14f), (double)control.orientation);*/
			// Prints all vectors info
			/* for (int i=0; i<pixy.line.numVectors; i++ ){
				PX4_INFO("Vector %d  %8.4f°\t x0 %8.4f\t y0 %8.4f\t x1 %8.4f\t y1 %8.4f\t",i,(double)actual_angle*181.0/3.14, (double)vecs[i].m_x0, (double) vecs[i].m_y0, (double)vecs[i].m_x1,(double)vecs[i].m_y1);
			} */
			// Set speed to linear_velocity paramater

		}

	}
	actual_angle = calculate_controller(actual_angle);
	if(yaw + actual_angle > PI){
		control.orientation = (yaw + actual_angle)- 6.2831853072f;
	}
	else{
		if(yaw + actual_angle < 0.0f-PI){
			control.orientation = (yaw + actual_angle) + 6.2831853072f;
		}
		else{
			control.orientation = yaw+actual_angle;
		}
	}
	control.speed = getSpeed(actual_angle);
	last_angle = actual_angle;
	return control;
}
