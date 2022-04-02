/**
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_race.h"
#include <math.h>
#include <time.h>
const int frame_width = 78;
const int frame_height = 51;
const float window_center = (frame_width / 2);
const float angular_velocity = -0.2f;
const float single_line_steer_scale = 1.2f;
const float linear_velocity = 0.05f;
const float homographyMatrix[3][3] = {
	{ 1.04096622e+00,  -7.54029404e-01, -2.55978456e+02},
	{ 4.45844843e-16,  -1.79968627e+00, -2.27373675e-13},
	{ 7.37615362e-05,  -9.67959146e-04,  6.17592121e-01}
};
clock_t interval_start=0;
clock_t interval_end;
float error_zero;
float integral_zero;
float kp=0.5;
float ki=1.0;
float kd=0.5;
float ka=0.8;
float bias=0;
float windup=0;
float old_angle=0;
float actual_angle=0;
/*struct poi{
	float x;
	float y;
};*/

float calculate_controller(float u, float read){
	//PX4_INFO("entering_controller");
	// Initializes clock for first run
	if(interval_start==0){
		interval_start=clock()-1;
		PX4_INFO("new clock");
	}
	// Gets actual time & calculates interval
	interval_end=clock();
	float interval_duration = (float)(interval_end-interval_start)/CLOCKS_PER_SEC;
	interval_start=interval_end;
	// LP filter
	actual_angle=old_angle+actual_angle;

	//Controller core
	float error = u-read;
	float integral_term=integral_zero+(error-ka*windup)*interval_duration;
	float derivative_term=(error-error_zero)/interval_duration;
	float output = kp*error+ki*integral_term+kd*derivative_term-bias;
	// Anti windup check
	if(output>1.0472f||output<1.0472f){
		if(output>0)
			windup=output-1.0472f;
		else
			windup=1.0472f+output;
	}
	else{
		windup=0.0f;
	}
	//PX4_INFO("error %8.4f\t integral_term %8.4f\t derivative_term %8.4f\t output %8.4f\t interval %8.4f\t",(double)error,(double)integral_term,(double)derivative_term,(double)output,(double)interval_duration);
	// Set obsolescence
	error_zero=error;
	integral_zero=integral_term;
	old_angle=output;
	return output;
}
struct vec {
	int mag, mod;
};
// TODO move to signed integers or recalculate the homography
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

float getVectorMagnitude(Vector v)
{
	return atan((v.m_y1 - v.m_y0) / (v.m_x1 - v.m_x0));
}
struct vec getWeightedAverage(Vector v1, Vector v2)
{
	float mod1 = getVectorModule(v1);
	float mod2 = getVectorModule(v2);
	float mag1 = getVectorMagnitude(v1);
	float mag2 = getVectorMagnitude(v2);
	struct vec avg;
	avg.mag = (mag1 + mag2)/2;
	avg.mod = (mod1 + mod2) / 2;
	return avg;
}
Vector homoTransform(Vector v)
{
	int x0=v.m_x0;
	int x1=v.m_x1;
	int y0=v.m_y0;
	int y1=v.m_y1;
	int one =1;
	int src0[3] = {x0,y0,one};
	int src1[3] = {x1,y1,one};
	float p0[3] = {0.0,0.0,0.0};
	float p1[3] = {0.0,0.0,0.0};
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			p0[i]+=homographyMatrix[i][j]*src0[j];
			p1[i]+=homographyMatrix[i][j]*src1[j];
		}
	}
	v.m_x0=p0[0]*p0[2];
	v.m_y0=p0[1]*p0[2];
	v.m_x1=p1[0]*p1[2];
	v.m_y1=p1[1]*p1[2];
	return v;
}

roverControl raceTrack(Pixy2 &pixy)
{
	roverControl control{};
	/* instert you algorithm here */
	pixy.line.getAllFeatures(LINE_VECTOR, true);

	//vec sum = sumVectors(Vec1,Vec2);
	if (pixy.line.numVectors == 0) {
		control.steer = 0.0f;
		control.speed = 0.0f;
	} else {
		if (pixy.line.numVectors == 1) {
			Vector single = orientPixyVector(pixy.line.vectors[0]);
			// Uncomment to activate the homography
			//single = homoTrasform(single);
			float mag = getVectorMagnitude(single);  //radian
			control.steer = calculate_controller(actual_angle,mag)/1.0472f;
			PX4_INFO("Single %8.4f°\t x0 %8.4f\t y0 %8.4f\t x1 %8.4f\t y1 %8.4f\t",(double)mag*180.0/3.14, (double)single.m_x0, (double) single.m_y0, (double)single.m_x1,(double)single.m_y1);
			PX4_INFO("Steering %8.4f", (double)control.steer*100.0);
			// Set speed to linear velocity parameter
			control.speed = linear_velocity;

		} else {
			// Find average of both top X values
			Vector vecs[5];
			for (int i=0; i<pixy.line.numVectors; i++ ){
				vecs[i]=orientPixyVector(pixy.line.vectors[i]);
				// Uncomment to activate the homography
				//vecs[i]=homoTransform(vecs[i]);
			}
			struct vec avg = getWeightedAverage(vecs[0], vecs[1]);
			control.steer = calculate_controller(actual_angle,avg.mag)/1.0472f;
			// Prints all vectors info
			for (int i=0; i<pixy.line.numVectors; i++ ){
				PX4_INFO("Vector %d  %8.4f°\t x0 %8.4f\t y0 %8.4f\t x1 %8.4f\t y1 %8.4f\t",i,(double)avg.mag*181.0/3.14, (double)vecs[i].m_x0, (double) vecs[i].m_y0, (double)vecs[i].m_x1,(double)vecs[i].m_y1);
			}
			PX4_INFO("Steering \t%8.4f", (double)control.steer*100.0);

			// Set speed to linear_velocity paramater
			control.speed = linear_velocity;
		}
	}

	return control;
}
