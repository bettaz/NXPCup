/**
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_race.h"
#include <math.h>
const int frame_width = 78;
const int frame_height = 51;
const float window_center = (frame_width / 2);
const float angular_velocity = -0.2f;
const float single_line_steer_scale = 1.2f;
const float linear_velocity = 0.0f;
const float homographyMatrix[3][3] = {
	{ 1.04096622e+00,  7.54029404e-01, -2.55978456e+02},
	{ 4.45844843e-16,  1.79968627e+00, -2.27373675e-13},
	{ 7.37615362e-05,  9.67959146e-04,  6.17592121e-01}
};
/*struct poi{
	float x;
	float y;
};*/
struct vec {
	float mag, mod;
};
Vector orientPixyVector(Vector v){
	v.m_y0=0-v.m_y0;
	v.m_y1=0-v.m_y1;
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
	avg.mag = mag1+mag2;
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
			//single = pixy.line.vectors[0];
			//float mod=getVectorModule(single);
			float mag = getVectorMagnitude(single);
			control.steer = 0-mag+(90*(float)M_PI/180);
			PX4_INFO("Vector %d point 1 x0\t%8.4f y0\t%8.4f x1\t%8.4f y1\t%8.4f",1, (double)single.m_x0, (double) single.m_y0, (double)single.m_x1,(double)single.m_y1);
			PX4_INFO("Steering \t%8.4f", (double)control.steer*180/M_PI);
			// Set speed to linear velocity parameter
			control.speed = linear_velocity;

		} else {
			// Find average of both top X values
			Vector vecs[5];
			for (int i=0; i<pixy.line.numVectors; i++ ){
				vecs[i]=orientPixyVector(pixy.line.vectors[i]);//homoTransform(pixy.line.vectors[i]);
			}
			//vec_0 = pixy.line.vectors[0];
			//vec_1 = pixy.line.vectors[1];
			struct vec avg = getWeightedAverage(vecs[0], vecs[1]);
			control.steer = 0-avg.mag;
			/*
			PX4_INFO("Vector 0 point 1 x0\t%8.4f y0\t%8.4f", (double)vec_0.m_x0, (double) vec_0.m_y0);
			PX4_INFO("Vector 0 point 2 x1\t%8.4f y1\t%8.4f", (double)vec_0.m_x1, (double)vec_0.m_y1);
			PX4_INFO("Vector 1 point 1 x0\t%8.4f y0\t%8.4f", (double)vec_1.m_x0, (double)vec_1.m_y0);
			PX4_INFO("Vector 1 point 2 x1\t%8.4f y1\t%8.4f", (double)vec_1.m_x1, (double)vec_1.m_y1);*/
			for (int i=0; i<pixy.line.numVectors; i++ ){
				PX4_INFO("Vector %d point 1 x0\t%8.4f y0\t%8.4f x1\t%8.4f y1\t%8.4f",i, (double)vecs[i].m_x0, (double) vecs[i].m_y0, (double)vecs[i].m_x1,(double)vecs[i].m_y1);
			}
			PX4_INFO("Steering \t%8.4f", (double)control.steer);

			// Set speed to linear_velocity paramater
			control.speed = linear_velocity;
		}
	}

	return control;
}
