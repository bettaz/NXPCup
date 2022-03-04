/**
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_race.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
const int frame_width = 78;
const int frame_height = 51;
const float window_center = (frame_width / 2);
const float angular_velocity = -0.6f;
const float single_line_steer_scale = 0.6f;
const float linear_velocity = 0.2f;
/*struct poi{
	float x;
	float y;
};*/
struct vec{
	/*poi start;
	poi end;*/
	float x0,x1,y0,y1;
};
vec getOrientedVector(Vector v){
	/*struct poi pt1, pt2;*/
	struct vec v_out;
	v_out.x0=v.m_x0;
	v_out.y0=v.m_y0;
	v_out.x1=v.m_x1;
	v_out.y1=v.m_y1;
	/*if(pt1.y>=pt2.y){
		v_out.start=pt2;
		v_out.end=pt1;
	}
	else{
		v_out.end=pt2;
		v_out.start=pt1;
	}*/
	/*v_out.start=pt1;
	v_out.end=pt2;*/
	return v_out;
}

roverControl raceTrack(Pixy2 &pixy)
{
	roverControl control{};
	/* instert you algorithm here */
	pixy.line.getAllFeatures(LINE_VECTOR, true);
	float x, y;
	//vec sum = sumVectors(Vec1,Vec2);
	if(pixy.line.numVectors==0){
		control.steer = 0.0f;
		control.speed = 0.0f;
	}
	else{
		if(pixy.line.numVectors==1){
		struct vec vec_1 = getOrientedVector(pixy.line.vectors[0]);
		// Find x/y values normalized to frame width and height
		if(vec_1.x1 > vec_1.x0){
			x = (vec_1.x1 - vec_1.x0) / frame_width;
                	y = (vec_1.y1 - vec_1.y0) / frame_height;
		}
		else{
			x = (vec_1.x0 - vec_1.x1) / frame_width;
			y = (vec_1.y0 - vec_1.y1) / frame_height;
		}
		// Use slope of line to determine steering angle
            	if(y > 0.0f || y < 0.0f){
			control.steer = (-angular_velocity) * (x / y) * single_line_steer_scale;
		}
		else{
			control.steer = 0.0f;
		}
		// Set speed to linear velocity parameter
            	control.speed = linear_velocity;
		}
		else{
			struct vec vec_0 = getOrientedVector(pixy.line.vectors[0]);
			struct vec vec_1 = getOrientedVector(pixy.line.vectors[1]);
			// Find average of both top X values
			float m_x1 = (vec_0.x1 + vec_1.x1) / 2;
			// Find distance from center of frame and use as steering value
            		control.steer = angular_velocity*(m_x1 - window_center) / frame_width;

            		// Set speed to linear_velocity paramater
            		control.speed = linear_velocity;
		}
	}
	return control;
}
