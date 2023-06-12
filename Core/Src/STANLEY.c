/*
 * PID.c
 *
 *  Created on: May 23, 2023
 *      Author: del
 */

#include "STANLEY.h"
#include <stdlib.h>
#include <math.h>

float normalizeAngle(float angle){
	const float PI = 3.141592653589f;

	while(angle > PI){
		angle = angle - 2*PI;
	}
	while(angle < -PI){
			angle = angle + 2*PI;
	}

	return angle;
}

float *polyFit(float x0, float x1, float y0, float y1){
	float m = (y1-y0)/(x1-x0);
	float c = y0 - m*x0;
	static float p[2];
	p[0]= m; p[1]= c;

	return p;
}

float *calcular_posActual(float posX, float posY, float current_speed_rpm, float yaw, float ang_direc, float T){
	posX = posX + current_speed_rpm*cos(yaw + ang_direc)*T;
	posY = posY + current_speed_rpm*sin(yaw + ang_direc)*T;

	float *arr_currPos = malloc(sizeof(float) * 2);

	arr_currPos[0]= posX; arr_currPos[1]= posY;

	return arr_currPos;
}

void StanleyController_Init(StanleyController *stanley){

	stanley->out = 0.0f;

}

float StanleyController_Update(StanleyController *stanley, float yaw, float vel, float arr_currPos[], float arr_path[]){

	float x = arr_currPos[0];
	float y = arr_currPos[1];
	float x0 = arr_path[0], y0 = arr_path[1], x1 = arr_path[2], y1 = arr_path[3];

	// path line equation
	float *p;
	p = polyFit(x0, x1, y0, y1);
	float a = *p, b = 1.0f, c = *(p+1);

	// path angle
	float path_angle = atan2f(y-y0, x-x0);

	// crosstrack error
	float crosstrack_error = (a*x + b*y + c)/sqrt(a*a + b*b);

	// angle_cross_track
	float angle_cross_track = atan2f(y-y0, x-x0);
	float angle_path2ct = path_angle - angle_cross_track;
	angle_path2ct = normalizeAngle(angle_path2ct);

	if(angle_path2ct > 0){
		crosstrack_error = fabs(crosstrack_error);
	}
	else{
		crosstrack_error = - fabs(crosstrack_error);
	}

	float crosstrack_steer = atan2f(stanley->k*crosstrack_error, vel+stanley->ks);

	// heading error
	float heading = path_angle - yaw;
	heading = normalizeAngle(heading);

	float steer = heading + crosstrack_steer;

	stanley->out = steer;

	return steer;

}

void StanleyController_Reset(StanleyController *stanley){

	stanley->out = 0.0f;

}

