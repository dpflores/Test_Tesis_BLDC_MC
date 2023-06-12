/*
 * PID.h
 *
 *  Created on: May 23, 2023
 *  Author: davpr
 *  Guia: https://github.com/dpflores/vehicle-control/blob/master/controllers.py#L115
 */

#ifndef STANLEY_CONTROLLER_H_
#define STANLEY_CONTROLLER_H_

typedef struct {
	float k;
	float ks;

	float out;

} StanleyController;


float normalizeAngle(float angle);
float *polyFit(float x0, float x1, float y0, float y1);
void StanleyController_Init(StanleyController *stanley);
float StanleyController_Update(StanleyController *stanley, float yaw, float vel, float arr_currPos[], float arr_path[]);
void StanleyController_Reset(StanleyController *stanley);

#endif
