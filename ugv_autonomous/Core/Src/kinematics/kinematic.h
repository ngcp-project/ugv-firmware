/*
 * KINEMATIC.h
 *
 *  Created on: March 08, 2025
 *      Author: Korbinian Schoerghuber
 *
 *      Kinematics for NGCP
 *
 *		Ver. 1.0
 */


#ifndef KINEMATIC_H
#define KINEMATIC_H

typedef struct{
	float L;
	float steering_angle;
	float velocity;
}kinematic;

extern kinematic kin;

typedef struct{
	float da;
	float dX;
	float dY;
	float ddistance;
	float a;
	float X;
	float Y;
}position;

extern position pos;

dead_reckoning(kinematic *k, position *p, float t);

#endif
