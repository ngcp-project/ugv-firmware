/*
 * kinematics.c
 *
 *  Created on: Mar 8, 2025
 *      Author: korbi
 */

#include "kinematic.h"
#include "math.h"

void dead_reckoning(kinematic *k, position *p, float t)
{
	p->da = ((k->velocity * 17.6 * 360.0 * tan((k->steering_angle/360.0) * 2 * M_PI)) * t) / (2 * M_PI * k->L);
	p->a += p->da;

	p->ddistance = k->velocity * 17.6 * t; //distance in inches

	p->dX = p->ddistance * sin((p->a/360.0) * 2 * M_PI);
	p->dY = p->ddistance * cos((p->a/360.0) * 2 * M_PI);

	p->X += p->dX;
	p->Y += p->dY;
}
