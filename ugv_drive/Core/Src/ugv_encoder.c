/*
 * ENCODER.c
 *
 *  Created on: Mar 08, 2025
 *      Author: Korbinian Schoerghuber
 *
 *      Ver. 1.0
 */

#include "ugv_encoder.h"


		float wheel_diameter = 5.255; //wheel diameter in inch
		float enc_counts = 1404; //encoder counts/rotation (old value 659.232)
		float time = 2400; //1min/25ms

float encoder(uint32_t e)
{
		// encoder variables
		float wheel_rpm = 0;
		float velocity = 0;
		static uint32_t encoder_value = 0;
		static uint32_t prev_encoder_value = 0;
		uint32_t delta_encoder_value = 0;

	// encoder
	prev_encoder_value = encoder_value;
	encoder_value = e;


	if((int)(encoder_value - prev_encoder_value) > 60000)
	{
		delta_encoder_value = (65536 - encoder_value) + prev_encoder_value;
	}
	else if((int)(encoder_value - prev_encoder_value) < (-60000))
	{
		delta_encoder_value = encoder_value - (prev_encoder_value - 65536);  // to handle the overflowed of a 2^16 bit variable
	}
	else
	{
		delta_encoder_value = abs(encoder_value - prev_encoder_value);
	}

	wheel_rpm = (delta_encoder_value / enc_counts) * time; //encoder / encoder counts/rotation * 1min/25ms
	velocity = wheel_diameter * PI * ft_in * wheel_rpm * ftpm_mph; // velocity in mph
	return velocity;
}






