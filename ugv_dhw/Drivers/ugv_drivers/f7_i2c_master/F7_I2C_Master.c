/*
 * F7_Master_L4_Slave.c
 *
 *  Created on: Jan 30, 2024
 *      Author: chris
 */

#include "F7_I2C_Master.h"
#include <stdlib.h>

//void ugv_init_F7_Master(F7_I2C_Master *Master, I2C_HandleTypeDef *hi2c, uint8_t buffSize,
//		uint8_t slaveAddress)
//{
//	Master->TxState = NOT_READY;
//	/* Assigning reference to hi2c to I2C_Handle */
//	Master->I2C_Handle = hi2c;
//	Master->buffSize = buffSize;
//	/* Allocate memory properly */
//	Master->TxBuffer = (uint8_t *)calloc(buffSize, sizeof(uint8_t));
//	Master->RxBuffer = (uint8_t *)calloc(buffSize, sizeof(uint8_t));
//	Master->slave_address = slaveAddress;
//	Master->write_slave = MASTER_W(slaveAddress);
//	Master->read_slave = MASTER_R(slaveAddress);
//}

void ugv_init_F7_Master(F7_I2C_Master *Master, I2C_HandleTypeDef *hi2c, uint8_t buffSize,
		uint8_t f3_slave_addr, uint8_t l4_slave_addr)
{
	Master->TxState = NOT_READY;
	/* Assigning reference to hi2c to I2C_Handle */
	Master->I2C_Handle = hi2c;
	Master->buffSize = buffSize;
	/* Allocate memory properly */
	/* should check for NULL after allocation */
	Master->TxBuffer = (uint8_t *)calloc(buffSize, sizeof(uint8_t));
	Master->RxBuffer = (uint8_t *)calloc(buffSize, sizeof(uint8_t));
	Master->f3_slave_address = f3_slave_addr;
	Master->l4_slave_address = l4_slave_addr;
}
