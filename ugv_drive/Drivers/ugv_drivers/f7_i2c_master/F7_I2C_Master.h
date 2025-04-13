/*
 * F7_Master.h
 *
 *  Created on: Jan 14, 2024
 *      Author: chris
 */

/* This header file contains a class declaration that facilitates communication between
 * the F7 Master and the L4 Slave device
 */

#ifndef INC_F7_I2C_MASTER_H_
#define INC_F7_I2C_MASTER_H_

#include "stm32f7xx_hal.h"

#define L4_SLAVE_ADDRESS 					(0x20U)
#define F3_SLAVE_ADDRESS 					(0x28U)
#define MASTER_R(SLAVE_ADDRESS) 			((SLAVE_ADDRESS << 1) | 1U)
#define MASTER_W(SLAVE_ADDRESS) 			((SLAVE_ADDRESS << 1) | 0U)
#define I2C_BUFF_SIZE						(30U)

enum txStatus
{
	NOT_READY,
	READY
};

/* Might need to make class volatile */
typedef struct f7_i2c_master
{
	/* Pointer to I2C handler that contains pertinent I2C
	 * Config information
	 */
	I2C_HandleTypeDef *I2C_Handle;
	uint8_t TxState;
	/* Pointer to buffer that will transmit I2C slave message */
	volatile uint8_t *TxBuffer;
	/* Pointer to buffer that will receive I2C slave message */
	uint8_t *RxBuffer;
	/* Specify the size of both the I2C tx and rx buffers */
	uint8_t buffSize;

	/* L4 Slave address */
	uint8_t slave_address;
	uint8_t write_slave;
	uint8_t read_slave;

	uint8_t f3_slave_address;
	uint8_t l4_slave_address;

}F7_I2C_Master;

void ugv_init_F7_Master(F7_I2C_Master *Master, I2C_HandleTypeDef *hi2c, uint8_t buffSize,
		uint8_t f3_slave_addr, uint8_t l4_slave_addr);

//Create something similar to a destructor in C?
//
//class F7_Master_L4_Slave
//{
//	public:
//		F7_Master_L4_Slave();
//		F7_Master_L4_Slave(I2C_HandleTypeDef *hi2c, uint8_t buffSize,
//							uint8_t slaveAddress);
//
//		/* Destructor to deallocate heap memory
//		 * that is pointed to by by txBuffer and rxBuffer
//		 */
//		~F7_Master_L4_Slave();
//
//		/* Create Boolean flag to indicate that transfer can happen */
//		volatile bool txState;
//
//		/* Create geters to return slave addresses */
//
//		/* Pointer to buffer that will transmit I2C slave message */
//		volatile uint8_t *txBuffer;
//
//		/* Pointer to buffer that will receive I2C slave message */
//		uint8_t *rxBuffer;
//
//		/* Specify the size of both the I2C tx and rx buffers */
//		volatile uint8_t buffSize;
//
//
//		/* L4 Slave address */
//		uint8_t slave_address;
//		uint8_t write_slave;
//		uint8_t read_slave;
//
//
//	private:
//		/* Pointer to I2C handler that contains pertinent I2C
//		 * Config information
//		 */
//		I2C_HandleTypeDef *I2C_Handle;
//};
#endif /* INC_F7_I2C_MASTER_H_ */
