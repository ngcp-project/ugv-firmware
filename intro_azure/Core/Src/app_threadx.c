/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "main.h"
#include "tx_api.h"
#include "nx_api.h"

#include "nxd_dhcp_client.h"
#include "stdint.h"

#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_STACK_SIZE (1024)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t thread_stack1[THREAD_STACK_SIZE];
TX_THREAD thread_ptr;
uint8_t thread_stack2[THREAD_STACK_SIZE];
TX_THREAD thread_ptr2;

uint8_t thread_stack3[THREAD_STACK_SIZE];
uint8_t thread_stack4[THREAD_STACK_SIZE];

ULONG ip_address;
ULONG network_mask;

#define DEMO_STACK_SIZE		(1024)

TX_THREAD			my_thread;
NX_PACKET_POOL 		my_pool;
NX_IP				my_ip;
NX_DHCP				my_dhcp;


uint8_t error_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID my_thread_entry(ULONG initial_input);
VOID my_thread_entry2(ULONG initial_input);

VOID my_dhcp_thread_entry(ULONG thread_input);
//VOID my_netx_driver(struct NX_IP_DRIVER_STRUCT *driver_req);

VOID  nx_stm32_eth_driver(NX_IP_DRIVER *driver_req_ptr);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
//  (void)byte_pool;
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
//  tx_thread_create(&thread_ptr, "my_thread", my_thread_entry, 0x1234, thread_stack,
//		  THREAD_STACK_SIZE, 15, 13, 1, TX_AUTO_START);
  UINT status;

  tx_thread_create(&my_thread, "my_thread", my_dhcp_thread_entry, 0,
		  thread_stack1, DEMO_STACK_SIZE, 2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);

//  byte_pool = byte_pool + DEMO_STACK_SIZE;
//  memory_ptr = memory_ptr + DEMO_STACK_SIZE;


  // initialize the NetX duo system
  nx_system_initialize();

  //Create a packet pool
//  status = nx_packet_pool_create(&my_pool, "NetX Main Packet Pool",
//		  1024, byte_pool, 64000);
  status = nx_packet_pool_create(&my_pool, "NetX Main Packet Pool",
		  128, thread_stack2, 1024);

//  byte_pool = byte_pool + 1024;
//  memory_ptr += 1024;

  // Check for pool creation error
  if (status)
	  error_counter++;

  //Create an IP instance without an IP address
  status = nx_ip_create(&my_ip, "My NetX IP Instance", IP_ADDRESS(0,0,0,0),
		  0xFFFFFF00, &my_pool, nx_stm32_eth_driver, thread_stack3, DEMO_STACK_SIZE, 1);

//  	  byte_pool = byte_pool + DEMO_STACK_SIZE;

//  memory_ptr += 1024;

  //Enable ARP supply ARP cache memory for my IP instance
  status = nx_arp_enable(&my_ip, (void *)thread_stack4, 1024);
//  byte_pool = byte_pool + 1024;
//  memory_ptr += 1024;


  //Check for ARP errors
  if (status)
	  error_counter++;

  //Enable UDP
  status = nx_udp_enable(&my_ip);
  if (status)
	  error_counter++;



  // Creating thread of higher priority
//  tx_thread_create(&thread_ptr2, "my_thread2", my_thread_entry2, 0x1234, thread_stack2,
//		  THREAD_STACK_SIZE, 14, 14, 1, TX_AUTO_START);
//  tx_trace_enable(&tracex_buffer, TRACEX_BUFFER_SIZE, 30);

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

//	ret = nx_ip_create(&EthIP, "NetX IP Instance 0", NULL_IP_ADDRESS, NULL_IP_ADDRESS,
//	&EthPool, nx_stm32_eth_driver, pointer, IP_MEMORY_SIZE, DEFAULT_PRIORITY);

	//nx_udp_enable()
  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

void my_dhcp_thread_entry(ULONG thread_input)
{
	UINT		status;
	ULONG		actual_status;
	NX_PACKET	*my_packet;

	// Wait for link to come up
	do
	{
		//Get link status
		status = nx_ip_status_check(&my_ip, NX_IP_LINK_ENABLED,
				&actual_status, 100);
	}while (status != NX_SUCCESS);

	//Create DHCP instance
	status = nx_dhcp_create(&my_dhcp, &my_ip, "My DHCP");

	if (status)
		error_counter++;

	//Start DHCP Client
	nx_dhcp_start(&my_dhcp);

	if (status)
		error_counter++;

	status = nx_ip_status_check(&my_ip, NX_IP_ADDRESS_RESOLVED,
			(ULONG *) &status, 10000);

    /* Check to see if we have a valid IP address.  */
    if (status)
    {
      error_counter++;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);

      return;
    }
    else
    {
    	status = nx_ip_address_get(&my_ip, &ip_address, &network_mask);


  /* Yes, a valid IP address is now on lease…  All NetX Duo
        services are available.*/
        if (!status)
        	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    }

}

//VOID my_thread_entry(ULONG initial_input)
//{
//	while(1)
//	{
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
//		HAL_Delay(500);
//		// The function argument describes the number of ticks that the thread is asleep
//		tx_thread_sleep(20);
//	}
//}
//
//VOID my_thread_entry2(ULONG initial_input)
//{
//	while(1)
//	{
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//		HAL_Delay(500);
//		// The function argument describes the number of ticks that the thread is asleep
//		tx_thread_sleep(20);
//	}
//}


/* USER CODE END 1 */
