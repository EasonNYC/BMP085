/**
  * @file main.c
  * @brief  Example usage for a driver for the BMP085 pressure sensor for the STM32-F4 ARM Discovery Board
	
  * @author Eason Smith
  *  
	* @date 12/23/2015
  */
	

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx.h"
//#include "cmsis_os.h"    // RTOS object definitions
#include "timer.h"       // for millis() functionality / refresh rate tracking
#include "bmp_085.h"     // pressure sensor


/** @addtogroup BMP085_Peripheral_Driver
  * @{
  */

void init_systick(void)
{
	SystemCoreClockUpdate();                         // Get Core Clock Frequency   
  if (SysTick_Config(SystemCoreClock / 1000)) {    // SysTick 1 msec interrupts  
    while (1);                                     // Capture error              
  }
}
 
int main(void)
{
	//init
	SystemInit();
	init_systick();
	
	//bmp085 stuff	
	bmp085_init();
	
	//timer + update rate stuff
	TIM2_init();
	extern uint8_t counter;
	uint32_t volatile time = millis();

	while (1) {
		
		//collect bmp data
	  bmp085_run();
		
		//track bmp update rate (in Hz) + print collected bmp data to the screen
	  if( millis() > (time + 1000)){
		  bmp085_print();
		  time = millis();
		  counter = 0;
		}
					
	}
		
}

/**
* @}
*/ 




