#Driver Requirements:
##Hardware
1)Bosch BMP085 pressure sensor
2)STM32-F4 ARM Cortex-M4 MCU (such as the STM32F4 Discovery Board)

###WIRING CONNECTIONS:
[STM32F4 PIN |	Bosch BMP085 Pressure Sensor PIN ]
  GND	| GND
  5V |	VIN
  PB7	| SDA
  PB6	| SCL
  PD0	| EOC

*It should be noted that pin assignments PB6, PB7, PD0 are not currently able to be re-configured, however this may be a feature added in the future. Currently you must use these pins to use this driver.*

##Software
-ARM Standard Peripheral Library must be included to properly use this driver. Specificly:
  "stm32f4xx.h"
  "stm32f4xx_i2c.h"
  "stm32f4xx_rcc.h"
  "stm32f4xx_gpio.h"
  "stm32f4xx_syscfg.h"
  "stm32f4xx_exti.h"

#Documentation:
All files have been commented and are Doxygen compatable. A doxyfile has been included for easily generating documentation. Input and output paths may need to be edited to the path of your download directory. 

#Usage:
In order to include the driver in a project, BMP085.h must be included in the main.c file. Within main.c, bmp085_init() must be called once to initialize the sensor on startup, and then bmp085_run() must be called in your main loop to collect Temperature Pressure and Altitude readings. To retrieve the update rate of the sensor (in Hz), a counter variable is used in conjunction with the TIM2 timer by including timer.h in main and using the extern counter variable located in "bmp_085.c". This functionality is temporary and will be encapsulated within "bmp_085.c" in a future update.
