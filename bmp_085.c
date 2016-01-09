/**
  * @file bmp085.c
  * @brief  A Driver for the BMP085 pressure sensor for the STM32-F4 ARM Discovery Board
  * @author Eason Smith
  * @date 12/23/2015
  *
  */
#include "bmp_085.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "stdio.h"
#include "math.h"


/** @addtogroup BMP085_Peripheral_Driver
  * @{
  */

/** @defgroup BMP085 
  * @brief BMP085 pressure sensor driver modules
  * @{
  */ 


#define BMP085_ADDRESS 0x77
#define BMP085_CONTROL_REGISTER 0xF4
#define BMP085_TEMPERATURE 0x2E
#define BMP085_PRESSURE 0xF4

//globals (mostly calibration stuff)
static short ac1 = 0xFFFF;
static short ac2 = 0xFFFF;
static short ac3 = 0xFFFF;
static unsigned short ac4 = 0xFFFF;
static unsigned short ac5 = 0xFFFF;
static unsigned short ac6 = 0xFFFF;
static short b1 = 0xFFFF;
static short b2 = 0xFFFF;
//static short mb = 0xFFFF;
static short mc = 0xFFFF;
static short md = 0xFFFF;
static long b5 =0x0;
volatile uint8_t procflag = 0; 				//state machine counter
uint8_t counter = 0; 									//used to calc update rate
uint32_t PRESSURE_SEALEVEL = 102250;  //standard pressure at sealevel

/**
	* @struct BMP_READINGS
  * @brief  A struct which holds the most current bmp readings.
  * @param  None
  * @retval None
  */
struct BMP_READINGS {
  float temperature;
	uint32_t pressure;
  float altitude;
	uint16_t ut;
	uint32_t up;
} bmp;

/**
  * @fn void init_gpio()
  * @brief  Configure GPIO Pins 6 and 7. 
  * @param  None
  * @retval None
  */
void init_gpio(){
	
	//initialize struct to hold settings
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);	// enable peripheral clocks for APB1/I2C
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);// enable periph clock for GPIO on SCL SDA
		
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //choose pins 6,7 or 7,8
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 					//alternate function mode
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 			//set gpio speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; 			 //set as open drain (only has to be pulled low)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 				 //enable internal pull up resistor
	
	GPIO_Init(GPIOB, &GPIO_InitStruct); 								//init GPIOB

	//Connect I2C1 pins(selected above) to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);   //AF for SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);   //AF for SDA
}

/**
  * @fn void init_i2c1()
  * @brief  Configure I2C. 
  * @param  None
  * @retval None
  */
void init_i2c1(){
		
	I2C_InitTypeDef I2c_InitStruct;
		
	//setup I2C1
	I2c_InitStruct.I2C_ClockSpeed = 100000; 				//Hz
	I2c_InitStruct.I2C_Mode = I2C_Mode_I2C; 				//I2C mode
	I2c_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; //50% 
	I2c_InitStruct.I2C_OwnAddress1 = 0x01; 					//pick an address (master)
	I2c_InitStruct.I2C_Ack = I2C_Ack_Enable;				//disable acknowledge
	I2c_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //use 7bit address length
	
	I2C_Init(I2C1, &I2c_InitStruct);								//init i2c1
	I2C_Cmd(I2C1, ENABLE);													//enable I2c1
	
}

/**
  * @fn void init_ext_int()
  * @brief  initialize external interrupt routine on PD0. Trigger on rising edge.
  * @param  None
  * @retval None
  */
void init_ext_int(){
	//create init structs
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	// Enable RCCs for GPIOD and sysconfig
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//set up GPIO 
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;  //use pin pd0
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct); //init GPIOD
	
	//use PD0 for EXTI_Line0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
	EXTI_InitStruct.EXTI_Line = EXTI_Line0; //PD0 connected to EXTI_Line0
	EXTI_InitStruct.EXTI_LineCmd = ENABLE; // Enable the line
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  // Interrupt mode 
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; //Trigger on rising edge
	EXTI_Init(&EXTI_InitStruct);  //init struct
	
	//NVIC stuff. Add IRQ to NVIC
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn; //PD0 is connected to EXTI_Line0 which has EXTI0_IRQn
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;  //lowest priority is 0x0F, highest is 0x00
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; //enable interupt
	NVIC_Init(&NVIC_InitStruct);  //init nvic
	
}

/**
  * @fn void I2C_start(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t dir)
  * @brief  send start condition to address in a particular direction.
  * @param  I2Cx the i2c1 object.
  * @param  addr the I2C address of the bmp085
  * @param  dir the direction of communication to request from the bmp085
  * @retval None
  */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t dir){
		
	//signal start
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	//wait for slave acknowledge
	uint32_t timeout = 10000;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if((timeout--) == 0){
			printf("timeout in I2C master mode select1");
			return;
		}
	}
	
	//Send slave addr for write
	I2C_Send7bitAddress(I2Cx, addr, dir);
	
	//wait for slave to acknowledge master transmit or master recieve mode
	if(dir == I2C_Direction_Transmitter){
		timeout = 10000;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
				if((timeout--) == 0){
					printf("timeout in I2C master mode selected");
					return;
			}
		}
	}
	else if (dir == I2C_Direction_Receiver){
		timeout = 10000;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
			if((timeout--) == 0){
					printf("timeout in I2C master receiver mode selected2");
					return;
			}
		}
	}
}


/**
  * @fn void I2C_write(I2C_TypeDef* I2Cx, uint8_t byte)
  * @brief  write one byte to i2C slave which was selected in I2C_start()<--send this first
  * @param  I2Cx The i2c1 object.
  * @param  byte The byte to send to the bmp085.
  * @retval None
  */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t byte) {
	
	//send the byte
	I2C_SendData(I2Cx, byte);
	
	//wait for event saying we succesfuly transmitted the byte
	uint32_t timeout = 10000;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
			if((timeout--) == 0){
					printf("timeout in i2cwrite");
					}
	}

}


/**
  * @fn void I2C_read(I2C_TypeDef* I2Cx, uint8_t* data, uint8_t num_bytes)
  * @brief  read multiple bytes from slave.
  * @param  I2Cx The i2c1 object.
  * @param  data The pointer to the object which will hold the data.
  * @param  num_bytes The size of the array pointed to by data
  * @retval None
  */
void I2C_read(I2C_TypeDef* I2Cx, uint8_t* data, uint8_t num_bytes){
		
	//send ack for all but last byte
	I2C_AcknowledgeConfig(I2Cx,ENABLE); 
		
	for(uint8_t i = 0; i < num_bytes; i++) {
		
		//if last byte send nack and stop bit
		if (i == (num_bytes-1)) {
			I2C_AcknowledgeConfig(I2Cx,DISABLE);
			I2C_GenerateSTOP(I2Cx, ENABLE);
		}
	
		//wait till byte received
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
		
		//put byte into our data array
		data[i] = I2C_ReceiveData(I2Cx);
		
	}
	I2C_AcknowledgeConfig(I2Cx,ENABLE);

}

/**
  * @fn void I2C_send_stop(I2C_TypeDef* I2Cx)
  * @brief  send stop bit.
  * @param  I2Cx The i2c1 object.
  * @retval None
  */
void I2C_send_stop(I2C_TypeDef* I2Cx) {
	//printf("sending stop:");
	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

//
/**
  * @fn void EXTI0_IRQHandler(void)
  * @brief  set PD0 Handler interrupt 
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void) {
	
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
			
		//handle state machine
		if (procflag == 0 || procflag == 2)
			procflag++;
				
		//Clear interrupt flag
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/**
  * @fn short bmp085ReadShort(uint8_t bmp_control_reg)
  * @brief  read two bytes of data and return as a short.
  * @param  bmp_control_reg The register to request data from
  * @retval Short containing data
  */
short bmp085ReadShort(uint8_t bmp_control_reg)
{
		
	uint8_t data[2]; 	
	//contact the bmp
	I2C_start(I2C1,BMP085_ADDRESS<<1,I2C_Direction_Transmitter); //contact bmp
	I2C_write(I2C1, bmp_control_reg); //tell what you want to read
	I2C_send_stop(I2C1);
	
  //read data	
	I2C_start(I2C1,BMP085_ADDRESS<<1,I2C_Direction_Receiver); //contact bmp
	I2C_read(I2C1, data, 2);
	
	//construct short
	uint8_t msb, lsb; 
	msb = data[0]; //read first byte
	lsb = data[1]; //read second byte
	
	return (short) msb<<8 | lsb; //combine both bytes as short
}

/**
  * @fn void bmp085_requestUT()
  * @brief  requests uncompensated temperature from the bmp085
  * @param  None
  * @retval None
  */
void bmp085_requestUT(){
	
	I2C_start(I2C1, BMP085_ADDRESS<<1, I2C_Direction_Transmitter);
	I2C_write(I2C1,BMP085_CONTROL_REGISTER);
	I2C_write(I2C1,BMP085_TEMPERATURE);
	I2C_send_stop(I2C1);
}

/**
  * @fn uint8_t bmp085Read(uint8_t bmp_control_reg)
  * @brief  Read 1 byte from a BMP085 control register
  * @param  bmp_control_reg The register to read data from
  * @retval A uint8_t containing data
  */
uint8_t bmp085Read(uint8_t bmp_control_reg){
	uint8_t* data; 	
	I2C_start(I2C1,BMP085_ADDRESS<<1,I2C_Direction_Transmitter); //contact bmp
	I2C_write(I2C1, bmp_control_reg); //tell what you want to read
	I2C_send_stop(I2C1);
	
	I2C_read(I2C1, data, 1);
	
	return *data;
}

/**
  * @fn unsigned short bmp085_readUT()
  * @brief  reads a uncompensated temperature value from the bmp085
  * @param  None
  * @retval unsigned short of uncompensated temperature
  */
unsigned short bmp085_readUT(){
	
	bmp.ut = bmp085ReadShort(0xF6);
	return bmp085ReadShort(0xF6); 
}	

/**
  * @fn void bmp085_calibration()
  * @brief Calibrates bmp085. Called at startup.
  * @param  None
  * @retval None
  */
void bmp085_calibration()
{
	printf(" calibrating BMP085\n");
	ac1 = bmp085ReadShort(0xAA);
	ac2 = bmp085ReadShort(0xAC);
	ac3 = bmp085ReadShort(0xAE);
	ac4 = bmp085ReadShort(0xB0);
	ac5 = bmp085ReadShort(0xB2);
	ac6 = bmp085ReadShort(0xB4);
	b1 = bmp085ReadShort(0xB6);
	b2 = bmp085ReadShort(0xB8);
	//mb = bmp085ReadShort(0xBA);
	mc = bmp085ReadShort(0xBC);
	md = bmp085ReadShort(0xBE);
	printf(" calibration complete.\n");
}

/**
  * @fn float bmp085_calcT(unsigned short ut)
  * @brief Calculates temperature given UT. Value returned will be in units of deg C
  * @param  ut Uncompensated Temperature
  * @retval returns a float of the temperature in degrees Celcius
  */
float bmp085_calcT(unsigned short ut){

	long x1, x2;
	x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
	x2 = ((long)mc << 11)/(x1 + md);
	b5 = x1 + x2;

	return ((b5 + 8)>>4)*.1; //in celcius
}
	
/**
  * @fn float bmp085_Conv_to_F(float Temp_C)
  * @brief convert temp to farenheight
  * @param  Temp_C Temperature in degrees C
  * @retval returns a float of the temperature in degrees Farenheit
  */
float bmp085_Conv_to_F(float Temp_C){
return 1.8f*Temp_C + 32.0f; //convert to farenheit 
}	

/**
  * @fn void bmp085_requestUP(uint8_t OSS)
  * @brief request an uncompensated pressure reading.
  * @param  OSS The oversampling setting [0,1,2,or 3] see datasheet
  * @retval None
  */
void bmp085_requestUP(uint8_t OSS){
	
	if (OSS > 3){
		printf("[ERROR]in REQUEST_UP: bad OSS\n");
		return;
	}
	
	//request UP
	I2C_start(I2C1, BMP085_ADDRESS<<1, I2C_Direction_Transmitter);
	I2C_write(I2C1,BMP085_CONTROL_REGISTER);
	I2C_write(I2C1,(0x34 + (OSS<<6))); //calc reg value to send given OSS setting
	I2C_send_stop(I2C1);

}

/**
  * @fn uint32_t bmp085_readUP(uint8_t bmp_control_reg, uint8_t OSS)
  * @brief Read 3 bytes from a BMP085 pressure control register, combine them into a short.
  * @param  bmp_control_reg The register to read data from.
  * @param  OSS The oversampling setting [0,1,2,or 3] see datasheet.
  * @retval unsigned 32bit uncompensated pressure reading.
  */
uint32_t bmp085_readUP(uint8_t bmp_control_reg, uint8_t OSS)
{
	
	uint8_t num_bytes = 3;
	uint8_t data[num_bytes];
	
	//contact the bmp
	I2C_start(I2C1,BMP085_ADDRESS<<1,I2C_Direction_Transmitter); //contact bmp
	I2C_write(I2C1, bmp_control_reg); //tell what you want to read
	I2C_send_stop(I2C1);
	
  //read 3 bytes for UP
	I2C_start(I2C1,BMP085_ADDRESS<<1,I2C_Direction_Receiver); //contact bmp
	I2C_read(I2C1, data, num_bytes);
	
	//assemble and return
	uint8_t msb, lsb, xlsb; 
	msb = data[0]; 
	lsb = data[1];
	xlsb = data[2];
	return (((uint32_t) msb << 16) | ((uint32_t) lsb << 8) | (uint32_t) xlsb) >> (8-OSS);
}

/**
  * @fn uint32_t bmp085_calcP(uint32_t up, uint8_t OSS)
  * @brief calculates pressure from UP. returns pascals. most of this is taken from the datasheet.
  * @param  up Uncompensated pressure reading.
  * @param  OSS The oversampling setting [0,1,2,or 3] see datasheet.
  * @retval unsigned 32bit pressure reading in Pascals.
  */
uint32_t bmp085_calcP(uint32_t up, uint8_t OSS){

	int32_t x1, x2, x3, b3, b6, p;
	uint32_t b4, b7;
	
	//Clac B6 using global variable B5
	b6 = b5 - 4000;
	
	// Calculate B3
	x1 = (b2 * (b6 * b6)>>12)>>11;
	x2 = (ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((int32_t)ac1)*4 + x3)<<OSS) + 2)>>2;
	
	// Calculate B4
	x1 = (ac3 * b6)>>13;
	x2 = (b1 * ((b6 * b6)>>12))>>16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (ac4 * (uint32_t)(x3 + 32768))>>15;
	
	// Calculate B7
	b7 = ((uint32_t)(up - b3) * (50000>>OSS));
	
	if (b7 < 0x80000000)
		p = (b7<<1)/b4;
	else
		p = (b7/b4)<<1;
	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	p += (x1 + x2 + 3791)>>4;
	
	return p;
}

/**
  * @fn float bmp085_calcAlt(void)
  * @brief calculate altitude in meters (see datasheet)
  * @param  None
  * @retval Float of the current altitude in meters.
  */
float bmp085_calcAlt(void){
float x =  pow(bmp.pressure/PRESSURE_SEALEVEL, 0.190295F);
float y = 1.0F-x;
float altitude = 44330.0F * y;
return 44330 * (1 - pow(((float) bmp.pressure/PRESSURE_SEALEVEL), 0.190295));

}

/**
  * @fn void bmp085_run(void)
  * @brief main state machine case statement(works side by side w/ extIRQ routine)
  * @param  None
  * @retval None
  */
void bmp085_run(void){
	
	switch(procflag){
		case 0:
			//state handled by IRQ 
			break;
		
		case 1:
			bmp.ut = bmp085_readUT();
			bmp.temperature = bmp085_calcT(bmp.ut); //save temp to avg_array for temp
			bmp085_requestUP(3);
			procflag = 2;
			break;
		
		case 2:
			//state handled by IRQ 
			break;
		
		case 3:
			bmp.up = bmp085_readUP(0xF6,3);
			bmp.pressure = bmp085_calcP(bmp.up,3);
			bmp.altitude = bmp085_calcAlt();
			counter++;
			bmp085_requestUT(); //start the next cycle
			procflag = 0;
			break;
		}
	}

	/**
  * @fn void bmp085_init()
  * @brief initializes bmp085 GPIO,I2C,exInt, calibrates and starts the state machine
  * @param  None
  * @retval None
  */
	void bmp085_init(){
	
		//initialize pins, I2C, and external interrupt
	init_gpio();
	init_i2c1();
	init_ext_int();
	
		//initialize BMP085 and request a temperature reading (starts state machine)
	bmp085_calibration();	
  bmp085_requestUT();	
}

/**
  * @fn void bmp085_print()
  * @brief printfs pressure, altitude and temperatures to the screen.
  * @param  None
  * @retval None
  */
void bmp085_print(){
	
float far = bmp085_Conv_to_F(bmp085_getTemperature());
	
printf("Cel: %.2f C || Far: %.2f F || Pr: %d || Alt: %.2f ft|| Hz: %d\n", 
	bmp085_getTemperature(),
	far, 
	bmp085_getPressure(), 
	bmp085_get_altitude_feet(), 
	counter);
		 
}

/**
  * @fn float bmp085_getTemperature(void)
  * @brief returns most recent temperature reading in Celsius.
  * @param  None
  * @retval Float of temperature reading in Celsius
  */
float bmp085_getTemperature(void){ return bmp.temperature;}

/**
  * @fn uint32_t bmp085_getPressure(void)
  * @brief returns most recent pressure reading in Pascals.
  * @param  None
  * @retval uint32 of pressure reading in Pascals
  */
uint32_t bmp085_getPressure(void){ return bmp.pressure; }

/**
  * @fn float bmp085_get_altitude_meters(void)
  * @brief returns most recent altitude reading in meters.
  * @param  None
  * @retval float of altitude reading in meters
  */
float bmp085_get_altitude_meters(void){return bmp.altitude; }

/**
  * @fn float bmp085_get_altitude_feet(void)
  * @brief returns most recent altitude reading in feet.
  * @param  None
  * @retval float of altitude reading in feet
  */
float bmp085_get_altitude_feet(void){ return bmp.altitude*3.28084F; }

/**
* @}
*/ 
/**
* @}
*/ 

