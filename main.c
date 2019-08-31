// INSTRUCTIONS:
// Call the following functions as needed:
// Basics: 
//	twi_init(); //call this first, enables NRF TWI drivers
//
// RGBW: (no special init function required)
//	turn on: RGBW_on(); returns 1 if failed, 0 if ok
//	turn off: RGBW_off(); returns 1 if failed, 0 if ok
//	get colors: RGBW_get(); returns a pointer to a static array of doubles [double R, doubleG, doubleB]
//
// GPIO expander 1 (main board right angle LEDs:
//	init: GPIOEXP1_init();
//	LED_BT_on();
//	LED_BT_off();
//
// MUX (both at the same time):
// Set the MUX to the sensor you want to read, then use the RGBW sensor code or Prox code to get data
//	MUX_init(); //sets output status on NRF GPIO pins
//	MUX_set(s3,s2,s1,s0); //s3 to s0 sets which sensor you get data from





/** @file
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h" 
//
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

#define NRF_LOG_MODULE_NAME "APP"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static const nrf_drv_twi_t m_twi1 = NRF_DRV_TWI_INSTANCE(1);


// UART code copied over
//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}



#ifdef ENABLE_LOOPBACK_TEST
/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    LEDS_ON(LEDS_MASK);
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}


#endif

  //

void MUX_init(){
		nrf_gpio_cfg_output(30); //S0
		nrf_gpio_cfg_output(0); //S1
		nrf_gpio_cfg_output(1); //S2
		nrf_gpio_cfg_output(2); //S3
}


void twi_init (void)
{
		//make sure TWO0 and TWI1 are both enabled in config text file somewhere else in project
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 16,
       .sda                = 15,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void twi1_init (void)
{
		//make sure TWO0 and TWI1 are both enabled in config text file somewhere else in project
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi1_config = {
       .scl                = 18,
       .sda                = 17,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi1, &twi1_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi1);
}

void writei2c(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write_high, uint8_t data_to_write_low){
	//power on spectrum sensor
	uint8_t dataToWrite[2] = {data_to_write_high, data_to_write_low};
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}

void writei2cOneByte(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write){
	//power on spectrum sensor
	uint8_t dataToWrite[1] = {data_to_write};
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}

uint16_t readi2cHighLow(uint8_t deviceAddr, uint8_t read_reg_addr){
	ret_code_t err_code; //to hold return code, currently do nothing with it
	
	//READ 1st byte
	uint16_t data_both;
	//first half of read
	uint8_t dataToSend4[1] = {read_reg_addr};
	err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToSend4[0], sizeof(dataToSend4), true);


	uint8_t read_data2[2];
	err_code = nrf_drv_twi_rx(&m_twi, deviceAddr, &read_data2[0], 2);
	//printf("addr: %x, data: 0x%x 0x%x \r\n",read_reg_addr ,read_data2[1], read_data2[0]);
	
	data_both = read_data2[0] | (read_data2[1] << 8);
	//printf("both: 0x%x \r\n",data_both);
	
	return data_both;
}

uint8_t readi2cOneByte(uint8_t deviceAddr, uint8_t read_reg_addr){
	ret_code_t err_code; //to hold return code, currently do nothing with it
	
	//READ 1st byte
	//first half of read
	uint8_t dataToSend4[1] = {read_reg_addr};
	err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToSend4[0], sizeof(dataToSend4), true);
	
	uint8_t read_data2[1];
	err_code = nrf_drv_twi_rx(&m_twi, deviceAddr, &read_data2[0], 1);
	
	return read_data2[0];
}

bool GPIOEXP1_init(){
	//make sure twi_init() was run before this function.
				//adr,R/!W		config hi low
	writei2c((0x23<<1), 0x0c, 0x0,0x0); //config port0 to output
	writei2c((0x23<<1), 0x0d, 0x0,0x0); //config port1 to output
	writei2c((0x23<<1), 0x0e, 0x0,0x0); //config port2 to output
	
	return 0;
}

uint8_t GPIOEXP1_readPort0(){
	return readi2cOneByte((0x23<<1)|0x1, 0x4);
}

bool LED_BT_on(){
	//make sure GPIOEXP1_init was run before this function.
	//read current GPIOEXP1 settings:
	uint8_t port0 = GPIOEXP1_readPort0();
	port0 |= 0x08;
	writei2cOneByte((0x23<<1),0x04, port0);
	
	return 0;
}

bool LED_BT_off(){
	//make sure GPIOEXP1_init was run before this function.
	//read current GPIOEXP1 settings:
	uint8_t port0 = GPIOEXP1_readPort0();
	port0 &= 0xF7;  // note F7 = 0b11110111
	writei2cOneByte((0x23<<1),0x04, port0);
	
	return 0;
}

void MUX_set(bool s3, bool s2, bool s1, bool s0){
	s3?nrf_gpio_pin_set(2):nrf_gpio_pin_clear(2);
	s2?nrf_gpio_pin_set(1):nrf_gpio_pin_clear(1);
	s1?nrf_gpio_pin_set(0):nrf_gpio_pin_clear(0);
	s0?nrf_gpio_pin_set(30):nrf_gpio_pin_clear(30);
}

bool RGBW_on(){
		//turns RGBW sensor on, returns 0 if success, 1 if falied
					//addr,  reg,  hi, low
		writei2c(0x10,0x00,0x00,0x00); //enable RGBW sensors with 80ms integration time

		//read back settings
		uint16_t i2cResult = readi2cHighLow(0x10, 0x00);
	
		return i2cResult != 0x00;
}
bool RGBW_off(){
		//turns RGBW sensor on, returns 0 if success, 1 if falied
					//addr,  reg,  hi, low
		writei2c(0x10,0x00,0x00,0x01); //enable RGBW sensors with 80ms integration time

		//read back settings
		uint16_t i2cResult = readi2cHighLow(0x10, 0x00);
	
		return i2cResult != 0x01;
}

//returns an array of doubles, max value is 255 for each color, skips matrix multiplications
double* RGBW_get(){
	bool printResult = false;
	static double RGBreturn[3] = {0,0,0};
	uint16_t r,g,b, w;
	
	//read colors
	r = readi2cHighLow(0x10, 0x08);	
	g = readi2cHighLow(0x10, 0x09);
	b = readi2cHighLow(0x10, 0x0A);
	w = readi2cHighLow(0x10, 0x0B);
	
	//do some math
	double normalizedR = r;
	double normalizedG = g;
	double normalizedB = b;
	normalizedR = normalizedR/w;
	normalizedG = normalizedG/w;
	normalizedB = normalizedB/w;
	int hexR, hexB, hexG;
	hexR = normalizedR*255;
	hexB = normalizedB*255;
	hexG = normalizedG*255;
	
	if(printResult){
		printf("  raw red: 0x%x\r\n",r );
		printf("raw green: 0x%x\r\n",g );
		printf(" raw blue:  0x%x\r\n",b );
		printf("raw white:  0x%x\r\n",w );
		printf("RGB: %x %x %x\r\n\r\n",hexR,hexG,hexB);
	}
	RGBreturn[0]= hexR;
	RGBreturn[1]= hexB;
	RGBreturn[2]= hexG;
	
	return RGBreturn;
	
}
 
/**
 * @brief Function for main application entry.
 */
int main(void)
{
		MUX_init();
	  LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
    uint32_t err_code_; // modified uint32_t err_code to uint32_t err_code_
    const app_uart_comm_params_t comm_params =
      {
          9,10,12,11,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code_);

    APP_ERROR_CHECK(err_code_);

#ifndef ENABLE_LOOPBACK_TEST
    printf("\r\nStart: \r\n");
//below original TWI code
    ret_code_t err_code;
    
		printf("\r\nStart before error check: \r\n");
    //APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    printf("TWI scanner.\r\n");

    twi_init();
		printf("\r\nFinished twi init: \r\n");
		
		
		nrf_delay_ms(1000);
    
		
    while (true)
    {
					/*
					//read VCNL prox snesor data
					uint8_t address = 0x60; //0x60  for prox					
					uint8_t dataToSend2[3] = {0x03,0x00,0x00};
					err_code = nrf_drv_twi_tx(&m_twi, address, &dataToSend2[0], sizeof(dataToSend2), false);
					
					//first half of read
					uint8_t dataToSend[1] = {0xF2};
					err_code = nrf_drv_twi_tx(&m_twi, address, &dataToSend[0], sizeof(dataToSend), true);


					uint8_t read_data[2];
      		err_code = nrf_drv_twi_rx(&m_twi, address, &read_data[0], 2);
					printf("recieved: 0x%x 0x%x\r\n",read_data[1], read_data[0]);
					if (err_code == NRF_SUCCESS){}
					NRF_LOG_FLUSH();
					*/
			
					//read spectrum snesor data
					uint8_t address2 = 0x39; //0x60  for prox					
					//uint8_t dataToSend2[3] = {0x03,0x00,0x00};
					//err_code = nrf_drv_twi_tx(&m_twi, address, &dataToSend2[0], sizeof(dataToSend2), false);
					
					/*
					printf("spectrum start=====>\r\n");
						
					//power on spectrum sensor
					uint8_t dataToSend3[2] = {0x80, 0x01};
					err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend3[0], sizeof(dataToSend3), false);
					
					//confirm above write
						uint8_t dataToSend5[1] = {0x80};
						err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend5[0], sizeof(dataToSend5), true);


						uint8_t read_data3[1];
						err_code = nrf_drv_twi_rx(&m_twi, address2, &read_data3[0], 1);
						printf("addr: 0x80, data: 0x%x \r\n",read_data3[0]);

						
						//start measurement
						dataToSend3[0] = 0x80;
						dataToSend3[1] = 0x03;
						err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend3[0], sizeof(dataToSend3), false);
					
						//confirm above write
						dataToSend5[1] = 0x80;
						err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend5[0], sizeof(dataToSend5), true);
						err_code = nrf_drv_twi_rx(&m_twi, address2, &read_data3[0], 1);
						printf("addr: 0x80, data: 0x%x \r\n",read_data3[0]);
						
					for(uint8_t reg = 0x95; reg < 0xA0; reg += 1){
						//first half of read
						uint8_t dataToSend4[1] = {reg};
						err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend4[0], sizeof(dataToSend4), true);


						uint8_t read_data2[1];
						err_code = nrf_drv_twi_rx(&m_twi, address2, &read_data2[0], 1);
						printf("addr: %x, data: 0x%x \r\n",reg,read_data2[0]);
						if (err_code == NRF_SUCCESS){}
						
						NRF_LOG_FLUSH();
					}
					printf("<====== spectrum end\r\n");
					*/
					
					
					RGBW_get(); //returns pointer to static [double R, doubleB, doubleG]
					
			nrf_delay_ms(250);
    }
    
#else

    // This part of the example is just for testing the loopback .
    while (true)
    {
        uart_loopback_test();
    }
#endif
	
	//above added UART code
	
}

/** @} */
