#ifndef INC_TASKS_H_
#define INC_TASKS_H_

#include "DHT11_22.h"
#include "button.h"
#include "mpu9250.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdlib.h>

/** UART **/

#define RX_BUFFER_SIZE 16

// Declaration and definition
extern char                rx_buffer[RX_BUFFER_SIZE];  // Buffer for received data
extern char                rx_command[RX_BUFFER_SIZE]; // Buffer for command processing
extern uint8_t             rx_byte;                    // Variable to hold received byte
extern uint8_t             rx_index;   // Index for the received data buffer
extern volatile uint8_t    data_ready; // Flag to indicate data is ready for processing
extern UART_HandleTypeDef* p_huart;

// functions prototypes
void UART2_Init_Receive_IT(UART_HandleTypeDef*);
void UART2_Receive_Handler(uint8_t);
void UART2_Process_Data_If_Ready(void);
void Process_UART2_Command(const char*);

/** End UART **/

/** DHT11/22 **/

// Declaration and definition
extern DHT11_22_t dht22;
extern float      tempTest, humTest;

// functions prototypes
void  DHT_Init(void);
void  DHT_Handle(void);
float DHT_readTemperature(void);
float DHT_readHumidity(void);

/** End DHT11/22 **/

#endif /* INC_TASKS_H_ */
