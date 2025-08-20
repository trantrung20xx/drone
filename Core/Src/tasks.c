#include "tasks.h"

/** UART **/

// Global variables for UART reception
char                rx_buffer[RX_BUFFER_SIZE];  // Buffer for received data
char                rx_command[RX_BUFFER_SIZE]; // Buffer for command processing
uint8_t             rx_byte    = 0;             // Variable to hold received byte
uint8_t             rx_index   = 0;             // Index for the received data buffer
volatile uint8_t    data_ready = 0; // Flag to indicate data is ready for processing
UART_HandleTypeDef* p_huart    = NULL;
// functions definitions
void UART2_Init_Receive_IT(UART_HandleTypeDef* huart) {
    if (huart == NULL)
        return;
    p_huart = huart;
    // Initialize UART reception in interrupt mode
    // Receive the firest byte
    HAL_UART_Receive_IT(p_huart, (uint8_t*)&rx_byte, 1);
}

void UART2_Receive_Handler(uint8_t byte) {
    if (rx_index < RX_BUFFER_SIZE - 1) {
        if (byte == '\n' || byte == '\r') {
            rx_buffer[rx_index] = '\0';           // Null-terminate the string
            strcpy(rx_command, rx_buffer);        // Copy to command buffer
            data_ready = 1;                       // Set flag to indicate data is ready
            rx_index   = 0;                       // Reset index for next command
            memset(rx_buffer, 0, RX_BUFFER_SIZE); // Clear the buffer
        } else {
            rx_buffer[rx_index++] = byte; // Store received byte in buffer
        }
    } else {
        // Buffer overflow, reset index
        rx_index = 0;
        memset(rx_buffer, 0, RX_BUFFER_SIZE); // Clear the buffer
    }
}

void UART2_Process_Data_If_Ready(void) {
    if (data_ready) {
        Process_UART2_Command(rx_command); // Process the command
        data_ready = 0;                    // Reset flag
    }
}

void Process_UART2_Command(const char* command) {
    // Process the command received from UART
    // TODO: Implement command processing logic
    UNUSED(command); // Suppress unused parameter warning
}

/** End UART **/
