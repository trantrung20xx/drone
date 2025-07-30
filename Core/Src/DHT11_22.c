#include "DHT11_22.h"

void DHT11_22_Init(DHT11_22_t* sensor, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    sensor->GPIOx    = GPIOx;
    sensor->GPIO_Pin = GPIO_Pin;
    for (int i = 0; i < 5; i++) {
        sensor->data[i] = 0; // Initialize data buffer
    }
    sensor->checksum = 0; // Initialize checksum
}

void DHT11_22__setToInputMode(DHT11_22_t* sensor) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = sensor->GPIO_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;         // No pull-up or pull-down
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW; // Low speed for input
    HAL_GPIO_Init(sensor->GPIOx, &GPIO_InitStruct);
}

void DHT11_22__setToOutputMode(DHT11_22_t* sensor) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = sensor->GPIO_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP; // Push-pull output
    GPIO_InitStruct.Pull             = GPIO_PULLUP;         // Use pull-up resistor
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW; // Low speed for output
    HAL_GPIO_Init(sensor->GPIOx, &GPIO_InitStruct);
}

void DHT11_22__sendStartSignal(DHT11_22_t* sensor) {
    DHT11_22__setToOutputMode(sensor); // Set to output mode
    HAL_GPIO_WritePin(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_RESET); // Pull pin low
    HAL_Delay(18); // Hold low for 18 ms
    HAL_GPIO_WritePin(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_SET); // Release pin
    DHT11_22__setToInputMode(sensor);                                 // Set to input mode
    while (HAL_GPIO_ReadPin(sensor->GPIOx, sensor->GPIO_Pin) == GPIO_PIN_SET)
        ; // Wait for the pin to go low
    while (HAL_GPIO_ReadPin(sensor->GPIOx, sensor->GPIO_Pin) == GPIO_PIN_RESET)
        ; // Wait for the pin to go high
    while (HAL_GPIO_ReadPin(sensor->GPIOx, sensor->GPIO_Pin) == GPIO_PIN_SET)
        ; // Wait for the pin to go low again
    // Now the sensor is ready to send data (40 bits)
}

uint8_t DHT11_22__readByte(DHT11_22_t* sensor) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        while (HAL_GPIO_ReadPin(sensor->GPIOx, sensor->GPIO_Pin) == GPIO_PIN_RESET)
            ; // Wait for the pin to go high (start of bit)
        // delay_us(50); // Wait for 50 us to read the bit
        if (HAL_GPIO_ReadPin(sensor->GPIOx, sensor->GPIO_Pin) == GPIO_PIN_SET) {
            // bit is 1
            byte = (byte << 1) | 1;
        } else {
            // bit is 0
            byte = (byte << 1) | 0;
        }
        while (HAL_GPIO_ReadPin(sensor->GPIOx, sensor->GPIO_Pin) == GPIO_PIN_SET)
            ; // Wait for the pin to go low (end of bit)
    }
    return byte; // Return the read byte
}

void DHT11_22_Handle(DHT11_22_t* sensor) {
    DHT11_22__sendStartSignal(sensor); // Send start signal to the sensor
    // Read 5 bytes of data
    // - byte 1: Humidity integer part
    // - byte 2: Humidity decimal part
    // - byte 3: Temperature integer part
    // - byte 4: Temperature decimal part
    // - byte 5: Checksum
    for (int i = 0; i < 5; i++) {
        sensor->data[i] = DHT11_22__readByte(sensor); // Read 5 bytes of data
    }
    // read checksum byte
    sensor->checksum = DHT11_22__readByte(sensor);
    // Validate checksum
    if (sensor->data[0] + sensor->data[1] + sensor->data[2] + sensor->data[3] !=
        sensor->checksum) {
        // Checksum is invalid, handle error
    }
}
