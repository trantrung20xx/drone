#include "DHT11_22.h"

void DHT11_22_Init(
    DHT11_22_t* sensor, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, DHT_Type type
) {
    sensor->GPIOx    = GPIOx;
    sensor->GPIO_Pin = GPIO_Pin;
    sensor->type     = type; // Set the sensor type (DHT11 or DHT22)
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
    DHT11_22__setToInputMode(sensor);
    if (!waitForPinState(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_RESET, DHT_TIMEOUT_MS))
        return; // Wait for the pin to go low
    if (!waitForPinState(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_SET, DHT_TIMEOUT_MS))
        return; // Wait for the pin to go high
    if (!waitForPinState(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_RESET, DHT_TIMEOUT_MS))
        return; // Wait for the pin to go low again
    // Now the sensor is ready to send data (40 bits)
}

uint8_t DHT11_22__readByte(DHT11_22_t* sensor) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (!waitForPinState(
                sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_SET, DHT_TIMEOUT_MS
            ))        // Wait for the pin to go high (start of bit)
            return 0; // Timeout reached, return 0
        delay_us(35); // Wait for 35 us to read the bit
        if (HAL_GPIO_ReadPin(sensor->GPIOx, sensor->GPIO_Pin) == GPIO_PIN_SET) {
            // bit is 1
            byte = (byte << 1) | 1;
        } else {
            // bit is 0
            byte = (byte << 1) | 0;
        }
        if (!waitForPinState(
                sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_RESET, DHT_TIMEOUT_MS
            ))        // Wait for the pin to go low (end of bit)
            return 0; // Timeout reached, return 0
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

float DHT11_22_ReadHumidity(DHT11_22_t* sensor) {
    if (sensor->type == DHT11) {
        return (float)sensor->data[0]; // For DHT11, humidity is in byte 1
    } else {                           // DHT22
        float raw_humidity =
            (sensor->data[0] << 8) | sensor->data[1]; // Combine bytes 1 and 2
        return raw_humidity / 10.0f;                  // Convert to percentage
    }
}

float DHT11_22_ReadTemperature(DHT11_22_t* sensor) {
    if (sensor->type == DHT11) {
        return (float)sensor->data[2]; // For DHT11, temperature is in byte 3
    } else {                           // DHT22
        uint16_t raw_temp =
            (sensor->data[2] << 8) | sensor->data[3]; // Combine bytes 3 and 4
        if (sensor->data[2] & 0x8000) {
            // If the sign bit is set, it's a negative temperature (DHT22)
            raw_temp &= 0x7FFF;         // Clear the sign bit
            return -(raw_temp / 10.0f); // Convert to negative temperature
        } else {
            // Positive temperature (DHT11 or positive DHT22)
            return raw_temp / 10.0f; // Convert to temperature
        }
    }
}

uint8_t waitForPinState(
    GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState state, uint32_t timeout
) {
    uint32_t start_time = HAL_GetTick(); // Get the current tick count
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != state) {
        if ((HAL_GetTick() - start_time) > timeout) {
            return 0; // Timeout reached
        }
    }
    return 1; // Desired pin state reached
}
