#include "DHT11_22.h"

void DHT11_22_Init(DHT11_22_t *sensor, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, DHT_Type type)
{
    sensor->GPIOx    = GPIOx;
    sensor->GPIO_Pin = GPIO_Pin;
    sensor->type     = type; // Set the sensor type (DHT11 or DHT22)

    // Initialize GPIO pin as open-drain output with pull-up resistor
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = GPIO_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_OD; // Open-drain
    GPIO_InitStruct.Pull             = GPIO_PULLUP;         // Internal pull-up
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // Default: release line

    // initialize data buffer and checksum
    for (int i = 0; i < 5; i++)
    {
        sensor->data[i] = 0; // Initialize data buffer
    }
    sensor->checksum = 0; // Initialize checksum
}

// void DHT11_22__setToInputMode(DHT11_22_t *sensor)
// {
//     GPIO_InitTypeDef GPIO_InitStruct = {0};
//     GPIO_InitStruct.Pin              = sensor->GPIO_Pin;
//     GPIO_InitStruct.Mode             = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull             = GPIO_PULLUP;         // Use pull-up resistor
//     GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW; // Low speed for input
//     HAL_GPIO_Init(sensor->GPIOx, &GPIO_InitStruct);
// }

// void DHT11_22__setToOutputMode(DHT11_22_t *sensor)
// {
//     GPIO_InitTypeDef GPIO_InitStruct = {0};
//     GPIO_InitStruct.Pin              = sensor->GPIO_Pin;
//     GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP; // Push-pull output
//     GPIO_InitStruct.Pull             = GPIO_NOPULL;         // No pull-up or pull-down
//     GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW; // Low speed for output
//     HAL_GPIO_Init(sensor->GPIOx, &GPIO_InitStruct);
// }

uint8_t DHT11_22__sendStartSignal(DHT11_22_t *sensor)
{
    // DHT11_22__setToOutputMode(sensor);                                  // Set to output mode
    HAL_GPIO_WritePin(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_RESET); // Pull pin low
    if (sensor->type == DHT11)
        HAL_Delay(20);  // Hold low for at least 20 ms for DHT11
    else                // DHT22
        delay_us(1500); // Hold low for 1.5 ms
    HAL_GPIO_WritePin(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_SET); // Release pin
    delay_us(30);                                                     // Wait for 20-40 us
    // DHT11_22__setToInputMode(sensor);
    if (!waitForPinState(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_RESET, DHT_TIMEOUT_US))
        return 0; // Wait for the pin to go low
    if (!waitForPinState(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_SET, DHT_TIMEOUT_US))
        return 0; // Wait for the pin to go high
    if (!waitForPinState(sensor->GPIOx, sensor->GPIO_Pin, GPIO_PIN_RESET, DHT_TIMEOUT_US))
        return 0; // Wait for the pin to go low again
    // Now the sensor is ready to send data (40 bits)
    return 1; // Start signal sent successfully
}

uint8_t DHT11_22__readByte(DHT11_22_t *sensor)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++)
    {
        if (!waitForPinState(sensor->GPIOx,
                             sensor->GPIO_Pin,
                             GPIO_PIN_SET,
                             DHT_TIMEOUT_US)) // Wait for the pin to go high (start of bit)
            return 0;                         // Timeout reached, return 0
        delay_us(50);                         // Wait for 50 us to read the bit
        if (HAL_GPIO_ReadPin(sensor->GPIOx, sensor->GPIO_Pin) == GPIO_PIN_SET)
        {
            // bit is 1
            byte = (byte << 1) | 1;
        }
        else
        {
            // bit is 0
            byte = (byte << 1) | 0;
        }
        if (!waitForPinState(sensor->GPIOx,
                             sensor->GPIO_Pin,
                             GPIO_PIN_RESET,
                             DHT_TIMEOUT_US)) // Wait for the pin to go low (end of bit)
            return 0;                         // Timeout reached, return 0
    }
    return byte; // Return the read byte
}

void DHT11_22_Handle(DHT11_22_t *sensor)
{
    // Send start signal to the sensor
    if (!DHT11_22__sendStartSignal(sensor))
    {
        // No response from sensor, handle error
        return;
    }
    // Read 5 bytes of data
    // - byte 1: Humidity integer part
    // - byte 2: Humidity decimal part
    // - byte 3: Temperature integer part
    // - byte 4: Temperature decimal part
    // - byte 5: Checksum
    __disable_irq(); // Disable interrupts
    for (int i = 0; i < 5; i++)
    {
        sensor->data[i] = DHT11_22__readByte(sensor); // Read 4 bytes of data
    }
    // read checksum byte
    sensor->checksum = sensor->data[4]; // byte 5 is checksum
    __enable_irq();                     // Enable interrupts
    // Validate checksum
    uint16_t sum = (uint16_t)sensor->data[0] + sensor->data[1] + sensor->data[2] + sensor->data[3];
    if ((sum & 0xFF) != sensor->checksum)
    {
        // Checksum is invalid, handle error
    }
}

float DHT11_22_ReadHumidity(DHT11_22_t *sensor)
{
    if (sensor->type == DHT11)
    {
        return (float)sensor->data[0]; // For DHT11, humidity is in byte 1
    }
    else
    {                                                                     // DHT22
        uint16_t raw_humidity = (sensor->data[0] << 8) | sensor->data[1]; // Combine bytes 1 and 2
        return raw_humidity / 10.0f;                                      // Convert to percentage
    }
}

float DHT11_22_ReadTemperature(DHT11_22_t *sensor)
{
    if (sensor->type == DHT11)
    {
        return (float)sensor->data[2]; // For DHT11, temperature is in byte 3
    }
    else
    {                                                                 // DHT22
        uint16_t raw_temp = (sensor->data[2] << 8) | sensor->data[3]; // Combine bytes 3 and 4
        if (raw_temp & 0x8000)
        {
            // If the sign bit is set, it's a negative temperature (DHT22)
            raw_temp &= 0x7FFF;         // Clear the sign bit
            return -(raw_temp / 10.0f); // Convert to negative temperature
        }
        else
        {
            // Positive temperature
            return raw_temp / 10.0f; // Convert to temperature
        }
    }
}

uint8_t
waitForPinState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state, uint32_t timeout_us)
{
    while (timeout_us--)
    {
        if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == state)
            return 1; // Desired pin state reached
        delay_us(1);  // Wait for 1 microsecond
    }
    return 0; // Timeout
}
