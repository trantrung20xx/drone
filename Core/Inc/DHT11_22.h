/******************************************************************************
 * @file    DHT11_22.h
 * @brief   Header file for DHT11 and DHT22 sensor handling.
 ******************************************************************************
 * @attention
 *
 * This file is part of a project that uses the C language.
 *
 ******************************************************************************
 */

#ifndef INCLUDED_DHT11_22_H
#define INCLUDED_DHT11_22_H

#include "main.h"

typedef enum {
    DHT11 = 0, // DHT11 sensor type
    DHT22      // DHT22 sensor type
} DHT_Type;

typedef struct {
    GPIO_TypeDef* GPIOx;    // GPIO port for the sensor
    uint16_t      GPIO_Pin; // GPIO pin number for the sensor
    uint8_t       data[5];  // Data buffer to hold sensor readings
    uint8_t       checksum; // Checksum for data validation
    DHT_Type      type;     // Type of DHT sensor (DHT11 or DHT22)
} DHT11_22_t;

void DHT11_22_Init(
    DHT11_22_t* sensor, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, DHT_Type type
);
void    DHT11_22__setToInputMode(DHT11_22_t* sensor);
void    DHT11_22__setToOutputMode(DHT11_22_t* sensor);
void    DHT11_22__sendStartSignal(DHT11_22_t* sensor);
uint8_t DHT11_22__readByte(DHT11_22_t* sensor);
void    DHT11_22_Handle(DHT11_22_t* sensor);
float   DHT11_22_ReadTemperature(DHT11_22_t* sensor);
float   DHT11_22_ReadHumidity(DHT11_22_t* sensor);

#endif // INCLUDED_DHT11_22_H
