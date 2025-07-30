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

typedef struct {
    GPIO_TypeDef* GPIOx;    // GPIO port for the sensor
    uint16_t      GPIO_Pin; // GPIO pin number for the sensor
    uint8_t       data[5];  // Data buffer to hold sensor readings
    uint8_t       checksum; // Checksum for data validation
    // uint8_t       state;    // Current state of the sensor (e.g., idle, reading)
} DHT11_22_t;

void    DHT11_22_Init(DHT11_22_t* sensor, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void    DHT11_22__setToInputMode(DHT11_22_t* sensor);
void    DHT11_22__setToOutputMode(DHT11_22_t* sensor);
void    DHT11_22__sendStartSignal(DHT11_22_t* sensor);
uint8_t DHT11_22__readByte(DHT11_22_t* sensor);
void    DHT11_22_Handle(DHT11_22_t* sensor);
float   DHT11_22_ReadTemperature(DHT11_22_t* sensor);
float   DHT11_22_ReadHumidity(DHT11_22_t* sensor);

#endif // INCLUDED_DHT11_22_H
