/* bh1750.h */
#ifndef BH1750_H
#define BH1750_H

#include "main.h"
#include <stdint.h>

/* BH1750 I2C Address (ADDR pin to GND = 0x23, ADDR pin to VCC = 0x5C) */
#define BH1750_ADDR_LOW    0x23  /* ADDR pin connected to GND */
#define BH1750_ADDR_HIGH   0x5C  /* ADDR pin connected to VCC */
#define BH1750_ADDR        BH1750_ADDR_LOW  /* Change this based on your wiring */

/* BH1750 Commands */
#define BH1750_POWER_DOWN  0x00
#define BH1750_POWER_ON    0x01
#define BH1750_RESET       0x07
#define BH1750_CONT_H_RES  0x10  /* Continuous H-Resolution Mode (1 lx resolution) */
#define BH1750_CONT_H_RES2 0x11  /* Continuous H-Resolution Mode 2 (0.5 lx resolution) */
#define BH1750_CONT_L_RES  0x13  /* Continuous L-Resolution Mode (4 lx resolution) */
#define BH1750_ONE_H_RES   0x20  /* One Time H-Resolution Mode */
#define BH1750_ONE_H_RES2  0x21  /* One Time H-Resolution Mode 2 */
#define BH1750_ONE_L_RES   0x23  /* One Time L-Resolution Mode */

/* Function prototypes */
HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef BH1750_ReadLight(I2C_HandleTypeDef* hi2c, float *lux);

#endif /* BH1750_H */
