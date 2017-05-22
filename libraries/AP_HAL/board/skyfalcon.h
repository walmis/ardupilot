/*
 * skyfalcon.h
 *
 *  Created on: Dec 28, 2016
 *      Author: walmis
 */

#ifndef LIBRARIES_AP_HAL_BOARD_SKYFALCON_H_
#define LIBRARIES_AP_HAL_BOARD_SKYFALCON_H_

#define HAL_BOARD_NAME "SKYFALCON"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150

#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
//#define HAL_PARAM_DEFAULTS_PATH "/etc/defaults.parm"

#define HAL_INS_DEFAULT HAL_INS_MPU60XX_I2C
#define HAL_BARO_DEFAULT HAL_BARO_MS5611_I2C
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define HAL_SERIAL0_BAUD_DEFAULT 115200
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE
#define HAL_STORAGE_SIZE            8192
#define HAL_BARO_MS5611_I2C_BUS  0
#define HAL_BARO_MS5611_I2C_ADDR 0x77
#define HAL_COMPASS_HMC5843_I2C_BUS 0
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#define HAL_INS_MPU60x0_I2C_BUS 0
#define HAL_INS_MPU60x0_I2C_ADDR 0x68
//#define HAL_HAVE_IMU_HEATER         1 // for Pixhawk2
//#define HAL_IMU_TEMP_DEFAULT       -1 // disabled
#define HAL_GPIO_A_LED_PIN        127
#define HAL_GPIO_B_LED_PIN        128
#define HAL_GPIO_C_LED_PIN        129
#define HAL_GPIO_LED_ON           LOW
#define HAL_GPIO_LED_OFF          HIGH



#endif /* LIBRARIES_AP_HAL_BOARD_SKYFALCON_H_ */
