/*
 * sensors.h
 *
 *  Created on: Jan 10, 2026
 *      Author: STM32 Project
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"
#include "SH1106.h"
#include <stdbool.h>

// Sensor types
typedef enum {
    SENSOR_DISTANCE,
    SENSOR_COLOR,
    SENSOR_COUNT
} SensorType_t;

// Sensor state
typedef enum {
    SENSOR_STATE_MENU,
    SENSOR_STATE_READING
} SensorState_t;

// Sensor pins definitions
// Ultrasound sensor uses PE7 (Trigger) and PE8 (Echo)
#define ULTRASOUND_TRIG_PIN GPIO_PIN_7
#define ULTRASOUND_TRIG_PORT GPIOE
#define ULTRASOUND_ECHO_PIN GPIO_PIN_8
#define ULTRASOUND_ECHO_PORT GPIOE

// Infrared sensor uses PE8 (digital input for black/white detection)
// Note: Shares PE8 with ultrasound echo - only one sensor active at a time
#define INFRARED_PIN GPIO_PIN_8
#define INFRARED_PORT GPIOE

// Sensor manager structure
typedef struct {
    SensorState_t state;
    SensorType_t currentSensor;
    uint32_t lastUpdateTime;
    bool exitConfirmed;
} SensorManager_t;

// Function prototypes
void Sensors_Init(void);
void Sensors_Start(void);
void Sensors_Update(void);
void Sensors_HandleInput(uint8_t button);
void Sensors_Draw(void);
void Sensors_Exit(void);

// Ultrasound sensor functions
void Ultrasound_Init(void);
float Ultrasound_ReadDistance(void);

// Infrared sensor functions
void Infrared_Init(void);
bool Infrared_IsWhite(void);

// Display functions
void Sensors_DrawMenu(void);
void Sensors_DrawDistanceReading(void);
void Sensors_DrawColorReading(void);

// External access to sensor state
extern SensorManager_t sensorManager;
extern bool sensorMode;

#endif /* INC_SENSORS_H_ */
