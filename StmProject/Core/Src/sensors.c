/*
 * sensors.c
 *
 *  Created on: Jan 10, 2026
 *      Author: STM32 Project
 */

#include "sensors.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Sensor global variables
SensorManager_t sensorManager = {0};
bool sensorMode = false;

// Private variables for sensors
static float lastDistance = 0.0f;
static bool lastColorIsWhite = true;

void Sensors_Init(void) {
    sensorManager.state = SENSOR_STATE_MENU;
    sensorManager.currentSensor = SENSOR_DISTANCE;
    sensorManager.lastUpdateTime = 0;
    sensorManager.exitConfirmed = false;
    
    // Don't initialize sensor hardware here - do it only when sensor is selected
    // This avoids GPIO conflicts with LEDs
}

void Sensors_Start(void) {
    sensorManager.state = SENSOR_STATE_MENU;
    sensorManager.lastUpdateTime = HAL_GetTick();
    sensorManager.exitConfirmed = false;
    sensorMode = true;
}

void Sensors_Update(void) {
    if (sensorManager.state != SENSOR_STATE_READING) return;
    
    uint32_t currentTime = HAL_GetTick();
    // Update sensor readings every 200ms
    if (currentTime - sensorManager.lastUpdateTime < 200) return;
    
    sensorManager.lastUpdateTime = currentTime;
    
    // Read sensor based on current selection
    switch(sensorManager.currentSensor) {
        case SENSOR_DISTANCE:
            lastDistance = Ultrasound_ReadDistance();
            break;
        case SENSOR_COLOR:
            lastColorIsWhite = Infrared_IsWhite();
            break;
        default:
            break;
    }
}

void Sensors_HandleInput(uint8_t button) {
    // Button mapping:
    // 1 = Exit (PC1)
    // 2 = Confirm/OK (PC2)
    // 3 = Up (PC3)
    // 4 = Down (PC4)
    
    // Exit button
    if (button == 1) {
        Sensors_Exit();
        return;
    }
    
    // Handle menu state
    if (sensorManager.state == SENSOR_STATE_MENU) {
        if (button == 3) { // Up button - previous sensor
            sensorManager.currentSensor = (sensorManager.currentSensor + SENSOR_COUNT - 1) % SENSOR_COUNT;
        }
        else if (button == 4) { // Down button - next sensor
            sensorManager.currentSensor = (sensorManager.currentSensor + 1) % SENSOR_COUNT;
        }
        else if (button == 2) { // Confirm button - start reading
            // Initialize the selected sensor hardware
            if (sensorManager.currentSensor == SENSOR_DISTANCE) {
                Ultrasound_Init();
            } else {
                Infrared_Init();
            }
            sensorManager.state = SENSOR_STATE_READING;
            sensorManager.lastUpdateTime = HAL_GetTick();
        }
    }
    // Handle reading state
    else if (sensorManager.state == SENSOR_STATE_READING) {
        if (button == 2 || button == 4) { // Confirm or Down button - return to menu
            sensorManager.state = SENSOR_STATE_MENU;
        }
    }
}

void Sensors_Draw(void) {
    switch(sensorManager.state) {
        case SENSOR_STATE_MENU:
            Sensors_DrawMenu();
            break;
        case SENSOR_STATE_READING:
            SH1106_Clear();
            if (sensorManager.currentSensor == SENSOR_DISTANCE) {
                Sensors_DrawDistanceReading();
            } else {
                Sensors_DrawColorReading();
            }
            break;
        default:
            break;
    }
    
    SH1106_UpdateScreen();
}

void Sensors_DrawMenu(void) {
    SH1106_Clear();

    SH1106_GotoXY(35, 2);
    SH1106_Puts("SENSORS", &Font_7x10, 1);

    // Draw sensor options
    const char* sensorNames[SENSOR_COUNT] = {
        "Distance",
        "Color"
    };

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        uint8_t yPos = 15 + i * 15;

        if (i == sensorManager.currentSensor) {
            // Selected item
            SH1106_DrawRectangle(20, yPos - 2, 88, 12, 1);
            SH1106_GotoXY(40, yPos);
            SH1106_Puts((char*)sensorNames[i], &Font_7x10, 0);
        } else {
            // Normal item
            SH1106_GotoXY(40, yPos);
            SH1106_Puts((char*)sensorNames[i], &Font_7x10, 1);
        }
    }

    // Instructions
    SH1106_GotoXY(10, 55);
    SH1106_Puts("UP/DN Nav, OK Sel", &Font_7x10, 1);
}

void Sensors_DrawDistanceReading(void) {
    char buffer[32];
    
    // Draw title
    SH1106_GotoXY(15, 0);
    SH1106_Puts("DISTANCE SENSOR", &Font_7x10, 1);
    
    // Draw distance value
    SH1106_GotoXY(10, 20);
    SH1106_Puts("Distance:", &Font_7x10, 1);
    
    snprintf(buffer, sizeof(buffer), "%.2f cm", lastDistance);
    SH1106_GotoXY(20, 35);
    SH1106_Puts(buffer, &Font_7x10, 1);
    
    // Draw instructions
    SH1106_GotoXY(10, 55);
    SH1106_Puts("EXIT: Return to menu", &Font_7x10, 1);
}

void Sensors_DrawColorReading(void) {
    // Draw title
    SH1106_GotoXY(20, 0);
    SH1106_Puts("COLOR SENSOR", &Font_7x10, 1);
    
    // Draw color detection result
    SH1106_GotoXY(10, 20);
    SH1106_Puts("Detected Color:", &Font_7x10, 1);
    
    const char* colorStr = lastColorIsWhite ? "WHITE" : "BLACK";
    SH1106_GotoXY(30, 35);
    SH1106_Puts((char*)colorStr, &Font_7x10, 1);
    
    // Draw instructions
    SH1106_GotoXY(10, 55);
    SH1106_Puts("EXIT: Return to menu", &Font_7x10, 1);
}

void Sensors_Exit(void) {
    sensorMode = false;
    sensorManager.state = SENSOR_STATE_MENU;
}

// ==================== Ultrasound Sensor Implementation ====================

void Ultrasound_Init(void) {
    // Configure trigger pin as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIOE clock if not already enabled
    __HAL_RCC_GPIOE_CLK_ENABLE();
    
    // Configure PD13 as output (trigger)
    GPIO_InitStruct.Pin = ULTRASOUND_TRIG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ULTRASOUND_TRIG_PORT, &GPIO_InitStruct);
    
    // Configure PD14 as input (echo)
    GPIO_InitStruct.Pin = ULTRASOUND_ECHO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ULTRASOUND_ECHO_PORT, &GPIO_InitStruct);
    
    // Set trigger pin low initially
    HAL_GPIO_WritePin(ULTRASOUND_TRIG_PORT, ULTRASOUND_TRIG_PIN, GPIO_PIN_RESET);
}

float Ultrasound_ReadDistance(void) {
    uint32_t timeout = 50000; // Timeout counter
    uint32_t startTime, endTime;
    float distance = 0.0f;
    
    // Send 10us trigger pulse
    HAL_GPIO_WritePin(ULTRASOUND_TRIG_PORT, ULTRASOUND_TRIG_PIN, GPIO_PIN_RESET);
    HAL_Delay(1); // Short delay
    HAL_GPIO_WritePin(ULTRASOUND_TRIG_PORT, ULTRASOUND_TRIG_PIN, GPIO_PIN_SET);
    
    // Create 10us pulse
    for (volatile int i = 0; i < 72; i++) { // ~10us at 72MHz
        __NOP();
    }
    
    HAL_GPIO_WritePin(ULTRASOUND_TRIG_PORT, ULTRASOUND_TRIG_PIN, GPIO_PIN_RESET);
    
    // Wait for echo pin to go high (start of echo)
    timeout = 50000;
    while (HAL_GPIO_ReadPin(ULTRASOUND_ECHO_PORT, ULTRASOUND_ECHO_PIN) == GPIO_PIN_RESET) {
        if (--timeout == 0) return lastDistance; // Timeout, return last value
    }
    
    startTime = HAL_GetTick() * 1000; // Convert to microseconds approximation
    
    // Wait for echo pin to go low (end of echo)
    timeout = 50000;
    while (HAL_GPIO_ReadPin(ULTRASOUND_ECHO_PORT, ULTRASOUND_ECHO_PIN) == GPIO_PIN_SET) {
        if (--timeout == 0) return lastDistance; // Timeout, return last value
    }
    
    endTime = HAL_GetTick() * 1000; // Convert to microseconds approximation
    
    // Calculate distance (speed of sound = 343 m/s = 0.0343 cm/us)
    // Distance = (Time * Speed) / 2 (divide by 2 because sound travels there and back)
    uint32_t duration = endTime - startTime;
    distance = (duration * 0.0343f) / 2.0f;
    
    // Validate distance (typical HC-SR04 range: 2-400 cm)
    if (distance < 2.0f || distance > 400.0f) {
        return lastDistance; // Return last valid value if out of range
    }
    
    return distance;
}

// ==================== Infrared Sensor Implementation ====================

void Infrared_Init(void) {
    // Configure infrared sensor pin as input
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIOE clock if not already enabled
    __HAL_RCC_GPIOE_CLK_ENABLE();
    
    // Note: PD14 is shared with ultrasound echo pin
    // This implementation assumes the infrared sensor can be on the same pin
    // or you may need to use a different pin
    GPIO_InitStruct.Pin = INFRARED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Pull-up for typical IR sensors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(INFRARED_PORT, &GPIO_InitStruct);
}

bool Infrared_IsWhite(void) {
    // Most IR sensors output:
    // HIGH (1) when detecting WHITE surface (high reflectivity)
    // LOW (0) when detecting BLACK surface (low reflectivity)
    GPIO_PinState pinState = HAL_GPIO_ReadPin(INFRARED_PORT, INFRARED_PIN);
    return (pinState == GPIO_PIN_SET);
}
