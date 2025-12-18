/**
 * @file    SH1106.h
 * @brief   Enhanced SH1106 OLED Display Driver for STM32
 * @author  Original: Tilen Majerle <tilen@majerle.eu>
 *          Modified: ControllersTech (www.controllerstech.com)
 *          Enhanced: 2024
 * @version 2.0
 *
 * @details This driver provides a complete interface for SH1106 OLED displays
 *          using I2C communication. Features include:
 *          - Basic drawing primitives (pixels, lines, shapes)
 *          - Text rendering with custom fonts
 *          - Bitmap support
 *          - Display control (on/off, contrast, inversion)
 *          - Partial screen updates for better performance
 *          - Hardware scrolling support
 *
 * Copyright (C) Alexander Lutsai, 2016
 * Copyright (C) Tilen Majerle, 2015
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 */

#ifndef SH1106_H
#define SH1106_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== Includes ==================== */
#include "main.h"
#include "fonts.h"
#include "stdlib.h"
#include "string.h"

/* ==================== Configuration ==================== */

/**
 * @defgroup SH1106_Config Configuration
 * @brief    Driver configuration parameters
 * @{
 */

/** @brief I2C device address (7-bit address left-shifted) */
#ifndef SH1106_I2C_ADDR
#define SH1106_I2C_ADDR         (0x3C << 1)
#endif

/** @brief Display width in pixels */
#ifndef SH1106_WIDTH
#define SH1106_WIDTH            128
#endif

/** @brief Display height in pixels */
#ifndef SH1106_HEIGHT
#define SH1106_HEIGHT           64
#endif

/** @brief I2C communication timeout in milliseconds */
#ifndef SH1106_I2C_TIMEOUT
#define SH1106_I2C_TIMEOUT      20000
#endif

/**
 * @}
 */

/* ==================== Type Definitions ==================== */

/**
 * @brief Color enumeration for SH1106 display
 */
typedef enum {
    SH1106_COLOR_BLACK = 0x00,  /**< Black color (pixel off) */
    SH1106_COLOR_WHITE = 0x01   /**< White color (pixel on) */
} SH1106_COLOR_t;

/* ==================== Core Functions ==================== */

/**
 * @defgroup SH1106_Core Core Functions
 * @brief    Essential display initialization and control
 * @{
 */

/**
 * @brief  Initialize the SH1106 display
 * @details Performs I2C device detection, sends initialization sequence,
 *          clears the display buffer, and turns on the display
 * @retval 0: Display not detected or initialization failed
 * @retval 1: Display initialized successfully
 * @note   Must be called before any other display operations
 */
uint8_t SH1106_Init(void);

/**
 * @brief  Update the entire display from internal buffer
 * @details Transfers the complete frame buffer to the display via I2C
 * @note   Call this after drawing operations to make changes visible
 * @note   For better performance, use SH1106_UpdateRegion() for partial updates
 */
void SH1106_UpdateScreen(void);

/**
 * @brief  Update only a specific region of the display
 * @param  x X coordinate of region start
 * @param  y Y coordinate of region start
 * @param  w Width of region in pixels
 * @param  h Height of region in pixels
 * @details More efficient than full screen update for small changes
 * @note   Region is automatically clipped to display boundaries
 */
void SH1106_UpdateRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

/**
 * @brief  Fill entire display buffer with specified color
 * @param  color Fill color (SH1106_COLOR_BLACK or SH1106_COLOR_WHITE)
 * @note   Call SH1106_UpdateScreen() after to apply changes
 */
void SH1106_Fill(SH1106_COLOR_t color);

/**
 * @brief  Clear the display (fill with black and update)
 * @details Convenience function that fills buffer with black and updates display
 */
void SH1106_Clear(void);

/**
 * @}
 */

/* ==================== Display Control ==================== */

/**
 * @defgroup SH1106_Control Display Control Functions
 * @brief    Hardware display control operations
 * @{
 */

/**
 * @brief  Turn display ON
 * @details Enables the DC-DC converter and display output
 */
void SH1106_ON(void);

/**
 * @brief  Turn display OFF
 * @details Disables display output while preserving buffer contents
 * @note   Display can be turned back on without re-initialization
 */
void SH1106_OFF(void);

/**
 * @brief  Set display contrast
 * @param  contrast Contrast level (0-255)
 * @details 0 = minimum brightness, 255 = maximum brightness
 */
void SH1106_SetContrast(uint8_t contrast);

/**
 * @brief  Invert display colors (hardware level)
 * @param  i Inversion state: 1 = inverted, 0 = normal
 * @details Hardware inversion - faster than software buffer inversion
 */
void SH1106_InvertDisplay(int i);

/**
 * @brief  Toggle software buffer inversion
 * @details Inverts all pixels in the frame buffer
 * @note   Call SH1106_UpdateScreen() after to apply changes
 */
void SH1106_ToggleInvert(void);

/**
 * @brief  Check if display is initialized
 * @retval 1 if initialized, 0 otherwise
 */
uint8_t SH1106_IsInitialized(void);

/**
 * @brief  Check if display is powered on
 * @retval 1 if display is ON, 0 if OFF
 */
uint8_t SH1106_IsDisplayOn(void);

/**
 * @}
 */

/* ==================== Drawing Primitives ==================== */

/**
 * @defgroup SH1106_Draw Drawing Functions
 * @brief    Primitive drawing operations
 * @{
 */

/**
 * @brief  Draw a single pixel
 * @param  x X coordinate (0 to SH1106_WIDTH-1)
 * @param  y Y coordinate (0 to SH1106_HEIGHT-1)
 * @param  color Pixel color
 * @note   Coordinates outside display bounds are ignored
 * @note   Call SH1106_UpdateScreen() after to apply changes
 */
void SH1106_DrawPixel(uint16_t x, uint16_t y, SH1106_COLOR_t color);

/**
 * @brief  Get pixel color at specified coordinates
 * @param  x X coordinate
 * @param  y Y coordinate
 * @retval Pixel state: 1 (white/on) or 0 (black/off)
 * @note   Returns 0 for out-of-bounds coordinates
 */
uint8_t SH1106_GetPixel(uint16_t x, uint16_t y);

/**
 * @brief  Draw a line using Bresenham's algorithm
 * @param  x0 Start point X coordinate
 * @param  y0 Start point Y coordinate
 * @param  x1 End point X coordinate
 * @param  y1 End point Y coordinate
 * @param  c Line color
 * @note   Optimized for horizontal and vertical lines
 */
void SH1106_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SH1106_COLOR_t c);

/**
 * @brief  Draw a rectangle outline
 * @param  x Top-left corner X coordinate
 * @param  y Top-left corner Y coordinate
 * @param  w Rectangle width in pixels
 * @param  h Rectangle height in pixels
 * @param  c Border color
 */
void SH1106_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SH1106_COLOR_t c);

/**
 * @brief  Draw a filled rectangle
 * @param  x Top-left corner X coordinate
 * @param  y Top-left corner Y coordinate
 * @param  w Rectangle width in pixels
 * @param  h Rectangle height in pixels
 * @param  c Fill color
 */
void SH1106_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SH1106_COLOR_t c);

/**
 * @brief  Draw a triangle outline
 * @param  x1 First vertex X coordinate
 * @param  y1 First vertex Y coordinate
 * @param  x2 Second vertex X coordinate
 * @param  y2 Second vertex Y coordinate
 * @param  x3 Third vertex X coordinate
 * @param  y3 Third vertex Y coordinate
 * @param  color Border color
 */
void SH1106_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                         uint16_t x3, uint16_t y3, SH1106_COLOR_t color);

/**
 * @brief  Draw a filled triangle
 * @param  x1 First vertex X coordinate
 * @param  y1 First vertex Y coordinate
 * @param  x2 Second vertex X coordinate
 * @param  y2 Second vertex Y coordinate
 * @param  x3 Third vertex X coordinate
 * @param  y3 Third vertex Y coordinate
 * @param  color Fill color
 */
void SH1106_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                               uint16_t x3, uint16_t y3, SH1106_COLOR_t color);

/**
 * @brief  Draw a circle outline using Midpoint Circle Algorithm
 * @param  x0 Center X coordinate
 * @param  y0 Center Y coordinate
 * @param  r Circle radius in pixels
 * @param  c Border color
 */
void SH1106_DrawCircle(int16_t x0, int16_t y0, int16_t r, SH1106_COLOR_t c);

/**
 * @brief  Draw a filled circle
 * @param  x0 Center X coordinate
 * @param  y0 Center Y coordinate
 * @param  r Circle radius in pixels
 * @param  c Fill color
 */
void SH1106_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SH1106_COLOR_t c);

/**
 * @}
 */

/* ==================== Text Rendering ==================== */

/**
 * @defgroup SH1106_Text Text Rendering Functions
 * @brief    Character and string display operations
 * @{
 */

/**
 * @brief  Set text cursor position
 * @param  x X coordinate for next character
 * @param  y Y coordinate for next character
 * @details Subsequent text output will begin at this position
 */
void SH1106_GotoXY(uint16_t x, uint16_t y);

/**
 * @brief  Write a single character at current cursor position
 * @param  ch Character to write (ASCII)
 * @param  Font Pointer to font structure
 * @param  color Text color
 * @retval Character written, or 0 if error (out of bounds)
 * @note   Cursor advances automatically after character is written
 * @note   Only printable ASCII characters (32-127) are supported
 */
char SH1106_Putc(char ch, FontDef_t* Font, SH1106_COLOR_t color);

/**
 * @brief  Write a null-terminated string at current cursor position
 * @param  str Pointer to string
 * @param  Font Pointer to font structure
 * @param  color Text color
 * @retval 0 on success, or the character that caused failure
 * @note   String must be null-terminated
 * @note   Writing stops if text exceeds display bounds
 */
char SH1106_Puts(char* str, FontDef_t* Font, SH1106_COLOR_t color);

/**
 * @}
 */

/* ==================== Bitmap Functions ==================== */

/**
 * @defgroup SH1106_Bitmap Bitmap Drawing Functions
 * @brief    Display bitmap images
 * @{
 */

/**
 * @brief  Draw a monochrome bitmap
 * @param  x Top-left corner X coordinate
 * @param  y Top-left corner Y coordinate
 * @param  bitmap Pointer to bitmap data (1 bit per pixel)
 * @param  w Bitmap width in pixels
 * @param  h Bitmap height in pixels
 * @param  color Color for set pixels (1 bits)
 * @details Bitmap format: horizontal byte alignment, MSB first
 * @note   Bitmap data must be in program memory or RAM
 */
void SH1106_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap,
                       int16_t w, int16_t h, uint16_t color);

/**
 * @}
 */

/* ==================== Scrolling Functions ==================== */

/**
 * @defgroup SH1106_Scroll Hardware Scrolling Functions
 * @brief    Display scrolling control
 * @{
 */

/**
 * @brief  Scroll display content to the right
 * @param  start_row Start page address (0-7)
 * @param  end_row End page address (0-7)
 * @note   Hardware scrolling - does not modify buffer
 */
void SH1106_ScrollRight(uint8_t start_row, uint8_t end_row);

/**
 * @brief  Scroll display content to the left
 * @param  start_row Start page address (0-7)
 * @param  end_row End page address (0-7)
 * @note   Hardware scrolling - does not modify buffer
 */
void SH1106_ScrollLeft(uint8_t start_row, uint8_t end_row);

/**
 * @brief  Diagonal scroll to the right
 * @param  start_row Start page address (0-7)
 * @param  end_row End page address (0-7)
 */
void SH1106_Scrolldiagright(uint8_t start_row, uint8_t end_row);

/**
 * @brief  Diagonal scroll to the left
 * @param  start_row Start page address (0-7)
 * @param  end_row End page address (0-7)
 */
void SH1106_Scrolldiagleft(uint8_t start_row, uint8_t end_row);

/**
 * @brief  Scroll display content down
 * @param  start_row Start page address (0-7)
 * @param  end_row End page address (0-7)
 */
void SH1106_ScrollDown(uint8_t start_row, uint8_t end_row);

/**
 * @brief  Stop all scrolling
 * @details Returns display to normal mode
 */
void SH1106_Stopscroll(void);

/**
 * @}
 */

/* ==================== Low-Level I2C Functions ==================== */

/**
 * @defgroup SH1106_I2C Low-Level I2C Communication
 * @brief    Direct I2C communication functions
 * @{
 */

/**
 * @brief  Initialize I2C peripheral
 * @note   Usually called automatically by SH1106_Init()
 */
void SH1106_I2C_Init(void);

/**
 * @brief  Write a single byte via I2C
 * @param  address I2C device address
 * @param  reg Register/command byte
 * @param  data Data byte to write
 */
void SH1106_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
void SH1106_Fill_Rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);

/**
 * @brief  Write multiple bytes via I2C
 * @param  address I2C device address
 * @param  reg Register/command byte
 * @param  data Pointer to data array
 * @param  count Number of bytes to write
 * @note   Maximum count is 255 bytes
 */
void SH1106_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count);

/**
 * @}
 */

/* ==================== Usage Example ==================== */

/**
 * @code
 * // Initialize display
 * if (SH1106_Init()) {
 *     // Set contrast
 *     SH1106_SetContrast(128);
 *
 *     // Clear screen
 *     SH1106_Clear();
 *
 *     // Draw text
 *     SH1106_GotoXY(10, 10);
 *     SH1106_Puts("Hello World!", &Font_7x10, SH1106_COLOR_WHITE);
 *
 *     // Draw shapes
 *     SH1106_DrawCircle(64, 32, 20, SH1106_COLOR_WHITE);
 *     SH1106_DrawRectangle(10, 40, 50, 20, SH1106_COLOR_WHITE);
 *
 *     // Update display
 *     SH1106_UpdateScreen();
 * }
 * @endcode
 */

#ifdef __cplusplus
}
#endif

#endif /* SH1106_H */
