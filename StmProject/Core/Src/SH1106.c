/**
 * Enhanced SH1106 OLED Display Driver
 * Original author: Tilen Majerle<tilen@majerle.eu>
 * Modification for SH1106: ControllersTech
 * Enhanced version with improved error handling, performance, and features
 */
#include "SH1106.h"

extern I2C_HandleTypeDef hi2c1;
#define SH1106_I2C &hi2c1

// Command and data write macros
#define SH1106_WRITECOMMAND(command) SH1106_I2C_Write(SH1106_I2C_ADDR, 0x00, (command))
#define SH1106_WRITEDATA(data) SH1106_I2C_Write(SH1106_I2C_ADDR, 0x40, (data))
#define ABS(x) ((x) > 0 ? (x) : -(x))

// Display control commands
#define SH1106_NORMALDISPLAY 0xA6
#define SH1106_INVERTDISPLAY 0xA7
#define SH1106_DISPLAYOFF 0xAE
#define SH1106_DISPLAYON 0xAF
#define SH1106_SETCONTRAST 0x81
#define SH1106_SEGREMAP 0xA1

// Frame buffer
static uint8_t SH1106_Buffer[SH1106_WIDTH * SH1106_HEIGHT / 8];

// Private structure
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
    uint8_t DisplayOn;
} SH1106_t;

static SH1106_t SH1106 = {0};

/**
 * @brief Initialize the SH1106 display
 * @return 1 if successful, 0 if failed
 */
uint8_t SH1106_Init(void) {
    // Check I2C connection with retry
    uint8_t retry = 3;
    while (retry--) {
        if (HAL_I2C_IsDeviceReady(SH1106_I2C, SH1106_I2C_ADDR, 1, 5000) == HAL_OK) {
            break;
        }
        HAL_Delay(10);
    }

    if (retry == 0) {
        return 0; // Device not ready
    }

    HAL_Delay(50); // Stabilization delay

    // Initialization sequence
    SH1106_WRITECOMMAND(0xAE); // Display off
    SH1106_WRITECOMMAND(0xD5); // Set display clock divide
    SH1106_WRITECOMMAND(0x80); // Divide ratio
    SH1106_WRITECOMMAND(0xA8); // Set multiplex ratio
    SH1106_WRITECOMMAND(0x3F); // 1/64 duty
    SH1106_WRITECOMMAND(0x00); // Set display offset
    SH1106_WRITECOMMAND(0x00); // No offset
    SH1106_WRITECOMMAND(0x40); // Set start line address
    SH1106_WRITECOMMAND(0xAD); // Set DC-DC enable
    SH1106_WRITECOMMAND(0x8B); // DC-DC ON
    SH1106_WRITECOMMAND(0xA1); // Set segment re-map (horizontal flip)
    SH1106_WRITECOMMAND(0xC8); // Set COM output scan direction (vertical flip)
    SH1106_WRITECOMMAND(0xDA); // Set COM pins hardware configuration
    SH1106_WRITECOMMAND(0x12);
    SH1106_WRITECOMMAND(0x81); // Set contrast control
    SH1106_WRITECOMMAND(0xFF); // Maximum contrast
    SH1106_WRITECOMMAND(0xD9); // Set pre-charge period
    SH1106_WRITECOMMAND(0x1F);
    SH1106_WRITECOMMAND(0xDB); // Set VCOMH deselect level
    SH1106_WRITECOMMAND(0x40);
    SH1106_WRITECOMMAND(0x33); // Set pump voltage (0x30-0x33)
    SH1106_WRITECOMMAND(0xA6); // Normal display mode
    SH1106_WRITECOMMAND(0xA4); // Display from RAM

    // Clear buffer and update screen
    SH1106_Fill(SH1106_COLOR_BLACK);
    SH1106_UpdateScreen();

    // Turn on display
    SH1106_WRITECOMMAND(0xAF);

    // Initialize structure
    SH1106.CurrentX = 0;
    SH1106.CurrentY = 0;
    SH1106.Inverted = 0;
    SH1106.Initialized = 1;
    SH1106.DisplayOn = 1;

    return 1;
}

/**
 * @brief Update the display with buffer contents
 */
void SH1106_UpdateScreen(void) {
    if (!SH1106.Initialized) return;

    for (uint8_t page = 0; page < 8; page++) {
        SH1106_WRITECOMMAND(0xB0 + page); // Set page address
        SH1106_WRITECOMMAND(0x02); // Set lower column address (offset for SH1106)
        SH1106_WRITECOMMAND(0x10); // Set higher column address
        SH1106_I2C_WriteMulti(SH1106_I2C_ADDR, 0x40, &SH1106_Buffer[SH1106_WIDTH * page], SH1106_WIDTH);
    }
}

/**
 * @brief Update only a specific region of the display
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param w Width
 * @param h Height
 */
void SH1106_UpdateRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    if (!SH1106.Initialized) return;

    uint8_t startPage = y / 8;
    uint8_t endPage = (y + h - 1) / 8;

    for (uint8_t page = startPage; page <= endPage && page < 8; page++) {
        SH1106_WRITECOMMAND(0xB0 + page);
        SH1106_WRITECOMMAND(0x02 + (x & 0x0F));
        SH1106_WRITECOMMAND(0x10 + ((x >> 4) & 0x0F));

        uint16_t startIdx = SH1106_WIDTH * page + x;
        uint16_t len = (w > SH1106_WIDTH - x) ? (SH1106_WIDTH - x) : w;
        SH1106_I2C_WriteMulti(SH1106_I2C_ADDR, 0x40, &SH1106_Buffer[startIdx], len);
    }
}

/**
 * @brief Fill entire screen with color
 * @param color Color to fill (SH1106_COLOR_BLACK or SH1106_COLOR_WHITE)
 */
void SH1106_Fill(SH1106_COLOR_t color) {
    memset(SH1106_Buffer, (color == SH1106_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SH1106_Buffer));
}

/**
 * @brief Clear the display (fill with black and update)
 */
void SH1106_Clear(void) {
    SH1106_Fill(SH1106_COLOR_BLACK);
    SH1106_UpdateScreen();
}

/**
 * @brief Draw a single pixel
 * @param x X coordinate
 * @param y Y coordinate
 * @param color Pixel color
 */
void SH1106_DrawPixel(uint16_t x, uint16_t y, SH1106_COLOR_t color) {
    if (x >= SH1106_WIDTH || y >= SH1106_HEIGHT) {
        return;
    }

    if (SH1106.Inverted) {
        color = (SH1106_COLOR_t)!color;
    }

    uint16_t idx = x + (y / 8) * SH1106_WIDTH;
    uint8_t bit = y % 8;

    if (color == SH1106_COLOR_WHITE) {
        SH1106_Buffer[idx] |= (1 << bit);
    } else {
        SH1106_Buffer[idx] &= ~(1 << bit);
    }
}

/**
 * @brief Get pixel color at coordinates
 * @param x X coordinate
 * @param y Y coordinate
 * @return Pixel color (0 or 1)
 */
uint8_t SH1106_GetPixel(uint16_t x, uint16_t y) {
    if (x >= SH1106_WIDTH || y >= SH1106_HEIGHT) {
        return 0;
    }

    uint16_t idx = x + (y / 8) * SH1106_WIDTH;
    uint8_t bit = y % 8;
    return (SH1106_Buffer[idx] & (1 << bit)) ? 1 : 0;
}

/**
 * @brief Set cursor position for text
 * @param x X coordinate
 * @param y Y coordinate
 */
void SH1106_GotoXY(uint16_t x, uint16_t y) {
    SH1106.CurrentX = x;
    SH1106.CurrentY = y;
}

/**
 * @brief Write a single character
 * @param ch Character to write
 * @param Font Pointer to font structure
 * @param color Text color
 * @return Character written or 0 on error
 */
char SH1106_Putc(char ch, FontDef_t* Font, SH1106_COLOR_t color) {
    if (!Font ||
        SH1106_WIDTH < (SH1106.CurrentX + Font->FontWidth) ||
        SH1106_HEIGHT < (SH1106.CurrentY + Font->FontHeight)) {
        return 0;
    }

    uint32_t charIdx = (ch - 32) * Font->FontHeight;

    for (uint32_t row = 0; row < Font->FontHeight; row++) {
        uint16_t b = Font->data[charIdx + row];
        for (uint32_t col = 0; col < Font->FontWidth; col++) {
            SH1106_COLOR_t pixelColor = ((b << col) & 0x8000) ? color : (SH1106_COLOR_t)!color;
            SH1106_DrawPixel(SH1106.CurrentX + col, SH1106.CurrentY + row, pixelColor);
        }
    }

    SH1106.CurrentX += Font->FontWidth;
    return ch;
}

/**
 * @brief Write a string
 * @param str String to write
 * @param Font Pointer to font structure
 * @param color Text color
 * @return 0 on success, failed character otherwise
 */
char SH1106_Puts(char* str, FontDef_t* Font, SH1106_COLOR_t color) {
    if (!str || !Font) return 0;

    while (*str) {
        if (SH1106_Putc(*str, Font, color) != *str) {
            return *str;
        }
        str++;
    }
    return 0;
}

/**
 * @brief Draw a line using Bresenham's algorithm
 */
void SH1106_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SH1106_COLOR_t c) {
    // Boundary clamping
    x0 = (x0 >= SH1106_WIDTH) ? (SH1106_WIDTH - 1) : x0;
    x1 = (x1 >= SH1106_WIDTH) ? (SH1106_WIDTH - 1) : x1;
    y0 = (y0 >= SH1106_HEIGHT) ? (SH1106_HEIGHT - 1) : y0;
    y1 = (y1 >= SH1106_HEIGHT) ? (SH1106_HEIGHT - 1) : y1;

    int16_t dx = ABS(x1 - x0);
    int16_t dy = ABS(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = ((dx > dy) ? dx : -dy) / 2;
    int16_t e2;

    // Optimized vertical line
    if (dx == 0) {
        uint16_t yMin = (y0 < y1) ? y0 : y1;
        uint16_t yMax = (y0 < y1) ? y1 : y0;
        for (uint16_t i = yMin; i <= yMax; i++) {
            SH1106_DrawPixel(x0, i, c);
        }
        return;
    }

    // Optimized horizontal line
    if (dy == 0) {
        uint16_t xMin = (x0 < x1) ? x0 : x1;
        uint16_t xMax = (x0 < x1) ? x1 : x0;
        for (uint16_t i = xMin; i <= xMax; i++) {
            SH1106_DrawPixel(i, y0, c);
        }
        return;
    }

    // Bresenham's line algorithm
    while (1) {
        SH1106_DrawPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;

        e2 = err;
        if (e2 > -dx) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dy) {
            err += dx;
            y0 += sy;
        }
    }
}

/**
 * @brief Draw a rectangle outline
 */
void SH1106_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SH1106_COLOR_t c) {
    if (x >= SH1106_WIDTH || y >= SH1106_HEIGHT) return;

    w = ((x + w) >= SH1106_WIDTH) ? (SH1106_WIDTH - x - 1) : w;
    h = ((y + h) >= SH1106_HEIGHT) ? (SH1106_HEIGHT - y - 1) : h;

    SH1106_DrawLine(x, y, x + w, y, c);
    SH1106_DrawLine(x, y + h, x + w, y + h, c);
    SH1106_DrawLine(x, y, x, y + h, c);
    SH1106_DrawLine(x + w, y, x + w, y + h, c);
}

/**
 * @brief Draw a filled rectangle
 */
void SH1106_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SH1106_COLOR_t c) {
    if (x >= SH1106_WIDTH || y >= SH1106_HEIGHT) return;

    w = ((x + w) >= SH1106_WIDTH) ? (SH1106_WIDTH - x) : w;
    h = ((y + h) >= SH1106_HEIGHT) ? (SH1106_HEIGHT - y) : h;

    for (uint16_t i = 0; i < h; i++) {
        SH1106_DrawLine(x, y + i, x + w - 1, y + i, c);
    }
}

/**
 * @brief Draw a triangle outline
 */
void SH1106_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                         uint16_t x3, uint16_t y3, SH1106_COLOR_t color) {
    SH1106_DrawLine(x1, y1, x2, y2, color);
    SH1106_DrawLine(x2, y2, x3, y3, color);
    SH1106_DrawLine(x3, y3, x1, y1, color);
}

/**
 * @brief Draw a filled triangle
 */
void SH1106_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                               uint16_t x3, uint16_t y3, SH1106_COLOR_t color) {
    int16_t deltax = ABS(x2 - x1);
    int16_t deltay = ABS(y2 - y1);
    int16_t x = x1, y = y1;
    int16_t xinc1, xinc2, yinc1, yinc2, den, num, numadd, numpixels;

    xinc1 = (x2 >= x1) ? 1 : -1;
    xinc2 = xinc1;
    yinc1 = (y2 >= y1) ? 1 : -1;
    yinc2 = yinc1;

    if (deltax >= deltay) {
        xinc1 = 0;
        yinc2 = 0;
        den = deltax;
        num = deltax / 2;
        numadd = deltay;
        numpixels = deltax;
    } else {
        xinc2 = 0;
        yinc1 = 0;
        den = deltay;
        num = deltay / 2;
        numadd = deltax;
        numpixels = deltay;
    }

    for (int16_t curpixel = 0; curpixel <= numpixels; curpixel++) {
        SH1106_DrawLine(x, y, x3, y3, color);
        num += numadd;
        if (num >= den) {
            num -= den;
            x += xinc1;
            y += yinc1;
        }
        x += xinc2;
        y += yinc2;
    }
}

/**
 * @brief Draw a circle outline using Midpoint Circle Algorithm
 */
void SH1106_DrawCircle(int16_t x0, int16_t y0, int16_t r, SH1106_COLOR_t c) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    SH1106_DrawPixel(x0, y0 + r, c);
    SH1106_DrawPixel(x0, y0 - r, c);
    SH1106_DrawPixel(x0 + r, y0, c);
    SH1106_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SH1106_DrawPixel(x0 + x, y0 + y, c);
        SH1106_DrawPixel(x0 - x, y0 + y, c);
        SH1106_DrawPixel(x0 + x, y0 - y, c);
        SH1106_DrawPixel(x0 - x, y0 - y, c);
        SH1106_DrawPixel(x0 + y, y0 + x, c);
        SH1106_DrawPixel(x0 - y, y0 + x, c);
        SH1106_DrawPixel(x0 + y, y0 - x, c);
        SH1106_DrawPixel(x0 - y, y0 - x, c);
    }
}

/**
 * @brief Draw a filled circle
 */
void SH1106_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SH1106_COLOR_t c) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    SH1106_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SH1106_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        SH1106_DrawLine(x0 - x, y0 - y, x0 + x, y0 - y, c);
        SH1106_DrawLine(x0 - y, y0 + x, x0 + y, y0 + x, c);
        SH1106_DrawLine(x0 - y, y0 - x, x0 + y, y0 - x, c);
    }
}

/**
 * @brief Draw a bitmap image
 * @param x X coordinate
 * @param y Y coordinate
 * @param bitmap Pointer to bitmap data
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color Color to draw
 */
void SH1106_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap,
                       int16_t w, int16_t h, uint16_t color) {
    if (!bitmap || w <= 0 || h <= 0) return;  // Validate input

    const int16_t byteWidth = (w + 7) / 8;    // Width in bytes

    for (int16_t row = 0; row < h; row++) {
        for (int16_t col = 0; col < w; col++) {
            // Calculate which byte and bit to check
            uint8_t byte = bitmap[row * byteWidth + col / 8];
            uint8_t bitMask = 0x80 >> (col % 8);

            if (byte & bitMask) {
                SH1106_DrawPixel(x + col, y + row, color);
            }
        }
    }
}

// Add this near your other SH1106 drawing functions or create it
void SH1106_Fill_Rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
    for (uint8_t i = 0; i < w; i++) {
        for (uint8_t j = 0; j < h; j++) {
            SH1106_DrawPixel(x + i, y + j, color);
        }
    }
}

/**
 * @brief Toggle display inversion
 */
void SH1106_ToggleInvert(void) {
    SH1106.Inverted = !SH1106.Inverted;
    for (uint16_t i = 0; i < sizeof(SH1106_Buffer); i++) {
        SH1106_Buffer[i] = ~SH1106_Buffer[i];
    }
}

/**
 * @brief Invert display colors (hardware level)
 * @param i 1 to invert, 0 for normal
 */
void SH1106_InvertDisplay(int i) {
    SH1106_WRITECOMMAND(i ? SH1106_INVERTDISPLAY : SH1106_NORMALDISPLAY);
}

/**
 * @brief Set display contrast
 * @param contrast Contrast value (0-255)
 */
void SH1106_SetContrast(uint8_t contrast) {
    SH1106_WRITECOMMAND(SH1106_SETCONTRAST);
    SH1106_WRITECOMMAND(contrast);
}

/**
 * @brief Turn display ON
 */
void SH1106_ON(void) {
    SH1106_WRITECOMMAND(0xAD);
    SH1106_WRITECOMMAND(0x8B);
    SH1106_WRITECOMMAND(SH1106_DISPLAYON);
    SH1106.DisplayOn = 1;
}

/**
 * @brief Turn display OFF
 */
void SH1106_OFF(void) {
    SH1106_WRITECOMMAND(0xAD);
    SH1106_WRITECOMMAND(0x8A);
    SH1106_WRITECOMMAND(SH1106_DISPLAYOFF);
    SH1106.DisplayOn = 0;
}

/**
 * @brief Get display initialization status
 * @return 1 if initialized, 0 otherwise
 */
uint8_t SH1106_IsInitialized(void) {
    return SH1106.Initialized;
}

/**
 * @brief Get display ON/OFF status
 * @return 1 if display is ON, 0 if OFF
 */
uint8_t SH1106_IsDisplayOn(void) {
    return SH1106.DisplayOn;
}

// ==================== Low-Level I2C Functions ====================

/**
 * @brief Write multiple bytes via I2C
 */
void SH1106_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
    if (!data || count == 0 || count > 255) return;

    uint8_t dt[256];
    dt[0] = reg;

    for (uint16_t i = 0; i < count; i++) {
        dt[i + 1] = data[i];
    }

    HAL_I2C_Master_Transmit(SH1106_I2C, address, dt, count + 1, 100);
}

/**
 * @brief Write a single byte via I2C
 */
void SH1106_I2C_Write(uint8_t address, uint8_t reg, uint8_t data) {
    uint8_t dt[2] = {reg, data};
    HAL_I2C_Master_Transmit(SH1106_I2C, address, dt, 2, 100);
}
