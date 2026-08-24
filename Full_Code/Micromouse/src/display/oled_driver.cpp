/**
 * @file oled_driver.cpp
 * @brief OLED driver implementation.
 * @see oled_driver.h
 */

#include "oled_driver.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

static Adafruit_SSD1306 _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool oled_init(void) {
    if (!_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        return false;
    }
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SSD1306_WHITE);
    _display.display();
    return true;
}

void oled_clear(void) {
    _display.clearDisplay();
}

void oled_update(void) {
    _display.display();
}

void oled_print(uint8_t x, uint8_t y, const char* str) {
    _display.setCursor(x, y);
    _display.print(str);
}

void oled_draw_maze(void) {
    /** TODO: Implementation */
}
