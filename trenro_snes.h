#ifndef __TRENRO_SNES_H
#define __TRENRO_SNES_H

#include <stdint.h>
#include <stdbool.h>

// The masks of the individual buttons as read from the gamepad.
typedef enum
{
    TRENRO_SNES_BUTTON_B           = 0x0001,
    TRENRO_SNES_BUTTON_Y           = 0x0002,
    TRENRO_SNES_BUTTON_SEL         = 0x0004,
    TRENRO_SNES_BUTTON_START       = 0x0008,
    TRENRO_SNES_BUTTON_UP_ARROW    = 0x0010,
    TRENRO_SNES_BUTTON_DOWN_ARROW  = 0x0020,
    TRENRO_SNES_BUTTON_LEFT_ARROW  = 0x0040,
    TRENRO_SNES_BUTTON_RIGHT_ARROW = 0x0080,
    TRENRO_SNES_BUTTON_A           = 0x0100,
    TRENRO_SNES_BUTTON_X           = 0x0200,
    TRENRO_SNES_BUTTON_L           = 0x0400,
    TRENRO_SNES_BUTTON_R           = 0x0800
} trenro_snes_button_t;

/**
 * Inits the button GPIOs.
 */
void trenro_snes_init(uint8_t latch_pin, uint8_t clock_pin, uint8_t data_pin);

/**
 * Returns false if snes_init hasn't been called.
 */
bool trenro_snes_read(uint16_t * button_mask);

/**
 * Returns true if the specified button is in the mask.
 */
bool trenro_snes_button_pressed(uint16_t button_mask, trenro_snes_button_t button);

#endif
