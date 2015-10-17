#ifndef __GAMEPAD_CTL_H
#define __GAMEPAD_CTL_H

#include "trenro_snes.h"

/**
 * Provides a simple interface for controlling the Syma 218 quadcopter with
 * a Trenro SNES gamepad.
 */

#define GAMEPAD_BUTTON_ROLL_LEFT      (TRENRO_SNES_BUTTON_LEFT_ARROW)
#define GAMEPAD_BUTTON_ROLL_RIGHT     (TRENRO_SNES_BUTTON_RIGHT_ARROW)
#define GAMEPAD_BUTTON_PITCH_FWD      (TRENRO_SNES_BUTTON_UP_ARROW)
#define GAMEPAD_BUTTON_PITCH_BWD      (TRENRO_SNES_BUTTON_DOWN_ARROW)
#define GAMEPAD_BUTTON_YAW_LEFT       (TRENRO_SNES_BUTTON_L)
#define GAMEPAD_BUTTON_YAW_RIGHT      (TRENRO_SNES_BUTTON_R)
#define GAMEPAD_BUTTON_THROTTLE_UP    (TRENRO_SNES_BUTTON_A)
#define GAMEPAD_BUTTON_THROTTLE_DOWN  (TRENRO_SNES_BUTTON_B)
#define GAMEPAD_BUTTON_FLIP           (TRENRO_SNES_BUTTON_X)
#define GAMEPAD_BUTTON_BIND           (TRENRO_SNES_BUTTON_START)

#define GAMEPAD_THROTTLE_BUTTON_DELTA (5UL)
#define GAMEPAD_CTL_BUTTON_DELTA      (50UL)

/**
 * Inits the gamepad interface for the Syma library.
 */
void gamepad_ctl_init(uint8_t latch_pin, uint8_t clock_pin, uint8_t data_pin);

/**
 * Prompts the gamepad to control the quadcopter.
 */
bool gamepad_ctl_update(void);

/**
 * Prompts the gamepad to relinquish control of the quadcopter.
 */
bool gamepad_ctl_disable(void);

#endif
