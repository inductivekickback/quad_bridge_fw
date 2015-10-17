#ifndef SYMA_218_BKT_H
#define SYMA_218_BKT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * The Syma X4 (218 BKT) quadcopter is flown with five inputs:
 *
 *     NAME:     RANGE:       DESCRIPTION:
 *     ------------------------------------------------------------------------
 *     THROTTLE: [0,    255]  The base speed of the motors: affects the quad's
 *               [OFF,  FULL] vertical position and movement speed.
 *
 *     YAW:      [-127, 127]  Rotates the quad left and right (about the axis
 *               [RIGHT,LEFT] that is perpindicular to the ground).
 *
 *     PITCH:    [-127, 127]  Causes the quad to move forward and backward by
 *               [BACK, FORW] orienting its 'nose' up or down.
 *
 *     ROLL:     [-127, 127]  Causes the quad to move left and right by
 *               [RIGHT,LEFT] dipping its left or right side.
 *
 *     FLIP:     [0,    1]    Commands the quad's onboard computer to perform
 *                            a flip along the pitch or roll axis. Also
 *                            expects one of the PITCH or ROLL values to have
 *                            maximum magnitude so it knows which way to flip.
 */

#define SYMA_UPDATE_INTERVAL_US           (4000UL)

#define SYMA_MIN_THROTTLE_VALUE           (0)
#define SYMA_MAX_THROTTLE_VALUE           (255)

#define SYMA_MIN_PITCH_ROLL_YAW_VALUE     (-127)
#define SYMA_MAX_PITCH_ROLL_YAW_VALUE     (127)
#define SYMA_NEUTRAL_PITCH_ROLL_YAW_VALUE (0)

#define SYMA_MIN_TRIM_VALUE               (-31)
#define SYMA_MAX_TRIM_VALUE               (31)

typedef enum
{
    SYMA_FLIP_FORWARD,
    SYMA_FLIP_BACKWARD,
    SYMA_FLIP_LEFT,
    SYMA_FLIP_RIGHT,
    SYMA_FLIP_COUNT
} syma_flip_t;

typedef enum
{
    SYMA_SPEED_LOW,
    SYMA_SPEED_HIGH,
    SYMA_SPEED_COUNT
} syma_speed_t;

/**
 * Resets the internal state of the library. Needs to be called before
 * any other functions are called.
 */
void syma_init(void);

/**
 * Initiates the binding sequence. Note that there is no feedback from the
 * quad so the binding sequence should only be started when it has a reasonable
 * chance of succeeding.
 */
void syma_bind(void);

/**
 * Allows the library to update the transmition parameters at the beginning
 * of each timeslot.
 */
void syma_update(uint8_t * rf_channel,
		 uint8_t * tx_addr,
		 uint8_t * tx_addr_len,
		 uint8_t * data,
		 uint8_t * data_len);

/**
 * Notifies the library that a timeslot was blocked or cancelled. This is
 * necessary to keep the transmitter and quad in sync.
 */
void syma_update_missed(void);

/**
 * Returns true if the binding process is complete.
 */
bool syma_is_bound(void);

/**
 * The get/set functions for the control variables return false if the
 * quad has not finished the binding sequence.
 */
bool syma_throttle_set(uint8_t val);
bool syma_yaw_set(int8_t val);
bool syma_pitch_set(int8_t val);
bool syma_roll_set(int8_t val);
bool syma_yaw_trim_set(int8_t val);
bool syma_pitch_trim_set(int8_t val);
bool syma_roll_trim_set(int8_t val);

bool syma_throttle_get(uint8_t * val);
bool syma_yaw_get(int8_t * val);
bool syma_pitch_get(int8_t * val);
bool syma_roll_get(int8_t * val);
bool syma_yaw_trim_get(int8_t * val);
bool syma_pitch_trim_get(int8_t * val);
bool syma_roll_trim_get(int8_t * val);

/**
 * Returns false if the quad has not finished the binding sequence or if the
 * throttle is set to zero. The current pitch or roll value will be saved
 * and restored after the flip is finished.
 */
bool syma_flip(syma_flip_t val);

/**
 * Returns false if the specified val is invalid.
 */
bool syma_speed_set(syma_speed_t val);

#endif
