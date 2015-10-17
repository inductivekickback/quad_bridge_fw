#include "gamepad_ctl.h"
#include "syma_218_bkt.h"

static bool    m_init = false;
static uint8_t m_throttle;
static int8_t  m_yaw;
static int8_t  m_pitch;
static int8_t  m_roll;


static void bind(void)
{
    syma_init();
    syma_bind();

    m_throttle = SYMA_MIN_THROTTLE_VALUE;
    m_yaw      = SYMA_NEUTRAL_PITCH_ROLL_YAW_VALUE;
    m_pitch    = SYMA_NEUTRAL_PITCH_ROLL_YAW_VALUE;
    m_roll     = SYMA_NEUTRAL_PITCH_ROLL_YAW_VALUE;
}


static int32_t increment(int32_t value, uint32_t increment, int32_t max_value)
{
    if ((max_value - increment) <= value)
    {
	return max_value;
    }
    else
    {
	return (value + increment);
    }
}


static int32_t decrement(int32_t value, uint32_t decrement, int32_t min_value)
{
    if ((min_value + decrement) >= value)
    {
	return min_value;
    }
    else
    {
	return (value - decrement);
    }
}


static bool update(uint16_t button_mask)
{
    bool x;
    bool y;

    if (trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_BIND))
    {
	bind();
	return true;
    }

    if (!syma_is_bound())
    {
	return true;
    }

    // Set the pitch.
    x = trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_PITCH_FWD);
    y = trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_PITCH_BWD);
    if (x^y)
    {
        if (x)
	{
            m_pitch = increment(m_pitch,
				    GAMEPAD_CTL_BUTTON_DELTA,
				    SYMA_MAX_PITCH_ROLL_YAW_VALUE);
	}
	else
	{
	    m_pitch = decrement(m_pitch,
				    GAMEPAD_CTL_BUTTON_DELTA,
				    SYMA_MIN_PITCH_ROLL_YAW_VALUE);
	}
    }
    else
    {
	m_pitch = SYMA_NEUTRAL_PITCH_ROLL_YAW_VALUE;
    }
    if (!syma_pitch_set(m_pitch))
    {
	return false;
    }

    // Set the roll.
    x = trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_ROLL_LEFT);
    y = trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_ROLL_RIGHT);
    if (x^y)
    {
	if (x)
	{
	    m_roll = increment(m_roll,
				   GAMEPAD_CTL_BUTTON_DELTA,
				   SYMA_MAX_PITCH_ROLL_YAW_VALUE);
	}
	else
	{
	    m_roll = decrement(m_roll,
				   GAMEPAD_CTL_BUTTON_DELTA,
				   SYMA_MIN_PITCH_ROLL_YAW_VALUE);
	}
    }
    else
    {
	m_roll = SYMA_NEUTRAL_PITCH_ROLL_YAW_VALUE;
    }
    if (!syma_roll_set(m_roll))
    {
	return false;
    }

    // Set the yaw.
    x = trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_YAW_LEFT);
    y = trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_YAW_RIGHT);
    if (x^y)
    {
	if (x)
	{
	    m_yaw = increment(m_yaw,
				  GAMEPAD_CTL_BUTTON_DELTA,
				  SYMA_MAX_PITCH_ROLL_YAW_VALUE);
	}
	else
	{
	    m_yaw = decrement(m_yaw,
				  GAMEPAD_CTL_BUTTON_DELTA,
				  SYMA_MIN_PITCH_ROLL_YAW_VALUE);
	}
    }
    else
    {
	m_yaw = SYMA_NEUTRAL_PITCH_ROLL_YAW_VALUE;
    }
    if (!syma_yaw_set(m_yaw))
    {
	return false;
    }

    // Set the throttle.
    x = trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_THROTTLE_UP);
    y = trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_THROTTLE_DOWN);
    if (x^y)
    {
	if (x)
	{
	    m_throttle = increment(m_throttle,
				       GAMEPAD_THROTTLE_BUTTON_DELTA,
				       SYMA_MAX_THROTTLE_VALUE);
	}
	else
	{
	    m_throttle = decrement(m_throttle,
				       GAMEPAD_THROTTLE_BUTTON_DELTA,
				       SYMA_MIN_THROTTLE_VALUE);
	}
    }
    if (!syma_throttle_set(m_throttle))
    {
	return false;
    }

    // Set flip.
    if (trenro_snes_button_pressed(button_mask, GAMEPAD_BUTTON_FLIP))
    {
	// The flip will fail if the throttle is set to the minimum.
	if (SYMA_MIN_THROTTLE_VALUE != m_throttle)
	{
	    if (!syma_flip(SYMA_FLIP_BACKWARD))
	    {
		return false;
	    }
	}
    }

    return true;
}


void gamepad_ctl_init(uint8_t latch_pin, uint8_t clock_pin, uint8_t data_pin)
{
    m_init = true;
    trenro_snes_init(latch_pin, clock_pin, data_pin);

    m_throttle = SYMA_MIN_THROTTLE_VALUE;
}


bool gamepad_ctl_update(void)
{
    uint16_t button_mask;

    if (!m_init)
    {
	return false;
    }

    if (trenro_snes_read(&button_mask))
    {
	return update(button_mask);
    }
    else
    {
	return false;
    }
}


bool gamepad_ctl_disable(void)
{
    if (!m_init)
    {
	return false;
    }

    if (syma_is_bound())
    {
	syma_throttle_set(SYMA_MIN_THROTTLE_VALUE);
    }

    return true;
}
