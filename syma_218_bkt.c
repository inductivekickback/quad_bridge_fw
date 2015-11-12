/**
 * The control packets have the following payload:
 *
 * 0: THROTTLE  [0    (off),   0xFF (full)]
 * 1: PITCH     [0xFF (back),  0x7F (forward)]
 * 2: YAW       [0xFF (right), 0x7F (left)]
 * 3: ROLL      [0xFF (right), 0x7F (left)]
 * 4: 0x00
 * 5: SPEED MODE (bits 7 and 6) LOW: 01b, HIGH: 11b (NOTE: HIGH SPEED MODE NOT TESTED)
 *    PITCH TRIM [0x3F, 0x1F]
 *              Up on right D-PAD increments to 0x1F
 *              Down on right D-PAD decrements to 0x3F
 * 6: FLIP (bit 6) Right bumper sets flag. Normally zero
 *    YAW TRIM [0x3F, 0x1F]
 *              Left on left D-PAD increments to 0x1F
 *              Right decrements until 0x3F
 * 7: ROLL TRIM [0x3F, 0x1F]
 *              Left on right D-PAD increment to 0x1F
 *              Right decrements until 0x3F.
 * 8: 0x00
 * 9: CHECKSUM
 *
 * Left bumper sets speed: 1 beep is low and 2 beeps is high.
 *
 * NOTE: MODE 2 (power button and right on right D-PAD at the same time)
 *       is not used. It swaps the roll and yaw controls on the transmitter.
 */

#include <string.h>
#include "syma_218_bkt.h"

typedef enum
{
    UNITIALIZED_STATE,
    UNBOUND_STATE,
    TRANSMITTER_ADDR_BROADCAST_STATE,
    THROTTLE_TOGGLE_STATE, // Required to complete the binding process.
    BOUND_STATE,
    STATE_COUNT
} state_t;

#define SIX_BIT_MASK               (0x1FUL) // 6bit values are used for trim values.
#define SIX_BIT_NEG_FLAG           (0x2UL)
#define EIGHT_BIT_MASK             (0x7FUL) // 8bit values are used for yaw, pitch, and roll.
#define EIGHT_BIT_NEG_FLAG         (0x80UL)

#define SPEED_BIT_MASK             (0x80UL)

#define FLIP_BIT_MASK              (0x40UL)
#define NUM_FLIP_PACKETS           (0x0AUL)

#define NUM_ADDR_BROADCAST_PACKETS (30UL)
#define NUM_TOGGLE_PACKETS         (90UL)

#define THROTTLE_TOGGLE_VALUE      (0xF6UL)

#define LOW_SPEED_VALUE            (0x40UL)
#define HIGH_SPEED_VALUE           (0xC0UL)

#define CTL_PACKET_LEN             (10UL)
#define THROTTLE_INDEX             (0UL)
#define PITCH_INDEX                (1UL)
#define YAW_INDEX                  (2UL)
#define ROLL_INDEX                 (3UL)
#define RESERVED_1_INDEX           (4UL)
#define PITCH_TRIM_SPEED_INDEX     (5UL)
#define YAW_TRIM_FLIP_INDEX        (6UL)
#define ROLL_TRIM_INDEX            (7UL)
#define RESERVED_2_INDEX           (8UL)
#define CHECKSUM_INDEX             (9UL)

#define ADDR_LEN                   (5UL)

#define RESERVED_DATA_VALUE        (0x00UL)

#define PACKETS_PER_CHANNEL        (2UL)

static const uint8_t BROADCAST_ADDR[ADDR_LEN] = {0xAB,0xAC,0xAD,0xAE,0xAF};
static const uint8_t CTL_ADDR[ADDR_LEN]       = {0x1C,0xDF,0x02,0x00,0xA2};

static const uint8_t BIND_PACKET[CTL_PACKET_LEN] = {0xA2,0x00,0x02,0xDF,0x1C,
						    0xAA,0xAA,0xAA,0x00,0x1E};

static const uint8_t BINDING_RF_CHANNELS[] = {0x4B,0x30,0x40,0x09};
static const uint8_t CTL_RF_CHANNELS[]     = {0x1E,0x3E,0x16,0x36};

static state_t  m_state;
static uint32_t m_state_count;

static uint8_t m_throttle;
static uint8_t m_pitch;
static uint8_t m_yaw;
static uint8_t m_roll;
static uint8_t m_yaw_trim_flip;
static uint8_t m_pitch_trim_speed;
static uint8_t m_roll_trim;

static uint8_t     m_flip_saved_value;
static uint8_t     m_flip_count;
static syma_flip_t m_flip_type;


/**
 * Calculates the checksum that is used in addition to the radio's checksum.
 */
static uint8_t checksum_calc(uint8_t * data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint32_t i=0; i < len; i++)
    {
        sum ^= data[i];
    }
    return (sum + 0x55);
}


static void ctl_packet_update(uint8_t * rf_channel,
                               uint8_t * tx_addr,
                               uint8_t * tx_addr_len,
                               uint8_t * data,
                               uint8_t * data_len)
{
    // Each channel is used twice.
    *rf_channel  = CTL_RF_CHANNELS[m_state_count / 2];

    memcpy(tx_addr, CTL_ADDR, sizeof(CTL_ADDR));

    *tx_addr_len = sizeof(CTL_ADDR);
    *data_len    = CTL_PACKET_LEN;

    data[THROTTLE_INDEX]         = m_throttle;
    data[PITCH_INDEX]            = m_pitch;
    data[YAW_INDEX]              = m_yaw;
    data[ROLL_INDEX]             = m_roll;
    data[RESERVED_1_INDEX]       = RESERVED_DATA_VALUE;
    data[PITCH_TRIM_SPEED_INDEX] = m_pitch_trim_speed;
    data[YAW_TRIM_FLIP_INDEX]    = m_yaw_trim_flip;
    data[ROLL_TRIM_INDEX]        = m_roll_trim;
    data[RESERVED_2_INDEX]       = RESERVED_DATA_VALUE;
    data[CHECKSUM_INDEX]         = checksum_calc(data, (CTL_PACKET_LEN - 1));

    m_state_count++;
    if ((sizeof(CTL_RF_CHANNELS) * PACKETS_PER_CHANNEL) == m_state_count)
    {
	m_state_count = 0;
    }

    if (m_flip_count)
    {
	m_flip_count--;
	if (0 == m_flip_count)
	{
	    m_yaw_trim_flip &= ~FLIP_BIT_MASK;
	    switch(m_flip_type)
	    {
	    case SYMA_FLIP_FORWARD:
	    case SYMA_FLIP_BACKWARD:
		m_pitch = m_flip_saved_value;
		break;
	    case SYMA_FLIP_LEFT:
	    case SYMA_FLIP_RIGHT:
		m_roll = m_flip_saved_value;
		break;
	    default:
		break;
	    }
	}
    }
}


static void throttle_toggle_update(uint8_t * rf_channel,
				   uint8_t * tx_addr,
				   uint8_t * tx_addr_len,
				   uint8_t * data,
				   uint8_t * data_len)
{
    uint8_t i = (m_state_count % (sizeof(CTL_RF_CHANNELS) * PACKETS_PER_CHANNEL));

    *rf_channel = CTL_RF_CHANNELS[i / 2];

    memcpy(tx_addr, CTL_ADDR, sizeof(CTL_ADDR));

    *tx_addr_len = sizeof(CTL_ADDR);
    *data_len    = CTL_PACKET_LEN;

    m_state_count++;

    if (m_state_count < NUM_TOGGLE_PACKETS)
    {
	data[THROTTLE_INDEX] = SYMA_MIN_THROTTLE_VALUE;
    }
    else if (m_state_count < (2 * NUM_TOGGLE_PACKETS))
    {
	data[THROTTLE_INDEX] = THROTTLE_TOGGLE_VALUE;
    }
    else  if (m_state_count < (3 * NUM_TOGGLE_PACKETS))
    {
	data[THROTTLE_INDEX] = SYMA_MIN_THROTTLE_VALUE;
    }
    else
    {
	data[THROTTLE_INDEX] = SYMA_MIN_THROTTLE_VALUE;
	m_state              = BOUND_STATE;
	m_state_count        = (i % (sizeof(CTL_RF_CHANNELS) * PACKETS_PER_CHANNEL));
    }

    data[PITCH_INDEX]            = RESERVED_DATA_VALUE;
    data[YAW_INDEX]              = RESERVED_DATA_VALUE;
    data[ROLL_INDEX]             = RESERVED_DATA_VALUE;
    data[RESERVED_1_INDEX]       = RESERVED_DATA_VALUE;
    data[PITCH_TRIM_SPEED_INDEX] = LOW_SPEED_VALUE;
    data[YAW_TRIM_FLIP_INDEX]    = RESERVED_DATA_VALUE;
    data[ROLL_TRIM_INDEX]        = RESERVED_DATA_VALUE;
    data[RESERVED_2_INDEX]       = RESERVED_DATA_VALUE;
    data[CHECKSUM_INDEX]         = checksum_calc(data, (CTL_PACKET_LEN - 1));
}


static void bind_packet_update(uint8_t * rf_channel,
			       uint8_t * tx_addr,
			       uint8_t * tx_addr_len,
			       uint8_t * data,
			       uint8_t * data_len)
{
    if (m_state_count < NUM_ADDR_BROADCAST_PACKETS)
    {
	uint8_t i = (((m_state_count - 1) / 2) % sizeof(BINDING_RF_CHANNELS));
	*rf_channel = BINDING_RF_CHANNELS[i];

	memcpy(tx_addr, BROADCAST_ADDR, sizeof(BROADCAST_ADDR));
	*tx_addr_len = sizeof(BROADCAST_ADDR);

	memcpy(data, BIND_PACKET, sizeof(BIND_PACKET));
	*data_len = sizeof(BIND_PACKET);

	m_state_count++;
    }
    else
    {
	m_state       = THROTTLE_TOGGLE_STATE;
	m_state_count = 0;
	throttle_toggle_update(rf_channel,
			       tx_addr,
			       tx_addr_len,
			       data,
			       data_len);
    }
}


static void seven_bit_value_set(uint8_t * var, int8_t val)
{
    if (0 > val)
    {
	*var = ((-1 * val) & EIGHT_BIT_MASK);
	*var |= EIGHT_BIT_NEG_FLAG;
    }
    else
    {
	*var = (val & EIGHT_BIT_MASK);
    }
}


static int8_t seven_bit_value_get(uint8_t var)
{
    if (var & EIGHT_BIT_NEG_FLAG)
    {
	int8_t result = (var & EIGHT_BIT_MASK);
	return (-1 * result);
    }
    else
    {
	return (int8_t)var;
    }
}


static void five_bit_value_set(uint8_t * var, int8_t val)
{
    if (0 > val)
    {
	*var = ((-1 * val) & SIX_BIT_MASK);
	*var |= SIX_BIT_NEG_FLAG;
    }
    else
    {
	*var = (val & SIX_BIT_MASK);
    }
}


static int8_t five_bit_value_get(uint8_t var)
{
    if (var & SIX_BIT_NEG_FLAG)
    {
	int8_t result = (var & SIX_BIT_MASK);
	return (-1 * result);
    }
    else
    {
	return (int8_t)var;
    }
}


bool syma_speed_set(syma_speed_t val)
{
    switch(val)
    {
    case SYMA_SPEED_LOW:
	m_pitch_trim_speed &= ~SPEED_BIT_MASK;
	return true;
    case SYMA_SPEED_HIGH:
	m_pitch_trim_speed |= SPEED_BIT_MASK;
	return true;
    default:
	return false;
    }
}


bool syma_flip(syma_flip_t val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }

    if (m_flip_count)
    {
	// A flip is already in process.
	return true;
    }

    switch(val)
    {
    case SYMA_FLIP_FORWARD:
        m_flip_saved_value = m_pitch;
	seven_bit_value_set(&m_pitch, SYMA_MAX_PITCH_ROLL_YAW_VALUE);
	break;
    case SYMA_FLIP_BACKWARD:
	m_flip_saved_value = m_pitch;
	seven_bit_value_set(&m_pitch, SYMA_MIN_PITCH_ROLL_YAW_VALUE);
	break;
    case SYMA_FLIP_LEFT:
	m_flip_saved_value = m_roll;
	seven_bit_value_set(&m_roll, SYMA_MAX_PITCH_ROLL_YAW_VALUE);
	break;
    case SYMA_FLIP_RIGHT:
	m_flip_saved_value = m_roll;
	seven_bit_value_set(&m_roll, SYMA_MIN_PITCH_ROLL_YAW_VALUE);
	break;
    default:
	return false;
    }

    m_yaw_trim_flip |= FLIP_BIT_MASK;
    m_flip_count     = NUM_FLIP_PACKETS;
    m_flip_type      = val;
    return true;
}


bool syma_roll_trim_get(int8_t * val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    *val = five_bit_value_get(m_roll_trim);
    return true;
}


bool syma_pitch_trim_get(int8_t * val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    *val = five_bit_value_get(m_pitch_trim_speed);
    return true;
}


bool syma_yaw_trim_get(int8_t * val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    *val = five_bit_value_get(m_yaw_trim_flip);
    return true;
}


bool syma_roll_get(int8_t * val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }

    if (0 == m_flip_count)
    {
	*val = seven_bit_value_get(m_roll);
    }
    else
    {
        switch(m_flip_type)
	{
	case SYMA_FLIP_LEFT:
	case SYMA_FLIP_RIGHT:
	    *val = seven_bit_value_get(m_flip_saved_value);
	    break;
	default:
	    *val = seven_bit_value_get(m_roll);
	    break;
	}
    }
    return true;
}


bool syma_pitch_get(int8_t * val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }

    if (0 == m_flip_count)
    {
	*val = seven_bit_value_get(m_pitch);
    }
    else
    {
        switch(m_flip_type)
	{
	case SYMA_FLIP_FORWARD:
	case SYMA_FLIP_BACKWARD:
	    *val = seven_bit_value_get(m_flip_saved_value);
	    break;
	default:
	    *val = seven_bit_value_get(m_pitch);
	    break;
	}
    }
    return true;
}


bool syma_yaw_get(int8_t * val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    *val = seven_bit_value_get(m_yaw);
    return true;
}


bool syma_throttle_get(uint8_t * val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    *val = m_throttle;
    return true;
}


bool syma_roll_trim_set(int8_t val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    five_bit_value_set(&m_roll_trim, val);
    return true;
}


bool syma_pitch_trim_set(int8_t val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    five_bit_value_set(&m_pitch_trim_speed, val);
    return true;
}


bool syma_yaw_trim_set(int8_t val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    five_bit_value_set(&m_yaw_trim_flip, val);
    return true;
}


bool syma_roll_set(int8_t val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }

    if (0 == m_flip_count)
    {
	seven_bit_value_set(&m_roll, val);
    }
    else
    {
        switch(m_flip_type)
	{
	case SYMA_FLIP_LEFT:
	case SYMA_FLIP_RIGHT:
	    seven_bit_value_set(&m_flip_saved_value, val);
	    break;
	default:
	    seven_bit_value_set(&m_roll, val);
	    break;
	}
    }
    return true;
}


bool syma_pitch_set(int8_t val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }

    if (0 == m_flip_count)
    {
	seven_bit_value_set(&m_pitch, val);
    }
    else
    {
        switch(m_flip_type)
	{
	case SYMA_FLIP_FORWARD:
	case SYMA_FLIP_BACKWARD:
	    seven_bit_value_set(&m_flip_saved_value, val);
	    break;
	default:
	    seven_bit_value_set(&m_pitch, val);
	    break;
	}
    }
    return true;
}


bool syma_yaw_set(int8_t val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    seven_bit_value_set(&m_yaw, val);
    return true;
}


bool syma_throttle_set(uint8_t val)
{
    if (BOUND_STATE != m_state)
    {
	return false;
    }
    m_throttle = val;
    return true;
}


bool syma_is_bound(void)
{
    return (BOUND_STATE == m_state);
}


void syma_update_missed(void)
{
    uint8_t dummy_buffer[CTL_PACKET_LEN];

    // Continue moving through the channels normally.
    syma_update(&dummy_buffer[0],
		dummy_buffer,
		&dummy_buffer[0],
		dummy_buffer,
		&dummy_buffer[0]);
}


void syma_update(uint8_t * rf_channel,
		 uint8_t * tx_addr,
		 uint8_t * tx_addr_len,
		 uint8_t * data,
		 uint8_t * data_len)
{
    switch (m_state)
    {
    case TRANSMITTER_ADDR_BROADCAST_STATE:
	bind_packet_update(rf_channel,
			   tx_addr,
			   tx_addr_len,
			   data,
			   data_len);
	break;
    case THROTTLE_TOGGLE_STATE:
	throttle_toggle_update(rf_channel,
			       tx_addr,
			       tx_addr_len,
			       data,
			       data_len);
	break;
    case BOUND_STATE:
	ctl_packet_update(rf_channel,
			   tx_addr,
			   tx_addr_len,
			   data,
			   data_len);
	break;
    case UNITIALIZED_STATE:
    case UNBOUND_STATE:
    default:
	break;
    }
}


void syma_bind(void)
{
    // TODO: Allow for partial addr override?
    // TODO: Generate a unique addr?
    // TODO: If addr is changed then update bind_packet.
    m_state       = TRANSMITTER_ADDR_BROADCAST_STATE;
    m_state_count = 0;
}


void syma_init(void)
{
    m_state               = UNBOUND_STATE;
    m_state_count         = 0;
    m_throttle            = 0;
    m_pitch               = 0;
    m_yaw                 = 0;
    m_roll                = 0;
    m_yaw_trim_flip       = 0;
    m_pitch_trim_speed    = LOW_SPEED_VALUE;
    m_roll_trim           = 0;
    m_flip_count          = 0;
}
