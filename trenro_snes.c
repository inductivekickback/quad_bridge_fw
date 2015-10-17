#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "trenro_snes.h"

#define SNES_WAIT_US    (5UL)

static bool m_init = false;

static uint8_t m_latch_pin;
static uint8_t m_clock_pin;
static uint8_t m_data_pin;


void trenro_snes_init(uint8_t latch_pin, uint8_t clock_pin, uint8_t data_pin)
{
    m_init = true;

    m_latch_pin = latch_pin;
    m_clock_pin = clock_pin;
    m_data_pin  = data_pin;

    nrf_gpio_cfg_output(latch_pin);
    nrf_gpio_cfg_output(clock_pin);
    nrf_gpio_cfg_input(data_pin, NRF_GPIO_PIN_NOPULL);

    nrf_gpio_pin_clear(latch_pin);
    nrf_gpio_pin_clear(clock_pin);
}


bool trenro_snes_read(uint16_t * button_mask)
{
    uint32_t mask;

    *button_mask = 0;

    nrf_gpio_pin_set(m_latch_pin);
    nrf_delay_us(SNES_WAIT_US);
    nrf_gpio_pin_clear(m_latch_pin);

    nrf_delay_us(SNES_WAIT_US);

    for (mask = 0x0001; mask < 0x10000; mask <<= 1)
    {
	nrf_gpio_pin_clear(m_clock_pin);
	nrf_delay_us(SNES_WAIT_US);

	// Data is active low.
	if (0 == nrf_gpio_pin_read(m_data_pin))
	{
	    *button_mask |= mask;
	}

	nrf_gpio_pin_set(m_clock_pin);
	nrf_delay_ms(SNES_WAIT_US);
    }

    return m_init;
}


__inline bool trenro_snes_button_pressed(uint16_t button_mask, trenro_snes_button_t button)
{
    return (button_mask & button);
}
