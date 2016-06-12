#include "key_driver.h"
#define COLUMN1 17
#define COLUMN2 12
#define COLUMN3 24

void button_gpiote_init(void)
{
  nrf_gpio_cfg_input(COLUMN1, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(COLUMN1, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(COLUMN1, NRF_GPIO_PIN_PULLDOWN);
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN2_Enabled << GPIOTE_INTENSET_IN2_Pos);
		nrf_gpiote_event_config(2, COLUMN1, NRF_GPIOTE_POLARITY_LOTOHI);
	    NVIC_EnableIRQ(GPIOTE_IRQn);
    __enable_irq();	
}