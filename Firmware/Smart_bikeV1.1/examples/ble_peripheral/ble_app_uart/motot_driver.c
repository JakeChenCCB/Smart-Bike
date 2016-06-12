#include "motot_driver.h"
APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}
void pwm_driver_init(void)
{
    ret_code_t err_code;
    
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(20000L, 9);//APP_PWM_DEFAULT_CONFIG_2CH(10000L, 0, 1);
    
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    
    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
	nrf_gpio_pin_clear(9);
}
void servo_right()
{
	nrf_gpio_pin_clear(8);
	nrf_gpio_pin_clear(9);
	app_pwm_enable(&PWM1);
	app_pwm_channel_duty_set(&PWM1, 0, 90.5);
	nrf_delay_ms(100);
	app_pwm_disable(&PWM1);
	nrf_gpio_pin_clear(9);
	
	
}
void servo_left()
{
	nrf_gpio_pin_clear(8);
	nrf_gpio_cfg_output(9);
	nrf_gpio_pin_clear(9);
	app_pwm_enable(&PWM1);
	app_pwm_channel_duty_set(&PWM1, 0, 92.5);
	nrf_delay_ms(100);
	app_pwm_disable(&PWM1);
		nrf_gpio_pin_clear(9);
	nrf_gpio_pin_set(8);
	
}
