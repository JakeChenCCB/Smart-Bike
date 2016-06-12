#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
//#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"
#include "nrf_gpio.h"

void pwm_driver_init(void);
void servo_right(void);
void servo_left(void);
