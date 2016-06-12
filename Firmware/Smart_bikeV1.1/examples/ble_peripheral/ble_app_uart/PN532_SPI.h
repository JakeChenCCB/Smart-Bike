
#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__
#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#if 0
#define DMSG(x) printf(x)
#define DMSG_HEX(x) printf("%c",x) 
#define DMSG_INT(x) printf("%d",x) 
#else
#define DMSG(x) 
#define DMSG_HEX(x)  
#define DMSG_INT(x) 
#endif
//#include "PN532Interface.h"
#define PN532_PREAMBLE                (0x00)
#define PN532_STARTCODE1              (0x00)
#define PN532_STARTCODE2              (0xFF)
#define PN532_POSTAMBLE               (0x00)

#define PN532_HOSTTOPN532             (0xD4)
#define PN532_PN532TOHOST             (0xD5)

#define PN532_ACK_WAIT_TIME           (10)  // ms, timeout of waiting for ACK

#define PN532_INVALID_ACK             (-1)
#define PN532_TIMEOUT                 (-2)
#define PN532_INVALID_FRAME           (-3)
#define PN532_NO_SPACE                (-4)

#define REVERSE_BITS_ORDER(b)         b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; \
                                      b = (b & 0xCC) >> 2 | (b & 0x33) << 2; \
                                      b = (b & 0xAA) >> 1 | (b & 0x55) << 1


#define NXP_PORT					GPIOB
#define NXP_SCK_PIN				GPIO_Pin_13
#define NXP_MISO_PIN			GPIO_Pin_14
#define NXP_MOSI_PIN			GPIO_Pin_15
#define NXP_SPI	          SPI2

//#define CS2_PORT				GPIOE
#define CS2_PIN					5


//#define CS1_PORT				GPIOB
#define CS1_PIN					5

#define SCK_PIN	2
#define MISO_PIN	3
#define MOSI_PIN	4
#define SCK_L	nrf_gpio_pin_clear(SCK_PIN)
#define SCK_H	nrf_gpio_pin_set(SCK_PIN)
#define MOSI_L	nrf_gpio_pin_clear(MOSI_PIN)
#define MOSI_H	nrf_gpio_pin_set(MOSI_PIN)
#define MISO_R nrf_gpio_pin_read(MISO_PIN)

#define u8	uint8_t
#define u32	uint32_t
#define SPI_INSTANCE  1
//uint8_t spi=0;
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static volatile bool spi_xfer_done=1;
//#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
//static uint8_t       m_rx_buf[sizeof(TEST_STRING)+1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

extern uint8_t spi2_CS_swich;

	void spi2_init(void);
	void  wakeup(void);
	
	int8_t writeCommand(const uint8_t *header, uint8_t hlen);
	int16_t readResponse(uint8_t buf[], uint8_t len);
	uint8_t isReady(void);
	void writeFrame(const uint8_t *header, uint8_t hlen);
	int8_t readAckFrame(void);
	void delay_ms(uint32_t time_value);
	uint32_t Time_get(void);
	void spi2_uninit(void);
	void spi2_enable(void);
	void spi2_disable(void);

#endif
