
#include "PN532_SPI.h"

#define STATUS_READ     2
#define DATA_WRITE      1
#define DATA_READ       3


static uint8_t command;
		
uint8_t write_flag=0;
uint8_t read_flag=0;
#define  write(data) SPI_WriteByte(data)
#define read() SPI_readByte()
void delay_ms(uint32_t time_value)
{
	nrf_delay_ms(time_value);
}

uint32_t Time_get(void)
{
	return 8;
}		
void PN532_CS_LOW(void)
{
	if(spi2_CS_swich)
		nrf_gpio_pin_clear(CS2_PIN);
	else
		nrf_gpio_pin_clear(CS1_PIN);
}

void PN532_CS_HIGH (void)
{
	if(spi2_CS_swich)
		nrf_gpio_pin_set(CS2_PIN);
	else
		nrf_gpio_pin_set(CS1_PIN);
}

uint8_t send_data[13];
uint8_t send_data_index=0;
u8 SPI_readByte()
{
	uint8_t RxData = 0;
	MOSI_L;
	SCK_L;
	for(char i=0;i<8;i++)
	{
		RxData>>=1;
		if(MISO_R==1)
			RxData|=0x80;
		SCK_H;
		SCK_L;
	}	
	return RxData;
}
u8 SPI_WriteByte(u8 TxData)
{	
	SCK_L;
	for(char i=0;i<8;i++)
	{
		SCK_L;
		if((TxData&0x01)==0x01)
			MOSI_H;
		else
			MOSI_L;
		SCK_H;
		TxData=TxData>>1;
		SCK_L;
	}
}
	

 void  wakeup(void)
{
    PN532_CS_LOW();
    delay_ms(2);
    PN532_CS_HIGH();
}


int8_t  writeCommand(const uint8_t *header, uint8_t hlen)
{
	uint16_t timeout=400;

    command = header[0];
    writeFrame(header, hlen);
    while (!isReady()) {
        delay_ms(1);
        timeout--;
        if (0 == timeout) {
            DMSG("Time out when waiting for ACK\n");
            return -2;
        }
    }
    if (readAckFrame()) {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }
    return 0;
}

int16_t  readResponse(uint8_t buf[], uint8_t len )
{
	uint8_t checksum;
	uint8_t sum , cmd ,i;
	uint8_t length;
	int16_t result;
  uint16_t time = 0;
	uint16_t timeout = 400; 
    while (!isReady()) {
        delay_ms(1);
        time++;
        if (timeout > 0 && time > timeout) {
            return PN532_TIMEOUT;
        }
    }

    PN532_CS_LOW();
    delay_ms(1);

    
    do {
        write(DATA_READ);

        if (0x00 != read()      ||       // PREAMBLE
                0x00 != read()  ||       // STARTCODE1
                0xFF != read()           // STARTCODE2
           ) {

            result = PN532_INVALID_FRAME;
            break;
        }

         length = read();
        if (0 != (uint8_t)(length + read())) {   // checksum of length
            result = PN532_INVALID_FRAME;
            break;
        }

         cmd = command + 1;               // response command
        if (PN532_PN532TOHOST != read() || (cmd) != read()) {
            result = PN532_INVALID_FRAME;
            break;
        }

        DMSG("read:  ");
        DMSG_HEX(cmd);

        length -= 2;
        if (length > len) {
            for ( i = 0; i < length; i++) {
                DMSG_HEX(read());                 // dump message
            }
            DMSG("\nNot enough space\n");
            read();
            read();
            result = PN532_NO_SPACE;  // not enough space
            break;
        }

         sum = PN532_PN532TOHOST + cmd;
        for ( i = 0; i < length; i++) {
            buf[i] = read();
            sum += buf[i];

            DMSG_HEX(buf[i]);
        }
        DMSG("\r\n");

        checksum = read();
        if (0 != (uint8_t)(sum + checksum)) {
            DMSG("checksum is not ok\n");
            result = PN532_INVALID_FRAME;
            break;
        }
        read();         // POSTAMBLE

        result = length;
    } while (0);

    PN532_CS_HIGH();

    return result;
}
uint8_t status;
uint8_t  isReady(void)
{
		//uint8_t status;
    PN532_CS_LOW();

    write(STATUS_READ);
     status = read() & 1;
    PN532_CS_HIGH();
    return status;
}

void  writeFrame(const uint8_t *header, uint8_t hlen)
{
		uint8_t checksum;
		uint8_t sum;
		uint8_t i;
		uint8_t length;
	
		write_flag=1;
	
    PN532_CS_LOW();
    delay_ms(2);               // wake up PN532

    write(DATA_WRITE);
    write(PN532_PREAMBLE);
    write(PN532_STARTCODE1);
    write(PN532_STARTCODE2);

     length = hlen  + 1;   // length of data field: TFI + DATA
    write(length);
    write(~length + 1);         // checksum of length

    write(PN532_HOSTTOPN532);
    sum = PN532_HOSTTOPN532;    // sum of TFI + DATA

    DMSG("write: ");

    for ( i = 0; i < hlen; i++) {
        write(header[i]);
        sum += header[i];

        DMSG_HEX(header[i]);
    }
   

    checksum = ~sum + 1;        // checksum of TFI + DATA
    write(checksum);
    write(PN532_POSTAMBLE);

    PN532_CS_HIGH();

    DMSG("\r\n");
}

int8_t  readAckFrame(void)
{
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
	uint8_t i;

    uint8_t ackBuf[sizeof(PN532_ACK)];

    PN532_CS_LOW();
    delay_ms(1);
    write(DATA_READ);

    for ( i = 0; i < sizeof(PN532_ACK); i++) {
        ackBuf[i] = read();
    }

    PN532_CS_HIGH();

    return memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK));
}
/*
static  void write(uint8_t data) 
{
	SPI_ReadWriteByte(data);

}

static  uint8_t read(void) 
{
	
		return SPI_ReadWriteByte(0);
		 
}
*/
