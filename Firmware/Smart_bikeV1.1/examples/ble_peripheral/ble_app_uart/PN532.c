/**************************************************************************/
/*!
    @file     PN532.c
*/
/**************************************************************************/

#include "PN532.h"	

uint8_t NFCState = 0; //  cs1->0x01,cs2->0x02,all good->0x03

//for  User
u8 keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; 
u8 len;

uint8_t last_uid[4];
uint32_t start_time;
uint32_t process_time;


uint8_t _uid[7];  // ISO14443A uid
uint8_t _uidLen;  // uid len
uint8_t _key[6];  // Mifare Classic key
uint8_t inListedTag; // Tg number of inlisted tag.

uint8_t pn532_packetbuffer[64];

uint8_t spi2_CS_swich = NFC_CS1;
extern uint8_t test_flag;
void NFC_select(uint8_t nfc_cs)
{
	
	spi2_CS_swich = nfc_cs;
}

void RTC_INIT()
{
	
}
		u8 card_uid[6];
		u8 uid_len;
		u8 nfc_buff[20];
extern uint8_t                 uid_data[16];
void Get_uid()
{
		nrf_gpio_pin_clear(10);
		NFC_init();
		wakeup();
		if(getFirmwareVersion())
		NFCState |= 1; 
		if(NFC_wait_card_once(NFC_CS1,card_uid,uid_len))
		{
				memcpy(uid_data,card_uid,6);
				test_flag=1;
		}
		nrf_gpio_pin_set(10);	
}
void  NFC_init(void)
{
		wakeup();
		delay_ms(50);
		SAMConfig();
		if(getFirmwareVersion())
		NFCState |= 1; 
}


/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters

    @param  data      Pointer to the uint8_t data
    @param  numBytes  Data length in bytes
*/
/*************************************************************************
void  PrintHex(const uint8_t *data, const uint32_t numBytes)
{
	uint8_t i;
#ifdef ARDUINO
    for (uint8_t i = 0; i < numBytes; i++) {
        if (data[i] < 0x10) {
            Serial.print(" 0");
        } else {
            Serial.print(' ');
        }
        Serial.print(data[i], HEX);
    }
    Serial.println("");
#else
    for ( i = 0; i < numBytes; i++) {
        printf(" %2X", data[i]);
    }
    printf("\n");
#endif
}
*/
/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters, along with
            the char equivalents in the following format

            00 00 00 00 00 00  ......

    @param  data      Pointer to the data
    @param  numBytes  Data length in bytes
*/
/*************************************************************************
void  PrintHexChar(const uint8_t *data, const uint32_t numBytes)
{
#ifdef ARDUINO
    for (uint8_t i = 0; i < numBytes; i++) {
        if (data[i] < 0x10) {
            Serial.print(" 0");
        } else {
            Serial.print(' ');
        }
        Serial.print(data[i], HEX);
    }
    Serial.print("    ");
    for (uint8_t i = 0; i < numBytes; i++) {
        char c = data[i];
        if (c <= 0x1f || c > 0x7f) {
            Serial.print('.');
        } else {
            Serial.print(c);
        }
    }
    Serial.println("");
#else
    for (uint8_t i = 0; i < numBytes; i++) {
        printf(" %2X", data[i]);
    }
    printf("    ");
    for (uint8_t i = 0; i < numBytes; i++) {
        char c = data[i];
        if (c <= 0x1f || c > 0x7f) {
            printf(".");
        } else {
            printf("%c", c);
        }
        printf("\n");
    }
#endif
}
*/
/**************************************************************************/
/*!
    @brief  Checks the firmware version of the PN5xx chip

    @returns  The chip's firmware version and ID
*/
/**************************************************************************/
uint32_t response;
uint32_t  getFirmwareVersion(void)
{
    //uint32_t response;
		int16_t status;
    pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

    if (writeCommand(pn532_packetbuffer, 1)) {
        return 0;
    }

    // read data packet
    status = readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
    if (0 > status) {
        return 0;
    }

    response = pn532_packetbuffer[0];
    response <<= 8;
    response |= pn532_packetbuffer[1];
    response <<= 8;
    response |= pn532_packetbuffer[2];
    response <<= 8;
    response |= pn532_packetbuffer[3];

    return response;
}
uint32_t  power_down_nfc(void)
{
    pn532_packetbuffer[0] = PN532_COMMAND_POWERDOWN;

    if (writeCommand(pn532_packetbuffer, 1)) {
        return 0;
    }

    return 1;
}

/**************************************************************************/
/*!
    Writes an 8-bit value that sets the state of the PN532's GPIO pins

    @warning This function is provided exclusively for board testing and
             is dangerous since it will throw an error if any pin other
             than the ones marked "Can be used as GPIO" are modified!  All
             pins that can not be used as GPIO should ALWAYS be left high
             (value = 1) or the system will become unstable and a HW reset
             will be required to recover the PN532.

             pinState[0]  = P30     Can be used as GPIO
             pinState[1]  = P31     Can be used as GPIO
             pinState[2]  = P32     *** RESERVED (Must be 1!) ***
             pinState[3]  = P33     Can be used as GPIO
             pinState[4]  = P34     *** RESERVED (Must be 1!) ***
             pinState[5]  = P35     Can be used as GPIO

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t  writeGPIO(uint8_t pinstate)
{
    // Make sure pinstate does not try to toggle P32 or P34
    pinstate |= (1 << PN532_GPIO_P32) | (1 << PN532_GPIO_P34);

    // Fill command buffer
    pn532_packetbuffer[0] = PN532_COMMAND_WRITEGPIO;
    pn532_packetbuffer[1] = PN532_GPIO_VALIDATIONBIT | pinstate;  // P3 Pins
    pn532_packetbuffer[2] = 0x00;    // P7 GPIO Pins (not used ... taken by I2C)

    DMSG("Writing P3 GPIO: ");
    DMSG_HEX(pn532_packetbuffer[1]);
    DMSG("\n");

    // Send the WRITEGPIO command (0x0E)
    if (writeCommand(pn532_packetbuffer, 3))
        return 0;

    return (0 < readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Reads the state of the PN532's GPIO pins

    @returns An 8-bit value containing the pin state where:

             pinState[0]  = P30
             pinState[1]  = P31
             pinState[2]  = P32
             pinState[3]  = P33
             pinState[4]  = P34
             pinState[5]  = P35
*/
/**************************************************************************/
uint8_t  readGPIO(void)
{
    pn532_packetbuffer[0] = PN532_COMMAND_READGPIO;

    // Send the READGPIO command (0x0C)
    if (writeCommand(pn532_packetbuffer, 1))
        return 0x0;

    readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));

    /* READGPIO response without prefix and suffix should be in the following format:

      byte            Description
      -------------   ------------------------------------------
      b0              P3 GPIO Pins
      b1              P7 GPIO Pins (not used ... taken by I2C)
      b2              Interface Mode Pins (not used ... bus select pins)
    */


    DMSG("P3 GPIO: "); DMSG_HEX(pn532_packetbuffer[7]);
    DMSG("P7 GPIO: "); DMSG_HEX(pn532_packetbuffer[8]);
    DMSG("I0I1 GPIO: "); DMSG_HEX(pn532_packetbuffer[9]);
    DMSG("\n");

    return pn532_packetbuffer[0];
}

/**************************************************************************/
/*!
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
uint8_t  SAMConfig(void)
{
    pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    pn532_packetbuffer[1] = 0x01; // normal mode;
    pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
    pn532_packetbuffer[3] = 0x00; // use IRQ pin!

    DMSG("SAMConfig\r\n");

    if (writeCommand(pn532_packetbuffer, 4))
        return false;

    return (0 < readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register

    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t  setPassiveActivationRetries(uint8_t maxRetries)
{
    pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
    pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
    pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
    pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
    pn532_packetbuffer[4] = maxRetries;

    if (writeCommand(pn532_packetbuffer, 5))
        return 0x0;  // no ACK

    return (0 < readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/***** ISO14443A Commands ******/

/**************************************************************************/
/*!
    Waits for an ISO14443A target to enter the field

    @param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t  readPassiveTargetID(uint8_t cardbaudrate, uint8_t *uid, uint8_t *uidLength )
{
//	uint16_t timeout = 3000;
	int16_t sens_res;
	uint8_t i ;
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
    pn532_packetbuffer[2] = cardbaudrate;

    if (writeCommand(pn532_packetbuffer, 3)) {
        return 0x0;  // command failed
    }

    // read data packet
    if (readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer)) < 0) {
        return 0x0;
    }

    // check some basic stuff
    /* ISO14443A card response should be in the following format:

      byte            Description
      -------------   ------------------------------------------
      b0              Tags Found
      b1              Tag Number (only one used in this example)
      b2..3           SENS_RES
      b4              SEL_RES
      b5              NFCID Length
      b6..NFCIDLen    NFCID
    */

    if (pn532_packetbuffer[0] != 1)
        return 0;

     sens_res = pn532_packetbuffer[2];
    sens_res <<= 8;
    sens_res |= pn532_packetbuffer[3];

    DMSG("ATQA: 0x");  DMSG_HEX(sens_res);
    DMSG("SAK: 0x");  DMSG_HEX(pn532_packetbuffer[4]);
    DMSG("\n");

    /* Card appears to be Mifare Classic */
    *uidLength = pn532_packetbuffer[5];

    for ( i = 0; i < pn532_packetbuffer[5]; i++) {
        uid[i] = pn532_packetbuffer[6 + i];
    }

    return 1;
}


/***** Mifare Classic Functions ******/

/**************************************************************************/
/*!
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
uint8_t  mifareclassic_IsFirstBlock (uint32_t uiBlock)
{
    // Test if we are in the small or big sectors
    if (uiBlock < 128)
        return ((uiBlock) % 4 == 0);
    else
        return ((uiBlock) % 16 == 0);
}

/**************************************************************************/
/*!
      Indicates whether the specified block number is the sector trailer
*/
/**************************************************************************/
uint8_t  mifareclassic_IsTrailerBlock (uint32_t uiBlock)
{
    // Test if we are in the small or big sectors
    if (uiBlock < 128)
        return ((uiBlock + 1) % 4 == 0);
    else
        return ((uiBlock + 1) % 16 == 0);
}

/**************************************************************************/
/*!
    Tries to authenticate a block of memory on a MIFARE card using the
    INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  uid           Pointer to a byte array containing the card UID
    @param  uidLen        The length (in bytes) of the card's UID (Should
                          be 4 for MIFARE Classic)
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  keyNumber     Which key type to use during authentication
                          (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
    @param  keyData       Pointer to a byte array containing the 6 bytes
                          key value

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t  mifareclassic_AuthenticateBlock (uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t *keyData)
{
    uint8_t i;

    // Hang on to the key and uid data
    memcpy (_key, keyData, 6);
    memcpy (_uid, uid, uidLen);
    _uidLen = uidLen;

    // Prepare the authentication command //
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
    pn532_packetbuffer[1] = 1;                              /* Max card numbers */
    pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
    pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
    memcpy (pn532_packetbuffer + 4, _key, 6);
    for (i = 0; i < _uidLen; i++) {
        pn532_packetbuffer[10 + i] = _uid[i];              /* 4 bytes card ID */
    }

    if (writeCommand(pn532_packetbuffer, 10 + _uidLen))
        return 0;

    // Read the response packet
    readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));

    // Check if the response is valid and we are authenticated???
    // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
    // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
    if (pn532_packetbuffer[0] != 0x00) {
        DMSG("Authentification failed\n");
        return 0;
    }

    return 1;
}

/**************************************************************************/
/*!
    Tries to read an entire 16-bytes data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          Pointer to the byte array that will hold the
                          retrieved data (if any)

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t  mifareclassic_ReadDataBlock (uint8_t blockNumber, uint8_t *data)
{
    DMSG("Trying to read 16 bytes from block ");
    DMSG_INT(blockNumber);

    /* Prepare the command */
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                      /* Card number */
    pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
    pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

    /* Send the command */
    if (writeCommand(pn532_packetbuffer, 4)) {
        return 0;
    }

    /* Read the response packet */
    readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));

    /* If byte 8 isn't 0x00 we probably have an error */
    if (pn532_packetbuffer[0] != 0x00) {
        return 0;
    }

    /* Copy the 16 data bytes to the output buffer        */
    /* Block content starts at byte 9 of a valid response */
    memcpy (data, pn532_packetbuffer + 1, 16);

    return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 16-bytes data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          The byte array that contains the data to write.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t  mifareclassic_WriteDataBlock (uint8_t blockNumber, uint8_t *data)
{
    /* Prepare the first command */
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                      /* Card number */
    pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
    pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
    memcpy (pn532_packetbuffer + 4, data, 16);        /* Data Payload */

    /* Send the command */
    if (writeCommand(pn532_packetbuffer, 20)) {
        return 0;
    }

    /* Read the response packet */
    return (0 < readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Formats a Mifare Classic card to store NDEF Records

    @returns 1 if everything executed properly, 0 for an error
*/
/*************************************************************************
uint8_t  mifareclassic_FormatNDEF (void)
{
    uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
    uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
    uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
    // for the MAD sector in NDEF records (sector 0)

    // Write block 1 and 2 to the card
    if (!(mifareclassic_WriteDataBlock (1, sectorbuffer1)))
        return 0;
    if (!(mifareclassic_WriteDataBlock (2, sectorbuffer2)))
        return 0;
    // Write key A and access rights card
    if (!(mifareclassic_WriteDataBlock (3, sectorbuffer3)))
        return 0;

    // Seems that everything was OK (?!)
    return 1;
}
*/
/**************************************************************************/
/*!
    Writes an NDEF URI Record to the specified sector (1..15)

    Note that this function assumes that the Mifare Classic card is
    already formatted to work as an "NFC Forum Tag" and uses a MAD1
    file system.  You can use the NXP TagWriter app on Android to
    properly format cards for this.

    @param  sectorNumber  The sector that the URI record should be written
                          to (can be 1..15 for a 1K card)
    @param  uriIdentifier The uri identifier code (0 = none, 0x01 =
                          "http://www.", etc.)
    @param  url           The uri text to write (max 38 characters).

    @returns 1 if everything executed properly, 0 for an error
*/
/*************************************************************************
uint8_t  mifareclassic_WriteNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char *url)
{
    // Figure out how long the string is
    uint8_t len = strlen(url);

    // Make sure we're within a 1K limit for the sector number
    if ((sectorNumber < 1) || (sectorNumber > 15))
        return 0;

    // Make sure the URI payload is between 1 and 38 chars
    if ((len < 1) || (len > 38))
        return 0;

    // Note 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A
    // in NDEF records

    // Setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
    uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, len + 5, 0xD1, 0x01, len + 1, 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (len <= 6) {
        // Unlikely we'll get a url this short, but why not ...
        memcpy (sectorbuffer1 + 9, url, len);
        sectorbuffer1[len + 9] = 0xFE;
    } else if (len == 7) {
        // 0xFE needs to be wrapped around to next block
        memcpy (sectorbuffer1 + 9, url, len);
        sectorbuffer2[0] = 0xFE;
    } else if ((len > 7) || (len <= 22)) {
        // Url fits in two blocks
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, len - 7);
        sectorbuffer2[len - 7] = 0xFE;
    } else if (len == 23) {
        // 0xFE needs to be wrapped around to final block
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, len - 7);
        sectorbuffer3[0] = 0xFE;
    } else {
        // Url fits in three blocks
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, 16);
        memcpy (sectorbuffer3, url + 23, len - 24);
        sectorbuffer3[len - 22] = 0xFE;
    }

    // Now write all three blocks back to the card
    if (!(mifareclassic_WriteDataBlock (sectorNumber * 4, sectorbuffer1)))
        return 0;
    if (!(mifareclassic_WriteDataBlock ((sectorNumber * 4) + 1, sectorbuffer2)))
        return 0;
    if (!(mifareclassic_WriteDataBlock ((sectorNumber * 4) + 2, sectorbuffer3)))
        return 0;
    if (!(mifareclassic_WriteDataBlock ((sectorNumber * 4) + 3, sectorbuffer4)))
        return 0;

    // Seems that everything was OK (?!)
    return 1;
}
*/
/***** Mifare Ultralight Functions ******/

/**************************************************************************/
/*!
    Tries to read an entire 4-bytes page at the specified address.

    @param  page        The page number (0..63 in most cases)
    @param  buffer      Pointer to the byte array that will hold the
                        retrieved data (if any)
*/
/**************************************************************************
uint8_t  mifareultralight_ReadPage (uint8_t page, uint8_t *buffer)
{
    if (page >= 64) {
        DMSG("Page value out of range\n");
        return 0;
    }

    // Prepare the command 
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                   / Card number 
    pn532_packetbuffer[2] = MIFARE_CMD_READ;     // Mifare Read command = 0x30 
    pn532_packetbuffer[3] = page;                // Page Number (0..63 in most cases) 

    // Send the command 
    if (writeCommand(pn532_packetbuffer, 4)) {
        return 0;
    }

    // Read the response packet 
    readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));

    // If byte 8 isn't 0x00 we probably have an error 
    if (pn532_packetbuffer[0] == 0x00) {
        // Copy the 4 data bytes to the output buffer         
        // Block content starts at byte 9 of a valid response 
        // Note that the command actually reads 16 bytes or 4  
        // pages at a time ... we simply discard the last 12  
        // bytes                                              
        memcpy (buffer, pn532_packetbuffer + 1, 4);
    } else {
        return 0;
    }

    // Return OK signal
    return 1;
}
*/

/**************************************************************************/
/*!
    @brief  Exchanges an APDU with the currently inlisted peer

    @param  send            Pointer to data to send
    @param  sendLength      Length of the data to send
    @param  response        Pointer to response data
    @param  responseLength  Pointer to the response data length
*/
/**************************************************************************
uint8_t  inDataExchange(uint8_t *send, uint8_t sendLength, uint8_t *response, uint8_t *responseLength)
{
    uint8_t i;
		int16_t status;
		uint8_t length;
	
	
    pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = inListedTag;

    if (writeCommand(pn532_packetbuffer, 2)) {
        return false;
    }

     status = readResponse(response, *responseLength);
    if (status < 0) {
        return false;
    }

    if ((response[0] & 0x3f) != 0) {
        DMSG("Status code indicates an error\n");
        return false;
    }

    length = status;
    length -= 1;

    if (length > *responseLength) {
        length = *responseLength; // silent truncation...
    }

    for (i = 0; i < length; i++) {
        response[i] = response[i + 1];
    }
    *responseLength = length;

    return true;
}
*/
/**************************************************************************/
/*!
    @brief  'InLists' a passive target. PN532 acting as reader/initiator,
            peer acting as card/responder.
*/
/*************************************************************************
uint8_t  inListPassiveTarget()
{
		int16_t status;
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;
    pn532_packetbuffer[2] = 0;

    DMSG("inList passive target\n");

    if (writeCommand(pn532_packetbuffer, 3)) {
        return false;
    }

    status = readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
    if (status < 0) {
        return false;
    }

    if (pn532_packetbuffer[0] != 1) {
        return false;
    }

    inListedTag = pn532_packetbuffer[1];

    return true;
}
*/
/*
int8_t  tgInitAsTarget(const uint8_t* command, const uint8_t len, const uint16_t timeout){
  
  int8_t status = writeCommand(command, len);
    if (status < 0) {
        return -1;
    }

    status = readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer), timeout);
    if (status > 0) {
        return 1;
    } else if (PN532_TIMEOUT == status) {
        return 0;
    } else {
        return -2;
    }
}
*/
/**
 * Peer to Peer
 
int8_t  tgInitAsTarget(uint16_t timeout)
{
    const uint8_t command[] = {
        PN532_COMMAND_TGINITASTARGET,
        0,
        0x00, 0x00,         //SENS_RES
        0x00, 0x00, 0x00,   //NFCID1
        0x40,               //SEL_RES

        0x01, 0xFE, 0x0F, 0xBB, 0xBA, 0xA6, 0xC9, 0x89, // POL_RES
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF,

        0x01, 0xFE, 0x0F, 0xBB, 0xBA, 0xA6, 0xC9, 0x89, 0x00, 0x00, //NFCID3t: Change this to desired value

        0x06, 0x46,  0x66, 0x6D, 0x01, 0x01, 0x10, 0x00// LLCP magic number and version parameter
    };
    return tgInitAsTarget(command, sizeof(command), timeout);
}
*/

/*
int16_t  tgGetData(uint8_t *buf, uint8_t len)
{
		int16_t status ;
		uint8_t i;
		uint16_t length;
    buf[0] = PN532_COMMAND_TGGETDATA;

    if (writeCommand(buf, 1)) {
        return -1;
    }

    status = readResponse(buf, len);
    if (0 >= status) {
        return status;
    }

     length = status - 1;


    if (buf[0] != 0) {
        DMSG("status is not ok\n");
        return -5;
    }

    for ( i = 0; i < length; i++) {
        buf[i] = buf[i + 1];
    }

    return length;
}
*/
/*
uint8_t  tgSetData(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    if (hlen > (sizeof(pn532_packetbuffer) - 1)) {
        if ((body != 0) || (header == pn532_packetbuffer)) {
            DMSG("tgSetData:buffer too small\n");
            return false;
        }

        pn532_packetbuffer[0] = PN532_COMMAND_TGSETDATA;
        if (writeCommand(pn532_packetbuffer, 1, header, hlen)) {
            return false;
        }
    } else {
        for (int8_t i = hlen - 1; i >= 0; i--){
            pn532_packetbuffer[i + 1] = header[i];
        }
        pn532_packetbuffer[0] = PN532_COMMAND_TGSETDATA;

        if (writeCommand(pn532_packetbuffer, hlen + 1, body, blen)) {
            return false;
        }
    }

    if (0 > readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer), 3000)) {
        return false;
    }

    if (0 != pn532_packetbuffer[0]) {
        return false;
    }

    return true;
}
*/
/*
int16_t  inRelease(const uint8_t relevantTarget){

    pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
    pn532_packetbuffer[1] = relevantTarget;

    if (writeCommand(pn532_packetbuffer, 2)) {
        return 0;
    }

    // read data packet
    return readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}
*/
/*
 uint8_t *getBuffer(uint8_t *len) 
{
		*len = sizeof(pn532_packetbuffer) - 4;
		return pn532_packetbuffer;
}
*/
		
		



/***************************************************************************
USER func
*/

uint8_t is_permit_block(uint8_t blockNumber)
{
	if(blockNumber < 4)
		return false;
	if((blockNumber%4) == 3)
		return false;
	
	return true;
}
		

uint8_t NFC_write(uint8_t blockNumber, uint8_t *data)
{
	u8 len;
	u8 cuid[4];

	if(!is_permit_block(blockNumber))
		return false;
		
	if(readPassiveTargetID(0,cuid,&len))
	{
		if(mifareclassic_AuthenticateBlock(cuid, len, blockNumber, 0, keya))
		{

			if(mifareclassic_WriteDataBlock(blockNumber,data))
				return true;
		}
	}

	return false;
}

uint8_t NFC_read(uint8_t blockNumber, uint8_t *data)
{
	u8 len;
	u8 cuid[4];
	//u8 keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; 

	if(!is_permit_block(blockNumber))
		return false;
	
	if(readPassiveTargetID(0,cuid,&len))
	{

		if(mifareclassic_AuthenticateBlock(cuid, len, blockNumber, 0, keya))
		{
			
			if(mifareclassic_ReadDataBlock (blockNumber,data))
				return true;
		}
	}
	return false;
}


/*
updata NFC card end_time and process_time

if now_time < start_time , set start_time as now_time.
*/
uint8_t updata_record(void)
{
	u32 rct;
	u8 data[16];
	
	rct = Time_get();
	if(rct > start_time)
	{	
		//memset(data,0,16);
		//update endtime
		process_time = rct - start_time;
		data[0] = 0x55;
		data[1] = 0xaa;
		data[2] = rct >> 24;
		data[3] = rct >> 16;
		data[4] = rct >> 8;
		data[5] = rct;
		
		if(!mifareclassic_AuthenticateBlock(last_uid, len, END_TIME_BLOCK, 0, keya))
			return false;
		if(!mifareclassic_WriteDataBlock (END_TIME_BLOCK,data))
			return false;
		
		//
		if(mifareclassic_AuthenticateBlock(last_uid, len, PROCESS_TIME_BLOCK, 0, keya))
		{
			if(mifareclassic_ReadDataBlock (PROCESS_TIME_BLOCK,data))
			{
				if(data[0] == 0x55 && data[1] == 0xaa)
				{
					process_time  = data[2]<<24;
					process_time += data[3]<<16;
					process_time += data[4]<<8;
					process_time += data[5];
					
					process_time += 60;
					
					data[2] = process_time >> 24;
					data[3] = process_time >> 16;
					data[4] = process_time >> 8;
					data[5] = process_time;
					if(!mifareclassic_AuthenticateBlock(last_uid, len, PROCESS_TIME_BLOCK, 0, keya))
						return false;
					if(!mifareclassic_WriteDataBlock (PROCESS_TIME_BLOCK,data))
						return false;
				}
				else
				{
					process_time = 0;
					data[0] = 0x55;
					data[1] = 0xaa;
					data[2] = process_time >> 24;
					data[3] = process_time >> 16;
					data[4] = process_time >> 8;
					if(!mifareclassic_WriteDataBlock (PROCESS_TIME_BLOCK,data))
						return false;
				}
						
			}else return false;
		}else return false;
		
	}
	else 
	{
		// if rct < start_time ,set rct as start time.
		start_time = Time_get();
		data[0] = 0x55;
		data[1] = 0xaa;
		data[2] = start_time >> 24;
		data[3] = start_time >> 16;
		data[4] = start_time >> 8;
		data[5] = start_time;
		if(!mifareclassic_AuthenticateBlock(last_uid, len, START_TIME_BLOCK, 0, keya))
			return false;
		if(!mifareclassic_WriteDataBlock (START_TIME_BLOCK,data))
			return false;
	}
	
	return true;
}

/*
NFC record time process function

if the card is the same , updata data;
if not the same card , check satrt time and updata; 
if it is a new card ,mark the start time .

if return true, it mean updata success
if return false,it mean no card or other err. please call again in a short time.
*/
uint8_t NFC_wait_card(u8 nfc_select)
{
	//u32 rct;
	u8 data[16];
	u8 cuid[4];
	
		
	//if(!NFCState)// nfc err
	//	return false;
	if(nfc_select == NFC_CS1)
	{
		if(!(NFCState&0x01))
			return false;
	}
	else if(nfc_select == NFC_CS2)
	{
		if(!(NFCState&0x02))
			return false;
	}
		
	spi2_CS_swich = nfc_select;
	//printf("\r\n NFC_wait_card:");
	if(readPassiveTargetID(0,cuid,&len))
	{
		if( memcmp(cuid,last_uid, len) != 0)
		{
			// not the same card . mark uid
			memcpy(last_uid,cuid, len);
			if(mifareclassic_AuthenticateBlock(last_uid, len, START_TIME_BLOCK, 0, keya))
			{
				if(mifareclassic_ReadDataBlock (START_TIME_BLOCK,data))
				{
					//printf("\r\nnot the same card \r\n");
					//if has been recorded start time , get start time
					if(data[0] == 0x55 && data[1] == 0xaa)
					{
						start_time  = data[2]<<24;
						start_time += data[3]<<16;
						start_time += data[4]<<8;
						start_time += data[5];
						
						}
						else
						{
							//printf("\r\n new card \r\n");
							// is a new card ,recorded start time and updata
							start_time = Time_get();
							data[0] = 0x55;
							data[1] = 0xaa;
							data[2] = start_time >> 24;
							data[3] = start_time >> 16;
							data[4] = start_time >> 8;
							data[5] = start_time;
							if(!mifareclassic_WriteDataBlock (START_TIME_BLOCK,data))
								return false;

						}
						
					}
			}
			
		}
		//updata nfc data
		if(updata_record())
			return true;
	}
	
	return false;
}




void getFilterTime(u8 nfc_select, u32 *st,u32 *et,u32 *pt)
{
	//_calendar_obj myt;
	
	u8 rbuf[17] = {0};
	
	if(nfc_select == NFC_CS1)
	{
		if(!(NFCState&0x01))
			return ;
	}
	else if(nfc_select == NFC_CS2)
	{
		if(!(NFCState&0x02))
			return ;
	}
	spi2_CS_swich = nfc_select;
	
	NFC_read(START_TIME_BLOCK,rbuf);
	*st  = rbuf[2]<<24;
	*st += rbuf[3]<<16;
	*st += rbuf[4]<<8;
	*st += rbuf[5];
	//printf("\r\n start time:");
	//Get_calendar(st,&myt);
	//showrtc(&myt);
	
	NFC_read(END_TIME_BLOCK,rbuf);
	*et  = rbuf[2]<<24;
	*et += rbuf[3]<<16;
	*et += rbuf[4]<<8;
	*et += rbuf[5];
	//printf("\r\n end time:");
	//Get_calendar(et,&myt);
	//showrtc(&myt);
	
	NFC_read(PROCESS_TIME_BLOCK,rbuf);
	*pt  = rbuf[2]<<24;
	*pt += rbuf[3]<<16;
	*pt += rbuf[4]<<8;
	*pt += rbuf[5];
	//printf("\r\n process time:");
	//Get_process_time(st,pt,&myt);
	//showrtc(&myt);
				
}
u8 NFC_CheckCard(void)
{
	u8 cuid[6];
//	u8 uid_len;
	
		//if( NFCState != 0x03 )  return false;
	  if( NFCState != 0x01 )  return false;
		
		//spi2_CS_swich = NFC_CS2;
		//if(!readPassiveTargetID(0,cuid,&len)) return false;
	 
		spi2_CS_swich = NFC_CS1;
			if(!readPassiveTargetID(0,cuid,&len)) return false;
	/*
	if(NFCState&0x01 )
	{
		spi2_CS_swich = NFC_CS2;
		if(!readPassiveTargetID(0,cuid,&len)) return false;
	}
	
	if(NFCState&0x02 )
	{
		spi2_CS_swich = NFC_CS1;
		if(!readPassiveTargetID(0,cuid,&len)) return false;
	}
			*/
	return true;
}

u8 NFC_wait_card_once(u8 nfc_select,u8 cuid[],u8 len)
{

//	if(nfc_select == NFC_CS1)
//	{
//		if(!(NFCState&0x01))
//			return false;
//	}
//	else if(nfc_select == NFC_CS2)
//	{
//		if(!(NFCState&0x02))
//			return false;
//	}
//	spi2_CS_swich = nfc_select;

			
		if(readPassiveTargetID(0,cuid,&len))
		{
			//printf("\r\nget a card, uid:");
			/* 
			if(mifareclassic_AuthenticateBlock(cuid, 4, 5, 0, keya))
			{
				printf("\r\nwrite 16 byte:");
				printf(wbuf);
				mifareclassic_WriteDataBlock(5,wbuf);
				//delay_ms(5);
				mifareclassic_ReadDataBlock (5,rbuf);
				printf("\r\nread  16 byte:");
				printf(rbuf);
				
					return true;
			}
			*/
			return true;
		}
	
	return false;
}

