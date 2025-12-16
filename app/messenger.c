#ifdef ENABLE_MESSENGER

#include <string.h>
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/bk4819.h"
#include "external/printf/printf.h"
#include "misc.h"
#include "settings.h"
#include "radio.h"
#include "app.h"
#include "audio.h"
#include "functions.h"
#include "frequencies.h"
#include "driver/system.h"
#include "app/messenger.h"
#include "common.h"
#include "ui/ui.h"
/* TODO TEST */
#include "dtmf.h"

#if defined(ENABLE_UART)
	#include "driver/uart.h"
#endif

#if defined(ENABLE_XMESH)
	#include <stdlib.h>
	#include "driver/eeprom.h"
#endif

typedef enum MsgStatus {
	READY,
  	SENDING,
  	RECEIVING,
} MsgStatus;

typedef union
{
	uint8_t u8[2];
	uint16_t u16;
	int16_t i16;
} Union_2B;

const uint8_t MAX_MSG_LENGTH = TX_MSG_LENGTH - 1;

const uint16_t TONE2_FREQ = 0x3065; // 0x2854

#define NEXT_CHAR_DELAY 100 // 10ms tick

char T9TableLow[9][4] = { {',', '.', '?', '!'}, {'a', 'b', 'c', '\0'}, {'d', 'e', 'f', '\0'}, {'g', 'h', 'i', '\0'}, {'j', 'k', 'l', '\0'}, {'m', 'n', 'o', '\0'}, {'p', 'q', 'r', 's'}, {'t', 'u', 'v', '\0'}, {'w', 'x', 'y', 'z'} };
char T9TableUp[9][4] = { {',', '.', '?', '!'}, {'A', 'B', 'C', '\0'}, {'D', 'E', 'F', '\0'}, {'G', 'H', 'I', '\0'}, {'J', 'K', 'L', '\0'}, {'M', 'N', 'O', '\0'}, {'P', 'Q', 'R', 'S'}, {'T', 'U', 'V', '\0'}, {'W', 'X', 'Y', 'Z'} };
unsigned char numberOfLettersAssignedToKey[9] = { 4, 3, 3, 3, 3, 3, 4, 3, 4 };

char T9TableNum[9][4] = { {'1', '\0', '\0', '\0'}, {'2', '\0', '\0', '\0'}, {'3', '\0', '\0', '\0'}, {'4', '\0', '\0', '\0'}, {'5', '\0', '\0', '\0'}, {'6', '\0', '\0', '\0'}, {'7', '\0', '\0', '\0'}, {'8', '\0', '\0', '\0'}, {'9', '\0', '\0', '\0'} };
unsigned char numberOfNumsAssignedToKey[9] = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };

char cMessage[TX_MSG_LENGTH];
char lastcMessage[TX_MSG_LENGTH];
char rxMessage[4][MAX_RX_MSG_LENGTH + 2];
unsigned char cIndex = 0;
unsigned char prevKey = 0, prevLetter = 0;
KeyboardType keyboardType = UPPERCASE;

MsgStatus msgStatus = READY;

uint8_t msgFSKBuffer[MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH];

uint16_t gErrorsDuringMSG;

uint8_t hasNewMessage = 0;

uint8_t keyTickCounter = 0;

#ifdef ENABLE_XMESH
#define HOP_COUNTER_MAX  			7
#define RX_TIME_EXPIRATION_10ms  	60000UL		// 10 minutes
#define TX_BASE_TIME_TO_SEND_10ms	1000UL		// 10 seconds
#define XMESH_BUFFER_SIZE			50
#if XMESH_BUFFER_SIZE >= 100
#warning "XMESH_BUFFER_SIZE increase RAM needed"
#elif XMESH_BUFFER_SIZE > 255
#error "XMESH_BUFFER_SIZE must be in uin8_t boundary"
#endif
#define XMESH_INDEX(x, y) 		(((x) + (y)) % XMESH_BUFFER_SIZE)

typedef struct
{
	uint16_t destination_id;
	uint16_t sender_id;
	uint16_t origin_id;
	uint8_t packet_id;
	uint8_t hop_counter;
	uint8_t reserved_byte;
	uint8_t crc8;
} MeshHeader_t;

typedef struct
{
	uint8_t payload[TX_MSG_LENGTH];
	MeshHeader_t header;
} MeshContent_t;

typedef struct
{
	MeshContent_t info;
	struct
	{
		uint32_t rx_timestamp;	// time when received by FSK; if already above RX_TIME_EXPIRATION, deleted buffer
		uint8_t rx_counter; 	// how many time received by FSK
		uint32_t tx_time;		// time to be sent; 0 if has been sent
	} state;
} MeshBuffer_t;

MeshBuffer_t xMeshBuffer[XMESH_BUFFER_SIZE];
uint16_t base_id = 0;
uint8_t xMeshIndexHead = 0;
uint8_t xMeshIndexTail = 0;
uint8_t xMeshSendPacketId = 0;
#endif

// -----------------------------------------------------

void MSG_FSKSendData() {

	uint16_t fsk_reg59;

	// REG_51
	//
	// <15>  TxCTCSS/CDCSS   0 = disable 1 = Enable
	//
	// turn off CTCSS/CDCSS during FFSK
	const uint16_t css_val = BK4819_ReadRegister(BK4819_REG_51);
	BK4819_WriteRegister(BK4819_REG_51, 0);

	// set the FM deviation level
	const uint16_t dev_val = BK4819_ReadRegister(BK4819_REG_40);
	//UART_printf("\n BANDWIDTH : 0x%.4X", dev_val);
	{
		uint16_t deviation = 850;
		switch (gEeprom.VfoInfo[gEeprom.TX_VFO].CHANNEL_BANDWIDTH)
		{
			case BK4819_FILTER_BW_WIDE:     deviation = 1050; break;
			case BK4819_FILTER_BW_NARROW:   deviation =  850; break;
			case BK4819_FILTER_BW_NARROWER: deviation =  750; break;
		}
		//BK4819_WriteRegister(0x40, (3u << 12) | (deviation & 0xfff));
		BK4819_WriteRegister(BK4819_REG_40, (dev_val & 0xf000) | (deviation & 0xfff));
	}

	// REG_2B   0
	//
	// <15> 1 Enable CTCSS/CDCSS DC cancellation after FM Demodulation   1 = enable 0 = disable
	// <14> 1 Enable AF DC cancellation after FM Demodulation            1 = enable 0 = disable
	// <10> 0 AF RX HPF 300Hz filter     0 = enable 1 = disable
	// <9>  0 AF RX LPF 3kHz filter      0 = enable 1 = disable
	// <8>  0 AF RX de-emphasis filter   0 = enable 1 = disable
	// <2>  0 AF TX HPF 300Hz filter     0 = enable 1 = disable
	// <1>  0 AF TX LPF filter           0 = enable 1 = disable
	// <0>  0 AF TX pre-emphasis filter  0 = enable 1 = disable
	//
	// disable the 300Hz HPF and FM pre-emphasis filter
	//
	const uint16_t filt_val = BK4819_ReadRegister(BK4819_REG_2B);
	BK4819_WriteRegister(BK4819_REG_2B, (1u << 2) | (1u << 0));

	// *******************************************
	// setup the FFSK modem as best we can

	// Uses 1200/1800 Hz FSK tone frequencies 1200 bits/s
	//
	BK4819_WriteRegister(BK4819_REG_58, // 0x37C3);   // 001 101 11 11 00 001 1
		(1u << 13) |		// 1 FSK TX mode selection
							//   0 = FSK 1.2K and FSK 2.4K TX .. no tones, direct FM
							//   1 = FFSK 1200/1800 TX
							//   2 = ???
							//   3 = FFSK 1200/2400 TX
							//   4 = ???
							//   5 = NOAA SAME TX
							//   6 = ???
							//   7 = ???
							//
		(7u << 10) |		// 0 FSK RX mode selection
							//   0 = FSK 1.2K, FSK 2.4K RX and NOAA SAME RX .. no tones, direct FM
							//   1 = ???
							//   2 = ???
							//   3 = ???
							//   4 = FFSK 1200/2400 RX
							//   5 = ???
							//   6 = ???
							//   7 = FFSK 1200/1800 RX
							//
		(0u << 8) |			// 0 FSK RX gain
							//   0 ~ 3
							//
		(0u << 6) |			// 0 ???
							//   0 ~ 3
							//
		(0u << 4) |			// 0 FSK preamble type selection
							//   0 = 0xAA or 0x55 due to the MSB of FSK sync byte 0
							//   1 = ???
							//   2 = 0x55
							//   3 = 0xAA
							//
		(1u << 1) |			// 1 FSK RX bandwidth setting
							//   0 = FSK 1.2K .. no tones, direct FM
							//   1 = FFSK 1200/1800
							//   2 = NOAA SAME RX
							//   3 = ???
							//   4 = FSK 2.4K and FFSK 1200/2400
							//   5 = ???
							//   6 = ???
							//   7 = ???
							//
		(1u << 0));			// 1 FSK enable
							//   0 = disable
							//   1 = enable

	// REG_72
	//
	// <15:0> 0x2854 TONE-2 / FSK frequency control word
	//        = freq(Hz) * 10.32444 for XTAL 13M / 26M or
	//        = freq(Hz) * 10.48576 for XTAL 12.8M / 19.2M / 25.6M / 38.4M
	//
	// tone-2 = 1200Hz
	// 18583,92
	BK4819_WriteRegister(BK4819_REG_72, TONE2_FREQ);

	// REG_70
	//
	// <15>   0 TONE-1
	//        1 = enable
	//        0 = disable
	//
	// <14:8> 0 TONE-1 tuning
	//
	// <7>    0 TONE-2
	//        1 = enable
	//        0 = disable
	//
	// <6:0>  0 TONE-2 / FSK tuning
	//        0 ~ 127
	//
	// enable tone-2, set gain
	//
	BK4819_WriteRegister(BK4819_REG_70,   // 0 0000000 1 1100000
		( 0u << 15) |    // 0
		( 0u <<  8) |    // 0
		( 1u <<  7) |    // 1
		(96u <<  0));    // 96

	// REG_59
	//
	// <15>  0 TX FIFO             1 = clear
	// <14>  0 RX FIFO             1 = clear
	// <13>  0 FSK Scramble        1 = Enable
	// <12>  0 FSK RX              1 = Enable
	// <11>  0 FSK TX              1 = Enable
	// <10>  0 FSK data when RX    1 = Invert
	// <9>   0 FSK data when TX    1 = Invert
	// <8>   0 ???
	//
	// <7:4> 0 FSK preamble length selection
	//       0  =  1 byte
	//       1  =  2 bytes
	//       2  =  3 bytes
	//       15 = 16 bytes
	//
	// <3>   0 FSK sync length selection
	//       0 = 2 bytes (FSK Sync Byte 0, 1)
	//       1 = 4 bytes (FSK Sync Byte 0, 1, 2, 3)
	//
	// <2:0> 0 ???
	//
	fsk_reg59 = (0u << 15) |   // 0/1     1 = clear TX FIFO
				(0u << 14) |   // 0/1     1 = clear RX FIFO
				(0u << 13) |   // 0/1     1 = scramble
				(0u << 12) |   // 0/1     1 = enable RX
				(0u << 11) |   // 0/1     1 = enable TX
				(0u << 10) |   // 0/1     1 = invert data when RX
				(0u <<  9) |   // 0/1     1 = invert data when TX
				(0u <<  8) |   // 0/1     ???
				(15u <<  4) |   // 0 ~ 15  preamble length .. bit toggling
				(1u <<  3) |   // 0/1     sync length
				(0u <<  0);    // 0 ~ 7   ???

	// Set packet length (not including pre-amble and sync bytes that we can't seem to disable)
	BK4819_WriteRegister(BK4819_REG_5D, ((MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH) << 8));

	// REG_5A
	//
	// <15:8> 0x55 FSK Sync Byte 0 (Sync Byte 0 first, then 1,2,3)
	// <7:0>  0x55 FSK Sync Byte 1
	//
	BK4819_WriteRegister(BK4819_REG_5A, 0x5555);                   // bytes 1 & 2

	// REG_5B
	//
	// <15:8> 0x55 FSK Sync Byte 2 (Sync Byte 0 first, then 1,2,3)
	// <7:0>  0xAA FSK Sync Byte 3
	//
	BK4819_WriteRegister(BK4819_REG_5B, 0x55AA);                   // bytes 2 & 3

	// CRC setting (plus other stuff we don't know what)
	//
	// REG_5C
	//
	// <15:7> ???
	//
	// <6>    1 CRC option enable    0 = disable  1 = enable
	//
	// <5:0>  ???
	//
	// disable CRC
	//
	// NB, this also affects TX pre-amble in some way
	//
	BK4819_WriteRegister(BK4819_REG_5C, 0x5625);   // 010101100 0 100101
//		BK4819_WriteRegister(0x5C, 0xAA30);   // 101010100 0 110000
//		BK4819_WriteRegister(0x5C, 0x0030);   // 000000000 0 110000

	BK4819_WriteRegister(BK4819_REG_59, (1u << 15) | (1u << 14) | fsk_reg59);   // clear FIFO's
	BK4819_WriteRegister(BK4819_REG_59, fsk_reg59);

	SYSTEM_DelayMs(100);

	{	// load the entire packet data into the TX FIFO buffer
		const uint16_t len_buff = (MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH);
		for (size_t i = 0, j = 0; i < len_buff; i += 2, j++) {
        	BK4819_WriteRegister(BK4819_REG_5F, (msgFSKBuffer[i + 1] << 8) | msgFSKBuffer[i]);
    	}
	}

	// enable FSK TX
	BK4819_WriteRegister(BK4819_REG_59, (1u << 11) | fsk_reg59);

	{
		// allow up to 310ms for the TX to complete
		// if it takes any longer then somethings gone wrong, we shut the TX down
		unsigned int timeout = 1000 / 5;

		while (timeout-- > 0)
		{
			SYSTEM_DelayMs(5);
			if (BK4819_ReadRegister(BK4819_REG_0C) & (1u << 0))
			{	// we have interrupt flags
				BK4819_WriteRegister(BK4819_REG_02, 0);
				if (BK4819_ReadRegister(BK4819_REG_02) & BK4819_REG_02_FSK_TX_FINISHED)
					timeout = 0;       // TX is complete
			}
		}
	}
	//BK4819_WriteRegister(BK4819_REG_02, 0);

	SYSTEM_DelayMs(100);

	// disable FSK
	BK4819_WriteRegister(BK4819_REG_59, fsk_reg59);

	// restore FM deviation level
	BK4819_WriteRegister(BK4819_REG_40, dev_val);

	// restore TX/RX filtering
	BK4819_WriteRegister(BK4819_REG_2B, filt_val);

	// restore the CTCSS/CDCSS setting
	BK4819_WriteRegister(BK4819_REG_51, css_val);

}

void MSG_EnableRX(const bool enable) {

	if (enable) {
		// REG_70
		//
		// <15>    0 TONE-1
		//         1 = enable
		//         0 = disable
		//
		// <14:8>  0 TONE-1 gain
		//
		// <7>     0 TONE-2
		//         1 = enable
		//         0 = disable
		//
		// <6:0>   0 TONE-2 / FSK gain
		//         0 ~ 127
		//
		// enable tone-2, set gain

		// REG_72
		//
		// <15:0>  0x2854 TONE-2 / FSK frequency control word
		//         = freq(Hz) * 10.32444 for XTAL 13M / 26M or
		//         = freq(Hz) * 10.48576 for XTAL 12.8M / 19.2M / 25.6M / 38.4M
		//
		// tone-2 = 1200Hz

		// REG_58
		//
		// <15:13> 1 FSK TX mode selection
		//         0 = FSK 1.2K and FSK 2.4K TX .. no tones, direct FM
		//         1 = FFSK 1200 / 1800 TX
		//         2 = ???
		//         3 = FFSK 1200 / 2400 TX
		//         4 = ???
		//         5 = NOAA SAME TX
		//         6 = ???
		//         7 = ???
		//
		// <12:10> 0 FSK RX mode selection
		//         0 = FSK 1.2K, FSK 2.4K RX and NOAA SAME RX .. no tones, direct FM
		//         1 = ???
		//         2 = ???
		//         3 = ???
		//         4 = FFSK 1200 / 2400 RX
		//         5 = ???
		//         6 = ???
		//         7 = FFSK 1200 / 1800 RX
		//
		// <9:8>   0 FSK RX gain
		//         0 ~ 3
		//
		// <7:6>   0 ???
		//         0 ~ 3
		//
		// <5:4>   0 FSK preamble type selection
		//         0 = 0xAA or 0x55 due to the MSB of FSK sync byte 0
		//         1 = ???
		//         2 = 0x55
		//         3 = 0xAA
		//
		// <3:1>   1 FSK RX bandwidth setting
		//         0 = FSK 1.2K .. no tones, direct FM
		//         1 = FFSK 1200 / 1800
		//         2 = NOAA SAME RX
		//         3 = ???
		//         4 = FSK 2.4K and FFSK 1200 / 2400
		//         5 = ???
		//         6 = ???
		//         7 = ???
		//
		// <0>     1 FSK enable
		//         0 = disable
		//         1 = enable

		// REG_5C
		//
		// <15:7>  ???
		//
		// <6>     1 CRC option enable
		//         0 = disable
		//         1 = enable
		//
		// <5:0>   ???
		//
		// disable CRC

		// REG_5D
		//
		// set the packet size

		const uint16_t fsk_reg59 =
			(0u << 15) |   // 1 = clear TX FIFO
			(0u << 14) |   // 1 = clear RX FIFO
			(0u << 13) |   // 1 = scramble
			(0u << 12) |   // 1 = enable RX
			(0u << 11) |   // 1 = enable TX
			(0u << 10) |   // 1 = invert data when RX
			(0u <<  9) |   // 1 = invert data when TX
			(0u <<  8) |   // ???
			(0u <<  4) |   // 0 ~ 15 preamble length selection ..
			(1u <<  3) |   // 0/1 sync length selection
			(0u <<  0);    // 0 ~ 7  ???

		// REG_70
		//
		// <15>   0 Enable TONE1
		//        1 = Enable
		//        0 = Disable
		//
		// <14:8> 0 TONE1 tuning gain
		//        0 ~ 127
		//
		// <7>    0 Enable TONE2
		//        1 = Enable
		//        0 = Disable
		//
		// <6:0>  0 TONE2/FSK tuning gain
		//        0 ~ 127
		//
		BK4819_WriteRegister(BK4819_REG_70,
			( 0u << 15) |    // 0
			( 0u <<  8) |    // 0
			( 1u <<  7) |    // 1
			(96u <<  0));    // 96

		// Tone2 baudrate 1200
		BK4819_WriteRegister(BK4819_REG_72, TONE2_FREQ);

		BK4819_WriteRegister(BK4819_REG_58,
			(1u << 13) |		// 1 FSK TX mode selection
								//   0 = FSK 1.2K and FSK 2.4K TX .. no tones, direct FM
								//   1 = FFSK 1200 / 1800 TX
								//   2 = ???
								//   3 = FFSK 1200 / 2400 TX
								//   4 = ???
								//   5 = NOAA SAME TX
								//   6 = ???
								//   7 = ???
								//
			(7u << 10) |		// 0 FSK RX mode selection
								//   0 = FSK 1.2K, FSK 2.4K RX and NOAA SAME RX .. no tones, direct FM
								//   1 = ???
								//   2 = ???
								//   3 = ???
								//   4 = FFSK 1200 / 2400 RX
								//   5 = ???
								//   6 = ???
								//   7 = FFSK 1200 / 1800 RX
								//
			(3u << 8) |			// 0 FSK RX gain
								//   0 ~ 3
								//
			(0u << 6) |			// 0 ???
								//   0 ~ 3
								//
			(0u << 4) |			// 0 FSK preamble type selection
								//   0 = 0xAA or 0x55 due to the MSB of FSK sync byte 0
								//   1 = ???
								//   2 = 0x55
								//   3 = 0xAA
								//
			(1u << 1) |			// 1 FSK RX bandwidth setting
								//   0 = FSK 1.2K .. no tones, direct FM
								//   1 = FFSK 1200 / 1800
								//   2 = NOAA SAME RX
								//   3 = ???
								//   4 = FSK 2.4K and FFSK 1200 / 2400
								//   5 = ???
								//   6 = ???
								//   7 = ???
								//
			(1u << 0));			// 1 FSK enable
								//   0 = disable
								//   1 = enable

		// REG_5A .. bytes 0 & 1 sync pattern
		//
		// <15:8> sync byte 0
		// < 7:0> sync byte 1
		BK4819_WriteRegister(BK4819_REG_5A, 0x5555);

		// REG_5B .. bytes 2 & 3 sync pattern
		//
		// <15:8> sync byte 2
		// < 7:0> sync byte 3
		BK4819_WriteRegister(BK4819_REG_5B, 0x55AA);

		// disable CRC
		BK4819_WriteRegister(BK4819_REG_5C, 0x5625);
		// BK4819_WriteRegister(BK4819_REG_5C, 0xAA30);   // 10101010 0 0 110000

		// set the almost full threshold
		BK4819_WriteRegister(BK4819_REG_5E, (64u << 3) | (1u << 0));  // 0 ~ 127, 0 ~ 7

		{	// packet size .. sync + 14 bytes - size of a single packet

			uint16_t size = (MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH);
			// size -= (fsk_reg59 & (1u << 3)) ? 4 : 2;
			size = (((size + 1) / 2) * 2) + 2;             // round up to even, else FSK RX doesn't work
			BK4819_WriteRegister(BK4819_REG_5D, (size << 8));
		}

		// clear FIFO's then enable RX
		BK4819_WriteRegister(BK4819_REG_59, (1u << 15) | (1u << 14) | fsk_reg59);
		BK4819_WriteRegister(BK4819_REG_59, (1u << 12) | fsk_reg59);

		BK4819_WriteRegister(BK4819_REG_02, 0);

	} else {
		BK4819_WriteRegister(BK4819_REG_70, 0);
		BK4819_WriteRegister(BK4819_REG_58, 0);
	}
}


// -----------------------------------------------------

// Polynomial for CRC-8 (x^8 + x^2 + x + 1)
#define CRC8_POLY			0x07

// Function to compute CRC-8
static uint8_t crc8_compute(const uint8_t *data, const size_t len) {
    uint8_t crc = 0x00; // Initial value
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void moveUP(char (*rxMessages)[MAX_RX_MSG_LENGTH + 2]) {
    // Shift existing lines up
    strcpy(rxMessages[0], rxMessages[1]);
	strcpy(rxMessages[1], rxMessages[2]);
	strcpy(rxMessages[2], rxMessages[3]);

    // Insert the new line at the last position
	memset(rxMessages[3], 0, sizeof(rxMessages[3]));
}


void DTMF_Send(const char txMessage[TX_MSG_LENGTH], bool bServiceMessage) {
	(void)bServiceMessage;
	if ( msgStatus != READY ) return;

	if ( strlen(txMessage) > 0 && (TX_freq_check(gCurrentVfo->pTX->Frequency) == 0) ) {
		/* TODO if txMessage contain invalid character */

		msgStatus = SENDING;
		memcpy(gDTMF_String,txMessage,15);

		RADIO_SetVfoState(VFO_STATE_NORMAL);
		BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);

		// mute the mic during TX
		gMuteMic = true;
		FUNCTION_Select(FUNCTION_TRANSMIT);
		SYSTEM_DelayMs(500);

		gDTMF_ReplyState=DTMF_REPLY_ANI;
		DTMF_Reply();

		SYSTEM_DelayMs(100);
		APP_EndTransmission(true);
		// this must be run after end of TX, otherwise radio will still TX transmit without even RED LED on
		FUNCTION_Select(FUNCTION_FOREGROUND);
		RADIO_SetVfoState(VFO_STATE_NORMAL);
		BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);

		msgStatus = READY;
	} else {
		AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
	}
}

uint16_t MSG_GetId()
{
	return base_id;
}

void MSG_SetId(const char name[8],const char id[8])
{
	uint8_t _number[8];

	memcpy(_number, id, 8);

	base_id = atoi(id);
	if (base_id == 0) {
		base_id = 1;
		snprintf((char*) _number, sizeof(_number), "1");
	}

	// save to EEPROM
	SETTINGS_SaveLogoInfo(name, (const char*) _number);
	// make sure in POWER_ON_DISPLAY_MODE_MESSAGE mode
	// 0E90..0E97
	EEPROM_ReadBuffer(0x0E90, _number, 8);
//	UART_printf("power on display=%x\n", _number[7]);
	_number[7] = POWER_ON_DISPLAY_MODE_MESSAGE;
	EEPROM_WriteBuffer(0x0E90, _number);
}

static void MSG_SendPacket()
{
	msgStatus = SENDING;

	RADIO_SetVfoState(VFO_STATE_NORMAL);
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
	BK4819_DisableDTMF();
	// mute the mic during TX
	gMuteMic = true;
	//RADIO_SetTxParameters();
	FUNCTION_Select(FUNCTION_TRANSMIT);
	SYSTEM_DelayMs(500);
//	BK4819_PlayRogerNormal(98);
//	SYSTEM_DelayMs(100);
	//BK4819_ExitTxMute();
	MSG_FSKSendData();
	SYSTEM_DelayMs(50);
	APP_EndTransmission(true);
	// this must be run after end of TX, otherwise radio will still TX transmit even without RED LED on
	FUNCTION_Select(FUNCTION_FOREGROUND);
	RADIO_SetVfoState(VFO_STATE_NORMAL);
	// disable mic mute after TX
	gMuteMic = false;
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
	MSG_EnableRX(true);

	msgStatus = READY;
}

#ifdef ENABLE_XMESH
static bool MSG_SendBuffer(const uint8_t index)
{
	if (index >= XMESH_BUFFER_SIZE)
		return false;
	if (msgStatus != READY)
		return false;
	if (FUNCTION_IsRx()) {
		UART_printf("[Err]Rx Blocked!\n");
		return false;
	}

	if ((TX_freq_check(gCurrentVfo->pTX->Frequency) == 0)
			&& strlen((const char*) xMeshBuffer[index].info.payload) > 0) {

		memset(msgFSKBuffer, 0, sizeof(msgFSKBuffer));
		// first 2 byte sync, message type
		msgFSKBuffer[0] = 'M';
		msgFSKBuffer[1] = 'S';
		memcpy(msgFSKBuffer + 2, xMeshBuffer[index].info.payload, TX_MSG_LENGTH);
		MeshHeader_t _h;
		memcpy(&_h, &xMeshBuffer[index].info.header, MSG_HEADER_LENGTH);
		//change sender id to base_id
		_h.sender_id = base_id;
		// make sure hop counter is above zero
		if (_h.hop_counter > 0)
			// counting down the hop counter in header
			_h.hop_counter--;
		memcpy(msgFSKBuffer + MAX_RX_MSG_LENGTH, &_h, MSG_HEADER_LENGTH);
		//update CRC
		_h.crc8 = msgFSKBuffer[MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH - 1] = crc8_compute(
				msgFSKBuffer, MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH - 1);

		MSG_SendPacket();

		moveUP(rxMessage);
		sprintf(rxMessage[3], "> %s", xMeshBuffer[index].info.payload);
		UART_printf("SMS%s\n",rxMessage[3]);
		memset(lastcMessage, 0, sizeof(lastcMessage));
		memcpy(lastcMessage, xMeshBuffer[index].info.payload, TX_MSG_LENGTH);
		cIndex = 0;
		prevKey = 0;
		prevLetter = 0;
		memset(cMessage, 0, sizeof(cMessage));


		return true;
	}
	else {
		AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
	}

	return false;
}


// all from FSK is save to buffer if message valid and buffer still available
static bool XMESH_AddBuffer(const char payload[TX_MSG_LENGTH], const MeshHeader_t header)
{
	const uint16_t rssi_reg67 = BK4819_ReadRegister(BK4819_REG_67) & 0x1FF;
	int16_t rssi_dBm = rssi_reg67 / 2 - 160;
	#ifdef ENABLE_MESSENGER_UART
	UART_printf("rssi<%ddBm\n", rssi_dBm);
	#endif


	uint8_t next_index = XMESH_INDEX(xMeshIndexHead, 1);
	// buffer full!!
	if (next_index == xMeshIndexTail) {
		UART_printf("[Err]Buffer full:T%d->H%d\n", xMeshIndexTail, xMeshIndexHead);
		return false;
	}
	else if (header.origin_id == base_id) {
		UART_printf("<<originated from here. msg ignored!\n");
		UART_printf("sender_id<%d\n", header.sender_id);
		/* TODO add checker that previous message has been sent */
		return false;
	}
	else {

		uint8_t buffP = xMeshIndexTail;
		/* TODO check there's already existed message; origin id & packet id is equal */
		while (buffP != xMeshIndexHead) {
			if (xMeshBuffer[buffP].info.header.origin_id == header.origin_id) {
				if (xMeshBuffer[buffP].info.header.packet_id == header.packet_id) {
					xMeshBuffer[buffP].state.rx_counter++;
					// add additional TX wait time
					if (xMeshBuffer[buffP].state.tx_time > 0) {
						xMeshBuffer[buffP].state.tx_time += 1000;
					}
					UART_printf("MSG existed[%d]!\n", xMeshBuffer[buffP].state.rx_counter);
					return false;
				}
			}
			buffP = XMESH_INDEX(buffP, 1);
		}

		// save to xMeshBuffer
		memcpy(xMeshBuffer[xMeshIndexHead].info.payload, payload, TX_MSG_LENGTH);
		memcpy(&xMeshBuffer[xMeshIndexHead].info.header, &header, MSG_HEADER_LENGTH);

		xMeshBuffer[xMeshIndexHead].state.rx_counter = 0;
		xMeshBuffer[xMeshIndexHead].state.rx_timestamp = Systick_Get10msTick();
		if (xMeshBuffer[xMeshIndexHead].info.header.hop_counter > 0) {
			/* TODO add random time based on RSSI */
			xMeshBuffer[xMeshIndexHead].state.tx_time = Systick_Get10msTick()
					+ TX_BASE_TIME_TO_SEND_10ms + (((base_id / 10) % 10) * 10)
					+ ((base_id % 10) * 20) + (rssi_dBm * 2);
		}
		else{
			// do not send; already the last hop
			xMeshBuffer[xMeshIndexHead].state.tx_time = 0;
		}

		xMeshIndexHead = XMESH_INDEX(xMeshIndexHead, 1);
		return true;
	}

	return false;
}
#endif // #ifdef ENABLE_XMESH

bool MSG_Send(const char txMessage[TX_MSG_LENGTH], const uint16_t destination) {

	if (msgStatus != READY)
		return false;

	if ( strlen(txMessage) > 0 && (TX_freq_check(gCurrentVfo->pTX->Frequency) == 0) ) {

		// next MSG_HEADER_LENGTH for header
		// [0..1] 	: destination ID
		// [2..3] 	: sender ID
		// [4..5] 	: origin ID
		// [6] 		: packet ID
		// [7]		: hop counter
		// [8]		: (reserved byte)
		// [9]		: CRC-8
		uint8_t headerMessage[MSG_HEADER_LENGTH];
		Union_2B u2b;
		u2b.u16 = destination;
		headerMessage[0] = u2b.u8[0];
		headerMessage[1] = u2b.u8[1];
		u2b.u16 = base_id;
		headerMessage[2] = u2b.u8[0];
		headerMessage[3] = u2b.u8[1];
		headerMessage[4] = u2b.u8[0];
		headerMessage[5] = u2b.u8[1];
		headerMessage[6] = xMeshSendPacketId++;
		headerMessage[7] = HOP_COUNTER_MAX;
		headerMessage[8] = headerMessage[9] = 0;

		memset(msgFSKBuffer, 0, sizeof(msgFSKBuffer));
		// first 2 byte sync, message type
		msgFSKBuffer[0] = 'M';
		msgFSKBuffer[1] = 'S';
		memcpy(msgFSKBuffer + 2, txMessage, TX_MSG_LENGTH);
		memcpy(msgFSKBuffer + MAX_RX_MSG_LENGTH, headerMessage, MSG_HEADER_LENGTH);
		msgFSKBuffer[MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH - 1] = headerMessage[9] = crc8_compute(
				msgFSKBuffer, MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH - 1);

		MSG_SendPacket();

		moveUP(rxMessage);
		sprintf(rxMessage[3], "> %s", txMessage);
		memset(lastcMessage, 0, sizeof(lastcMessage));
		memcpy(lastcMessage, txMessage, TX_MSG_LENGTH);
		cIndex = 0;
		prevKey = 0;
		prevLetter = 0;
		memset(cMessage, 0, sizeof(cMessage));

		return true;
	} else {
		AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
	}

	return false;
}

uint8_t validate_char( uint8_t rchar ) {
	if ( (rchar == 0x1b) || (rchar >= 32 && rchar <= 127) ) {
		return rchar;
	}
	return 32;
}

#ifdef ENABLE_XMESH
void MSG_StorePacket(const uint16_t interrupt_bits) {
	const bool rx_sync             = (interrupt_bits & BK4819_REG_02_FSK_RX_SYNC) ? true : false;
	const bool rx_fifo_almost_full = (interrupt_bits & BK4819_REG_02_FSK_FIFO_ALMOST_FULL) ? true : false;
	const bool rx_finished         = (interrupt_bits & BK4819_REG_02_FSK_RX_FINISHED) ? true : false;

	if (rx_sync) {
		gFSKWriteIndex = 0;
		memset(msgFSKBuffer, 0, sizeof(msgFSKBuffer));
		msgStatus = RECEIVING;
	}

	if (rx_fifo_almost_full && msgStatus == RECEIVING) {

		const uint16_t count = BK4819_ReadRegister(BK4819_REG_5E) & (7u << 0);  // almost full threshold
		for (uint16_t i = 0; i < count; i++) {
			const uint16_t word = BK4819_ReadRegister(BK4819_REG_5F);
			if (gFSKWriteIndex < sizeof(msgFSKBuffer))
				msgFSKBuffer[gFSKWriteIndex++] = (word >> 0) & 0xFF;
			if (gFSKWriteIndex < sizeof(msgFSKBuffer))
				msgFSKBuffer[gFSKWriteIndex++] = (word >> 8) & 0xFF;
		}

		SYSTEM_DelayMs(10);
	}

	if (rx_finished) {

		const uint16_t fsk_reg59 = BK4819_ReadRegister(BK4819_REG_59) & ~((1u << 15) | (1u << 14) | (1u << 12) | (1u << 11));

		BK4819_WriteRegister(BK4819_REG_59, (1u << 15) | (1u << 14) | fsk_reg59);
		BK4819_WriteRegister(BK4819_REG_59, (1u << 12) | fsk_reg59);
		msgStatus = READY;

		if (gFSKWriteIndex > 0) {

			uint8_t crc_val = crc8_compute(msgFSKBuffer, MSG_HEADER_LENGTH + MAX_RX_MSG_LENGTH - 1);

			if (msgFSKBuffer[0] == 'M' && msgFSKBuffer[1] == 'S') {

				char _payload[TX_MSG_LENGTH];
				MeshHeader_t _header;

				memset(_payload, 0, TX_MSG_LENGTH);
				snprintf(_payload, TX_MSG_LENGTH, "%s", &msgFSKBuffer[2]);
				memcpy(&_header, &msgFSKBuffer[MAX_RX_MSG_LENGTH], MSG_HEADER_LENGTH);

				// next MSG_HEADER_LENGTH for header
				// [0..1] 	: destination ID
				// [2..3] 	: sender ID
				// [4..5] 	: origin ID
				// [6] 		: packet ID
				// [7]		: hop counter
				// [8]		: (reserved byte)
				// [9]		: CRC-8
				UART_printf("header<%d,%d,%d,%d,%d,%d\n",
						_header.destination_id,
						_header.sender_id,
						_header.origin_id,
						_header.packet_id,
						_header.hop_counter,
						_header.crc8);

				/* TODO CRC check */
				if (crc_val == _header.crc8)
					UART_printf("CRC<VALID\n");
				else
					UART_printf("CRC<MISMATCH\n");

				if (XMESH_AddBuffer(_payload, _header)) {

					moveUP(rxMessage);
					snprintf(rxMessage[3], TX_MSG_LENGTH + 2, "< %s", &msgFSKBuffer[2]);
					if (crc_val != _header.crc8)
						rxMessage[3][0] = 'C';

					#ifdef ENABLE_MESSENGER_UART
					UART_printf("SMS%s\n", rxMessage[3]);
					#endif
				}
			}
			else {
				moveUP(rxMessage);
				snprintf(rxMessage[3], TX_MSG_LENGTH + 2, "? %s", &msgFSKBuffer[2]);
			}

			if ( gScreenToDisplay != DISPLAY_MSG ) {
				hasNewMessage = 1;
				gUpdateStatus = true;
				gUpdateDisplay = true;
				#ifdef ENABLE_MESSENGER_NOTIFICATION
				gPlayMSGRing = true;
				#endif
			}
			else {
				gUpdateDisplay = true;
			}
		}

		gFSKWriteIndex = 0;
		#ifdef ENABLE_MESSENGER_DELIVERY_NOTIFICATION
		// Transmit a message to the sender that we have received the message (Unless it's a service message)
		if (msgFSKBuffer[0] == 'M' && msgFSKBuffer[1] == 'S' && msgFSKBuffer[2] != 0x1b) {
			MSG_Send("\x1b\x1b\x1bRCVD", true);
		}
		#endif
	}
}
#else	//#ifdef ENABLE_XMESH
void MSG_StorePacket(const uint16_t interrupt_bits) {

	//const uint16_t rx_sync_flags   = BK4819_ReadRegister(BK4819_REG_0B);

	const bool rx_sync             = (interrupt_bits & BK4819_REG_02_FSK_RX_SYNC) ? true : false;
	const bool rx_fifo_almost_full = (interrupt_bits & BK4819_REG_02_FSK_FIFO_ALMOST_FULL) ? true : false;
	const bool rx_finished         = (interrupt_bits & BK4819_REG_02_FSK_RX_FINISHED) ? true : false;

	//UART_printf("\nMSG : S%i, F%i, E%i | %i", rx_sync, rx_fifo_almost_full, rx_finished, interrupt_bits);

//	/* trap dtmf tone here for real-time */
//	const bool drx_s_lost			= (interrupt_bits & BK4819_REG_02_SQUELCH_LOST) ? true : false;
//	const bool drx_s_found			= (interrupt_bits & BK4819_REG_02_SQUELCH_FOUND) ? true : false;
//	UART_printf("\nDTMF : L%i,F%i | %X", drx_s_lost,drx_s_found, interrupt_bits);
//	if((drx_s_lost==0)&&(drx_s_found==0)){
//		uint16_t dtmf_tone   = BK4819_ReadRegister(BK4819_REG_0B);
//		UART_printf("\nTONE:%d",(dtmf_tone>>8)&0x0F);
//	}

	if (rx_sync) {
		gFSKWriteIndex = 0;
		memset(msgFSKBuffer, 0, sizeof(msgFSKBuffer));
		msgStatus = RECEIVING;
	}

	if (rx_fifo_almost_full && msgStatus == RECEIVING) {

		const uint16_t count = BK4819_ReadRegister(BK4819_REG_5E) & (7u << 0);  // almost full threshold
		for (uint16_t i = 0; i < count; i++) {
			const uint16_t word = BK4819_ReadRegister(BK4819_REG_5F);
			if (gFSKWriteIndex < sizeof(msgFSKBuffer))
				msgFSKBuffer[gFSKWriteIndex++] = validate_char((word >> 0) & 0xff);
			if (gFSKWriteIndex < sizeof(msgFSKBuffer))
				msgFSKBuffer[gFSKWriteIndex++] = validate_char((word >> 8) & 0xff);
		}

		SYSTEM_DelayMs(10);

	}

	if (rx_finished) {

		const uint16_t fsk_reg59 = BK4819_ReadRegister(BK4819_REG_59) & ~((1u << 15) | (1u << 14) | (1u << 12) | (1u << 11));

		BK4819_WriteRegister(BK4819_REG_59, (1u << 15) | (1u << 14) | fsk_reg59);
		BK4819_WriteRegister(BK4819_REG_59, (1u << 12) | fsk_reg59);
		msgStatus = READY;

		if (gFSKWriteIndex > 2) {

			// If there's three 0x1b bytes, then it's a service message
			if (msgFSKBuffer[2] == 0x1b && msgFSKBuffer[3] == 0x1b && msgFSKBuffer[4] == 0x1b) {
			#ifdef ENABLE_MESSENGER_DELIVERY_NOTIFICATION
				// If the next 4 bytes are "RCVD", then it's a delivery notification
				if (msgFSKBuffer[5] == 'R' && msgFSKBuffer[6] == 'C' && msgFSKBuffer[7] == 'V' && msgFSKBuffer[8] == 'D') {
					UART_printf("SVC<RCPT\n");
					rxMessage[3][strlen(rxMessage[3])] = '+';
					gUpdateStatus = true;
					gUpdateDisplay = true;
				}
			#endif
			} else {
				moveUP(rxMessage);
				if (msgFSKBuffer[0] != 'M' || msgFSKBuffer[1] != 'S') {
					snprintf(rxMessage[3], TX_MSG_LENGTH + 2, "? unknown msg format!");
				}
				else
				{
					snprintf(rxMessage[3], TX_MSG_LENGTH + 2, "< %s", &msgFSKBuffer[2]);
					#ifdef ENABLE_MESSENGER_UART
					UART_printf("SMS%s\n", rxMessage[3]);
					#endif

					const uint16_t rssi_reg67 = BK4819_ReadRegister(BK4819_REG_67) & 0x1FF;
					int16_t rssi_dBm = rssi_reg67 / 2 - 160;
					UART_printf("rssi=%ddBm\n", rssi_dBm);
				}			

				if ( gScreenToDisplay != DISPLAY_MSG ) {
					hasNewMessage = 1;
					gUpdateStatus = true;
					gUpdateDisplay = true;
			#ifdef ENABLE_MESSENGER_NOTIFICATION
					gPlayMSGRing = true;
			#endif
				}
				else {
					gUpdateDisplay = true;
				}
			}
		}

		gFSKWriteIndex = 0;
		#ifdef ENABLE_MESSENGER_DELIVERY_NOTIFICATION		
		// Transmit a message to the sender that we have received the message (Unless it's a service message)
		if (msgFSKBuffer[0] == 'M' && msgFSKBuffer[1] == 'S' && msgFSKBuffer[2] != 0x1b) {
			MSG_Send("\x1b\x1b\x1bRCVD", true);
		}
		#endif
	}
}
#endif	//#ifdef ENABLE_XMESH

#ifdef ENABLE_XMESH

uint8_t XMESH_GetBufferAvailable()
{
	uint8_t len = 0, h = xMeshIndexTail;

	while (h != xMeshIndexHead) {
		len++;
		h = XMESH_INDEX(h, 1);
	}

	return (XMESH_BUFFER_SIZE - len);
}

void XMESH_INIT()
{
	for ( int i = 0; i < XMESH_BUFFER_SIZE; ++i ) {
		memset(&xMeshBuffer[i], 0, sizeof(MeshBuffer_t));
	}

	char _name[8], _number[8];
	SETTINGS_LoadLogoInfo(_name, _number);
	base_id = atoi(_number);
	if (gEeprom.POWER_ON_DISPLAY_MODE == POWER_ON_DISPLAY_MODE_VOLTAGE || base_id == 0) {
		base_id = 1;
		snprintf(_name, sizeof(_name), "net01");
		snprintf(_number, sizeof(_number), "%d", base_id);
		MSG_SetId(_name, _number);
	}
	else {
		UART_printf("net=%s:id=%d", _name, base_id);
	}
}

void XMESH_TimeSlice500ms()
{
//	static bool rx_state = false;
//
//	bool _rx_now = FUNCTION_IsRx();
//	if (rx_state != _rx_now) {
//		rx_state = _rx_now;
//		UART_printf("[%ums]rx_st=%d\n", Systick_Get10msTick(), rx_state);
//	}

	// check buffer expiration time
	if (xMeshIndexHead != xMeshIndexTail) {
		if (Systick_Get10msTick()>xMeshBuffer[xMeshIndexHead].state.rx_timestamp+RX_TIME_EXPIRATION_10ms) {
			xMeshIndexTail = XMESH_INDEX(xMeshIndexTail, 1);
		}
	}

	// check buffer time to send
	uint8_t index = xMeshIndexTail;
	while (index != xMeshIndexHead) {
		if (xMeshBuffer[index].state.tx_time > 0) {
			if (Systick_Get10msTick() >= xMeshBuffer[index].state.tx_time) {
				/* TODO send current buffer */
				if(MSG_SendBuffer(index)){
					xMeshBuffer[index].state.tx_time = 0;
				}
				break;
			}
		}
		index = XMESH_INDEX(index, 1);
	}
}

#endif

void MSG_Init() {
	memset(rxMessage, 0, sizeof(rxMessage));
	memset(cMessage, 0, sizeof(cMessage));
	memset(lastcMessage, 0, sizeof(lastcMessage));
	hasNewMessage = 0;
	msgStatus = READY;
	prevKey = 0;
    prevLetter = 0;
	cIndex = 0;
}

// ---------------------------------------------------------------------------------

void insertCharInMessage(uint8_t key) {
	if ( key == KEY_0 ) {
		if ( keyboardType == NUMERIC ) {
			cMessage[cIndex] = '0';
		} else {
			cMessage[cIndex] = ' ';
		}
		if ( cIndex < MAX_MSG_LENGTH ) {
			cIndex++;
		}
	} else if (prevKey == key)
	{
		cIndex = (cIndex > 0) ? cIndex - 1 : 0;
		if ( keyboardType == NUMERIC ) {
			cMessage[cIndex] = T9TableNum[key - 1][(++prevLetter) % numberOfNumsAssignedToKey[key - 1]];
		} else if ( keyboardType == LOWERCASE ) {
			cMessage[cIndex] = T9TableLow[key - 1][(++prevLetter) % numberOfLettersAssignedToKey[key - 1]];
		} else {
			cMessage[cIndex] = T9TableUp[key - 1][(++prevLetter) % numberOfLettersAssignedToKey[key - 1]];
		}
		if ( cIndex < MAX_MSG_LENGTH ) {
			cIndex++;
		}
	}
	else
	{
		prevLetter = 0;
		if ( cIndex >= MAX_MSG_LENGTH ) {
			cIndex = (cIndex > 0) ? cIndex - 1 : 0;
		}
		if ( keyboardType == NUMERIC ) {
			cMessage[cIndex] = T9TableNum[key - 1][prevLetter];
		} else if ( keyboardType == LOWERCASE ) {
			cMessage[cIndex] = T9TableLow[key - 1][prevLetter];
		} else {
			cMessage[cIndex] = T9TableUp[key - 1][prevLetter];
		}
		if ( cIndex < MAX_MSG_LENGTH ) {
			cIndex++;
		}

	}
	cMessage[cIndex] = '\0';
	if ( keyboardType == NUMERIC ) {
		prevKey = 0;
		prevLetter = 0;
	} else {
		prevKey = key;
	}
}

void processBackspace() {
	cIndex = (cIndex > 0) ? cIndex - 1 : 0;
	cMessage[cIndex] = '\0';
	prevKey = 0;
    prevLetter = 0;
}

void  MSG_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld) {

	if (bKeyPressed && bKeyHeld) {
		switch (Key)
		{
			case KEY_F:
				if (gEeprom.KEY_LOCK && gKeypadLocked > 0) {
		 			COMMON_KeypadLockToggle();
				} else {
					MSG_Init();
				}
				break;
			default:
				AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
				break;
		}

	} else if (bKeyPressed && !bKeyHeld) {

		switch (Key)
		{
			case KEY_0:
			case KEY_1:
			case KEY_2:
			case KEY_3:
			case KEY_4:
			case KEY_5:
			case KEY_6:
			case KEY_7:
			case KEY_8:
			case KEY_9:
				if ( keyTickCounter > NEXT_CHAR_DELAY) {
					prevKey = 0;
    				prevLetter = 0;
				}
				insertCharInMessage(Key);
				keyTickCounter = 0;
				break;
			case KEY_STAR:
				keyboardType = (KeyboardType)((keyboardType + 1) % END_TYPE_KBRD);
				break;
			case KEY_F:
				processBackspace();
				break;
			case KEY_UP:
				memset(cMessage, 0, sizeof(cMessage));
				memcpy(cMessage, lastcMessage, TX_MSG_LENGTH);
				cIndex = strlen(cMessage);
				break;
			/*case KEY_DOWN:
				break;*/
			case KEY_MENU:
			case KEY_PTT:
				// Send message
				MSG_Send(cMessage, 0);
				break;
			case KEY_EXIT:
				gRequestDisplayScreen = DISPLAY_MAIN;
				break;

			default:
				AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
				break;
		}

	}

}


#endif
