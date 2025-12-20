/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include <string.h>

#if !defined(ENABLE_OVERLAY)
	#include "ARMCM0.h"
#endif
#ifdef ENABLE_FMRADIO
	#include "app/fm.h"
#endif
#if defined(ENABLE_MESSENGER) || defined(ENABLE_MESSENGER_UART)
	#include <strings.h>
	#include <stdlib.h>
	#include "app/messenger.h"
  	#include "external/printf/printf.h"
#endif
#include "app/uart.h"
#include "bsp/dp32g030/dma.h"
#include "driver/uart.h"
#include "misc.h"
#include "settings.h"
#include "ui/ui.h"		//change frequency
#include "common.h"		//change channel
#include "driver/system.h"

#if defined(ENABLE_OVERLAY)
	#include "sram-overlay.h"
#endif

#ifdef ENABLE_SCREEN_DUMP
	#include "driver/keyboard.h"
	#include "driver/st7565.h"
#endif

#define DMA_INDEX(x, y) (((x) + (y)) % sizeof(UART_DMA_Buffer))
#define COMMAND_MAX_BUFSIZE				(TX_MSG_LENGTH + 16)
const uint16_t RX_DMA_LENGTH = sizeof(UART_DMA_Buffer);
char gCmdMessage[COMMAND_MAX_BUFSIZE];

static bool UART_Change_Frequency_Command(const char *message)
{
	// get new frequency
	if (strlen(message) == 6) {
		char freq_input[6];
		uint8_t _i_freq_count=0;
		while(_i_freq_count<6){
			if ((message[_i_freq_count] >= '0')
					|| (message[_i_freq_count] <= '9')) {

				freq_input[_i_freq_count] =
						message[_i_freq_count];
				_i_freq_count++;
			}
			else
				break;
		}

		const uint8_t Vfo = gEeprom.TX_VFO;
		// user is entering a frequency
		if ((_i_freq_count >= 6) && IS_FREQ_CHANNEL(gTxVfo->CHANNEL_SAVE))
		{
			uint32_t Frequency = StrToUL(freq_input) * 100;
//			UART_printf("\nfr=%d", Frequency);

			// clamp the frequency entered to some valid value
			if (Frequency < frequencyBandTable[0].lower) {
				Frequency = frequencyBandTable[0].lower;
			}
			else if (Frequency >= BX4819_band1.upper && Frequency < BX4819_band2.lower) {
				const uint32_t center = (BX4819_band1.upper + BX4819_band2.lower) / 2;
				Frequency = (Frequency < center) ? BX4819_band1.upper : BX4819_band2.lower;
			}
			else if (Frequency > frequencyBandTable[BAND_N_ELEM - 1].upper) {
				Frequency = frequencyBandTable[BAND_N_ELEM - 1].upper;
			}

			const FREQUENCY_Band_t band = FREQUENCY_GetBand(Frequency);
			if (gTxVfo->Band != band) {
				gTxVfo->Band               = band;
				gEeprom.ScreenChannel[Vfo] = band + FREQ_CHANNEL_FIRST;
				gEeprom.FreqChannel[Vfo]   = band + FREQ_CHANNEL_FIRST;

				SETTINGS_SaveVfoIndices();

				RADIO_ConfigureChannel(Vfo, VFO_CONFIGURE_RELOAD);
			}

			Frequency = FREQUENCY_RoundToStep(Frequency, gTxVfo->StepFrequency);

			if (Frequency >= BX4819_band1.upper && Frequency < BX4819_band2.lower)
			{	// clamp the frequency to the limit
				const uint32_t center = (BX4819_band1.upper + BX4819_band2.lower) / 2;
				Frequency = (Frequency < center) ? BX4819_band1.upper - gTxVfo->StepFrequency : BX4819_band2.lower;
			}

			gTxVfo->freq_config_RX.Frequency = Frequency;
			gTxVfo->freq_config_TX.Frequency = Frequency;

			gRequestDisplayScreen = DISPLAY_MAIN;
			gUpdateDisplay = 1;
			return true;
		}
	}

	return false;
}

static void _id_parse(const char *str, char *name, char *id)
{
	const char *start = str;
	const char *p = str;

	while (*p) {
		if (*p == ',') {
			snprintf(name, 8, "%.*s", (int) (p - start), start);
//			UART_printf("name=%.*s=%s\n", (int) (p - start), start, name);
			start = p + 1;	// move past comma
		}
		p++;
	}
	// last word
	if(p>start){
		snprintf(id, 8, "%.*s", (int) (p - start), start);
//		UART_printf("id=%.*s=%s\n", (int) (p - start), start, id);
	}
}

bool UART_IsCommandAvailable(void)
{
	static uint16_t uart_rx_index = 0;
	uint16_t DmaIndex = DMA_CH0->ST & 0xFFFU;
	static uint8_t waitDelay10msCounter = 0;
	char _c[2] = { 0, 0 };

	if (uart_rx_index == DmaIndex) {
		waitDelay10msCounter = 0;
		return false;
	}

	waitDelay10msCounter++;

	if (waitDelay10msCounter > 10) {
		waitDelay10msCounter = 0;

//		UART_printf("index=%d,%d,%d\n", uart_rx_index, DmaIndex,
//				(DmaIndex > uart_rx_index) ?
//						(int) (DmaIndex - uart_rx_index) :
//						(int) (DmaIndex + sizeof(UART_DMA_Buffer) - uart_rx_index));
		memset(gCmdMessage, 0, sizeof(gCmdMessage));
		while (uart_rx_index != DmaIndex) {
			if ((UART_DMA_Buffer[uart_rx_index] == '\r')
					|| (UART_DMA_Buffer[uart_rx_index] == '\n')
					|| (UART_DMA_Buffer[uart_rx_index] == '\0')) {

				uart_rx_index = DmaIndex;
				return true;
			}
			_c[0] = UART_DMA_Buffer[uart_rx_index];
			strcat(gCmdMessage, _c);
			uart_rx_index = DMA_INDEX(uart_rx_index, 1);
		}
	}

	return false;
}

void UART_HandleCommand(void)
{
	char *tmp;
	UART_printf("cmd[%dB]=%s\n", strlen(gCmdMessage), gCmdMessage);

	if (strncmp(gCmdMessage, "SMS:", 4) == 0) {
		tmp = gCmdMessage + 4;
//		UART_printf("s[%dB]:%s\n", strlen(tmp), tmp);

		char *payload = tmp;
		char dest_id[8] = { 0 };
		uint8_t comma_pos = 0;
		uint16_t dID = 0;
		bool comma_found = false;
		char c[2] = {0,0};

		for ( comma_pos = 0; comma_pos < strlen(tmp); comma_pos++ ) {
			// comma_pos exceed dest_id length
			if (comma_pos >= 8)
				break;
			c[0] = *(tmp + comma_pos);
			if (c[0] == ',') {
				comma_found = true;
//				UART_printf("destID=%d\n", dest_id);
				break;
			}
			strcat(dest_id, c);
		}

		if (comma_found) {
//			UART_printf("offset=%d\n", comma_pos + 1);
			payload = tmp + comma_pos + 1;
			dID = atoi(dest_id);
		}

//		UART_printf("%s=%d,%s\n", tmp, dID, payload);

		if (strlen(payload) > 0) {
			if (!MSG_Send(payload, strlen(payload), dID)) {
				UART_printf("in RX state!\n");
			}
			UART_printf("SMS>%s\r\n", payload);
			gUpdateDisplay = true;
		}
	}
	else if (strncmp(gCmdMessage, "FREQ:", 5) == 0) {
		tmp = gCmdMessage + 5;
		if (strlen(tmp) > 0) {
			if (UART_Change_Frequency_Command(tmp)) {
				UART_printf("\nFREQ>%d\n", gTxVfo->freq_config_TX.Frequency);
			}
		}
	}
	else if (strncmp(gCmdMessage, "DTMF:", 5) == 0) {
		tmp = gCmdMessage + 5;
		if (strlen(tmp) > 0) {
			DTMF_Send(tmp, false);
			UART_printf("\nDTMF>%s\n", tmp);
			gUpdateDisplay = true;
		}
	}
	else if (strncmp(gCmdMessage, "SID:", 4) == 0) {
		tmp = gCmdMessage + 4;
		char _name[8], _number[8];

		if (strlen(tmp) > 0) {
			_id_parse(tmp, _name, _number);
			UART_printf("SID:%s->%s,%s\n", tmp, _name, _number);
			MSG_SetId(_name, _number);
			UART_printf("\nID>%d\n", MSG_GetId());
		}
	}
	else if (strncmp(gCmdMessage, "--id", 4) == 0) {
		UART_printf("\nID>%d\n", MSG_GetId());
	}
	else if(strncmp(gCmdMessage,"--buflen",8)==0){
		UART_printf("\nBuffer free,time: %d,%ums\n", XMESH_GetBufferAvailable(),
				Systick_Get10msTick() * 10);
	}
	else if (strncmp(gCmdMessage, "--help", 6) == 0) {
		UART_printf("available command:\n");
		UART_printf("SMS:123,payload\t\tsend to point 123 with \"payload\" data\n");
		UART_printf("DTMF:12345\t\tsend dtmf tone of \"12345\"\n");
		UART_printf("FREQ:145000\t\tset freq to 145000kHz\n");
		UART_printf("SID:net1,234\t\tSet to net name=net1 and id=123\n");
		UART_printf("--id\t\tGet ID of this device\n");
		UART_printf("--buflen\t\tGet meshtastic buffer available\n");
	}
}

