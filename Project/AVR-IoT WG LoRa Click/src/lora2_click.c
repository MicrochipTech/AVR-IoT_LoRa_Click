/**
 * \file
 *
 *
 (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

#include "lora2_click.h"
#include "usart_basic.h"
#include <atmel_start_pins.h>
#include <driver_rn2903.h>
#include <clock_config.h>
#include <util/delay.h>

uint8_t LoRa2ResponseIndex                      = 0;
char    LoRa2ResponseBuffer[responseBufferSize] = {0};

//*********************************************************
//          ISR Call back Function
//*********************************************************
void LoRa2_CaptureReceivedMessage(void)
{
	uint8_t readByte = LoRa_get_data();
	if ((readByte != '\0') && (LoRa2ResponseIndex < responseBufferSize))
		LoRa2ResponseBuffer[LoRa2ResponseIndex++] = readByte;
}

//*********************************************************
//          Other Functions
//*********************************************************

void LoRa2_RegisterISRCallback(void)
{
	LoRa_set_ISR_cb(&LoRa2_CaptureReceivedMessage, RX_CB);
}

void LoRa2_ReadyReceiveBuffer(void)
{
	LoRa2ResponseIndex = 0;
	for (uint8_t position = 0; position < responseBufferSize; position++)
		LoRa2ResponseBuffer[position] = 0;
}

char *LoRa2_GetResponse(void)
{
	return LoRa2ResponseBuffer;
}

void LoRa2_blockingWait(uint16_t limit)
{
	for (uint16_t counter = 0; counter < limit; counter++)
		_delay_ms(15);
}
