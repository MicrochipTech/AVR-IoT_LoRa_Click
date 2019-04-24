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

#include <stdio.h>
#include <string.h>
#include <click_example.h>
#include <clock_config.h>
#include <util/delay.h>
#include "lora2_click.h"
#include <driver_rn2903.h>

void LoRa2_Example(void) // Call After System Init; Above while(1) Loop in Main.c
{
	char  exampleReadStorage[responseBufferSize] = {0};
	char *exampleRead;
	LoRa2_RegisterISRCallback();
	rn2903_SetHardwareReset(false);
	LoRa2_blockingWait(2);
	rn2903_SetHardwareReset(true);
	LoRa2_blockingWait(400);
	exampleRead = LoRa2_GetResponse();
	strcpy(exampleReadStorage, exampleRead);
	LoRa2_ReadyReceiveBuffer();
	rn2903_SendString("sys factoryRESET");
	LoRa2_blockingWait(400);
	exampleRead = LoRa2_GetResponse(); // Read Version String from Buffer
	strcpy(exampleReadStorage, exampleRead);
}
