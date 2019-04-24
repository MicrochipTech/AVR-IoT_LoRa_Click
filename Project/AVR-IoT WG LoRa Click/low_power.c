/*
    \file   low_power.c

    \brief  low power handler source file.

    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/

#include "low_power.h"
#include "lora_handling.h"
#include "atmel_start_pins.h"
#include "clock_config.h"
#include "lora2_click.h"
#include "driver_rn2903.h"
#include "led.h"
#include "usart_basic.h"
#include <util/delay.h>
#include <avr/sleep.h>
#include <ccp.h>
#include <stdbool.h>

#define SHORT_DELAY                             2
#define MEDIUM_DELAY                            15
#define LONG_DELAY                              1000
#define BREAK_CONDITION                         0x00
#define WAKE_UP_STRING                          0x55
#define SLEEP_TIME                              "100000000"

static void initLowPower(void);
static void deinitLowPower(void);
static void tcbDisable(void);
static void tcbEnable(void);

bool has_entered_low_power_at_least_once = false;

extern void LORA_HANDLING_createCommand(const char* command, const char* value);

extern const char RN_cmd_sys_sleep[];
extern char response[];
extern char complete_command[]; 

static void initLowPower(void)
{
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB,
    CLKCTRL_PDIV_64X_gc /* 64 */
    | 1 << CLKCTRL_PEN_bp ); /* Prescaler enable: enabled */

    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA,
    CLKCTRL_CLKSEL_OSCULP32K_gc ); /* 32KHz Internal Ultra Low Power Oscillator (OSCULP32K) */

    /* wait for system oscillator changing to finish */
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm)
    {
        ;
    }
}

static void deinitLowPower(void)
{
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB,
    CLKCTRL_PDIV_4X_gc /* 4 */
    | 1 << CLKCTRL_PEN_bp ); /* Prescaler enable: enabled */

    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA,
    CLKCTRL_CLKSEL_OSC20M_gc ); /* 20MHz main oscillator */

    /* wait for system oscillator changing to finish */
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm)
    {
        ;
    }
}

void LOW_POWER_disableWINC(void)
{
    PF3_set_level(0);
}

static void tcbDisable(void)
{
    TCB0.CTRLA &= ~(1 << TCB_ENABLE_bp);
}

static void tcbEnable(void)
{
    TCB0.CTRLA |= (1 << TCB_ENABLE_bp);
}

void LOW_POWER_enterLowPower(void)
{
    LORA_HANDLING_createCommand(RN_cmd_sys_sleep, SLEEP_TIME);
    rn2903_SendString(complete_command);
    has_entered_low_power_at_least_once = true;
    LoRa2_ReadyReceiveBuffer();
    LED_YELLOW_set_level(LED_OFF);
    LoRa2_blockingWait(MEDIUM_DELAY);
    initLowPower();
    tcbEnable();
    sleep_mode();
}

void LOW_POWER_exitLowPower(void)
{
    tcbDisable();
    deinitLowPower();
    LED_YELLOW_set_level(LED_ON);
    if(has_entered_low_power_at_least_once == true)
    {
        LoRa_write(BREAK_CONDITION);
        LoRa_write(WAKE_UP_STRING);
    }
    LoRa2_blockingWait(MEDIUM_DELAY);
}
