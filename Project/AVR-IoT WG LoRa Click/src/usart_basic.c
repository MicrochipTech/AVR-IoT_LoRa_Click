/**
 * \file
 *
 * \brief USART basic driver.
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

/**
 * \defgroup doc_driver_usart_basic USART Basic
 * \ingroup doc_driver_usart
 *
 * \section doc_driver_usart_basic_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */
#include <compiler.h>
#include <clock_config.h>
#include <usart_basic.h>
#include <atomic.h>

/* Static Variables holding the ringbuffer used in IRQ mode */
static uint8_t          LoRa_rxbuf[LoRa_RX_BUFFER_SIZE];
static volatile uint8_t LoRa_rx_head;
static volatile uint8_t LoRa_rx_tail;
static volatile uint8_t LoRa_rx_elements;
static uint8_t          LoRa_txbuf[LoRa_TX_BUFFER_SIZE];
static volatile uint8_t LoRa_tx_head;
static volatile uint8_t LoRa_tx_tail;
static volatile uint8_t LoRa_tx_elements;

void LoRa_default_rx_isr_cb(void);
void (*LoRa_rx_isr_cb)(void) = &LoRa_default_rx_isr_cb;
void LoRa_default_udre_isr_cb(void);
void (*LoRa_udre_isr_cb)(void) = &LoRa_default_udre_isr_cb;

void LoRa_default_rx_isr_cb(void)
{
	uint8_t data;
	uint8_t tmphead;

	/* Read the received data */
	data = USART1.RXDATAL;
	/* Calculate buffer index */
	tmphead = (LoRa_rx_head + 1) & LoRa_RX_BUFFER_MASK;

	if (tmphead == LoRa_rx_tail) {
		/* ERROR! Receive buffer overflow */
	} else {
		/* Store new index */
		LoRa_rx_head = tmphead;

		/* Store received data in buffer */
		LoRa_rxbuf[tmphead] = data;
		LoRa_rx_elements++;
	}
}

void LoRa_default_udre_isr_cb(void)
{
	uint8_t tmptail;

	/* Check if all data is transmitted */
	if (LoRa_tx_elements != 0) {
		/* Calculate buffer index */
		tmptail = (LoRa_tx_tail + 1) & LoRa_TX_BUFFER_MASK;
		/* Store new index */
		LoRa_tx_tail = tmptail;
		/* Start transmission */
		USART1.TXDATAL = LoRa_txbuf[tmptail];
		LoRa_tx_elements--;
	}

	if (LoRa_tx_elements == 0) {
		/* Disable UDRE interrupt */
		USART1.CTRLA &= ~(1 << USART_DREIE_bp);
	}
}

/**
 * \brief Set call back function for LoRa
 *
 * \param[in] cb The call back function to set
 *
 * \param[in] type The type of ISR to be set
 *
 * \return Nothing
 */
void LoRa_set_ISR_cb(usart_cb_t cb, usart_cb_type_t type)
{
	switch (type) {
	case RX_CB:
		LoRa_rx_isr_cb = cb;
		break;
	case UDRE_CB:
		LoRa_udre_isr_cb = cb;
		break;
	default:
		// do nothing
		break;
	}
}

/* Interrupt service routine for RX complete */
ISR(USART1_RXC_vect)
{
	if (LoRa_rx_isr_cb != NULL)
		(*LoRa_rx_isr_cb)();
}

/* Interrupt service routine for Data Register Empty */
ISR(USART1_DRE_vect)
{
	if (LoRa_udre_isr_cb != NULL)
		(*LoRa_udre_isr_cb)();
}

bool LoRa_is_tx_ready()
{
	return (LoRa_tx_elements != LoRa_TX_BUFFER_SIZE);
}

bool LoRa_is_rx_ready()
{
	return (LoRa_rx_elements != 0);
}

bool LoRa_is_tx_busy()
{
	return (!(USART1.STATUS & USART_TXCIF_bm));
}

/**
 * \brief Read one character from LoRa
 *
 * Function will block if a character is not available.
 *
 * \return Data read from the LoRa module
 */
uint8_t LoRa_read(void)
{
	uint8_t tmptail;

	/* Wait for incoming data */
	while (LoRa_rx_elements == 0)
		;
	/* Calculate buffer index */
	tmptail = (LoRa_rx_tail + 1) & LoRa_RX_BUFFER_MASK;
	/* Store new index */
	LoRa_rx_tail = tmptail;
	ENTER_CRITICAL(R);
	LoRa_rx_elements--;
	EXIT_CRITICAL(R);

	/* Return data */
	return LoRa_rxbuf[tmptail];
}

/**
 * \brief Write one character to LoRa
 *
 * Function will block until a character can be accepted.
 *
 * \param[in] data The character to write to the USART
 *
 * \return Nothing
 */
void LoRa_write(const uint8_t data)
{
	uint8_t tmphead;

	/* Calculate buffer index */
	tmphead = (LoRa_tx_head + 1) & LoRa_TX_BUFFER_MASK;
	/* Wait for free space in buffer */
	while (LoRa_tx_elements == LoRa_TX_BUFFER_SIZE)
		;
	/* Store data in buffer */
	LoRa_txbuf[tmphead] = data;
	/* Store new index */
	LoRa_tx_head = tmphead;
	ENTER_CRITICAL(W);
	LoRa_tx_elements++;
	EXIT_CRITICAL(W);
	/* Enable UDRE interrupt */
	USART1.CTRLA |= (1 << USART_DREIE_bp);
}

/**
 * \brief Initialize USART interface
 * If module is configured to disabled state, the clock to the USART is disabled
 * if this is supported by the device's clock system.
 *
 * \return Initialization status.
 * \retval 0 the USART init was successful
 * \retval 1 the USART init was not successful
 */
int8_t LoRa_init()
{

	USART1.BAUD = (uint16_t)USART1_BAUD_RATE(57600); /* set baud rate register */

	USART1.CTRLA = 0 << USART_ABEIE_bp    /* Auto-baud Error Interrupt Enable: disabled */
	               | 0 << USART_DREIE_bp  /* Data Register Empty Interrupt Enable: disabled */
	               | 0 << USART_LBME_bp   /* Loop-back Mode Enable: disabled */
	               | USART_RS485_OFF_gc   /* RS485 Mode disabled */
	               | 1 << USART_RXCIE_bp  /* Receive Complete Interrupt Enable: enabled */
	               | 0 << USART_RXSIE_bp  /* Receiver Start Frame Interrupt Enable: disabled */
	               | 0 << USART_TXCIE_bp; /* Transmit Complete Interrupt Enable: disabled */

	USART1.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	               | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	               | 1 << USART_RXEN_bp     /* Reciever enable: enabled */
	               | USART_RXMODE_NORMAL_gc /* Normal mode */
	               | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	               | 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */

	// USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* Asynchronous Mode */
	//		 | USART_CHSIZE_8BIT_gc /* Character size: 8 bit */
	//		 | USART_PMODE_DISABLED_gc /* No Parity */
	//		 | USART_SBMODE_1BIT_gc; /* 1 stop bit */

	// USART1.DBGCTRL = 0 << USART_DBGRUN_bp; /* Debug Run: disabled */

	// USART1.EVCTRL = 0 << USART_IREI_bp; /* IrDA Event Input Enable: disabled */

	// USART1.RXPLCTRL = 0x0 << USART_RXPL_gp; /* Receiver Pulse Length: 0x0 */

	// USART1.TXPLCTRL = 0x0 << USART_TXPL_gp; /* Transmit pulse length: 0x0 */

	uint8_t x;

	/* Initialize ringbuffers */
	x = 0;

	LoRa_rx_tail     = x;
	LoRa_rx_head     = x;
	LoRa_rx_elements = x;
	LoRa_tx_tail     = x;
	LoRa_tx_head     = x;
	LoRa_tx_elements = x;

	return 0;
}

/**
 * \brief Enable RX and TX in LoRa
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX and TX enable-bits in the USART control register
 *
 * \return Nothing
 */
void LoRa_enable()
{
	USART1.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

/**
 * \brief Enable RX in LoRa
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX enable-bit in the USART control register
 *
 * \return Nothing
 */
void LoRa_enable_rx()
{
	USART1.CTRLB |= USART_RXEN_bm;
}

/**
 * \brief Enable TX in LoRa
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the TX enable-bit in the USART control register
 *
 * \return Nothing
 */
void LoRa_enable_tx()
{
	USART1.CTRLB |= USART_TXEN_bm;
}

/**
 * \brief Disable LoRa
 * 1. Disables the USART module by clearing the enable-bit(s) in the USART control register
 * 2. If supported by the clock system, disables the clock to the USART
 *
 * \return Nothing
 */
void LoRa_disable()
{
	USART1.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

/**
 * \brief Get recieved data from LoRa
 *
 * \return Data register from LoRa module
 */
uint8_t LoRa_get_data()
{
	return USART1.RXDATAL;
}

#include <stdio.h>

#if defined(__GNUC__)

int Terminal_printCHAR(char character, FILE *stream)
{
	Terminal_write(character);
	return 0;
}

FILE Terminal_stream = FDEV_SETUP_STREAM(Terminal_printCHAR, NULL, _FDEV_SETUP_WRITE);

#elif defined(__ICCAVR__)

int putchar(int outChar)
{
	USART_0_write(outChar);
	return outChar;
}
#endif

/* Static Variables holding the ringbuffer used in IRQ mode */
static uint8_t          Terminal_rxbuf[Terminal_RX_BUFFER_SIZE];
static volatile uint8_t Terminal_rx_head;
static volatile uint8_t Terminal_rx_tail;
static volatile uint8_t Terminal_rx_elements;
static uint8_t          Terminal_txbuf[Terminal_TX_BUFFER_SIZE];
static volatile uint8_t Terminal_tx_head;
static volatile uint8_t Terminal_tx_tail;
static volatile uint8_t Terminal_tx_elements;

void Terminal_default_rx_isr_cb(void);
void (*Terminal_rx_isr_cb)(void) = &Terminal_default_rx_isr_cb;
void Terminal_default_udre_isr_cb(void);
void (*Terminal_udre_isr_cb)(void) = &Terminal_default_udre_isr_cb;

void Terminal_default_rx_isr_cb(void)
{
	uint8_t data;
	uint8_t tmphead;

	/* Read the received data */
	data = USART2.RXDATAL;
	/* Calculate buffer index */
	tmphead = (Terminal_rx_head + 1) & Terminal_RX_BUFFER_MASK;

	if (tmphead == Terminal_rx_tail) {
		/* ERROR! Receive buffer overflow */
	} else {
		/* Store new index */
		Terminal_rx_head = tmphead;

		/* Store received data in buffer */
		Terminal_rxbuf[tmphead] = data;
		Terminal_rx_elements++;
	}
}

void Terminal_default_udre_isr_cb(void)
{
	uint8_t tmptail;

	/* Check if all data is transmitted */
	if (Terminal_tx_elements != 0) {
		/* Calculate buffer index */
		tmptail = (Terminal_tx_tail + 1) & Terminal_TX_BUFFER_MASK;
		/* Store new index */
		Terminal_tx_tail = tmptail;
		/* Start transmission */
		USART2.TXDATAL = Terminal_txbuf[tmptail];
		Terminal_tx_elements--;
	}

	if (Terminal_tx_elements == 0) {
		/* Disable UDRE interrupt */
		USART2.CTRLA &= ~(1 << USART_DREIE_bp);
	}
}

/**
 * \brief Set call back function for Terminal
 *
 * \param[in] cb The call back function to set
 *
 * \param[in] type The type of ISR to be set
 *
 * \return Nothing
 */
void Terminal_set_ISR_cb(usart_cb_t cb, usart_cb_type_t type)
{
	switch (type) {
	case RX_CB:
		Terminal_rx_isr_cb = cb;
		break;
	case UDRE_CB:
		Terminal_udre_isr_cb = cb;
		break;
	default:
		// do nothing
		break;
	}
}

/* Interrupt service routine for RX complete */
ISR(USART2_RXC_vect)
{
	if (Terminal_rx_isr_cb != NULL)
		(*Terminal_rx_isr_cb)();
}

/* Interrupt service routine for Data Register Empty */
ISR(USART2_DRE_vect)
{
	if (Terminal_udre_isr_cb != NULL)
		(*Terminal_udre_isr_cb)();
}

bool Terminal_is_tx_ready()
{
	return (Terminal_tx_elements != Terminal_TX_BUFFER_SIZE);
}

bool Terminal_is_rx_ready()
{
	return (Terminal_rx_elements != 0);
}

bool Terminal_is_tx_busy()
{
	return (!(USART2.STATUS & USART_TXCIF_bm));
}

/**
 * \brief Read one character from Terminal
 *
 * Function will block if a character is not available.
 *
 * \return Data read from the Terminal module
 */
uint8_t Terminal_read(void)
{
	uint8_t tmptail;

	/* Wait for incoming data */
	while (Terminal_rx_elements == 0)
		;
	/* Calculate buffer index */
	tmptail = (Terminal_rx_tail + 1) & Terminal_RX_BUFFER_MASK;
	/* Store new index */
	Terminal_rx_tail = tmptail;
	ENTER_CRITICAL(R);
	Terminal_rx_elements--;
	EXIT_CRITICAL(R);

	/* Return data */
	return Terminal_rxbuf[tmptail];
}

/**
 * \brief Write one character to Terminal
 *
 * Function will block until a character can be accepted.
 *
 * \param[in] data The character to write to the USART
 *
 * \return Nothing
 */
void Terminal_write(const uint8_t data)
{
	uint8_t tmphead;

	/* Calculate buffer index */
	tmphead = (Terminal_tx_head + 1) & Terminal_TX_BUFFER_MASK;
	/* Wait for free space in buffer */
	while (Terminal_tx_elements == Terminal_TX_BUFFER_SIZE)
		;
	/* Store data in buffer */
	Terminal_txbuf[tmphead] = data;
	/* Store new index */
	Terminal_tx_head = tmphead;
	ENTER_CRITICAL(W);
	Terminal_tx_elements++;
	EXIT_CRITICAL(W);
	/* Enable UDRE interrupt */
	USART2.CTRLA |= (1 << USART_DREIE_bp);
}

/**
 * \brief Initialize USART interface
 * If module is configured to disabled state, the clock to the USART is disabled
 * if this is supported by the device's clock system.
 *
 * \return Initialization status.
 * \retval 0 the USART init was successful
 * \retval 1 the USART init was not successful
 */
int8_t Terminal_init()
{

	USART2.BAUD = (uint16_t)USART2_BAUD_RATE(9600); /* set baud rate register */

	USART2.CTRLA = 0 << USART_ABEIE_bp    /* Auto-baud Error Interrupt Enable: disabled */
	               | 0 << USART_DREIE_bp  /* Data Register Empty Interrupt Enable: disabled */
	               | 0 << USART_LBME_bp   /* Loop-back Mode Enable: disabled */
	               | USART_RS485_OFF_gc   /* RS485 Mode disabled */
	               | 1 << USART_RXCIE_bp  /* Receive Complete Interrupt Enable: enabled */
	               | 0 << USART_RXSIE_bp  /* Receiver Start Frame Interrupt Enable: disabled */
	               | 0 << USART_TXCIE_bp; /* Transmit Complete Interrupt Enable: disabled */

	USART2.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	               | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	               | 1 << USART_RXEN_bp     /* Reciever enable: enabled */
	               | USART_RXMODE_NORMAL_gc /* Normal mode */
	               | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	               | 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */

	// USART2.CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* Asynchronous Mode */
	//		 | USART_CHSIZE_8BIT_gc /* Character size: 8 bit */
	//		 | USART_PMODE_DISABLED_gc /* No Parity */
	//		 | USART_SBMODE_1BIT_gc; /* 1 stop bit */

	// USART2.DBGCTRL = 0 << USART_DBGRUN_bp; /* Debug Run: disabled */

	// USART2.EVCTRL = 0 << USART_IREI_bp; /* IrDA Event Input Enable: disabled */

	// USART2.RXPLCTRL = 0x0 << USART_RXPL_gp; /* Receiver Pulse Length: 0x0 */

	// USART2.TXPLCTRL = 0x0 << USART_TXPL_gp; /* Transmit pulse length: 0x0 */

	uint8_t x;

	/* Initialize ringbuffers */
	x = 0;

	Terminal_rx_tail     = x;
	Terminal_rx_head     = x;
	Terminal_rx_elements = x;
	Terminal_tx_tail     = x;
	Terminal_tx_head     = x;
	Terminal_tx_elements = x;

#if defined(__GNUC__)
	stdout = &Terminal_stream;
#endif

	return 0;
}

/**
 * \brief Enable RX and TX in Terminal
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX and TX enable-bits in the USART control register
 *
 * \return Nothing
 */
void Terminal_enable()
{
	USART2.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

/**
 * \brief Enable RX in Terminal
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX enable-bit in the USART control register
 *
 * \return Nothing
 */
void Terminal_enable_rx()
{
	USART2.CTRLB |= USART_RXEN_bm;
}

/**
 * \brief Enable TX in Terminal
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the TX enable-bit in the USART control register
 *
 * \return Nothing
 */
void Terminal_enable_tx()
{
	USART2.CTRLB |= USART_TXEN_bm;
}

/**
 * \brief Disable Terminal
 * 1. Disables the USART module by clearing the enable-bit(s) in the USART control register
 * 2. If supported by the clock system, disables the clock to the USART
 *
 * \return Nothing
 */
void Terminal_disable()
{
	USART2.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

/**
 * \brief Get recieved data from Terminal
 *
 * \return Data register from Terminal module
 */
uint8_t Terminal_get_data()
{
	return USART2.RXDATAL;
}
