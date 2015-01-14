/*
 * uart.c
 *
 *  Created on: Dec 9, 2014
 *      Author: abamberger
 */

#include "hardware/uart.h"
#include "helpers/ringbuf.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

RingBuf rx_ringbuf;
RingBuf tx_ringbuf;
unsigned char rx_buffer[BUFFER_SIZE];
unsigned char tx_buffer[BUFFER_SIZE];

void init_uart()
{
    // Initialize the rx and tx ring buffers
    InitRingBuf(&rx_ringbuf, rx_buffer, BUFFER_SIZE);
    InitRingBuf(&tx_ringbuf, tx_buffer, BUFFER_SIZE);

    // Configure the character format, protocol, and communications mode
    UART_SCI_PORT.SCICCR.bit.STOPBITS = 0; // One stop bit
    UART_SCI_PORT.SCICCR.bit.PARITYENA = 0; // Disable parity
    UART_SCI_PORT.SCICCR.bit.LOOPBKENA = 0; // Disable loopback test mode
    UART_SCI_PORT.SCICCR.bit.ADDRIDLE_MODE = 0; // Set idle-line mode for RS-232 compatibility
    UART_SCI_PORT.SCICCR.bit.SCICHAR = 0x7; // Select 8-bit character length

    // Enable the transmitter and receiver
    UART_SCI_PORT.SCICTL1.bit.RXENA = 1;
    UART_SCI_PORT.SCICTL1.bit.TXENA = 1;

    // Set initial baud rate to 115200
    // Baud Rate Register = (LSPCLK (20 MHz) / (Baud Rate * 8)) - 1
    // For 115200, BRR = 20.701, set BRR to 21 for 113636 effective baud rate, for 1.3% deviation from nominal baud rate
    UART_SCI_PORT.SCIHBAUD = 0;
    UART_SCI_PORT.SCILBAUD = 21;

    // Configure SCI peripheral to free-run when the processor is suspended (debugging at a breakpoint)
    UART_SCI_PORT.SCIPRI.bit.SOFT = 0;
    UART_SCI_PORT.SCIPRI.bit.FREE = 1;

    // Configure the transmit and receive FIFOs
    UART_SCI_PORT.SCIFFTX.bit.SCIRST = 0; // Reset the SCI transmit and receive channels
    UART_SCI_PORT.SCIFFTX.bit.SCIFFENA = 1; // Enable FIFO module
    UART_SCI_PORT.SCIFFTX.bit.TXFIFOXRESET = 0; // Reset the transmit FIFO to clear any junk in it before we start
    UART_SCI_PORT.SCIFFTX.bit.TXFIFOXRESET = 1; // Enable transmit FIFO operation
    UART_SCI_PORT.SCIFFTX.bit.TXFFINT = 1; // Clear the transmit FIFO int flag if it is set
    UART_SCI_PORT.SCIFFTX.bit.TXFFIENA = 0; // Disable the transmit interrupt for now.  It will be re-enabled when there's something to send
    UART_SCI_PORT.SCIFFTX.bit.TXFFIL = 0; // Configure tx FIFO to generate interrupts when the tx FIFO is empty

    UART_SCI_PORT.SCIFFRX.bit.RXFFOVRCLR = 1; // Clear the rx overflow flag if it is set
    UART_SCI_PORT.SCIFFRX.bit.RXFIFORESET = 0; // Reset the receive FIFO to clear any junk in it before we start
    UART_SCI_PORT.SCIFFRX.bit.RXFIFORESET = 1; // Enable receive FIFO operation
    UART_SCI_PORT.SCIFFRX.bit.RXFFINTCLR = 1; // Clear the receive FIFO int flag if it is set
    UART_SCI_PORT.SCIFFRX.bit.RXFFIL = 0x1; // Configure rx FIFO to generate interrupts when it is has 1 character in it
                                            // This doesn't really use the FIFO as a FIFO, but if we use more than 1 level
                                            // of the receive FIFO, there needs to be some kind of periodic background task
                                            // running that periodically flushes the FIFO, so that characters don't get stuck
                                            // in it.  For now, I mostly want to use the TX FIFO to lower the number of TX interrupts
                                            // generated, so I'm just bypassing the functionality of the RX FIFO for now
    UART_SCI_PORT.SCIFFRX.bit.RXFFIENA = 1; // Enable the FIFO receive interrupt

    UART_SCI_PORT.SCIFFCT.bit.FFTXDLY = 0; // Set FIFO transfer delay to 0

    // Enable FIFO operation
    UART_SCI_PORT.SCIFFTX.bit.SCIRST = 1;

    // Enable the SCI module
    UART_SCI_PORT.SCICTL1.bit.SWRESET = 1;
}

int uart_chars_available()
{
    return rx_ringbuf.size(&rx_ringbuf);
}

unsigned char uart_get_char()
{
    return rx_ringbuf.pop(&rx_ringbuf);
}

int uart_read_available_chars(char* buffer, int buffer_size)
{
    int chars_read = 0;
    while (rx_ringbuf.size(&rx_ringbuf) > 0) {
        buffer[chars_read++] = rx_ringbuf.pop(&rx_ringbuf);

        // Don't overflow the buffer the caller has provided to us
        if (chars_read >= buffer_size) {
            return chars_read;
        }
    }

    return chars_read;
}

void uart_printf(const char* format, ...)
{
    // Format the string into the format buffer
    static char buffer[UART_STRING_LIMIT + 1];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buffer, UART_STRING_LIMIT, format, ap);
    va_end(ap);

    // Transmit the formatted string
    int string_len = strlen(buffer);
    uart_send_data((Uint8*)buffer, string_len);

    /*
    int txbuf_start_size = tx_ringbuf.size(&tx_ringbuf);
    int i;
    for (i = 0; i < string_len; ++i) {
        tx_ringbuf.push(&tx_ringbuf, buffer[i]);
    }

    // If the transmit ring buffer was empty to begin with, enable the transmit interrupt
    // (this will start copying the contents of the transmit ring buffer into the transmit FIFO)
    if (txbuf_start_size == 0) {
        UART_SCI_PORT.SCIFFTX.bit.TXFFIENA = 1;
    }
    */

    return;
}

void uart_send_data(Uint8* data, int length)
{
    int txbuf_start_size = tx_ringbuf.size(&tx_ringbuf);
    int i;
    for (i = 0; i < length; i++) {
        tx_ringbuf.push(&tx_ringbuf, data[i]);
    }

    // If the transmit ring buffer was empty to begin with, enable the transmit interrupt
    // (this will start copying the contents of the transmit ring buffer into the transmit FIFO)
    if (txbuf_start_size == 0) {
        UART_SCI_PORT.SCIFFTX.bit.TXFFIENA = 1;
    }
}

interrupt void uart_tx_isr(void)
{
    // Attempt to load up to 4 bytes into the TX FIFO
    int i;
    for (i = 0; i < 4; i++) {
        if ((tx_ringbuf.size(&tx_ringbuf) > 0)&&(UART_SCI_PORT.SCICTL2.bit.TXRDY)) {
            UART_SCI_PORT.SCITXBUF = tx_ringbuf.pop(&tx_ringbuf);
            //while (!UART_SCI_PORT.SCICTL2.bit.TXRDY)            {}
        }
    }

    // If we've emptied the transmit ring buffer, turn off the transmit interrupt (it will be re-enabled when there is more data to send)
    // otherwise, leave it on so we're interrupted when the current transmission has finished and can continue emptying the transmit ring buffer
    if (tx_ringbuf.size(&tx_ringbuf) == 0) {
        UART_SCI_PORT.SCIFFTX.bit.TXFFIENA = 0;
    }

    // Clear the transmit interrupt flag
    UART_SCI_PORT.SCIFFTX.bit.TXFFINTCLR = 1;

    // Acknowledge CPU interrupt to receive more interrupts from PIE group 9
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

interrupt void uart_rx_isr(void)
{
    // Empty the FIFO into the receive ring buffer
    while (UART_SCI_PORT.SCIFFRX.bit.RXFFST > 0) {
        rx_ringbuf.push(&rx_ringbuf, UART_SCI_PORT.SCIRXBUF.bit.RXDT);
    }

    // Clear the overflow flag if it is set
    // TODO: Handle this condition instead of just clearing it
    if (UART_SCI_PORT.SCIFFRX.bit.RXFFOVF) {
        UART_SCI_PORT.SCIFFRX.bit.RXFFOVRCLR = 1;
    }

    // Clear the receive interrupt flag
    UART_SCI_PORT.SCIFFRX.bit.RXFFINTCLR = 1;

    // Acknowledge CPU interrupt to receive more interrupts from PIE group 9
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
