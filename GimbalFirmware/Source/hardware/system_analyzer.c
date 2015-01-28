/*
 * system_analyzer.c
 *
 *  Created on: Nov 19, 2014
 *      Author: abamberger
 */

#include "hardware/system_analyzer.h"

SpiPortDescriptor sys_analyzer_desc = {
        &SpibRegs,                              // SPI Control Regs
        15,                                     // Slave select GPIO number
        CLOCK_POLARITY_NORMAL,                  // Spi Clock Polarity
        CLOCK_PHASE_NORMAL,                     // Spi Clock Phase
        CHAR_LENGTH_8_BITS,                     // Spi Character Length
        0                                       // Baud rate configuration (20MHz, Baud rate = LSPCLK / (baud_rate_configure + 1), LSPCLK = 20MHz)
};

void InitSystemAnalyzer()
{
    // Initialize the SPI port for 24 bit reads and writes.  The DAC uses 24-bit transactions, while
    // the ADC only uses 16-bit transactions, but we want to read and write in the same operation,
    // so we'll use a 24-bit transaction and throw away 8 bits of the value read
    InitSpiPort(&sys_analyzer_desc);
}

int16 SystemAnalyzerSendReceive(int16 data)
{
    // The incoming data is a signed 16-bit number, and the DAC outputs data in an unsigned 16-bit range,
    // so we shift the zero-centered incoming data up to be 0-64k mid-scale centered
    Uint16 corrected_data = ((int32)data) + 32768;

    Uint32 readVal = SpiSendRecv24Bit(&sys_analyzer_desc, corrected_data);

    // Since the DAC write is 24 bits, and the ADC read is only 16-bits, the received
    // ADC value has been shifted left by 8 bits, so undo that here
    return readVal >> 8;
}
