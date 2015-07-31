#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"

void setup_serial_port()
{
	EALLOW;
	// Configure the character format, protocol, and communications mode
	ScibRegs.SCICCR.bit.STOPBITS = 0; // One stop bit
	ScibRegs.SCICCR.bit.PARITYENA = 0; // Disable parity
	ScibRegs.SCICCR.bit.LOOPBKENA = 0; // Disable loopback test mode
	ScibRegs.SCICCR.bit.ADDRIDLE_MODE = 0; // Set idle-line mode for RS-232 compatibility
	ScibRegs.SCICCR.bit.SCICHAR = 0x7; // Select 8-bit character length

	// Enable the transmitter and receiver
	ScibRegs.SCICTL1.bit.RXENA = 1;
	ScibRegs.SCICTL1.bit.TXENA = 1;

	/*
	// Set initial baud rate to 115200
	// Baud Rate Register = (LSPCLK (20 MHz) / (Baud Rate * 8)) - 1
	// For 115200, BRR = 20.701, set BRR to 21 for 113636 effective baud rate, for 1.3% deviation from nominal baud rate
	ScibRegs.SCIHBAUD = 0;
	ScibRegs.SCILBAUD = 21;*/

	// Set initial baud rate to 230400
	// For 230400, BRR = 9.851, set BRR to 10 for 227272 effective baud rate, for 1.36% deviation from nominal baud rate
	ScibRegs.SCIHBAUD = 0;
	ScibRegs.SCILBAUD = 10;

	/*// Set initial baud rate to 500000
	// For 500000, BRR = 4.0, set BRR to 4 for 500000 effective baud rate, for 0% deviation from nominal baud rate
	ScibRegs.SCIHBAUD = 0;
	ScibRegs.SCILBAUD = 4;*/

	// Configure SCI peripheral to free-run when the processor is suspended (debugging at a breakpoint)
	ScibRegs.SCIPRI.bit.SOFT = 0;
	ScibRegs.SCIPRI.bit.FREE = 1;

	// Configure the transmit and receive FIFOs
	ScibRegs.SCIFFTX.bit.SCIRST = 0; // Reset the SCI transmit and receive channels
	ScibRegs.SCIFFTX.bit.SCIFFENA = 1; // Enable FIFO module
	ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 0; // Reset the transmit FIFO to clear any junk in it before we start
	ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1; // Enable transmit FIFO operation
	ScibRegs.SCIFFTX.bit.TXFFINT = 1; // Clear the transmit FIFO int flag if it is set
	ScibRegs.SCIFFTX.bit.TXFFIENA = 0; // Disable the transmit interrupt for now.  It will be re-enabled when there's something to send
	ScibRegs.SCIFFTX.bit.TXFFIL = 0; // Configure tx FIFO to generate interrupts when the tx FIFO is empty

	ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1; // Clear the rx overflow flag if it is set
	ScibRegs.SCIFFRX.bit.RXFIFORESET = 0; // Reset the receive FIFO to clear any junk in it before we start
	ScibRegs.SCIFFRX.bit.RXFIFORESET = 1; // Enable receive FIFO operation
	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1; // Clear the receive FIFO int flag if it is set
	ScibRegs.SCIFFRX.bit.RXFFIL = 0x1; // Configure rx FIFO to generate interrupts when it is has 1 character in it
											// This doesn't really use the FIFO as a FIFO, but if we use more than 1 level
											// of the receive FIFO, there needs to be some kind of periodic background task
											// running that periodically flushes the FIFO, so that characters don't get stuck
											// in it.  For now, I mostly want to use the TX FIFO to lower the number of TX interrupts
											// generated, so I'm just bypassing the functionality of the RX FIFO for now
	ScibRegs.SCIFFRX.bit.RXFFIENA = 0; // Disable the FIFO receive interrupt

	ScibRegs.SCIFFCT.bit.FFTXDLY = 0; // Set FIFO transfer delay to 0

	// Enable FIFO operation
	ScibRegs.SCIFFTX.bit.SCIRST = 1;

	// Enable the SCI module
	ScibRegs.SCICTL1.bit.SWRESET = 1;
   EDIS;

}

int read_serial_port(unsigned char *data, unsigned int max_size)
{
	int bytes_read = 0;
	Uint16 timeout = 0;
	while (max_size > 0) {
		// Check whether this was an interrupt due to a received character or a receive error
		if (ScibRegs.SCIRXST.bit.RXERROR) {
			// This was an error interrupt

			// Reset the peripheral to clear the error condition
			ScibRegs.SCICTL1.bit.SWRESET = 0;
			ScibRegs.SCICTL1.bit.SWRESET = 1;
		} else {
			// Empty the FIFO into the receive ring buffer
			while ((ScibRegs.SCIFFRX.bit.RXFFST > 0)&&(max_size > 0)) {
				*data = ScibRegs.SCIRXBUF.bit.RXDT;
				data++;
				max_size--;
				bytes_read++;
				timeout = 0;
			}
		}

		// Clear the overflow flag if it is set
		// TODO: Handle this condition instead of just clearing it
		if (ScibRegs.SCIFFRX.bit.RXFFOVF) {
			ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
		}

		// @todo: handle timeout here
		// bytes_read == 0 --> timeout after 65535 cycles
		// bytes_read > 0  --> timeout after 256 cycles
		timeout++;
		if (((bytes_read == 0)&&(timeout >= 0xFFFF))||
			((bytes_read > 0)&&(timeout > 0x100))) {
			return bytes_read;
		}
	}

	return bytes_read;
}

int send_serial_port(unsigned char *data, unsigned int size)
{
	while (size > 0) {
		if (ScibRegs.SCIFFTX.bit.TXFFST < 4) {
			ScibRegs.SCITXBUF = *data;
			data++;
			size--;
		}
	}
	return 0;
}
