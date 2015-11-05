#include "f2806x_int8.h"
#include "hardware/i2c.h"

#include <stdlib.h>

#include "gopro_interface.h"

typedef struct {
    const uint8_t *p;
    uint16_t len;
} tx_transaction_t;

typedef struct {
    uint8_t *p;
    uint16_t len;
    uint16_t maxlen;
} rx_transaction_t;

static tx_transaction_t tx;
static rx_transaction_t rx = {
    .len = 0,
    .maxlen = 0
};

I2CIntACallback int_a_callback = NULL;

static void i2c_init_interrupts(void);

void init_i2c(I2CIntACallback interrupt_a_callback)
{
    // Initialize the rx and tx buffers
    tx.len = 0;
    rx.len = 0;

    // Register the interrupt A callback function
    int_a_callback = interrupt_a_callback;

    // Hold the I2C module in reset so we can configure it
    I2caRegs.I2CMDR.bit.IRS = 0;

    // Configure the I2C mode register
    I2caRegs.I2CMDR.bit.FREE = 0; //TODO: For testing, disable free run // Set I2C module to free run while the processor is halted on a breakpoint
    I2caRegs.I2CMDR.bit.MST = 0; // Configure us as an I2C slave
    I2caRegs.I2CMDR.bit.XA = 0; // Select 7-bit addressing mode
    I2caRegs.I2CMDR.bit.DLB = 0; // Disable on-chip loopback
    I2caRegs.I2CMDR.bit.FDF = 0; // Disable free data format
    I2caRegs.I2CMDR.bit.BC = 0x0; // 8 bits of data per transmitted byte

    // Configure I2C module interrupts
    I2caRegs.I2CIER.bit.AAS = 1; // Enable interrupts for when we're addressed as a slave
    I2caRegs.I2CIER.bit.SCD = 1; // Enable interrupt for Stop Condition Detected
    I2caRegs.I2CIER.bit.RRDY = 1; // Enable interrupts for when data is received
    I2caRegs.I2CIER.bit.XRDY = 0; // Disable the transmit interrupt for now.  It will be enabled when there is data to send

    // Configure the I2C module clock prescaler
    I2caRegs.I2CPSC.bit.IPSC = 6; // I2C module clock = CPU Clock / (IPSC + 1).  Per spec, I2C module clock must be between 7 and 12 MHz.
                                  // At CPU clock frequency of 80MHz, IPSC = 6 gives the highest possible module clock within the spec (11.429 MHz)

    // The GoPro expects the controller to be at address 0x60, so set that as our slave address
    I2caRegs.I2COAR = 0x0060;

    i2c_init_interrupts();

    // Enable the I2C module
    I2caRegs.I2CMDR.bit.IRS = 1;
}

static void i2c_init_interrupts(void)
{
    EALLOW;
    PieVectTable.I2CINT2A = &i2c_fifo_isr; // I2C Tx and Rx fifo interrupts are handled by the same ISR
    PieVectTable.I2CINT1A = &i2c_int_a_isr; // All non-fifo I2C interrupts are handled by the same ISR
    EDIS;

    // Enable PIE group 8 interrupt 2 for I2C FIFO interrupts
    PieCtrlRegs.PIEIER8.bit.INTx2 = 1;
    // Enable PIE group 8 interrupt 1 for Regular I2C interrupts (the only one we're currently using is addressed as slave (AAS))
    PieCtrlRegs.PIEIER8.bit.INTx1 = 1;

    // Enable CPU INT3 which is connected to EPWMx INTs:
    IER |= M_INT8;
}

Uint16 i2c_get_aas()
{
    return I2caRegs.I2CSTR.bit.AAS;
}

Uint16 i2c_get_sdir()
{
    return I2caRegs.I2CSTR.bit.SDIR;
}

Uint16 i2c_get_bb()
{
    return I2caRegs.I2CSTR.bit.BB;
}

Uint16 i2c_get_scd()
{
    return I2caRegs.I2CSTR.bit.SCD;
}

void i2c_clr_scd()
{
    I2caRegs.I2CSTR.bit.SCD = 1;
}

static void send_next_byte()
{
    /*
     * Helper to write the next byte
     * in the current tx transaction.
     */

    if (tx.len > 0) {
        I2caRegs.I2CDXR = *(tx.p);
        tx.p++;
        tx.len--;
    }
}

void i2c_begin_tx(const uint8_t* data, int length)
{
    /*
     * Kick off transmission of 'data'.
     *
     * Load the first byte, and enabled the XRDY isr
     * to get notified when we can write the next.
     */

    tx.p = data;
    tx.len = length;

    send_next_byte();
    I2caRegs.I2CIER.bit.XRDY = 1;
}

void i2c_begin_rx(uint8_t* data, int maxlen)
{
    /*
     * Prepare to receive into 'data'.
     */

    rx.p = data;
    rx.maxlen = maxlen;
    rx.len = 0;
}

int i2c_get_rx_len()
{
    return rx.len;
}

void i2c_enable_scd_isr()
{
    I2caRegs.I2CIER.bit.SCD = 1;
}

void i2c_disable_scd_isr()
{
    I2caRegs.I2CIER.bit.SCD = 0;
}

interrupt void i2c_fifo_isr(void)
{
    /*
     * It might be nice to some day use the fifo-based ISR for data transfer,
     * but for now we've been unable to get it to behave reliably.
     *
     * Specifically, the TX FIFO ISR does not appear to fire when configured
     * to fire upon the emptying of the TX fifo. Further, the TX FIFO INT pending
     * bit gets set diring *rx* operations. Why.
     *
     * For now, we do data transfer byte-wise, and this routine is really
     * only left around in case we ever need to investigate this functionality again.
     */

    // Acknowledge CPU interrupt to receive more interrupts from PIE group 8
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

interrupt void i2c_int_a_isr(void)
{
    I2CAIntSrc int_src = (I2CAIntSrc)I2caRegs.I2CISRC.bit.INTCODE;

    // We handle the receive ready and transmit ready interrupts here as a special case,
    // otherwise, we call the registered callback function
    switch (int_src) {
    case I2C_INT_SRC_RX_READY:
        if (rx.len < rx.maxlen) {
            *(rx.p) = I2caRegs.I2CDRR;
            rx.p++;
            rx.len++;
        }
        break;

    case I2C_INT_SRC_TX_READY:
        send_next_byte();
        if (tx.len == 0) {
            I2caRegs.I2CIER.bit.XRDY = 0;
        }
        break;

    case I2C_INT_SRC_REGS_READY:
        // According to the datasheet, reading the interrupt source register clears the corresponding interrupt
        // flag bit except for the register access ready, receive ready, and transmit ready interrupts.
        // The transmit ready flag is read-only and is cleared automatically when new data is loaded to be sent.
        // The receive ready flag is cleared automatically when the received data is read.  Therefore, we only
        // need to manually clear the register access ready flag here
        I2caRegs.I2CSTR.bit.ARDY = 1;
        break;

    default:
        // Check the pointer value has been initialised, as this interrupt may
        // be triggered after interrupts are enabled, but before i2c init has
        // been called, which may be never if GoPro control is disabled.
        if(int_a_callback != NULL) {
            // Call the callback with the value of the interrupt source register
            (*int_a_callback)(int_src);
        }
        break;
    }

    // Acknowledge CPU interrupt to receive more interrupts from PIE group 8
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}
