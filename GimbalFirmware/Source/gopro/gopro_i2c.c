#include "gopro_i2c.h"
#include "gopro_interface.h"
#include "hardware/i2c.h"

#include <stdint.h>


static void gopro_i2c_isr(I2CAIntSrc int_src);
static void gopro_i2c_on_addressed();


void gopro_i2c_init()
{
    init_i2c(&gopro_i2c_isr);
    i2c_clr_scd();
}

bool gopro_i2c_in_progress()
{
    if (!i2c_get_bb() && i2c_get_scd()) {
        return false;
    }

    return true;
}

void gopro_i2c_send(const Uint8 *buf, Uint8 len)
{
    /*
     * Send pending data, and clear INTR line
     * since our request to be read has been acked.
     *
     * Size of message is count in size field of message, +1 for size byte
     */

    i2c_send_data(buf, len);
}

void gopro_i2c_isr(I2CAIntSrc int_src)
{
    /*
     * Called back from within the i2c interrupt.
     * Dispatch to the appropriate handler.
     */

    switch (int_src) {
    case I2C_INT_SRC_STOP_DETECTED:
        // polling on this in main thread for now
        break;

    case I2C_INT_SRC_ADDRESSED_AS_SLAVE:
        gopro_i2c_on_addressed();
        break;
    }
}

void gopro_i2c_on_addressed()
{
    /*
     * Called from ISR context when we're addressed as an i2c slave by the camera.
     */

    // must capture transaction direction before clearing stop condition
    bool addressed_as_slave_tx = i2c_get_sdir();

    gp_deassert_intr(); // we've been acknowledged, can deassert
    i2c_clr_scd();      // clear previous stop condition so we can be notified on the upcoming one

    // notify gp that we've been addressed
    gp_on_slave_address(addressed_as_slave_tx);
}

void gopro_i2c_on_timeout()
{
    /*
     * Called by our client when they've determined
     * that an operation has taken too long.
     */

    // could cancel transaction here,
    // nothing to do here for now
    i2c_clr_scd();
}
