/*
 * gyro.c
 *
 *  Created on: Sep 10, 2014
 *      Author: abamberger
 */
#include "hardware/gyro.h"
#include "F2806x_Examples.h" // For DELAY_US

#define INTER_COMMAND_DELAY 2

SpiPortDescriptor gyro_spi_desc = {
        &SpiaRegs,                              // SPI Control Regs
        19,                                     // Slave select GPIO number
        CLOCK_POLARITY_NORMAL,                  // Spi Clock Polarity
        CLOCK_PHASE_HALF_CYCLE_DELAY,           // Spi Clock Phase
        CHAR_LENGTH_16_BITS,                    // Spi Character Length
        19                                      // Baud rate configuration (1MHz (max supported by MPU-6K), Baud rate = LSPCLK / (baud_rate_configure + 1), LSPCLK = 20MHz)
};

void InitGyro()
{
    // Initialize the appropriate SPI peripheral to talk to the gyro
    InitSpiPort(&gyro_spi_desc);

    // Read the who am I register to see if we're talking to the MPU-6K successfully
    Uint8 who_am_i = SpiReadReg8Bit(&gyro_spi_desc, MPU_WHO_AM_I_REG);
    DELAY_US(INTER_COMMAND_DELAY);

    // Disable the I2C interface, enable the SPI interface, disable FIFO
    SpiWriteReg8Bit(&gyro_spi_desc, MPU_USER_CTRL_REG, FIFO_DISABLED | I2C_MASTER_DISABLED | I2C_INTERFACE_DISABLED);

    DELAY_US(INTER_COMMAND_DELAY);

    // Disable the fsync pin, configure the highest bandwidth filter
    SpiWriteReg8Bit(&gyro_spi_desc, MPU_CONFIG_REG, FSYNC_DISABLED | LPF_0);

    DELAY_US(INTER_COMMAND_DELAY);

    // Select gyro 500 deg/s full scale range
    SpiWriteReg8Bit(&gyro_spi_desc, MPU_GYRO_CONFIG_REG, GYRO_FS_500);

    DELAY_US(INTER_COMMAND_DELAY);

    // Configure interrupt
    SpiWriteReg8Bit(&gyro_spi_desc, MPU_INT_PIN_CFG_REG, INT_ACTIVE_LOW | INT_PUSH_PULL | INT_PULSE | INT_CLEAR_READ_ANY | FSYNC_INT_DISABLED | I2C_BYPASS_DISABLED);

    DELAY_US(INTER_COMMAND_DELAY);

    // Disable the temperature sensor
    SpiWriteReg8Bit(&gyro_spi_desc, MPU_PWR_MGMT_1_REG, TEMP_SENSE_DISABLED | CLK_SRC_PLL_GYRO_X);

    DELAY_US(INTER_COMMAND_DELAY);

    // Set the sample rate to 1KHz (based on an 8KHz gyro output rate, change this if that changes)
    // TODO: Temporarily changing this to 4 kHz for testing
    SpiWriteReg8Bit(&gyro_spi_desc, MPU_SMPRT_DIV_REG, 7); // Sample rate = Gyro Output Rate (8KHz) / (1 + Sample Rate Divider) = 1 KHz

    /*
    // Read current clock source.  If set to anything other than PLL_GYRO_X, set it to PLL_GYRO_X
    Uint8 pwrMgmt = SpiReadReg(MPU_PWR_MGMT_1_REG);
    DELAY_US(1);
    if ((pwrMgmt & CLK_SRC_MASK) != CLK_SRC_PLL_GYRO_X) {
        SpiWriteReg(MPU_PWR_MGMT_1_REG, (pwrMgmt & ~CLK_SRC_MASK) | CLK_SRC_PLL_GYRO_X);
    }
    */

    DELAY_US(INTER_COMMAND_DELAY);

    // Put the accelerometers into standby mode (aren't using them for now)
    SpiWriteReg8Bit(&gyro_spi_desc, MPU_PWR_MGMT_2_REG, ACCEL_X_STBY | ACCEL_Y_STBY | ACCEL_Z_STBY);

    DELAY_US(INTER_COMMAND_DELAY);

    // Enable interrupt
    SpiWriteReg8Bit(&gyro_spi_desc, MPU_INT_ENABLE_REG, DATA_READY_INT_ENABLED);

    DELAY_US(INTER_COMMAND_DELAY);

    // Read the who am I register to see if we're talking to the MPU-6K successfully
    who_am_i = SpiReadReg8Bit(&gyro_spi_desc, MPU_WHO_AM_I_REG);

    // Reconfigure the gyro SPI port to run at 5MHz.  The data result registers can be read at 20MHz, but the configuration registers
    // can only be read and written at 1MHz, so we do all configuration at 1MHz and raise the speed to 5MHz after we're done, so subsequent data
    // reads complete much faster.  5MHz is the limit of the piccolo's SPI clock at the current peripheral clock settings
    ChangeSpiClockRate(&gyro_spi_desc, 0);

    return;
}

void ReadGyro(int16* gyro_x, int16* gyro_y, int16* gyro_z)
{
    Uint16 response1 = 0;
    Uint16 response2 = 0;
    Uint16 response3 = 0;
    Uint16 response4 = 0;

    // Take the slave select line low to begin the transaction
    // NOTE: It's important to read all of these registers in one transaction
    // to ensure that all data comes from the same sample (the MPU-6K guarantees
    // this by returning results from a shadow register set that is only updated
    // when the SPI interface is idle)
    SSAssert(&gyro_spi_desc);

    // Need at least 8ns set up time
    DELAY_US(1);

    // Perform the burst read of the gyro registers

    // Read gyro x high byte
    response1 = SpiSendRecvAddressedReg(&gyro_spi_desc, MPU_GYRO_XOUT_H_REG, 0x00, SPI_READ);

    // Read gyro x low byte and gyro y high byte
    response2 = SpiSendRecvAddressedReg(&gyro_spi_desc, 0x00, 0x00, SPI_READ);

    // Read gyro y low byte and gyro z high byte
    response3 = SpiSendRecvAddressedReg(&gyro_spi_desc, 0x00, 0x00, SPI_READ);

    // Read gyro z low byte (and 1 byte of garbage)
    response4 = SpiSendRecvAddressedReg(&gyro_spi_desc, 0x00, 0x00, SPI_READ);

    // Need at least 500ns hold time
    DELAY_US(1);

    // Take the slave select line high to complete the transaction
    SSDeassert(&gyro_spi_desc);

    // Unpack and return the results
    *gyro_x = (int16)(((response1 << 8) & 0xFF00) | ((response2 >> 8) & 0x00FF));
    *gyro_y = (int16)(((response2 << 8) & 0xFF00) | ((response3 >> 8) & 0x00FF));
    *gyro_z = (int16)(((response3 << 8) & 0xFF00) | ((response4 >> 8) & 0x00FF));

    return;
}

// Returns the x gyro high and low bytes, and the y gyro high byte
// packed into a 32-bit integer as follows: 0x00-x_high-x_low_y_high
Uint32 ReadGyroPass1()
{
	Uint32 response1 = 0;
	Uint32 response2 = 0;
	Uint32 assembled_response = 0;

	// Take the slave select line low to begin the transaction
	SSAssert(&gyro_spi_desc);

	// Need at least 8ns set up time
	DELAY_US(1);

	// Perform the burst read of the gyro registers

	// Read gyro x high byte
	response1 = SpiSendRecvAddressedReg(&gyro_spi_desc, MPU_GYRO_XOUT_H_REG, 0x00, SPI_READ);

	// Read gyro x low byte and gyro y high byte
	response2 = SpiSendRecvAddressedReg(&gyro_spi_desc, 0x00, 0x00, SPI_READ);

	// Need at least 500ns hold time
	DELAY_US(1);

	// Take the slave select line high to complete the transaction
	SSDeassert(&gyro_spi_desc);

	// Unpack and return the results
	assembled_response = ((response1 << 16) & 0x00FF0000) | (response2 & 0x0000FFFF);

	return assembled_response;
}

// Returns the y gyro low byte and z gyro low and high bytes,
// packed into a 32-bit integer as follows: 0x00-y_low-z_high_z_low
Uint32 ReadGyroPass2()
{
	Uint32 response1 = 0;
	Uint32 response2 = 0;
	Uint32 assembled_response = 0;

	// Take the slave select line low to begin the transaction
	SSAssert(&gyro_spi_desc);

	// Need at least 8ns set up time
	DELAY_US(1);

	// Perform the burst read of the gyro registers

	// Read gyro y low byte
	response1 = SpiSendRecvAddressedReg(&gyro_spi_desc, MPU_GYRO_YOUT_L_REG, 0x00, SPI_READ);

	// Read gyro z high byte and gyro z low byte
	response2 = SpiSendRecvAddressedReg(&gyro_spi_desc, 0x00, 0x00, SPI_READ);

	// Need at least 500ns hold time
	DELAY_US(1);

	// Take the slave select line high to complete the transaction
	SSDeassert(&gyro_spi_desc);

	// Unpack and return the results
	assembled_response = ((response1 << 16) & 0x00FF0000) | (response2 & 0x0000FFFF);

	return assembled_response;
}