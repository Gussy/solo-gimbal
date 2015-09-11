#include "hardware/device_init.h"
#include "hardware/interrupts.h"

// The following includes contain funtions that are configured as ISR
#include "PM_Sensorless.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/led.h"
#include "can/cand.h"


void InitInterrupts()
{
    GimbalAxis board_hw_id = (GimbalAxis)GetBoardHWID();

    // *******************************************
    // Configure interrupts that apply to all axes
    // *******************************************
    EALLOW;
    PieVectTable.ECAP1_INT = &MainISR; // Main ISR is driven from ECAP1 timer
    PieVectTable.XINT2 = &MotorDriverFaultIntISR; // Motor driver fault is driven from external interrupt 2
    EDIS;

    // Enable PIE group 4 interrupt 1 for ECAP1_INT (for main 10kHz loop)
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
    // Enable PIE group 1 interrupt 5 for XINT2 (for motor driver fault line)
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;

    // Configure and enable ECAP1 (for main 10KHz loop)
    ECap1Regs.ECEINT.bit.CTR_EQ_PRD1 = 0x1;  //Enable ECAP1 Period Match interrupt
    ECap1Regs.ECCLR.bit.CTR_EQ_PRD1 = 0x1; //Clear any existing interrupts

    // Configure and enable XINT2 (for motor driver fault line)
    EALLOW;
    GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 13; // Select GPIO 13 as the XINT2 source
    EDIS;
    XIntruptRegs.XINT2CR.bit.POLARITY = 0x00; // Interrupt on falling edge
    XIntruptRegs.XINT2CR.bit.ENABLE = 1; // Enable interrupt

    // Enable CPU INT4 for ECAP1_INT:
    IER |= M_INT4;
    // Enable CPU INT1 for XINT1 and XINT2
    IER |= M_INT1;

    // ******************************************
    // Configure interrupts that only apply to AZ
    // ******************************************
    if (board_hw_id == AZ) {
        EALLOW;
        PieVectTable.SCIRXINTB = &uart_rx_isr; // Uart RX ISR is driven from SCI-B receive
        PieVectTable.SCITXINTB = &uart_tx_isr; // Uart TX ISR is driven from SCI-B transmit
        EDIS;

        // Enable PIE group 9 interrupt 3 for SCI-B rx
        PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
        // Enable PIE group 9 interrupt 4 for SCI-B tx
        PieCtrlRegs.PIEIER9.bit.INTx4 = 1;

        // Enable CPU INT9 for SCI-B
        IER |= M_INT9;
    }

    // ******************************************
    // Configure interrupts that only apply to EL
    // ******************************************
    if (board_hw_id == EL) {
        EALLOW;
        PieVectTable.SCIRXINTA = &uart_rx_isr; // Uart RX ISR is driven from SCI-B receive
        PieVectTable.SCITXINTA = &uart_tx_isr; // Uart TX ISR is driven from SCI-B transmit

        PieVectTable.XINT1 = &GyroIntISR; // Gyro ISR is driven from external interrupt 1
        PieVectTable.I2CINT2A = &i2c_fifo_isr; // I2C Tx and Rx fifo interrupts are handled by the same ISR
        PieVectTable.I2CINT1A = &i2c_int_a_isr; // All non-fifo I2C interrupts are handled by the same ISR
        EDIS;

        // Enable PIE group 9 interrupt 1 for SCI-A rx
        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
        // Enable PIE group 9 interrupt 2 for SCI-A tx
        PieCtrlRegs.PIEIER9.bit.INTx2 = 1;

        // Enable CPU INT9 for SCI-A
        IER |= M_INT9;

        // Enable PIE group 1 interrupt 4 for XINT1 (for gyro interrupt line)
        PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
        // Enable PIE group 8 interrupt 2 for I2C FIFO interrupts
        PieCtrlRegs.PIEIER8.bit.INTx2 = 1;
        // Enable PIE group 8 interrupt 1 for Regular I2C interrupts (the only one we're currently using is addressed as slave (AAS))
        PieCtrlRegs.PIEIER8.bit.INTx1 = 1;

        // Configure and enable XINT1 (for gyro interrupt line)
        EALLOW;
        GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 12; // Select GPIO 12 as the XINT1 source
        EDIS;
        XIntruptRegs.XINT1CR.bit.POLARITY = 0x01; // Interrupt on rising edge
        XIntruptRegs.XINT1CR.bit.ENABLE = 1; // Enable interrupt

        // Enable CPU INT8 for I2C FIFO and I2C addressed as slave (AAS)
        IER |= M_INT8;

        // Initialise the Beacon LED specific interrupts
        init_led_interrupts();
    }

    // For boards that receive the torque commands
    if(board_hw_id != EL) {
        // Set the ISR Handler
        EALLOW;
        PieVectTable.ECAN0INTA = &eCAN0INT_ISR;
        EDIS;

        // Enable PIE group 9 interrupt 5 for ECANAINT0
        PieCtrlRegs.PIEIER9.bit.INTx5 = 1;

        // Enable CPU INT9 for ECANAINT0
        IER |= M_INT9;
    }

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
}

//============================================================================
// NOTE:
// IN MOST APPLICATIONS THE FUNCTIONS AFTER THIS POINT CAN BE LEFT UNCHANGED
// THE USER NEED NOT REALLY UNDERSTAND THE BELOW CODE TO SUCCESSFULLY RUN THIS
// APPLICATION.
//============================================================================

// This function initializes the PIE control registers to a known state.
//
void PieCntlInit(void)
{
    // Disable Interrupts at the CPU level:
    DINT;

    // Disable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

	// Clear all PIEIER registers:
	PieCtrlRegs.PIEIER1.all = 0;
	PieCtrlRegs.PIEIER2.all = 0;
	PieCtrlRegs.PIEIER3.all = 0;	
	PieCtrlRegs.PIEIER4.all = 0;
	PieCtrlRegs.PIEIER5.all = 0;
	PieCtrlRegs.PIEIER6.all = 0;
	PieCtrlRegs.PIEIER7.all = 0;
	PieCtrlRegs.PIEIER8.all = 0;
	PieCtrlRegs.PIEIER9.all = 0;
	PieCtrlRegs.PIEIER10.all = 0;
	PieCtrlRegs.PIEIER11.all = 0;
	PieCtrlRegs.PIEIER12.all = 0;

	// Clear all PIEIFR registers:
	PieCtrlRegs.PIEIFR1.all = 0;
	PieCtrlRegs.PIEIFR2.all = 0;
	PieCtrlRegs.PIEIFR3.all = 0;	
	PieCtrlRegs.PIEIFR4.all = 0;
	PieCtrlRegs.PIEIFR5.all = 0;
	PieCtrlRegs.PIEIFR6.all = 0;
	PieCtrlRegs.PIEIFR7.all = 0;
	PieCtrlRegs.PIEIFR8.all = 0;
	PieCtrlRegs.PIEIFR9.all = 0;
	PieCtrlRegs.PIEIFR10.all = 0;
	PieCtrlRegs.PIEIFR11.all = 0;
	PieCtrlRegs.PIEIFR12.all = 0;
}	

void PieVectTableInit(void)
{
	int16 i;
   	PINT *Dest = &PieVectTable.TINT1;

   	EALLOW;
   	for(i=0; i < 115; i++) 
    *Dest++ = &ISR_ILLEGAL;
   	EDIS;
 
   	// Enable the PIE Vector Table
   	PieCtrlRegs.PIECTRL.bit.ENPIE = 1; 	
}

interrupt void ISR_ILLEGAL(void)   // Illegal operation TRAP
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm("          ESTOP0");
  for(;;);

}
