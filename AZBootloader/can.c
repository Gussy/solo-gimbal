#include "boot/Boot.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"

void CAN_Init()
{

/* Create a shadow register structure for the CAN control registers. This is
 needed, since only 32-bit access is allowed to these registers. 16-bit access
 to these registers could potentially corrupt the register contents or return
 false data. This is especially true while writing to/reading from a bit
 (or group of bits) among bits 16 - 31 */

   struct ECAN_REGS ECanaShadow;

   EALLOW;

/* Enable CAN clock  */

   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK=1;

/* Configure eCAN-A pins using GPIO regs*/

   // GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1; // GPIO30 is CANRXA
   // GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1; // GPIO31 is CANTXA
   GpioCtrlRegs.GPAMUX2.all |= 0x50000000;

/* Enable internal pullups for the CAN pins  */

   // GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;
   // GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;
   GpioCtrlRegs.GPAPUD.all &= 0x3FFFFFFF;

/* Asynch Qual */

   GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/

    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

/* Initialize all bits of 'Message Control Register' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero

    ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;

/* Clear all RMPn, GIFn bits */
// RMPn, GIFn bits are zero upon reset and are cleared again as a precaution.

   ECanaRegs.CANRMP.all = 0xFFFFFFFF;

/* Clear all interrupt flag bits */

   ECanaRegs.CANGIF0.all = 0xFFFFFFFF;
   ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

/* Configure bit timing parameters for eCANA*/

	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    ECanaShadow.CANES.all = ECanaRegs.CANES.all;

    do
	{
	    ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1 );  		// Wait for CCE bit to be set..

    ECanaShadow.CANBTC.all = 0;

    /* CAN bitrate of 1Mbps */
	ECanaShadow.CANBTC.bit.BRPREG = 1;
	ECanaShadow.CANBTC.bit.TSEG2REG = 4;
	ECanaShadow.CANBTC.bit.TSEG1REG = 13;

    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    ECanaShadow.CANES.all = ECanaRegs.CANES.all;

    do
    {
       ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 ); 		// Wait for CCE bit to be  cleared..

/* Disable all Mailboxes  */

   ECanaRegs.CANME.all = 0;     // Required before writing the MSGIDs

/* Assign MSGID to MBOX1 */

   ECanaRegs.CANTRR.bit.TRR1 = 1;
   while ( ECanaRegs.CANTRS.bit.TRS1 == 1);
   ECanaMboxes.MBOX1.MSGID.all = 0x00040000;	// Standard ID of 1, Acceptance mask disabled
   //ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = 0x3FF;
   ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = 0x0FF;
   ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 2;

/* Configure MBOX1 to be a send MBOX */

   {
	   volatile Uint16 i;
	   ECanaRegs.CANMD.all = 0x0000;
	   // delay here for a bit to allow the other two boards to come up
	   for (i = 0; i < 0x4000; i++) {
	   }
   }

/* Enable MBOX1 */

   ECanaRegs.CANME.all = 0x0002;

   EDIS;
    return;
}

//#################################################
// Uint16 CAN_GetWordData(void)
//-----------------------------------------------
// This routine fetches two bytes from the CAN-A
// port and puts them together to form a single
// 16-bit value.  It is assumed that the host is
// sending the data in the order LSB followed by MSB.
//-----------------------------------------------


Uint16 CAN_GetWordData()
{
   Uint16 wordData;
   Uint16 byteData;

   wordData = 0x0000;
   byteData = 0x0000;

// Fetch the LSB
   while(ECanaRegs.CANRMP.all == 0) { }
   wordData =  (Uint16) ECanaMboxes.MBOX1.MDL.byte.BYTE0;   // LS byte

   // Fetch the MSB

   byteData =  (Uint16)ECanaMboxes.MBOX1.MDL.byte.BYTE1;    // MS byte

   // form the wordData from the MSB:LSB
   wordData |= (byteData << 8);

/* Clear all RMPn bits */

    ECanaRegs.CANRMP.all = 0xFFFFFFFF;

   return wordData;
}

#define CAN_SEND_WORD_LED_TIMEOUT_1 500000
#define CAN_SEND_WORD_LED_TIMEOUT_2 50000

Uint16 CAN_SendWordData(Uint16 data)
{
   ECanaMboxes.MBOX1.MDL.byte.BYTE0 = (data&0xFF);   // LS byte

   // Fetch the MSB
   ECanaMboxes.MBOX1.MDL.byte.BYTE1 = ((data >> 8)&0xFF);    // MS byte

   ECanaRegs.CANTRS.all = (1ul<<1);	// "writing 0 has no effect", previously queued boxes will stay queued
   //while (ECanaRegs.CANTA.bit.TA1 != 1);
   ECanaRegs.CANTA.all = (1ul<<1);		// "writing 0 has no effect", clears pending interrupt, open for our tx
   //while (ECanaRegs.CANTA.bit.TA1 != 0);
   struct ECAN_REGS ECanaShadow;
   // wait for it to be sent
   ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;
   Uint32 timeout = 0;
   Uint32 timeout_max = CAN_SEND_WORD_LED_TIMEOUT_1;
   while ((ECanaShadow.CANTRS.bit.TRS1 == 1)) {
       while ((ECanaShadow.CANTRS.bit.TRS1 == 1) && (timeout++ < timeout_max)) {
           ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;
       }
       // Blink the status LED to show that we're doing something
       timeout = 0;
       if (timeout_max == CAN_SEND_WORD_LED_TIMEOUT_1) {
           timeout_max = CAN_SEND_WORD_LED_TIMEOUT_2;
       } else {
           timeout_max = CAN_SEND_WORD_LED_TIMEOUT_1;
       }
       led_status_toggle();
   }

   return data;
}
