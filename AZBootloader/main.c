/*
 * main.c
 */
#include "Boot.h"

#include "data.h"


//#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_EXTERNAL_RX_BUFFER
#define MAVLINK_EXTERNAL_RX_STATUS

#include "protocol_c2000.h"

#if 0
#ifndef MAVLINK_MESSAGE_CRC
#pragma    DATA_SECTION(mavlink_message_crcs,"DMARAML5");
	extern uint8_t mavlink_message_crcs[256];

#define MAVLINK_MESSAGE_CRC(msgid) mavlink_message_crcs[msgid]
#endif
#endif

#include "mavlink.h"


#define	FLASH_F2806x 1
#include "Flash2806x_API_Library.h"

#include "F2806x_SysCtrl.h"



//uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

#pragma    DATA_SECTION(m_mavlink_buffer,"DMARAML5");
#pragma    DATA_SECTION(m_mavlink_status,"DMARAML5");

mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

// External functions
extern void CopyData(void);
extern Uint32 GetLongData(void);
extern void ReadReservedFn(void);
void  WatchDogEnable(void);

#pragma    DATA_SECTION(endRam,".endmem");
Uint16 endRam;


#define MAVLINK_SYSTEM_ID 50
#define MAVLINK_COMPONENT_ID 230

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

	ECanaShadow.CANBTC.bit.BRPREG = 1;
	ECanaShadow.CANBTC.bit.TSEG2REG = 7;
	ECanaShadow.CANBTC.bit.TSEG1REG = 15;

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
   Uint16 count = 0;
   while ((ECanaShadow.CANTRS.bit.TRS1 == 1)&&(count < 0x100)) {
	   ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;
   }

   return data;
}

unsigned int location = 0;

static void reset_datapointer(void) {
	location = 0;
}

Uint16 read_Data_and_Send()
{
	extern const unsigned short DATA[];
	Uint16 retval = 0;
	retval = DATA[location++];
	retval = ((retval & 0xFF00)>>8)|((retval & 0x00FF)<<8);
	CAN_SendWordData(retval);
	return retval;
}

Uint16 read_Data()
{
	extern const unsigned short DATA[];
	Uint16 retval = 0;
	retval = DATA[location++];
	retval = ((retval & 0xFF00)>>8)|((retval & 0x00FF)<<8);
	return retval;
}

static const Uint32 crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

Uint32 crc32_add(Uint16 value, Uint32 crc)
{
	Uint32 retval;
#if 0
	retval = crc32_tab[(crc ^ ((value&0xFF00)>>8))] ^ (crc >> 8);
	retval = crc32_tab[(retval ^ ((value&0xFF)))] ^ (retval >> 8);
#else
	retval = (crc ^ value)&0xFFFF;
#endif
	return retval;
}

int verify_data_checksum(void)
{
   Uint32 stored_checksum = 0;
   Uint32 calculated_checksum = 0xFFFFFFFF;
   reset_datapointer();

   // Asign GetWordData to the CAN-A version of the
   // function.  GetWordData is a pointer to a function.
   GetWordData = read_Data;
   if (GetWordData() != 0x08AA) return 0;
   calculated_checksum = crc32_add(0x08AA,calculated_checksum);

   Uint16 i;
   // Read and discard the 8 reserved words.
   for(i = 1; i <= 8; i++)
   {
	   calculated_checksum = crc32_add(GetWordData(),calculated_checksum);
   }
   // Entry Addr
   calculated_checksum = crc32_add(GetWordData(),calculated_checksum);
   calculated_checksum = crc32_add(GetWordData(),calculated_checksum);

   struct HEADER {
      Uint16 BlockSize;
      Uint32 DestAddr;
    } BlockHeader;

    Uint16 wordData;
 //#define VERIFY
 #ifdef VERIFY
    Uint32 errors = 0;
 #endif

    // Get the size in words of the first block
    BlockHeader.BlockSize = (*GetWordData)();
    calculated_checksum = crc32_add(BlockHeader.BlockSize,calculated_checksum);


    // While the block size is > 0 copy the data
    // to the DestAddr.  There is no error checking
    // as it is assumed the DestAddr is a valid
    // memory location

    while((BlockHeader.BlockSize != (Uint16)0x0000)&&(BlockHeader.BlockSize != 0xFFFF))
    {
       BlockHeader.DestAddr = GetLongData();
       calculated_checksum = crc32_add(BlockHeader.DestAddr>>16,calculated_checksum);
       calculated_checksum = crc32_add(BlockHeader.DestAddr&0xFFFF,calculated_checksum);
       for(i = 1; i <= BlockHeader.BlockSize; i++)
       {
     	  extern Uint16 endRam;
          wordData = (*GetWordData)();
          calculated_checksum = crc32_add(wordData,calculated_checksum);
       }

       // Get the size of the next block
       BlockHeader.BlockSize = (*GetWordData)();
       calculated_checksum = crc32_add(BlockHeader.BlockSize,calculated_checksum);
    }

   stored_checksum = (*GetWordData)();
   //stored_checksum <<= 16;
   //stored_checksum |= (*GetWordData)();

   reset_datapointer();

   if (stored_checksum == calculated_checksum) return 1;
   return 0;
}

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
	ScibRegs.SCILBAUD = 21;
	*/
	// Set initial baud rate to 230400
	// For 230400, BRR = 9.851, set BRR to 10 for 227272 effective baud rate, for 1.36% deviation from nominal baud rate
	ScibRegs.SCIHBAUD = 0;
	ScibRegs.SCILBAUD = 10;

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
		// Empty the FIFO into the receive ring buffer
		while ((ScibRegs.SCIFFRX.bit.RXFFST > 0)&&(max_size > 0)) {
			*data = ScibRegs.SCIRXBUF.bit.RXDT;
			data++;
			max_size--;
			bytes_read++;
			timeout = 0;
		}

		// Clear the overflow flag if it is set
		// TODO: Handle this condition instead of just clearing it
		if (ScibRegs.SCIFFRX.bit.RXFFOVF) {
			ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
		}

		// @todo: handle timeout here
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


/* all these may be overwritten when loading the image */
#pragma    DATA_SECTION(msg,"DMARAML5");
#pragma    DATA_SECTION(inmsg,"DMARAML5");
#pragma    DATA_SECTION(status,"DMARAML5");
#pragma    DATA_SECTION(buffer,"DMARAML5");
mavlink_message_t msg, inmsg;
mavlink_status_t status;
unsigned char buffer[300];

#if 0
uint16_t mavlink_msg_request_encapsulated_data_pack(uint8_t system_id,
		                                            uint8_t component_id,
		                                            mavlink_message_t *msg,
		                                            uint16_t seqnr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	_mav_put_uint16_t(buf, 0, seqnr);
#else
	mavlink_encapsulated_data_t *packet = (mavlink_encapsulated_data_t *)msg;
	packet->seqnr = seqnr;
#endif

	msg->msgid = MAVLINK_MSG_ID_ENCAPSULATED_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, 20, MAVLINK_MSG_ID_ENCAPSULATED_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, 20);
#endif
}
#endif

static Uint16 Example_CsmUnlock()
{
    volatile Uint16 temp;
	Uint16 PRG_key0 = 0xFFFF;        //   CSM Key values

    // Load the key registers with the current password
    // These are defined in Example_Flash2806x_CsmKeys.asm

    EALLOW;
    CsmRegs.KEY0 = PRG_key0;
    CsmRegs.KEY1 = PRG_key0;
    CsmRegs.KEY2 = PRG_key0;
    CsmRegs.KEY3 = PRG_key0;
    CsmRegs.KEY4 = PRG_key0;
    CsmRegs.KEY5 = PRG_key0;
    CsmRegs.KEY6 = PRG_key0;
    CsmRegs.KEY7 = PRG_key0;
    EDIS;

    // Perform a dummy read of the password locations
    // if they match the key values, the CSM will unlock

    temp = CsmPwl.PSWD0;
    temp = CsmPwl.PSWD1;
    temp = CsmPwl.PSWD2;
    temp = CsmPwl.PSWD3;
    temp = CsmPwl.PSWD4;
    temp = CsmPwl.PSWD5;
    temp = CsmPwl.PSWD6;
    temp = CsmPwl.PSWD7;

    // If the CSM unlocked, return succes, otherwise return
    // failure.
    if ( (CsmRegs.CSMSCR.all & 0x0001) == 0) return STATUS_SUCCESS;
    else return STATUS_FAIL_CSM_LOCKED;

}


Uint32 MAVLINK_Flash()
{
	int getting_messages = 0;
	Uint16 seq = 0, length, last_seq = 0;
	FLASH_ST FlashStatus = {0};
	Uint16  Status;
	Uint16  *Flash_ptr = (Uint16 *)0x3DC000;     // Pointer to a location in flash
	Uint16 blink_state = 0;
    Uint16 blink_counter = 20;

	memset(&msg,0,sizeof(msg));
	memset(&status,0,sizeof(status));
	memset(&m_mavlink_buffer,0,sizeof(m_mavlink_buffer[0]));
	memset(&m_mavlink_status,0,sizeof(m_mavlink_status[0]));
	setup_serial_port();

	EALLOW;
	Flash_CPUScaleFactor = SCALE_FACTOR;
	EDIS;


	// wait for an image to arrive over mavlink serial
	while (1) {
		// send mavlink message to request data stream
		// mavlink_msg_request_data_stream_pack(SYSTEM_ID,COMPONENT_ID,&msg,0,0,0,1000,1);
		//mavlink_msg_encapsulated_data_pack(MAVLINK_SYSTEM_ID,MAVLINK_COMPONENT_ID,&msg,seq,NULL);
#define DATA_SIZE 250
		mavlink_msg_data_transmission_handshake_pack(MAVLINK_SYSTEM_ID,MAVLINK_COMPONENT_ID,&msg,MAVLINK_TYPE_UINT16_T, 0 /* size */, seq /* width */, 0 /*uint16_t height*/, 0 /*uint16_t packets*/, DATA_SIZE /*uint8_t payload*/, 0 /*uint8_t jpg_quality*/);
		length = mavlink_msg_to_send_buffer(buffer,&msg);
		send_serial_port(buffer,length);
		getting_messages = 1;
		while (getting_messages) {
			int read_size = 0;
			// read the serial port, and if I get no messages, timeout
			memset(buffer,0,sizeof(buffer[0])*300);
			if ((read_size = read_serial_port(buffer,300)) == 0) {
				getting_messages = 0;
				memset(&msg,0,sizeof(msg));
				memset(&status,0,sizeof(status));
				memset(&m_mavlink_buffer,0,sizeof(m_mavlink_buffer[0]));
				memset(&m_mavlink_status,0,sizeof(m_mavlink_status[0]));
			} else {
				int i;
				getting_messages = 1;
				for (i = 0; i < read_size; i++) {
					if (mavlink_parse_char(MAVLINK_COMM_0,buffer[i]&0xFF,&inmsg,&status)) {
						if ((inmsg.sysid == MAVLINK_SYSTEM_ID)&&
								(inmsg.compid == MAVLINK_COMPONENT_ID)&&
								(inmsg.msgid == MAVLINK_MSG_ID_ENCAPSULATED_DATA)) {
							if (inmsg.len > 2) {
								seq = mavlink_msg_encapsulated_data_get_seqnr(&inmsg);
								mavlink_msg_encapsulated_data_get_data(&inmsg,buffer);
								getting_messages = 0;
								// fix the buffer...
								{
									int j;
									for (j = 0; j < DATA_SIZE/2; j++) {
										buffer[j] = (buffer[j*2+1]<<8)|
												    (buffer[j*2+0]);
									}
								}
								if (seq == 0) {
									Example_CsmUnlock();

									/* don't erase SECTOR A */
									Status = Flash_Erase(SECTORB, &FlashStatus);
									Status = Flash_Erase(SECTORC, &FlashStatus);
									Status = Flash_Erase(SECTORD, &FlashStatus);
									Status = Flash_Erase(SECTORE, &FlashStatus);
									Status = Flash_Erase(SECTORF, &FlashStatus);
									Status = Flash_Erase(SECTORG, &FlashStatus);
									/* don't erase SECTOR H */
									// write data to flash
									Flash_Program(Flash_ptr,(Uint16 *)buffer,DATA_SIZE/2,&FlashStatus);
									Flash_ptr += DATA_SIZE/2;
									seq++;
								} else if (seq == (last_seq+1)) {
									// write data to flash
									Flash_Program(Flash_ptr,(Uint16 *)buffer,DATA_SIZE/2,&FlashStatus);
									Flash_ptr += DATA_SIZE/2;
									last_seq = seq;
									seq++;
								}
							} else {
							}
						} else if ((inmsg.sysid == MAVLINK_SYSTEM_ID)&&
								(inmsg.compid == MAVLINK_COMPONENT_ID)&&
								(inmsg.msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE)) {
							/* must be done */
							WatchDogEnable();
							while(1);
						}
					}
				}
			}
		}

		// If we're here, we've timed out and are about to send another mavlink request for a boot image
		// Toggle the LED in here to show that we're doing something
        if (++blink_counter >= 20) {
            if (blink_state == 0) {
                GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
                blink_state = 1;
            } else {
                GpioDataRegs.GPASET.bit.GPIO7 = 1;
                blink_state = 0;
            }
            blink_counter = 0;
        }
	}
	// reset?
	return 0;
}

Uint32 CAN_Boot()
{
   Uint32 EntryAddr;

   location = 0;

   // If the missing clock detect bit is set, just
   // loop here.
   if(SysCtrlRegs.PLLSTS.bit.MCLKSTS == 1)
   {
      for(;;);
   }

   // Asign GetWordData to the CAN-A version of the
   // function.  GetWordData is a pointer to a function.
   GetWordData = read_Data_and_Send;

   CAN_Init();

   // If the KeyValue was invalid, abort the load
   // and return the flash entry point.
   if (GetWordData() != 0x08AA) return FLASH_ENTRY_POINT;

   ReadReservedFn();

   EntryAddr = GetLongData();

   CopyData();

   return EntryAddr;
}

//---------------------------------------------------------------
// This module disables the watchdog timer.
//---------------------------------------------------------------

void  WatchDogDisable()
{
   EALLOW;
   SysCtrlRegs.WDCR = 0x0068;               // Disable watchdog module
   EDIS;
}

//---------------------------------------------------------------
// This module enables the watchdog timer.
//---------------------------------------------------------------

void  WatchDogEnable()
{
   EALLOW;
   SysCtrlRegs.WDCR = 0x0028;               // Enable watchdog module
   SysCtrlRegs.WDKEY = 0x55;                // Clear the WD counter
   SysCtrlRegs.WDKEY = 0xAA;
   EDIS;
}

// This function initializes the PLLCR register.
//void InitPll(Uint16 val, Uint16 clkindiv)
static void PLLset(Uint16 val)
{
   volatile Uint16 iVol;

   // Make sure the PLL is not running in limp mode
   if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
   {
	  EALLOW;
      // OSCCLKSRC1 failure detected. PLL running in limp mode.
      // Re-enable missing clock logic.
      SysCtrlRegs.PLLSTS.bit.MCLKCLR = 1;
      EDIS;
      // Replace this line with a call to an appropriate
      // SystemShutdown(); function.
      asm("        ESTOP0");     // Uncomment for debugging purposes
   }

   // DIVSEL MUST be 0 before PLLCR can be changed from
   // 0x0000. It is set to 0 by an external reset XRSn
   // This puts us in 1/4
   if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
   {
       EALLOW;
       SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
       EDIS;
   }

   // Change the PLLCR
   if (SysCtrlRegs.PLLCR.bit.DIV != val)
   {

      EALLOW;
      // Before setting PLLCR turn off missing clock detect logic
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
      SysCtrlRegs.PLLCR.bit.DIV = val;
      EDIS;

      // Optional: Wait for PLL to lock.
      // During this time the CPU will switch to OSCCLK/2 until
      // the PLL is stable.  Once the PLL is stable the CPU will
      // switch to the new PLL value.
      //
      // This time-to-lock is monitored by a PLL lock counter.
      //
      // Code is not required to sit and wait for the PLL to lock.
      // However, if the code does anything that is timing critical,
      // and requires the correct clock be locked, then it is best to
      // wait until this switching has completed.

      // Wait for the PLL lock bit to be set.
      // The watchdog should be disabled before this loop, or fed within
      // the loop via ServiceDog().

	  // Uncomment to disable the watchdog
      WatchDogDisable();

      while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1) {}

      EALLOW;
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
	  EDIS;
	  }

	  //divide down SysClk by 2 to increase stability
	  EALLOW;
      SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
      EDIS;
}

void DeviceInit()
{
	//The Device_cal function, which copies the ADC & oscillator calibration values
	// from TI reserved OTP into the appropriate trim registers, occurs automatically
	// in the Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC and oscillators to function according
	// to specification.
		EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1; // Enable ADC peripheral clock
		(*Device_cal)();					  // Auto-calibrate from TI OTP
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0; // Return ADC clock to original state
		EDIS;

	// Switch to Internal Oscillator 1 and turn off all other clock
	// sources to minimize power consumption
		EALLOW;
		SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0;
	    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL=0;  // Clk Src = INTOSC1
		SysCtrlRegs.CLKCTL.bit.XCLKINOFF=1;     // Turn off XCLKIN
	//	SysCtrlRegs.CLKCTL.bit.XTALOSCOFF=1;    // Turn off XTALOSC
		SysCtrlRegs.CLKCTL.bit.INTOSC2OFF=1;    // Turn off INTOSC2
		SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;  //Select external crystal for osc2
		SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;  //Select osc2
	    EDIS;

	// SYSTEM CLOCK speed based on Internal OSC = 10 MHz
	// 0x10=  80    MHz		(16)
	// 0xF =  75    MHz		(15)
	// 0xE =  70    MHz		(14)
	// 0xD =  65    MHz		(13)
	// 0xC =  60	MHz		(12)
	// 0xB =  55	MHz		(11)
	// 0xA =  50	MHz		(10)
	// 0x9 =  45	MHz		(9)
	// 0x8 =  40	MHz		(8)
	// 0x7 =  35	MHz		(7)
	// 0x6 =  30	MHz		(6)
	// 0x5 =  25	MHz		(5)
	// 0x4 =  20	MHz		(4)
	// 0x3 =  15	MHz		(3)
	// 0x2 =  10	MHz		(2)

	PLLset( 0x8 );	// choose from options above
	EALLOW;
    SysCtrlRegs.LOSPCP.all = 0x0002;		// Sysclk / 4 (20 MHz)
	SysCtrlRegs.XCLK.bit.XCLKOUTDIV=0;	//divide by 4 default

	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;    // ADC

    // Configure the hardware ID pins first.  This is done out of order because some of the
    // pins are configured differently depending on which board we're dealing with
    //--------------------------------------------------------------------------------------
    //  GPIO-20 - PIN FUNCTION = ID Pin 0
        GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    // 0=GPIO,  1=EQEP1A,  2=MDXA,  3=COMP1OUT
        GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;     // 1=OUTput,  0=INput
        GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;     // Enable internal pullups
    //  GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;   // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO20 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-21 - PIN FUNCTION = ID Pin 1
        GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;    // 0=GPIO,  1=EQEP1B,  2=MDRA,  3=COMP2OUT
        GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;     // 1=OUTput,  0=INput
        GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;     // Enable internal pullups
    //  GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;   // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO21 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    // SCI-B is used for MAVLink communication with the parent system
        SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 0;  // SCI-A
        SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 1;	// SCI-B
    // If this is the AZ board, GPIOs 16, 17, 18 and 19 get configured as SCI pins
    //--------------------------------------------------------------------------------------
    //  GPIO-16 - PIN FUNCTION = RTS in from copter
        GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;    // 0=GPIO, 1=SPISIMOA, 2=Resv CAN-B, 3=TZ2n
        GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;     // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;   // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO16 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-17 - PIN FUNCTION = CTS out to copter
        GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;    // 0=GPIO, 1=SPISOMIA, 2=Resv CAN-B, 3=TZ3n
        GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;     // 1=OUTput,  0=INput
        GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;   // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO17 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-7 - PIN FUNCTION = User LED (Active Low)
        GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;     // 0=GPIO, 1=EPWM4B, 2=SCIRXDA, 3=ECAP2
        GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;      // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;    // uncomment if --> Set Low initially
        GpioDataRegs.GPASET.bit.GPIO7 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-18 - PIN FUNCTION = Tx to copter
        GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;    // 0=GPIO, 1=SPICLKA, 2=SCITXDB, 3=XCLKOUT
    //  GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;     // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;   // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO18 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-19 - PIN FUNCTION = Rx from copter
        GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;    // 0=GPIO, 1=SPISTEA, 2=SCIRXDB, 3=ECAP1
    //  GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;   // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO19 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    EDIS;

}

Uint32 SelectBootMode()
{
	  Uint32 EntryAddr;

	  EALLOW;

	  // Watchdog Service
	  SysCtrlRegs.WDKEY = 0x0055;
	  SysCtrlRegs.WDKEY = 0x00AA;

	  // Before waking up the flash
	  // set the POR to the minimum trip point
	  // If the device was configured by the factory
	  // this write will have no effect.

	  *BORTRIM = 0x0100;

	  // At reset we are in /4 mode.  Change to /1
	  // Calibrate the ADC and internal OSCs
	  SysCtrlRegs.PLLSTS.bit.DIVSEL = DIVSEL_BY_1;
	  SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
	  (*Device_cal)();
	  SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0;

	  // Init two locations used by the flash API with 0x0000
	  Flash_CPUScaleFactor = 0;
	  Flash_CallbackPtr = 0;
	  EDIS;

	  DeviceInit();

	  // Read the password locations - this will unlock the
	  // CSM only if the passwords are erased.  Otherwise it
	  // will not have an effect.
	  CsmPwl.PSWD0;
	  CsmPwl.PSWD1;
	  CsmPwl.PSWD2;
	  CsmPwl.PSWD3;
	  CsmPwl.PSWD4;
	  CsmPwl.PSWD5;
	  CsmPwl.PSWD6;
	  CsmPwl.PSWD7;

	  EALLOW;

	  if (verify_data_checksum()) {
		  EntryAddr = CAN_Boot();
		  reset_datapointer();
	  } else {
		  EntryAddr = MAVLINK_Flash();
	  }
	return EntryAddr;
}
