/*
 * cand.c
 *
 *  Created on: Mar 6, 2014
 *      Author: rmorrill
 */

//#include "F2806x_Device.h"     // F2806x Headerfile Include File
//#include "F2806x_Examples.h"   // F2806x Examples Include File

//#include "F2806x_Gpio.h"     // F2806x Headerfile Include File

#include "PeripheralHeaderIncludes.h"

#include "cand_BitFields.h"
#include "cand.h"
#include "HWSpecific.h"
#include "device_init.h"
#include "PM_Sensorless.h"

#include <stdlib.h>
#include <stdint.h>

static void ECanInitGpio( void );
static int ECanTx( struct MBOX* outbox );
static int ECanRx( struct MBOX* inbox );

//#define FORCE_CAN_ID_TO_AZ

#ifndef BOOL
typedef enum { FALSE = 0, TRUE } BOOL;
#endif

#define CAN_TX_MBOX_CNT 16

void ECanInit( void )        // Initialize eCAN-A module
{
	volatile union CANLAM_REG* lam;
	volatile struct MBOX* mbox;
	int i;

	ECanInitGpio();

	/* Create a shadow register structure for the CAN control registers. This is
	 needed, since only 32-bit access is allowed to these registers. 16-bit access
	 to these registers could potentially corrupt the register contents or return
	 false data. */

	struct ECAN_REGS ECanaShadow;

    EALLOW;     // EALLOW enables access to protected bits

    /* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/
    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

    /* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */
    // HECC mode also enables time-stamping feature
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.SCB = 1;
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

	// Disable Mailboxes
	ECanaRegs.CANME.all = 0;        // Required before writing the MSGIDs

	ECanaRegs.CANMD.all  = 0xffffffff<<CAN_TX_MBOX_CNT;	///< Boxes 0-15 are Tx, 16-31 Rx, (1 = rx)
	//ECanaRegs.CANMD.all  = 0xFFFF0000;						///< Boxes 0-15 are Tx, 16-31 Rx, (1 = rx)

	// Setup Rx Mailboxes
	for( i=16; i<32; i++ ){
		// CTRL reg needs cleared, setting ID to default value for testing
		mbox = &ECanaMboxes.MBOX0+i;
		mbox->MSGCTRL.all = 0x00000000;
		mbox->MSGID.bit.IDE = 0;
		mbox->MSGID.bit.STDMSGID = 0x7ff;
		mbox->MSGID.bit.AME = 1;

		// All Rx boxes accept all messages
		lam = &ECanaLAMRegs.LAM0+i;
		lam->all = 0x1fffffff;
	}

	// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
	//  as a matter of precaution.

    ECanaRegs.CANTA.all = 0xFFFFFFFF;   /* Clear all TAn bits */
    ECanaRegs.CANRMP.all = 0xFFFFFFFF;  /* Clear all RMPn bits */
    ECanaRegs.CANGIF0.all = 0xFFFFFFFF; /* Clear all interrupt flag bits */
    ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

    /* Configure bit timing parameters for eCANA*/

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU has been granted permission to change the configuration registers
    do
    {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1 );       // Wait for CCE bit to be set..

    ECanaShadow.CANBTC.all = 0;
    /* The following block is for 80 MHz SYSCLKOUT. (40 MHz CAN module clock Bit rate = 1 Mbps
       See Note at end of file. */

    ECanaShadow.CANBTC.bit.BRPREG = 1;
    ECanaShadow.CANBTC.bit.TSEG2REG = 4;
    ECanaShadow.CANBTC.bit.TSEG1REG = 13;

    ECanaShadow.CANBTC.bit.SAM = 1;
    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU no longer has permission to change the configuration registers
    do
    {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 );       // Wait for CCE bit to be  cleared..

	ECanaRegs.CANMIM.all = 0xFFFFFFFF;				///< Enable Interrupt Mask on all Mailboxes
	ECanaRegs.CANME.all = 0xFFFFFFFF;				///< Enable all Mailboxes

    EDIS;
}

static void ECanInitGpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected CAN pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     // Enable pull-up for GPIO31 (CANTXA)

/* Set qualification for selected CAN pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)

/* Configure eCAN-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAN functional pins.

    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;    // Configure GPIO30 for CANRXA operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;    // Configure GPIO31 for CANTXA operation

    EDIS;
}

/**
 * Copy outbox to next slot in the transmit queue. If
 * a message is already in that slot, it will be overwritten
 * without warning. Flags box for immediate transmission
 */
static int ECanTx( struct MBOX* outbox )
{
	static uint8_t can_tx_mbox = 0;
	volatile struct MBOX *mptr;

	//get next output box
	mptr = &ECanaMboxes.MBOX0+can_tx_mbox;

	//ECanaRegs.CANME.all &= ~((0x00000001ul)<<can_tx_mbox) //didnt work, mmk, then i'll make a shadow?
	unsigned long temp = ECanaRegs.CANME.all;
	temp  &= ~((0x00000001ul)<<can_tx_mbox);

	ECanaRegs.CANME.all = temp;

	mptr->MDH = outbox->MDH;
	mptr->MDL = outbox->MDL;
	mptr->MSGCTRL = outbox->MSGCTRL;
	mptr->MSGID = outbox->MSGID;

	temp |= (1ul<<can_tx_mbox);
	ECanaRegs.CANME.all = temp;
	ECanaRegs.CANTRS.all = (1ul<<can_tx_mbox);	// "writing 0 has no effect", previously queued boxes will stay queued
	ECanaRegs.CANTA.all = (1ul<<can_tx_mbox);		// "writing 0 has no effect", clears pending interrupt, open for our tx

	if (++can_tx_mbox >= CAN_TX_MBOX_CNT) {
		can_tx_mbox = 0;
	}

	return 0;
}

/**
 * Check if any Mailbox is currently pending, if
 * so, copy to inbox
 */
static int ECanRx( struct MBOX* inbox )
{
	volatile struct MBOX *mptr;

	//Check if Rx Message Pending
	if ( ECanaRegs.CANRMP.all ) {
		Uint32 b, f;

		for ( f=0x80000000, b=31; f; f>>=1, b-- ){
			if (ECanaRegs.CANRMP.all&f) {

				mptr = &ECanaMboxes.MBOX0+b;
				inbox->MDH.all = mptr->MDH.all;
				inbox->MDL.all = mptr->MDL.all;
				inbox->MSGCTRL.all = mptr->MSGCTRL.all;
				inbox->MSGID.all = mptr->MSGID.all;

				ECanaRegs.CANRMP.all = f;

				return 1;
			}
		}
	}
	return 0;
}

CAND_Result cand_init( void )
{
#if !defined(FORCE_CAN_ID_TO_AZ) && !defined(FORCE_CAN_ID_TO_EL) && !defined(FORCE_CAN_ID_TO_ROLL)
	//Initialize ID GPIO
	//GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;		//disable internal pullups (pulled up externally by switch)
	//GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;

	//GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;		//TODO: lookup, make sure mux 0 = gpio
	//GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;		//TODO: lookup
#endif

	// Be sure that the switches have been moved from their factory positions,
	// if not, send out a fault message with the errant SenderID
	if (CAND_GetSenderID() == CAND_ID_INVALID)
		cand_tx_fault(CAND_FAULT_UNKNOWN_AXIS_ID);

	return CAND_SUCCESS;
}

CAND_SenderID CAND_GetSenderID( void )
{
	//Read system jumpers to know whoami
	int sw = GetBoardHWID();

#ifdef FORCE_CAN_ID_TO_AZ
    return CAND_ID_AZ;
#endif
#ifdef FORCE_CAN_ID_TO_EL
    return CAND_ID_EL;
#endif
#ifdef FORCE_CAN_ID_TO_ROLL
    return CAND_ID_ROLL;
#endif

    switch (sw) {
    	case 0:		return CAND_ID_EL;
    	case 1:		return CAND_ID_AZ;
    	case 2:		return CAND_ID_ROLL;
    	case 3:
    	default:	return CAND_ID_INVALID;
    }
}

BOOL CAND_InDestinationList( CAND_DestinationID did )
{
    CAND_SenderID me = CAND_GetSenderID();

    if (me == CAND_ID_INVALID) {
        return FALSE;
    }

    if (did == CAND_ID_ALL_AXES) {
        return TRUE;
    }

    if ( me == did ) {
    	return TRUE;
    }

    return FALSE;
}


CAND_Result cand_rx( struct cand_message * msg )
{
	CAND_Result ret = CAND_RX_EMPTY;
	struct MBOX mbox;

	if (ECanRx(&mbox)) {
		//main parser.
		CAND_SID sid;

		sid.field.SID = mbox.MSGID.bit.STDMSGID;

		switch (sid.all.m_id) {
			case CAND_MID_FAULT:
			{
				// All fault msgs are broadcast, so no destination check
				msg->sender_id = (CAND_SenderID) sid.fault.s_id;

				msg->fault_code = (CAND_FaultCode) sid.fault.fault_code;

				ret = CAND_RX_FAULT;
			}
				break;
			case CAND_MID_COMMAND:
				// Check if we are on the recipient list
				if ( CAND_InDestinationList( (CAND_DestinationID) sid.directive.d_id) == FALSE ) {
					break;
				}
				msg->command = (CAND_Command) sid.directive.command;
				ret = CAND_RX_COMMAND;
				break;
			case CAND_MID_PARAMETER_SET:

				// Check if we are on the recipient list
				if ( CAND_InDestinationList( (CAND_DestinationID) sid.param_set.all.d_id) == FALSE ) {
					break;
				}

				// Addressing mode sets whether to interpret the parameter
				// ID as a register with up to 5 parameter flags set, or just a
				// single parameter ID.
				if (sid.param_set.all.addr_mode == CAND_ADDR_MODE_IMMEDIATE) {

					// If there are multiple destinations for a single immediate parameter
					// then our data is packed in with the others and we have to get it out:
					if ( sid.param_set.all.d_id == CAND_ID_ALL_AXES ) {
						// Extract out our parameter
						msg->param_id[0] = (CAND_ParameterID) sid.param_set.immediate.param_reg;
						msg->param_cnt = 1;							// there can only be 1 param in multicast of this type

						switch (CAND_GetSenderID()) {
							case CAND_ID_AZ:	msg->param[0] = mbox.MDL.word.HI_WORD;		break;
							case CAND_ID_ROLL:	msg->param[0] = mbox.MDL.word.LOW_WORD;		break;
							default:			break;
						}


					} else {
						// Must just be a parameter for us (don't need to check broadcast, immediate
						// parameters for broadcast aren't a thing)
						uint8_t p_flag, p_cnt;

						for( p_flag=1, p_cnt=0; p_flag<=0x10 && p_cnt<4; p_flag<<=1 ) {

							if ( p_flag&sid.param_set.immediate.param_reg ) {
								msg->param_id[p_cnt] = (CAND_ParameterID) (sid.param_set.immediate.param_reg&p_flag);

								switch (p_cnt) {
									case 0: 	msg->param[p_cnt] = mbox.MDL.word.HI_WORD;		break;
									case 1:		msg->param[p_cnt] = mbox.MDL.word.LOW_WORD;		break;
									case 2:		msg->param[p_cnt] = mbox.MDH.word.HI_WORD;		break;
									case 3:		msg->param[p_cnt] = mbox.MDH.word.LOW_WORD;		break;
									default: 													break;
								}
								p_cnt++;
							}
						}
						msg->param_cnt = p_cnt;
					}


				} else {
					// Extended mode, rather than having a register with flags for parameters, we
					// get a list of ID/params to set, first ID is in the last 5 bits of the arbitration
					// field
					int byte_in_payload=0, p_cnt=0;

					while( mbox.MSGCTRL.bit.DLC > byte_in_payload && p_cnt < 3) {

						// Get the special case ID0 out of the arbitration field, else
						// the next byte in the payload is the next ID
						if (p_cnt == 0) {
							msg->param_id[p_cnt] = (CAND_ParameterID) sid.param_set.extended.param_id;
						} else {
							msg->param_id[p_cnt] = (CAND_ParameterID) CANMD8(mbox, byte_in_payload);
							byte_in_payload++;
						}

						// param IDs of <= 16 are 2 bytes, else just 1, list is
						// packed, so we'll have to count our way through
						if (msg->param_id[p_cnt] < CAND_PID_WORD_CUTOFF) {
							msg->param[p_cnt] = CANMD8(mbox, byte_in_payload)<<8 | CANMD8(mbox, byte_in_payload+1);
							byte_in_payload += 2;
						} else {
							msg->param[p_cnt] = CANMD8(mbox, byte_in_payload);
							byte_in_payload += 1;
						}

						p_cnt++;
					}
					msg->param_cnt = p_cnt;
				}
				ret = CAND_RX_PARAM_SET;
				break;
			case CAND_MID_PARAMETER_QUERY:

				// Check if we are on the recipient list
				if ( CAND_InDestinationList( (CAND_DestinationID) sid.param_query.d_id) == FALSE ) {
					break;
				}

				msg->sender_id = (CAND_SenderID) sid.param_query.s_id;

				// This is either a new query or a response to a query that we sent out
				if (sid.param_query.dir == CAND_DIR_QUERY) {
					// New Query! Extract out the IDs for queried params
					int p_cnt=0;
					msg->param_repeat = sid.param_query.repeat;

					while( p_cnt < mbox.MSGCTRL.bit.DLC ) {
						msg->param_request_id[p_cnt] = (CAND_ParameterID) CANMD8(mbox, p_cnt);
						p_cnt++;
					}
					msg->param_request_cnt = p_cnt;
					ret = CAND_RX_PARAM_QUERY;
				} else {
					// Response! Extract out the ID/param pairs
					int byte_in_payload=0, p_cnt=0;

					// similar deal as set parameter in extended mode, except all
					// ID/value pairs are in the payload
					while( mbox.MSGCTRL.bit.DLC > byte_in_payload && p_cnt < 4) {

						msg->param_response_id[p_cnt] = (CAND_ParameterID) CANMD8(mbox, byte_in_payload);
						byte_in_payload++;

						if (msg->param_response_id[p_cnt] < CAND_PID_WORD_CUTOFF) {
							msg->param_response[p_cnt] = CANMD8(mbox, byte_in_payload)<<8 | CANMD8(mbox, byte_in_payload+1);
							byte_in_payload += 2;
						} else {
							msg->param_response[p_cnt] = CANMD8(mbox, byte_in_payload);
							byte_in_payload += 1;
						}

						p_cnt++;
					}
					msg->param_response_cnt = p_cnt;
					ret = CAND_RX_PARAM_RESPONSE;
				}

				break;
			//case CAND_MID_DEBUG:
				//note that debug data will actually get caught by CAND_MID_PARAMETER_QUERY,
				// then get rejected because it's an invalid destination ID, leaving as known
				// issue for now, hence TODO: fixme
			default:
				ret = CAND_RX_PARSE_ERROR;
				break;
		}
	}
	return ret;
}

CAND_Result cand_tx_multi_param(CAND_DestinationID did, CAND_ParameterID* pid, Uint16* param, Uint8 param_cnt)
{
    CAND_SID sid;
    Uint8 payload[8], payload_cnt = 0;

    if (param_cnt == 0) {
        return CAND_SUCCESS;
    }

    sid.sidWord = 0;
    sid.all.m_id = CAND_MID_PARAMETER_SET;
    sid.param_set.all.d_id = did;
    sid.param_set.extended.addr_mode = CAND_ADDR_MODE_EXTENDED;

    //need to handle a few different things here:
    // 1: that mdb is a multi-endpoint desitination, and we're sending a single pid
    // 2: that mdb is a multi-endpoint desitination, and we're sending multiple pids
    // 3: that mdb is a single endpoint and we're sending multiple pids
    // 4: that mdb is a single endpoint and we're sending 1 pid (then you should be using SetParam...)

    // This is nearly done, but TODO: add logic to detect when Immediate mode
    // could be used, current implementation isnt quite as efficient as it could
    // be

    if (did == CAND_ID_ALL_AXES) {
        //  Sending to Multiple Destinations, packet format is as follows:
        //  Param ID is loaded into the arbitration field, and the params
        //  are loaded into the payload as an ordered list, MSParam is the
        //  lowest destination id in the destination list

        // when sending to the two axes, it could be an 1x1 byte, 2x1 byte or
        // 1x2 byte params
        if ( pid[0] < CAND_PID_WORD_CUTOFF) {
            // only 1 param of 2 byte to this destination

            sid.param_set.all.param    = pid[0];

            payload[0] = (param[0]>>8)&0xff;
            payload[1] = param[0]&0xff;
            payload[2] = (param[1]>>8)&0xff;
            payload[3] = param[1]&0xff;
            payload_cnt = 4;
            sid.param_set.extended.addr_mode    = CAND_ADDR_MODE_IMMEDIATE;

        } else if ( param_cnt == 1) {
            // 1x 1byte param
            sid.param_set.all.param    = pid[0];

            payload[0] = param[0]&0xff;
            payload[1] = param[1]&0xff;
            payload_cnt = 2;

        } else if ( param_cnt == 2 ) {
            // 2x 1byte param, this is the only case '2' from above
            sid.param_set.all.param    = pid[0];

            payload[0] = param[0]&0xff;
            payload[1] = param[1]&0xff;
            payload[2] = pid[1];
            payload[3] = param[2]&0xff;
            payload[4] = param[3]&0xff;
            payload_cnt = 5;

        }
    } else {
        // Now we must be sending multiple parameters to a single endpoint
        if ( param_cnt == 1 ) {
            // you should call the right function:
            return cand_tx_param(did, pid[0], param[0]);
        }

        // this is where we need to check whether all pids are <CAND_PID_WORD_CUTOFF,
        // so we know we can send in immediate mode

        int param_in_cnt=0, pid_cnt=0;
        payload_cnt=0;

        for ( param_in_cnt=0; param_in_cnt<param_cnt && payload_cnt<8; param_in_cnt++, pid_cnt++ ) {

            if (pid_cnt == 0) {
                // first id is stuffed into arbitration field
                sid.param_set.all.param = pid[pid_cnt];
            } else {
                payload[payload_cnt++] = pid[pid_cnt];
            }

            if (pid[pid_cnt] < CAND_PID_WORD_CUTOFF) {
                //16 bit param:
                payload[payload_cnt] = (param[param_in_cnt]>>8)&0xff;
                payload[payload_cnt+1] = param[param_in_cnt]&0xff;
                payload_cnt += 2;
            } else {
                payload[payload_cnt] = param[param_in_cnt]&0xff;
                payload_cnt += 1;
            }
        }
    }

    return cand_tx(sid, payload, payload_cnt);
}

CAND_Result cand_tx_param(CAND_DestinationID did, CAND_ParameterID pid, Uint16 param)
{
    // assemble packet
    CAND_SID sid;
    Uint8 payload[2];

    sid.sidWord                 = 0;
    sid.all.m_id                = CAND_MID_PARAMETER_SET;
    sid.param_set.all.d_id    = did;
    sid.param_set.all.param   = pid;
    sid.param_set.extended.addr_mode = CAND_ADDR_MODE_EXTENDED;

    if (pid < CAND_PID_WORD_CUTOFF) {
        //16 bit param:
        payload[0] = (param>>8)&0xff;
        payload[1] = param&0xff;
    } else {
        payload[0] = param&0xff;
    }

    // send message to destinations provided
    return cand_tx(sid, payload, (pid < CAND_PID_WORD_CUTOFF) ? 2 : 1);
}

CAND_Result cand_tx_multi_response( CAND_ParameterID *pid, Uint16 *val, uint8_t resp_cnt )
{
	CAND_SID sid;
	uint8_t payload[10], pcnt=0;

	sid.sidWord = 0;
	sid.param_query.m_id = CAND_MID_PARAMETER_QUERY;
	sid.param_query.d_id = CAND_ID_EL; // EL axis is control board
	sid.param_query.s_id = CAND_GetSenderID();
	sid.param_query.dir = CAND_DIR_RESPONSE;

	while(resp_cnt) {
		payload[pcnt++] = pid[resp_cnt-1];
		if (pid[resp_cnt-1] < CAND_PID_WORD_CUTOFF) {
			payload[pcnt++] = (val[resp_cnt-1]>>8)&0xff;
		}
		payload[pcnt++] = val[resp_cnt-1]&0xff;
		resp_cnt--;
	}

	return cand_tx(sid, payload, pcnt);
}

CAND_Result cand_tx_response( CAND_DestinationID did, CAND_ParameterID pid, Uint16 val )
{
	CAND_SID sid;
	uint8_t payload[10], pcnt=0;

	sid.sidWord = 0;
	sid.param_query.m_id = CAND_MID_PARAMETER_QUERY;
	sid.param_query.d_id = did;
	sid.param_query.s_id = CAND_GetSenderID();
	sid.param_query.dir = CAND_DIR_RESPONSE;

	payload[pcnt++] = pid;
	if (pid < CAND_PID_WORD_CUTOFF) {
		payload[pcnt++] = (val>>8)&0xff;
	}

	payload[pcnt++] = val&0xff;
	return cand_tx(sid, payload, pcnt);
}

CAND_Result cand_tx_command(CAND_DestinationID did, CAND_Command cmd)
{
    CAND_SID sid;

    sid.sidWord = 0;
    sid.all.m_id = CAND_MID_COMMAND;
    sid.directive.d_id = did;
    sid.directive.command = cmd;

    return cand_tx(sid, NULL, 0);
}

CAND_Result cand_tx_fault( CAND_FaultCode fault_code )
{
	CAND_SID sid;

	sid.sidWord             = 0;
	sid.all.m_id            = CAND_MID_FAULT;     //000 0000 0100
	sid.fault.s_id 			= CAND_GetSenderID();
	sid.fault.fault_code    = fault_code;

	return cand_tx(sid, NULL, 0);
}

CAND_Result cand_tx(CAND_SID sid, uint8_t* p_data, uint8_t p_data_size)
{
	Uint16 i;
	struct MBOX mbox;

	mbox.MSGCTRL.all = 0;
	mbox.MSGID.all = 0;
	mbox.MSGID.bit.STDMSGID = sid.field.SID;
	mbox.MSGCTRL.bit.DLC = p_data_size;

	mbox.MDL.all = 0;
	mbox.MDH.all = 0;

	for (i=0; i<p_data_size; i++) {
		if (i<4) {
			mbox.MDL.all |= (Uint32) p_data[i]<<(8*(3-i));
		} else {
			mbox.MDH.all |= (Uint32) p_data[i]<<(8*((3-i)-4));
		}
	}

	ECanTx(&mbox);

	return CAND_SUCCESS;
}

