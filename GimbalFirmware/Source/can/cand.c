#include "PeripheralHeaderIncludes.h"
#include "F2806x_Examples.h" // For DELAY_US

#include "can/cand_BitFields.h"
#include "can/cand.h"
#include "hardware/HWSpecific.h"
#include "hardware/device_init.h"
#include "PM_Sensorless.h"
#include "hardware/uart.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

static void ECanInitGpio(void);
static void ECanTx(struct MBOX* outbox);
static int ECanRx(struct MBOX* inbox);

#define CAN_RX_MBOX_CNT 16
#define CAN_TX_MBOX_CNT 16

static uint8_t can_tx_mbox = 0;
static sysid_t sysid = {0},temp_sysid = {0};
static uint8_t msg_pass=0,torque_pass=0,gyro_pass=0,accel_pass=0;//,pass=0;
CAND_ParameterID immediate_pid_lookup_buffer[6] = {
    CAND_PID_TORQUE,
    CAND_PID_POSITION,
    CAND_PID_INVALID,
    CAND_PID_INVALID,
    CAND_PID_INVALID,
    CAND_PID_INVALID
};

void ECanInit(void)        // Initialize eCAN-A module
{
	volatile union CANLAM_REG* lam;
	volatile struct MBOX* mbox;
	int i;

	/* Create a shadow register structure for the CAN control registers. This is
     needed, since only 32-bit access is allowed to these registers. 16-bit access
     to these registers could potentially corrupt the register contents or return
     false data. */

	volatile struct ECAN_REGS ECanaShadow;

	ECanInitGpio();

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
    ECanaShadow.CANMC.bit.SUSP = 1; // Set peripheral to free run while processor is halted at a breakpoint
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

	// Disable Mailboxes
	ECanaRegs.CANME.all = 0;        // Required before writing the MSGIDs

	ECanaRegs.CANMD.all = 0xFFFFFFFF << CAN_TX_MBOX_CNT;	///< Boxes 0-15 are Tx, 16-31 Rx, (1 = rx)

	// Configure overwrite protection for all rx mailboxes
	ECanaRegs.CANOPC.all = 0xFFFFFFFF << CAN_TX_MBOX_CNT; // 1 = overwrite protection

	// Setup Rx Mailboxes
	for(i = CAN_TX_MBOX_CNT; i < (CAN_TX_MBOX_CNT + CAN_RX_MBOX_CNT); i++) {
		// CTRL reg needs cleared, setting ID to default value for testing
		mbox = &ECanaMboxes.MBOX0 + i;
		mbox->MSGCTRL.all = 0x00000000;
		mbox->MSGID.bit.IDE = 0;
		mbox->MSGID.bit.STDMSGID = 0x7FF;
		mbox->MSGID.bit.AME = 1;

		// All Rx boxes accept all messages
		lam = &ECanaLAMRegs.LAM0 + i;
		lam->all = 0x1FFFFFFF;
	}

	// Setup torque command specific mailbox
    ECanaMboxes.MBOX31.MSGID.bit.STDMSGID = 0x581; // 0x581 is the SID signature of the torque command message
    ECanaLAMRegs.LAM31.all = 0x0003FFFF; // 11 Message ID bits must match exactly
	ECanaRegs.CANOPC.bit.OPC31 = 0; // Disable over-write protection (mbox will hold latest value)

	// Enable interrupts for the torque mailbox
	ECanaRegs.CANMIM.bit.MIM31 = 1; // Interrupt enable mask (enabled)
	ECanaRegs.CANMIL.bit.MIL31 = 0; // Interrupt level mask (level 0)

	//ECanaShadow.CANGIM.all = ECanaRegs.CANGIM.all;
	ECanaRegs.CANGIM.bit.I0EN = 1; // Enable ECAN0INT
	ECanaRegs.CANGIM.bit.GIL = 1; // Global interrupts trigger ECAN1INT
	//ECanaRegs.CANGIM.all = ECanaShadow.CANGIM.all;

	// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
	//  as a matter of precaution.

    ECanaRegs.CANTA.all = 0xFFFFFFFF;   /* Clear all TAn bits */
    ECanaRegs.CANRMP.all = 0xFFFFFFFF;  /* Clear all RMPn bits */
    ECanaRegs.CANGIF0.all = 0xFFFFFFFF; /* Clear all interrupt flag bits */
    ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

    /* Configure bit timing parameters for eCANA*/

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 1;            // Set CCR = 1
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU has been granted permission to change the configuration registers
    do
    {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1);       // Wait for CCE bit to be set..

    ECanaShadow.CANBTC.all = 0;
    /* The following block is for 80 MHz SYSCLKOUT. (40 MHz CAN module clock Bit rate = 1 Mbps
       See Note at end of file. */

    ECanaShadow.CANBTC.bit.BRPREG = 1;
    ECanaShadow.CANBTC.bit.TSEG2REG = 4;
    ECanaShadow.CANBTC.bit.TSEG1REG = 13;

    ECanaShadow.CANBTC.bit.SAM = 1;
    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 0;            // Set CCR = 0
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU no longer has permission to change the configuration registers
    do
    {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0);       // Wait for CCE bit to be  cleared..

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
static void ECanTx(struct MBOX* outbox)
{
	volatile struct MBOX *mptr;
	unsigned long temp;

    //get next output box
    mptr = &ECanaMboxes.MBOX0 + can_tx_mbox;

    temp = ECanaRegs.CANME.all;
    temp  &= ~((0x00000001ul) << can_tx_mbox);

    ECanaRegs.CANME.all = temp;

    mptr->MDH = outbox->MDH;
    mptr->MDL = outbox->MDL;
    mptr->MSGCTRL = outbox->MSGCTRL;
    mptr->MSGID = outbox->MSGID;

    temp |= (1ul << can_tx_mbox);
    ECanaRegs.CANME.all = temp;
    ECanaRegs.CANTRS.all = (1ul << can_tx_mbox);	// "writing 0 has no effect", previously queued boxes will stay queued
    ECanaRegs.CANTA.all = (1ul << can_tx_mbox);		// "writing 0 has no effect", clears pending interrupt, open for our tx

    // Reset the round-robin counter
    if (++can_tx_mbox >= CAN_TX_MBOX_CNT) {
        can_tx_mbox = 0;
    }
}

/**
 * Check if any Mailbox is currently pending, if
 * so, copy to inbox
 */
static int ECanRx(struct MBOX* inbox)
{
	volatile struct MBOX *mptr;

	//Check if Rx Message Pending
	if (ECanaRegs.CANRMP.all) {
		Uint32 b, f;

		for (f = 0x80000000, b = ((CAN_TX_MBOX_CNT + CAN_RX_MBOX_CNT) - 1); ; f >>= 1, b--){
			if (ECanaRegs.CANRMP.all & f) {

				mptr = &ECanaMboxes.MBOX0 + b;
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

CAND_Result cand_init(void)
{
    // Make sure that the board hw id pins are pulled correctly.  The ID all axes isn't a valid
    // id for a single axis to have (it's used for broadcast to all axes).  If this is the board hw id,
    // send out a fault message to indicate the bad board ID
	if (CAND_GetSenderID() == CAND_ID_ALL_AXES) {
	    return CAND_INIT_BAD_HW_ID;
	} else {
	    return CAND_SUCCESS;
	}
}

CAND_SenderID CAND_GetSenderID(void)
{
    switch (GetBoardHWID()) {
    	case EL:
    	    return CAND_ID_EL;

    	case AZ:
    	    return CAND_ID_AZ;

    	case ROLL:
    	    return CAND_ID_ROLL;

    	default:
    	    return CAND_ID_ALL_AXES;
    }
}

bool CAND_InDestinationList(CAND_DestinationID did)
{
    CAND_SenderID me = CAND_GetSenderID();

    if (did == CAND_ID_ALL_AXES) {
        return true;
    }

    if (me == did) {
    	return true;
    }

    return false;
}

CAND_Result cand_rx(struct cand_message * msg)
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
				msg->sender_id = (CAND_SenderID)sid.fault.s_id;

				msg->fault_code = (CAND_FaultCode)sid.fault.fault_code;
				msg->fault_type = (CAND_FaultType)sid.fault.fault_type;

				ret = CAND_RX_FAULT;
			}
			break;

			case CAND_MID_COMMAND:
				// Check if we are on the recipient list
				if (CAND_InDestinationList((CAND_DestinationID)sid.directive.d_id) == false) {
					break;
				}
				msg->command = (CAND_Command)sid.directive.command;
				ret = CAND_RX_COMMAND;
				break;

			case CAND_MID_PARAMETER_SET:
				// Check if we are on the recipient list
				if (CAND_InDestinationList((CAND_DestinationID)sid.param_set.all.d_id) == false) {
					break;
				}

				// Addressing mode sets whether to interpret the parameter
				// ID as a register with up to 6 parameter flags set, or just a
				// single parameter ID.
				if (sid.param_set.all.addr_mode == CAND_ADDR_MODE_IMMEDIATE) {

					// If there are multiple destinations for a single immediate parameter
					// then our data is packed in with the others and we have to get it out:
					if (sid.param_set.all.d_id == CAND_ID_ALL_AXES) {
						// Extract out our parameter
					    // Convert the 1 hot parameter ID back to a normal parameter ID
					    uint8_t p_flag, shift_cnt;
					    for (p_flag = 1, shift_cnt = 0; p_flag <= 0x20; p_flag <<= 1, shift_cnt++) {
					        if (p_flag & sid.param_set.immediate.param_reg) {
					            msg->param_id[0] = immediate_pid_lookup_buffer[shift_cnt];
					            break;
					        }
					    }
                        uint8_t format_id = SID_FORMAT_MSG,format_size = sizeof(sid_format_t), data_size = sizeof(sysid), data_id = SID_DATA_MSG;
                        uint8_t fmt_name[4]="FMT";
                        uint8_t fmt_fmt[16] = "BBnNZ";
                        uint8_t fmt_labels[64] = "Type,Length,Name,Format,Columns";

                        uint8_t data_name[4]="SID";
                        uint8_t data_fmt[16] = "IBbbhhhhhhh";

                        uint8_t data_labels[64] = "seq,torque_axis,roll,el,torque,gy_x,gy_y,gy_z,acc_x,acc_y,acc_z";
                        uint8_t head_bytes[] = {HEAD_BYTE1,HEAD_BYTE2,128};
                        static uint8_t format_sent = 0;
                        static uint32_t seq_no = 0;
                        //char tdata[10];
                        //uint8_t tdata_size;
						switch (CAND_GetSenderID()) {
							case CAND_ID_AZ:
								    if(format_sent == 0){
				                        uart_send_data((Uint8*)&head_bytes,sizeof(head_bytes));
				                        uart_send_data((Uint8*)&format_id,sizeof(format_id));
				                        uart_send_data((Uint8*)&format_size,sizeof(format_size));
				                        uart_send_data((Uint8*)fmt_name,sizeof(fmt_name));
				                        uart_send_data((Uint8*)fmt_fmt,sizeof(fmt_fmt));
				                        uart_send_data((Uint8*)fmt_labels,sizeof(fmt_labels));
				                        
				                        uart_send_data((Uint8*)&head_bytes,sizeof(head_bytes));
				                        uart_send_data((Uint8*)&data_id,sizeof(data_id));
				                        uart_send_data((Uint8*)&data_size,sizeof(data_size));
				                        uart_send_data((Uint8*)data_name,sizeof(data_name));
				                        uart_send_data((Uint8*)data_fmt,sizeof(data_fmt));
				                        uart_send_data((Uint8*)data_labels,sizeof(data_labels));
				                        format_sent++;
                                    }

							        switch(mbox.MDL.word.HI_WORD){
							            case 0:
							                    msg->param_cnt = 0;
							                    seq_no++;
							                    msg_pass++;
							                    //pass++;
							                    //tdata_size = sprintf(tdata,"0,%d ",msg_pass);
							                    //uart_send_data((Uint8*)tdata,tdata_size);
							                    if(msg_pass > 1) {
							                        temp_sysid.seq_no[3] = (seq_no>>24)&0xff;
							                        temp_sysid.seq_no[2] = (seq_no>>16)&0xff;
							                        temp_sysid.seq_no[1] = (seq_no>>8)&0xff;
							                        temp_sysid.seq_no[0] = seq_no&0xff;
							                        temp_sysid.torque_axis = mbox.MDL.word.LOW_WORD;
							                        temp_sysid.roll_ang = mbox.MDH.word.HI_WORD;
							                        temp_sysid.el_ang = mbox.MDH.word.LOW_WORD;
							                        break;
							                    }

							                    sysid.seq_no[3] = (seq_no>>24)&0xff;
							                    sysid.seq_no[2] = (seq_no>>16)&0xff;
							                    sysid.seq_no[1] = (seq_no>>8)&0xff;
							                    sysid.seq_no[0] = seq_no&0xff;
							                    
							                    sysid.torque_axis = mbox.MDL.word.LOW_WORD;
							                    sysid.roll_ang = mbox.MDH.word.HI_WORD;
							                    sysid.el_ang = mbox.MDH.word.LOW_WORD;
							                    break;
							             case 1:
							                    msg->param_cnt = 1;
							                    msg->param[0] = mbox.MDH.word.LOW_WORD;
							                    torque_pass++;
							                    //pass++;
							                    //tdata_size = sprintf(tdata,"1,%d ",torque_pass);
							                    //uart_send_data((Uint8*)tdata,tdata_size);
							                    if(torque_pass>1) {
							                        switch(sysid.torque_axis){
							                            case ROLL: temp_sysid.torque[1] = mbox.MDL.word.LOW_WORD>>8;
							                                    temp_sysid.torque[0] = mbox.MDL.word.LOW_WORD&0xff;
							                                    break;
							                            case EL: temp_sysid.torque[1] = mbox.MDH.word.HI_WORD>>8;
							                                    temp_sysid.torque[0] = mbox.MDH.word.HI_WORD&0xff;
							                                    break;
							                            case AZ: temp_sysid.torque[1] = mbox.MDH.word.LOW_WORD>>8;
							                                    temp_sysid.torque[0] = mbox.MDH.word.LOW_WORD&0xff;
							                                    break;
							                         }
							                         break;
							                    }
							                    
							                    switch(sysid.torque_axis){
							                        case ROLL: sysid.torque[1] = mbox.MDL.word.LOW_WORD>>8;
							                                sysid.torque[0] = mbox.MDL.word.LOW_WORD&0xff;
							                                break;
							                        case EL: sysid.torque[1] = mbox.MDH.word.HI_WORD>>8;
							                                sysid.torque[0] = mbox.MDH.word.HI_WORD&0xff;
							                                break;
							                        case AZ: sysid.torque[1] = mbox.MDH.word.LOW_WORD>>8;
							                                sysid.torque[0] = mbox.MDH.word.LOW_WORD&0xff;
							                                break;
							                    }

							                    break;
							              case 2:
							                    msg->param_cnt = 0;
							                    gyro_pass++;
							                    //pass++;
							                    //tdata_size = sprintf(tdata,"2,%d ",gyro_pass);
							                    //uart_send_data((Uint8*)tdata,tdata_size);
							                    if(gyro_pass > 1) {
							                        temp_sysid.gyro_x[1] = mbox.MDL.word.LOW_WORD>>8;
							                        temp_sysid.gyro_x[0] = mbox.MDL.word.LOW_WORD & 0xff;
							                        
							                        temp_sysid.gyro_y[1] = mbox.MDH.word.HI_WORD>>8;
							                        temp_sysid.gyro_y[0] = mbox.MDH.word.HI_WORD & 0xff;
							                        
							                        temp_sysid.gyro_z[1] = mbox.MDH.word.LOW_WORD>>8;
							                        temp_sysid.gyro_z[0] = mbox.MDH.word.LOW_WORD & 0xff;
							                        break;
							                    }
							                    sysid.gyro_x[1] = mbox.MDL.word.LOW_WORD>>8;
							                    sysid.gyro_x[0] = mbox.MDL.word.LOW_WORD & 0xff;
							                    
							                    sysid.gyro_y[1] = mbox.MDH.word.HI_WORD>>8;
							                    sysid.gyro_y[0] = mbox.MDH.word.HI_WORD & 0xff;
							                    
							                    sysid.gyro_z[1] = mbox.MDH.word.LOW_WORD>>8;
							                    sysid.gyro_z[0] = mbox.MDH.word.LOW_WORD & 0xff;
							                    

							                    break;
							              case 3:
							                    msg->param_cnt = 0;
							                    accel_pass++;
							                    //pass++;
							                    //tdata_size = sprintf(tdata,"3,%d ",accel_pass);
							                    //uart_send_data((Uint8*)tdata,tdata_size);
							                    if(accel_pass > 1) {
							                        temp_sysid.accel_x[1] = mbox.MDL.word.LOW_WORD>>8;
							                        temp_sysid.accel_x[0] = mbox.MDL.word.LOW_WORD & 0xff;
							                        
							                        temp_sysid.accel_y[1] = mbox.MDH.word.HI_WORD>>8;
							                        temp_sysid.accel_y[0] = mbox.MDH.word.HI_WORD&0xff;
							                        
							                        temp_sysid.accel_z[1] = mbox.MDH.word.LOW_WORD>>8;
							                        temp_sysid.accel_z[0] = mbox.MDH.word.LOW_WORD&0xff;
							                        
							                        break;
							                    }
							                    sysid.accel_x[1] = mbox.MDL.word.LOW_WORD>>8;
							                    sysid.accel_x[0] = mbox.MDL.word.LOW_WORD & 0xff;
							                    
							                    sysid.accel_y[1] = mbox.MDH.word.HI_WORD>>8;
							                    sysid.accel_y[0] = mbox.MDH.word.HI_WORD&0xff;
							                    
							                    sysid.accel_z[1] = mbox.MDH.word.LOW_WORD>>8;
							                    sysid.accel_z[0] = mbox.MDH.word.LOW_WORD&0xff;

							                    break;
							         }
							         if(millis() <=8000){
		                                accel_pass = 0;
		                                gyro_pass = 0;
		                                torque_pass = 0;
		                                msg_pass = 0;    
		                             }
							         if(accel_pass == 0 || gyro_pass == 0 || torque_pass == 0 || msg_pass == 0) {
							            break;
							         }
							         
		                            sysid.head1 = HEAD_BYTE1;
		                            sysid.head2 = HEAD_BYTE2;
		                            sysid.msgid = 129;
                                    if(accel_pass == gyro_pass && gyro_pass == msg_pass && gyro_pass == torque_pass) {
                                        accel_pass=0;
                                        gyro_pass=0;
                                        msg_pass=0;
                                        torque_pass=0;
		                                if(millis() >=8000){
		                                    //tdata_size = sprintf(tdata," %ld,%d\n", seq_no,sysid.roll_ang);
                                            //uart_send_data((Uint8*)tdata,tdata_size);
                                            //pass = 0;
		                                    uart_send_data((Uint8*)&sysid, sizeof(sysid));
		                                }
                                            
                                    } else {
                                        accel_pass--;
                                        gyro_pass--;
                                        msg_pass--;
                                        torque_pass--;
                                        memcpy(&sysid, &temp_sysid,sizeof(temp_sysid));
                                    }
							    break;

							case CAND_ID_ROLL:
							    msg->param[0] = mbox.MDL.word.LOW_WORD;
							    break;

							default:
							    break;
						}
					} else {
						// Must just be a parameter for us (don't need to check broadcast, immediate
						// parameters for broadcast aren't a thing)
						uint8_t p_flag, p_cnt, shift_cnt;

						for (p_flag = 1, p_cnt = 0, shift_cnt = 0; p_flag <= 0x20 && p_cnt < 4; p_flag <<= 1, shift_cnt++ ) {

							if (p_flag & sid.param_set.immediate.param_reg) {
							    // Convert the 1 hot parameter ID back to a normal parameter ID
								msg->param_id[p_cnt] = (CAND_ParameterID)immediate_pid_lookup_buffer[shift_cnt];

								switch (p_cnt) {
									case 0:
									    msg->param[p_cnt] = mbox.MDL.word.HI_WORD;
									    break;

									case 1:
									    msg->param[p_cnt] = mbox.MDL.word.LOW_WORD;
									    break;

									case 2:
									    msg->param[p_cnt] = mbox.MDH.word.HI_WORD;
									    break;

									case 3:
									    msg->param[p_cnt] = mbox.MDH.word.LOW_WORD;
									    break;

									default:
									    break;
								}
								p_cnt++;
							}
						}
						msg->param_cnt = p_cnt;
					}
				} else {
					// Extended mode, rather than having a register with flags for parameters, we
					// get a list of ID/params to set, first ID is in the last 6 bits of the arbitration
					// field

				    // First need to check if this message contains extended parameter IDs
				    // If it does, it requires special processing
				    CAND_ParameterID first_parameter_id = (CAND_ParameterID)sid.param_set.extended.param_id;
				    if (first_parameter_id == CAND_PID_EXTENDED) {
				        int i;
				        msg->param_id[0] = CAND_PID_EXTENDED;
				        msg->extended_param_id = (CAND_ExtendedParameterID)CANMD8(mbox, 0); // First byte of payload is extended parameter ID
				        // Extract the remainder of the message payload into the parsed message
				        for (i = 1; i < mbox.MSGCTRL.bit.DLC; i++) {
				            msg->extended_param[i - 1] = CANMD8(mbox, i);
				        }
				        msg->extended_param_length = mbox.MSGCTRL.bit.DLC - 1; // Subtract 1 for the extended parameter ID
				        msg->param_cnt = 1;
				    } else {
				        int byte_in_payload = 0, p_cnt = 0;

                        while ((mbox.MSGCTRL.bit.DLC > byte_in_payload)) {

                            // Get the special case ID0 out of the arbitration field, else
                            // the next byte in the payload is the next ID
                            if (p_cnt == 0) {
                                msg->param_id[p_cnt] = first_parameter_id;
                            } else {
                                msg->param_id[p_cnt] = (CAND_ParameterID)CANMD8(mbox, byte_in_payload);
                                byte_in_payload++;
                            }

                            // Params can be either 4, 2, or 1 byte.  We can determine which size each param is by
                            // its parameter ID.  List is packed, so we'll have to count our way through
                            if (msg->param_id[p_cnt] < CAND_PID_4_BYTE_CUTOFF) {
                                // Make sure we're not going to overrun the message payload by trying to read this parameter out of it.
                                // If we would, this is an ill formed message, so we just give up here and return the parameters that
                                // have already been parsed
                                if (byte_in_payload + 4 > 8) {
                                    break;
                                }

                                msg->param[p_cnt] = (CANMD8(mbox, byte_in_payload) << 24) | (CANMD8(mbox, byte_in_payload + 1) << 16) | (CANMD8(mbox, byte_in_payload + 2) << 8) | CANMD8(mbox, byte_in_payload + 3);
                                byte_in_payload += 4;
                            } else if (msg->param_id[p_cnt] < CAND_PID_2_BYTE_CUTOFF) {
                                // Make sure we're not going to overrun the message payload by trying to read this parameter out of it.
                                // If we would, this is an ill formed message, so we just give up here and return the parameters that
                                // have already been parsed
                                if (byte_in_payload + 2 > 8) {
                                    break;
                                }

                                msg->param[p_cnt] = ((CANMD8(mbox, byte_in_payload) << 8) | CANMD8(mbox, byte_in_payload + 1)) & 0x0000FFFF;
                                byte_in_payload += 2;
                            } else {
                                // Make sure we're not going to overrun the message payload by trying to read this parameter out of it.
                                // If we would, this is an ill formed message, so we just give up here and return the parameters that
                                // have already been parsed
                                if (byte_in_payload + 1 > 8) {
                                    break;
                                }

                                msg->param[p_cnt] = CANMD8(mbox, byte_in_payload) & 0x000000FF;
                                byte_in_payload += 1;
                            }

                            p_cnt++;
                        }
                        msg->param_cnt = p_cnt;
				    }
				}
				ret = CAND_RX_PARAM_SET;
				break;

			case CAND_MID_PARAMETER_QUERY:
				// Check if we are on the recipient list
				if (CAND_InDestinationList((CAND_DestinationID)sid.param_query.d_id) == false) {
					break;
				}

				msg->sender_id = (CAND_SenderID)sid.param_query.s_id;

				// This is either a new query or a response to a query that we sent out
				if (sid.param_query.dir == CAND_DIR_QUERY) {
					// New Query! Extract out the IDs for queried params
					int p_cnt=0;
					msg->param_repeat = sid.param_query.repeat;

					while (p_cnt < mbox.MSGCTRL.bit.DLC) {
						msg->param_request_id[p_cnt] = (CAND_ParameterID)CANMD8(mbox, p_cnt);
						p_cnt++;
					}

					msg->param_request_cnt = p_cnt;
					ret = CAND_RX_PARAM_QUERY;
				} else {
					// Response! Extract out the ID/param pairs
					int byte_in_payload=0, p_cnt=0;

					// similar deal as set parameter in extended mode, except all
					// ID/value pairs are in the payload
					while( mbox.MSGCTRL.bit.DLC > byte_in_payload) {

						msg->param_response_id[p_cnt] = (CAND_ParameterID)CANMD8(mbox, byte_in_payload);
						byte_in_payload++;

						// Params can be either 4, 2, or 1 byte.  We can determine which size each param is by
                        // its parameter ID.  List is packed, so we'll have to count our way through
                        if (msg->param_response_id[p_cnt] < CAND_PID_4_BYTE_CUTOFF) {
                            // Make sure we're not going to overrun the message payload by trying to read this parameter out of it.
                            // If we would, this is an ill formed message, so we just give up here and return the parameters that
                            // have already been parsed
                            if (byte_in_payload + 4 > 8) {
                                break;
                            }

                            msg->param_response[p_cnt] = (CANMD8(mbox, byte_in_payload) << 24) | (CANMD8(mbox, byte_in_payload + 1) << 16) | (CANMD8(mbox, byte_in_payload + 2) << 8) | CANMD8(mbox, byte_in_payload + 3);
                            byte_in_payload += 4;
                        } else if (msg->param_response_id[p_cnt] < CAND_PID_2_BYTE_CUTOFF) {
                            // Make sure we're not going to overrun the message payload by trying to read this parameter out of it.
                            // If we would, this is an ill formed message, so we just give up here and return the parameters that
                            // have already been parsed
                            if (byte_in_payload + 2 > 8) {
                                break;
                            }

                            msg->param_response[p_cnt] = ((CANMD8(mbox, byte_in_payload) << 8) | CANMD8(mbox, byte_in_payload + 1)) & 0x0000FFFF;
                            byte_in_payload += 2;
                        } else {
                            // Make sure we're not going to overrun the message payload by trying to read this parameter out of it.
                            // If we would, this is an ill formed message, so we just give up here and return the parameters that
                            // have already been parsed
                            if (byte_in_payload + 1 > 8) {
                                break;
                            }

                            msg->param_response[p_cnt] = CANMD8(mbox, byte_in_payload) & 0x000000FF;
                            byte_in_payload += 1;
                        }

						p_cnt++;
					}
					msg->param_response_cnt = p_cnt;
					ret = CAND_RX_PARAM_RESPONSE;
				}

				break;

			default:
				ret = CAND_RX_PARSE_ERROR;
				break;
		}
	}
	return ret;
}

CAND_Result cand_tx_multi_param(CAND_DestinationID did, CAND_ParameterID* pid, Uint32* param, Uint8 param_cnt)
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
    // 1: that the destination id is a multi-endpoint desitination, and we're sending a single pid
    // 2: that the destination id is a multi-endpoint desitination, and we're sending multiple pids
    // 3: that the destination id is a single endpoint and we're sending multiple pids
    // 4: that the destination id is a single endpoint and we're sending 1 pid (then you should be using cand_tx_param...)

    // This is nearly done, but TODO: add logic to detect when Immediate mode
    // could be used, current implementation isn't quite as efficient as it could
    // be

    if (did == CAND_ID_ALL_AXES) {
        //  Sending to Multiple Destinations, packet format is as follows:
        //  Param ID is loaded into the arbitration field, and the params
        //  are loaded into the payload as an ordered list, MSParam is the
        //  lowest destination id in the destination list

        // when sending to the two axes, it could be a 1x1 byte, 2x1 byte or
        // 1x2 byte params
        if ((pid[0] < CAND_PID_2_BYTE_CUTOFF) && (pid[0] > CAND_PID_4_BYTE_CUTOFF)) {
            // only 1 param of 2 byte to this destination

            Uint8 one_hot_id = 0;
            int i;
            for (i = 0; i < 6; i++) {
                if (immediate_pid_lookup_buffer[i] == pid[0]) {
                    one_hot_id = 1 << i;
                    break;
                }
            }

            sid.param_set.all.param = one_hot_id;

            payload[0] = (param[0] >> 8) & 0xff;
            payload[1] = param[0] & 0xff;
            payload[2] = (param[1] >> 8) & 0xff;
            payload[3] = param[1] & 0xff;
            payload_cnt = 4;
            sid.param_set.extended.addr_mode = CAND_ADDR_MODE_IMMEDIATE;

        } else if (param_cnt == 1) {
            // 1x 1 byte param
            sid.param_set.all.param = pid[0];

            payload[0] = param[0] & 0xff;
            payload[1] = param[1] & 0xff;
            payload_cnt = 2;

        } else if (param_cnt == 2) {
            // 2x 1 byte param, this is the only case '2' from above
            sid.param_set.all.param = pid[0];

            payload[0] = param[0] & 0xff;
            payload[1] = param[1] & 0xff;
            payload[2] = pid[1];
            payload[3] = param[2] & 0xff;
            payload[4] = param[3] & 0xff;
            payload_cnt = 5;
        }
    } else {
        Uint8 param_in_cnt = 0, pid_cnt = 0;

        // Now we must be sending multiple parameters to a single endpoint
        if (param_cnt == 1) {
            // you should call the right function:
            return cand_tx_param(did, pid[0], param[0]);
        }

        // this is where we need to check whether all pids are <CAND_PID_WORD_CUTOFF,
        // so we know we can send in immediate mode

        payload_cnt = 0;

        for (param_in_cnt = 0; (param_in_cnt < param_cnt) && (payload_cnt < 8); param_in_cnt++, pid_cnt++) {

            if (pid_cnt == 0) {
                // first id is stuffed into arbitration field
                sid.param_set.all.param = pid[pid_cnt];
            } else {
                payload[payload_cnt++] = pid[pid_cnt];
            }

            if ((pid[pid_cnt] < CAND_PID_2_BYTE_CUTOFF) && (pid[pid_cnt] > CAND_PID_4_BYTE_CUTOFF)) {
                // 16-bit param
                payload[payload_cnt] = (param[param_in_cnt] >> 8) & 0xff;
                payload[payload_cnt + 1] = param[param_in_cnt] & 0xff;
                payload_cnt += 2;
            } else if (pid[pid_cnt] > CAND_PID_2_BYTE_CUTOFF) {
                // 8-bit param
                payload[payload_cnt] = param[param_in_cnt] & 0xff;
                payload_cnt += 1;
            } else {
                // 32-bit param.  We don't support multi-param tramsmits with 4-byte parameters, so return an error code
                return CAND_TX_UNSUPPORTED_PARAM;
            }
        }
    }

    return cand_tx(sid, payload, payload_cnt);
}

CAND_Result cand_tx_param(CAND_DestinationID did, CAND_ParameterID pid, Uint32 param)
{
    // assemble packet
    CAND_SID sid;
    Uint8 payload[4];
    uint8_t payload_size = 0;

    sid.sidWord = 0;
    sid.all.m_id = CAND_MID_PARAMETER_SET;
    sid.param_set.all.d_id = did;
    sid.param_set.all.param = pid;
    sid.param_set.extended.addr_mode = CAND_ADDR_MODE_EXTENDED;

    if (pid < CAND_PID_4_BYTE_CUTOFF) {
        // 32-bit param
        payload[0] = (param >> 24) & 0x000000FF;
        payload[1] = (param >> 16) & 0x000000FF;
        payload[2] = (param >> 8) & 0x000000FF;
        payload[3] = param & 0x000000FF;
        payload_size = 4;
    } else if (pid < CAND_PID_2_BYTE_CUTOFF) {
        // 16-bit param
        payload[0] = (param >> 8) & 0x000000FF;
        payload[1] = param & 0x000000FF;
        payload_size = 2;
    } else {
        // 8-bit param
        payload[0] = param & 0x000000FF;
        payload_size = 1;
    }

    // send message to destinations provided
    return cand_tx(sid, payload, payload_size);
}

CAND_Result cand_tx_extended_param(CAND_DestinationID did, CAND_ExtendedParameterID epid, uint8_t* param, int param_length)
{
    CAND_SID sid;
    Uint8 payload[8];
    int payload_length = param_length;
    int i;

    sid.sidWord = 0;
    sid.all.m_id = CAND_MID_PARAMETER_SET;
    sid.param_set.all.d_id = did;
    sid.param_set.all.param = CAND_PID_EXTENDED;
    sid.param_set.extended.addr_mode = CAND_ADDR_MODE_EXTENDED;

    // Extended parameter ID is first byte in payload
    payload[0] = epid;

    // Make sure we don't overflow the payload
    // Max size of extended param is 7 bytes
    if (payload_length > 7) {
        payload_length = 7;
    }

    // Rest of payload is actual parameter
    for (i = 0; i < payload_length; i++) {
        payload[i + 1] = param[i];
    }

    return cand_tx(sid, payload, payload_length + 1);
}

CAND_Result cand_tx_request(CAND_DestinationID did, CAND_ParameterID pid)
{
    CAND_SID sid;
    uint8_t payload;

    sid.sidWord = 0;
    sid.param_query.m_id = CAND_MID_PARAMETER_QUERY;
    sid.param_query.d_id = did;
    sid.param_query.s_id = CAND_GetSenderID();
    sid.param_query.dir = CAND_DIR_QUERY;
    sid.param_query.repeat = 0;

    payload = pid;

    return cand_tx(sid, &payload, 1);
}

CAND_Result cand_tx_multi_request(CAND_DestinationID did, CAND_ParameterID* pids, uint8_t request_cnt)
{
    CAND_SID sid;
    uint8_t payload[8];
    uint8_t i;

    // We only support requesting up to 8 parameters at a time
    if (request_cnt > 8) {
        return CAND_TX_TOO_MANY_PARAM_REQUEST_PIDS;
    }

    sid.sidWord = 0;
    sid.param_query.m_id = CAND_MID_PARAMETER_QUERY;
    sid.param_query.d_id = did;
    sid.param_query.s_id = CAND_GetSenderID();
    sid.param_query.dir = CAND_DIR_QUERY;
    sid.param_query.repeat = 0;

    for (i = 0; i < request_cnt; i++) {
        payload[i] = pids[i];
    }

    return cand_tx(sid, payload, request_cnt);
}

CAND_Result cand_tx_multi_response(CAND_DestinationID did, CAND_ParameterID *pid, Uint32 *val, uint8_t resp_cnt)
{
	CAND_SID sid;
	uint8_t payload[10], pcnt=0;

	sid.sidWord = 0;
	sid.param_query.m_id = CAND_MID_PARAMETER_QUERY;
	sid.param_query.d_id = did;
	sid.param_query.s_id = CAND_GetSenderID();
	sid.param_query.dir = CAND_DIR_RESPONSE;

	while (resp_cnt && (pcnt < 8)) {
		payload[pcnt++] = pid[resp_cnt - 1];

		if (pid[resp_cnt - 1] < CAND_PID_4_BYTE_CUTOFF) {
		    // 4 byte param response

		    // Make sure we won't overflow the payload by adding this parameter.  If we would,
		    // give up and transmit the param responses that have already been inserted into the payload
		    if (pcnt + 4 > 8) {
		        break;
		    }
		    payload[pcnt++] = (val[resp_cnt - 1] >> 24) & 0x000000FF;
		    payload[pcnt++] = (val[resp_cnt - 1] >> 16) & 0x000000FF;
		    payload[pcnt++] = (val[resp_cnt - 1] >> 8) & 0x000000FF;
		    payload[pcnt++] = val[resp_cnt - 1] & 0x000000FF;
		} else if (pid[resp_cnt - 1] < CAND_PID_2_BYTE_CUTOFF) {
		    // 2 byte param response

		    // Make sure we won't overflow the payload by adding this parameter.  If we would,
            // give up and transmit the param responses that have already been inserted into the payload
            if (pcnt + 2 > 8) {
                break;
            }
            payload[pcnt++] = (val[resp_cnt - 1] >> 8) & 0x000000FF;
            payload[pcnt++] = val[resp_cnt - 1] & 0x000000FF;
		} else {
		    // 1 byte param response

            // Make sure we won't overflow the payload by adding this parameter.  If we would,
            // give up and transmit the param responses that have already been inserted into the payload
            if (pcnt + 1 > 8) {
                break;
            }
            payload[pcnt++] = val[resp_cnt - 1] & 0x000000FF;
		}

		resp_cnt--;
	}

	return cand_tx(sid, payload, pcnt);
}

CAND_Result cand_tx_response(CAND_DestinationID did, CAND_ParameterID pid, Uint32 val)
{
	CAND_SID sid;
	uint8_t payload[8], pcnt=0;

	sid.sidWord = 0;
	sid.param_query.m_id = CAND_MID_PARAMETER_QUERY;
	sid.param_query.d_id = did;
	sid.param_query.s_id = CAND_GetSenderID();
	sid.param_query.dir = CAND_DIR_RESPONSE;

	payload[pcnt++] = pid;

	if (pid < CAND_PID_4_BYTE_CUTOFF) {
	    // 4 byte param response
	    payload[pcnt++] = (val >> 24) & 0x000000FF;
        payload[pcnt++] = (val >> 16) & 0x000000FF;
        payload[pcnt++] = (val >> 8) & 0x000000FF;
        payload[pcnt++] = val & 0x000000FF;
	} else if (pid < CAND_PID_2_BYTE_CUTOFF) {
	    // 2 byte param response
	    payload[pcnt++] = (val >> 8) & 0x000000FF;
        payload[pcnt++] = val & 0x000000FF;
	} else {
	    // 1 byte param response
	    payload[pcnt++] = val & 0x000000FF;
	}

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

CAND_Result cand_tx_fault(CAND_FaultCode fault_code, CAND_FaultType fault_type)
{
	CAND_SID sid;

	sid.sidWord = 0;
	sid.all.m_id = CAND_MID_FAULT;     //000 0000 0100
	sid.fault.s_id = CAND_GetSenderID();
	sid.fault.fault_code = fault_code;
	sid.fault.fault_type = fault_type;

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

	for (i = 0; i < p_data_size; i++) {
		if (i < 4) {
			mbox.MDL.all |= (Uint32)p_data[i] << (8 * (3 - i));
		} else {
			mbox.MDH.all |= (Uint32)p_data[i] << (8 * ((3 - i) - 4));
		}
	}

	ECanTx(&mbox);

	return CAND_SUCCESS;
}

