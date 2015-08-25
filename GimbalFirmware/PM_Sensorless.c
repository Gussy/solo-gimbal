#include "PM_Sensorless.h"
#include "PM_Sensorless-Settings.h"
#include "PeripheralHeaderIncludes.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "hardware/encoder.h"
#include "hardware/led.h"
#include "hardware/uart.h"
#include "hardware/interrupts.h"
#include "hardware/gyro.h"
#include "hardware/timing.h"
#include "can/cand_BitFields.h"
#include "can/cand.h"
#include "can/cb.h"
#include "can/can_message_processor.h"
#include "can/can_parameter_updates.h"
#include "control/gyro_kinematics_correction.h"
#include "control/PID.h"
#include "control/average_power_filter.h"
#include "control/running_average_filter.h"
#include "control/filt2p.h"
#include "motor/motor_drive_state_machine.h"
#include "motor/motor_commutation.h"
#include "parameters/flash_params.h"
#include "parameters/load_axis_parms_state_machine.h"
#include "parameters/mavlink_parameter_interface.h"
#include "flash/flash.h"
#include "flash/flash_init.h"
#include "hardware/watchdog.h"
#include "control/rate_loops.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "gopro/gopro_interface.h"
#include "helpers/fault_handling.h"
#include "version_git.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define getTempSlope() (*(int (*)(void))0x3D7E82)();
#define getTempOffset() (*(int (*)(void))0x3D7E85)();

static void check_rate_cmd_timeout(void);
static void update_LEDs(void);
static void check_gp_responses(void);
static void send_mav_heartbeat(void);
static void send_can_heartbeat(void);
static void read_temp_and_voltage(void);

struct SchedTask scheduled_tasks[] = {
    {.task_func=&check_rate_cmd_timeout,        .interval_ms=3,     .last_run_ms=1},
    {.task_func=&gp_interface_state_machine,    .interval_ms=3,     .last_run_ms=2},
    {.task_func=&update_LEDs,                   .interval_ms=150,   .last_run_ms=0},
    {.task_func=&check_gp_responses,            .interval_ms=150,   .last_run_ms=1},
    {.task_func=&read_temp_and_voltage,         .interval_ms=150,   .last_run_ms=2},
    {.task_func=&send_can_heartbeat,            .interval_ms=1000,  .last_run_ms=0},
    {.task_func=&send_mav_heartbeat,            .interval_ms=1000,  .last_run_ms=1}
};

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

// Global variables used in this system
int16 DegreesC;
static int16 TempOffset;
static int16 TempSlope;
static Uint8 board_hw_id = 0;
static float DcBusVoltage = 0.0f;

static Uint32 IsrTicker = 0;

static volatile bool GyroDataReadyFlag = false;
static volatile bool TorqueDemandAvailableFlag = false;

EncoderParms encoder_parms = {
    .raw_theta = 0,
    .virtual_counts = 0,
    .virtual_counts_accumulator = 0,
    .virtual_counts_accumulated = 0,
    .mech_theta = 0.0,
    .corrected_mech_theta = 0.0,
    .elec_theta = 0.0,
    .calibration_slope = 0.0,
    .calibration_intercept = 0.0
};

AxisParms axis_parms = {
    .blink_state = BLINK_INIT,
    .enable_flag = FALSE,
    .all_init_params_recvd = FALSE,
    .other_axis_hb_recvd = {FALSE, FALSE, FALSE},
    .other_axis_init_params_recvd = {FALSE, FALSE, FALSE},
    .other_axis_enable_retry_counter = 0
};

ControlBoardParms control_board_parms = {
    .gyro_readings = {0, 0, 0},
    .integrated_raw_gyro_readings = {0, 0, 0},
    .integrated_raw_accel_readings = {0, 0, 0},
    .encoder_readings = {0, 0, 0},
    .last_axis_fault = {CAND_FAULT_NONE, CAND_FAULT_NONE, CAND_FAULT_NONE},
    .encoder_value_received = {FALSE, FALSE, FALSE},
    .axes_homed = {FALSE, FALSE, FALSE},
    .calibration_status = {
        GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN,
        GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN,
        GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN
    },
    .rate_cmd_inject = {0, 0, 0},
    .rate_cmd_inject_filtered = {0, 0, 0},
    .rate_loop_step = 0,
	.control_type = CONTROL_TYPE_POS,
	.max_allowed_torque = LOW_TORQUE_MODE_MAX,
    .initialized = FALSE,
    .enabled = FALSE,
    .param_set = {0}
};

LoadAxisParmsStateInfo load_ap_state_info = {
	.current_load_offset = 0,
	.current_request_load_offset = 0,
	.total_words_to_load = 0,
    .request_retry_counter = REQUEST_RETRY_PERIOD,
	.axis_parms_checksum_verified = FALSE,
    .axis_parms_load_complete = FALSE
};

MavlinkGimbalInfo mavlink_gimbal_info = {
    .mav_state = MAV_STATE_UNINIT,
    .mav_mode = MAV_MODE_GIMBAL_UNINITIALIZED,
    .mavlink_processing_state = MAVLINK_STATE_PARSE_INPUT,
    .rate_cmd_timeout_counter = 0,
    .gimbal_active = FALSE
};

DebugData debug_data = {
    .debug_1 = 0,
    .debug_2 = 0,
    .debug_3 = 0
};

AveragePowerFilterParms power_filter_parms = {
    .iq_filter = 0.0,        // Iq filter output
    .iq_filter_prev = 0.0,        // Iq filter previous
    .alpha = 0.0,        // Alpha factor
    .current_limit = 0.0,        // Current limit
    .iq_over = FALSE,      // Iq over current
};

MotorDriveParms motor_drive_parms = {
    .motor_drive_state = STATE_INIT,
    .motor_drive_state_after_initialisation = STATE_RUNNING,
    .park_xform_parms = PARK_DEFAULTS,
    .clarke_xform_parms = CLARKE_DEFAULTS,
    .ipark_xform_parms = IPARK_DEFAULTS,
    .pid_id = {
        .param = {
            .Kp = 3.5,
            .Ki = 1600,
            .R = 7,
            .V = 16.8,
            .Vmin = 12,
            .V_filt_tc = 1.0,
            .Idem = 0,
            .I = 0,
            .max_dt = 2.0*COMMUTATION_PERIOD_SEC
        },
        .state = {
            .V_filtered = 16.8,
            .integrator = 0,
            .last_run_us = 0,
            .output = 0
        }
    },
    // IQ PID controller parameters
    .pid_iq = {
        .param = {
            .Kp = 3.5,
            .Ki = 1600,
            .R = 7,
            .V = 16.8,
            .Vmin = 12,
            .V_filt_tc = 1.0,
            .Idem = 0,
            .I = 0,
            .max_dt = 2.0*COMMUTATION_PERIOD_SEC
        },
        .state = {
            .V_filtered = 16.8,
            .integrator = 0,
            .last_run_us = 0,
            .output = 0
        }
    },
    .svgen_parms = SVGENDQ_DEFAULTS,
    .pwm_gen_parms = PWMGEN_DEFAULTS,
    .cal_offset_A = _IQ15(0.5),
    .cal_offset_B = _IQ15(0.5),
    .cal_filt_gain = _IQ15(COMMUTATION_PERIOD_SEC/(COMMUTATION_PERIOD_SEC+TC_CAL)),
    .Idem = 0.0,
    .current_cal_timer = 0,
    .pre_init_timer = 0,
    .fault_revive_counter = 0,
    .md_initialized = FALSE
};

static Uint32 can_init_fault_message_resend_counter = 0;
static Uint32 OldIsrTicker = 0;

void main(void)
{
	DeviceInit();	// Device Life support & GPIO
    board_hw_id = GetBoardHWID();

    // Program the EEPROM on every boot
    if(board_hw_id == EL) {
    	gp_write_eeprom();
    }

	// Initialize CAN peripheral, and CAND backend
	ECanInit();
	if (cand_init() != CAND_SUCCESS) {
	    // If the CAN module didn't initialize, we busy wait here forever and send an error message at roughly 1Hz
	    while (1) {
	        // Rough approximation of 1-second of busy waiting, doesn't need to be super accurate
	        if (++can_init_fault_message_resend_counter >= 0x7B124) {
	            can_init_fault_message_resend_counter = 0;
	            AxisFault(CAND_FAULT_UNKNOWN_AXIS_ID, CAND_FAULT_TYPE_UNRECOVERABLE, &control_board_parms, &motor_drive_parms);
	        }
	    }
	}

	// Initialize flash (must be after CAN, in case the migration fails and resets all axes)
	if (board_hw_id == AZ) {
        init_flash();
    }

	init_param_set(control_board_parms.param_set);

	// Only El and Roll load parameters over CAN
	if ((board_hw_id == EL) || (board_hw_id == ROLL)) {
	    InitAxisParmsLoader(&load_ap_state_info);
	}
    timer_init();
    scheduler_init(scheduled_tasks, sizeof(scheduled_tasks)/sizeof(struct SchedTask));

    // Initialize PWM module
	//add 1 as math in pwm macro does math and truncates
    motor_drive_parms.pwm_gen_parms.PeriodMax = SYSTEM_FREQUENCY_HZ*COMMUTATION_PERIOD_SEC/2.5/2 + 1;  // Prescaler X1 (T1), ISR period = T x 1
	PWM_INIT_MACRO(motor_drive_parms.pwm_gen_parms);

    // Initialize ECAP Module for use as a timer
    ECap1Regs.CAP3 = SYSTEM_FREQUENCY_HZ*COMMUTATION_PERIOD_SEC;  //Set Period Value, Main ISR
	ECap1Regs.CTRPHS = 0x0;  //Set Phase to 0
	ECap1Regs.ECCTL2.bit.CAP_APWM = 0x1;  //Set APWM Mode
	ECap1Regs.ECCTL2.bit.APWMPOL = 0x0;  //Set Polarity Active HI
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 0x0;  //Disable Sync In
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0x3;  //Disable Sync Out
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0x1;  //Set to free run Mode
	ECap1Regs.CAP4 = SYSTEM_FREQUENCY_HZ*COMMUTATION_PERIOD_SEC/2;  //  Set Compare Value

    // Initialize ADC module
	init_adc();

    if (board_hw_id == EL) {
        // Initialize Gyro
        InitGyro();

        // Initialize the HeroBus interface
        init_gp_interface();

        // Initialize the beacon LED
        init_led();

        InitRateLoops();
    }

    // If we're the AZ board, initialize UART for MAVLink communication
    // Also initialize the MAVLink subsystem
    if (board_hw_id == AZ) {
        init_uart();
        init_mavlink();
    } else {
        // Initialise the mavlink param data-structure so we can
        // translate incoming mavlink params to flash params
        init_default_mavlink_params();
    }

    // Initialize the average power filter
    // Current sample frequency is frequency of main ISR
    // Tau = 840 seconds per CW's calculations on 5/1/15
    // Current limit = 0.2 Amps^2 per CW's calculations on 5/1/15
    init_average_power_filter(&power_filter_parms, COMMUTATION_FREQUENCY_HZ, 840, 0.07);
	
	// Get temp sensor calibration coefficients
	TempOffset = getTempOffset();
	TempSlope = getTempSlope();

	if (board_hw_id == AZ) {
        set_axis_enable(true);
	}

	// Un-assert the DRV chip RESET signal to make the power stage active
	GpioDataRegs.GPASET.bit.GPIO1 = 1;
	GpioDataRegs.GPASET.bit.GPIO3 = 1;
	GpioDataRegs.GPASET.bit.GPIO5 = 1;

    InitInterrupts();

    DEBUG_OFF;

	// IDLE loop. Just sit and loop forever:
	for(;;)  //infinite loop
	{
        run_scheduler();

        // Process and respond to any waiting CAN messages
        Process_CAN_Messages(&axis_parms, &motor_drive_parms, &control_board_parms, &load_ap_state_info);

		// If we're the AZ board, we also have to process messages from the MAVLink interface
        if (board_hw_id == AZ) {
            mavlink_state_machine(&mavlink_gimbal_info, &motor_drive_parms);
        }

        if (board_hw_id == EL) {
            // Disable the MainISR when the pitch axis is timing off the gyro
            ECap1Regs.ECEINT.bit.CTR_EQ_PRD1 = control_board_parms.enabled ? 0 : 1;
        }

        // If we're the elevation board, check to see if we need to run the rate loops
        if (board_hw_id == EL && control_board_parms.enabled) {
            // If there is new gyro data to be processed, and all axes have been homed (the rate loop has been enabled), run the rate loops
            if (GyroDataReadyFlag) {
                GyroDataReadyFlag = false;

                DEBUG_ON;
                RunRateLoops(&control_board_parms);
                DEBUG_OFF;

                MotorCommutationLoop(&control_board_parms,
                                     &axis_parms,
                                     &motor_drive_parms,
                                     &encoder_parms,
                                     &power_filter_parms,
                                     &load_ap_state_info);
            }
        } else if(OldIsrTicker != IsrTicker || TorqueDemandAvailableFlag) {
            // If the 10kHz loop timer has ticked since the last time we ran the motor commutation loop, run the commutation loop
            OldIsrTicker = IsrTicker;
            TorqueDemandAvailableFlag = false;

            DEBUG_ON;
            MotorCommutationLoop(&control_board_parms,
                                 &axis_parms,
                                 &motor_drive_parms,
                                 &encoder_parms,
                                 &power_filter_parms,
                                 &load_ap_state_info);
            DEBUG_OFF;
        }

        if (board_hw_id == AZ) {
            // Update any parameters that have changed due to CAN messages
            ProcessParamUpdates(&control_board_parms, &debug_data);
        }
	}
} //END MAIN CODE

bool get_axis_enable(void) {
    return axis_parms.enable_flag;
}

void set_axis_enable(bool axis_enable) {
    if (axis_parms.enable_flag != axis_enable) {
        if (axis_enable) {
            // Enable the motor driver
            GpioDataRegs.GPBSET.bit.GPIO39 = 1;

            EALLOW;
            EPwm1Regs.TZCLR.bit.OST=1;
            EPwm2Regs.TZCLR.bit.OST=1;
            EPwm3Regs.TZCLR.bit.OST=1;
            EDIS;
        } else {
            // Disable the motor driver
            GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;

            EALLOW;
            EPwm1Regs.TZFRC.bit.OST=1;
            EPwm2Regs.TZFRC.bit.OST=1;
            EPwm3Regs.TZFRC.bit.OST=1;
            EDIS;
        }
        axis_parms.enable_flag = axis_enable;
    }
}

static void check_rate_cmd_timeout(void)
{
    // If we miss more than 10 rate commands in a row (roughly 100ms),
    // disable the gimbal axes.  They'll be re-enabled when we get a new
    // rate command
    if (GetBoardHWID() == AZ) {
        if (mavlink_gimbal_info.gimbal_active) {
            if (++mavlink_gimbal_info.rate_cmd_timeout_counter >= 33) {
                // Disable the other two axes
                cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_RELAX);
                // Disable ourselves
                RelaxAZAxis();
                mavlink_gimbal_info.gimbal_active = FALSE;
            }
        }
    }
}

static void update_LEDs(void)
{
    static BlinkState last_blink_state = BLINK_INIT; // Initialise with BLINK_ERROR so the first cycle detects a changed state
    static const LED_RGBA rgba_red = {.red = 0xff, .green = 0, .blue = 0, .alpha = 0xff};
    static const LED_RGBA rgba_green = {.red = 0, .green = 0xff, .blue = 0, .alpha = 0xff};
    static const LED_RGBA rgba_blue = {.red = 0, .green = 0, .blue = 0xff, .alpha = 0xff};

    if(board_hw_id == EL) {
        if (axis_parms.blink_state == BLINK_INIT && millis() > 5000 && !(axis_parms.other_axis_hb_recvd[AZ] && axis_parms.other_axis_hb_recvd[ROLL])) {
            axis_parms.blink_state = BLINK_NO_COMM;
        }
        if (axis_parms.blink_state != last_blink_state) {
            switch (axis_parms.blink_state) {
                case BLINK_NO_COMM:
                    led_set_mode(LED_MODE_BLINK_FOREVER, rgba_blue, 0);
                    break;

                case BLINK_INIT:
                    led_set_mode(LED_MODE_FADE_IN, rgba_green, 0);
                    break;

                case BLINK_RUNNING:
                    led_set_mode(LED_MODE_BREATHING, rgba_green, 0);
                    break;

                case BLINK_ERROR:
                    led_set_mode(LED_MODE_BLINK_FOREVER, rgba_red, 0);
                    break;

                case BLINK_ERROR_UNRECOVERABLE:
                    led_set_mode(LED_MODE_SOLID, rgba_red, 0);
                    break;

                case BLINK_CALIBRATING:
                    led_set_mode(LED_MODE_DISCO, rgba_red, 0);
                    break;

                case BLINK_OVERRIDE:
                default:
                    break;
            }
            last_blink_state = axis_parms.blink_state;
        }

        led_update_state();
    }
}

static void check_gp_responses(void)
{
    // If we're the EL board, periodically check if there are any new GoPro responses that we should send back to the AZ board
    if (board_hw_id == EL) {
        if (gp_new_heartbeat_available()) {
            // If there is a heartbeat status, get it and send out over CAN.
            GPHeartbeatStatus status = gp_heartbeat_status();
            cand_tx_response(CAND_ID_AZ, CAND_PID_GOPRO_HEARTBEAT, (uint32_t)status);
        }

        gp_transaction_t *txn;
        if (gp_get_completed_transaction(&txn)) {

            CAND_ParameterID can_id;
            Uint32 response_buffer = 0;
            response_buffer |= ((((Uint32)txn->mav_cmd) << 8) & 0x0000FF00);

            if (txn->reqtype == GP_REQUEST_GET) {
                // XXX: only returning first byte for now.
                //      want to be able to return larger chunks in the future
                response_buffer |= ((((Uint32)txn->payload[0]) << 0) & 0x000000FF);
                can_id = CAND_PID_GOPRO_GET_RESPONSE;

            } else {
                response_buffer |= ((((Uint32)txn->status) << 0) & 0x000000FF);
                can_id = CAND_PID_GOPRO_SET_RESPONSE;
            }

            cand_tx_response(CAND_ID_AZ, can_id, response_buffer);
        }
    }
}

static void send_mav_heartbeat(void)
{
    if (board_hw_id == AZ) {
        send_mavlink_heartbeat(mavlink_gimbal_info.mav_state, mavlink_gimbal_info.mav_mode);
    }
}

static void send_can_heartbeat(void)
{
    CBSendStatus();
}

static void read_temp_and_voltage(void)
{
    DcBusVoltage = (((float)AdcResult.ADCRESULT14 / 4096.0f) * 3.30f) * VBUS_DIV_MULTIPLIER; // DC Bus voltage meas.
	DegreesC = ((((long)((AdcResult.ADCRESULT15 - (long)TempOffset) * (long)TempSlope))>>14) + 1)>>1;

    // update the current control loop
    motor_drive_parms.pid_iq.param.V = DcBusVoltage;
    motor_drive_parms.pid_id.param.V = DcBusVoltage;

	// software start of conversion for temperature measurement and Bus Voltage Measurement
	AdcRegs.ADCSOCFRC1.bit.SOC14 = 1;
	AdcRegs.ADCSOCFRC1.bit.SOC15 = 1;
}

interrupt void GyroIntISR(void)
{
    // Notify the main loop that there is new gyro data to process
    GyroDataReadyFlag = true;

    // Acknowledge interrupt to receive more interrupts from PIE group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void MotorDriverFaultIntISR()
{
    // Process the motor drive fault
    AxisFault(CAND_FAULT_MOTOR_DRIVER_FAULT, CAND_FAULT_TYPE_UNRECOVERABLE, &control_board_parms, &motor_drive_parms);

    // TODO: May want to have some sort of recovery code here
    // Some motor driver faults need the motor driver chip to be
    // reset to clear the fault

    // Acknowledge interrupt to receive more interrupts from PIE group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// MainISR
interrupt void MainISR(void)
{
    // Verifying the ISR
    IsrTicker++;

    // Enable more interrupts from this timer
    // KRK Changed to ECAP1 interrupt
    ECap1Regs.ECCLR.bit.CTR_EQ_PRD1 = 0x1;
    ECap1Regs.ECCLR.bit.INT = 0x1;

    // Acknowledge interrupt to receive more interrupts from PIE group 3
    // KRK Changed to Group 4 to use ECAP interrupt.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

interrupt void eCAN0INT_ISR(void)
{
    DEBUG_ON;
    int16_t mailbox = ECanaRegs.CANGIF0.bit.MIV0;
    if(ECanaRegs.CANGIF0.bit.GMIF0 == 1 && mailbox == 31) {
        TorqueDemandAvailableFlag = true;
    }

    // Reenable core interrupts and CAN int from PIE module
    PieCtrlRegs.PIEACK.bit.ACK9 = 1; // Enables PIE to drive a pulse into the CPU
    DEBUG_OFF;
}

void power_down_motor()
{
    EPwm1Regs.CMPA.half.CMPA = 0; // PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA = 0; // PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA = 0; // PWM 3A - PhaseC
}

int GetAxisHomed(void)
{
    return control_board_parms.axes_homed[board_hw_id];
}

Uint16 GetAxisParmsLoaded(void)
{
    return axis_parms.all_init_params_recvd;
}

void EnableAZAxis(void)
{
    if (motor_drive_parms.md_initialized) {
        motor_drive_parms.motor_drive_state = STATE_RUNNING;
    } else {
        motor_drive_parms.motor_drive_state_after_initialisation = STATE_RUNNING;
    }
}

void RelaxAZAxis(void)
{
    if (motor_drive_parms.md_initialized) {
        motor_drive_parms.motor_drive_state = STATE_DISABLED;
    } else {
        motor_drive_parms.motor_drive_state_after_initialisation = STATE_DISABLED;
    }
}

void SetMavlinkGimbalEnabled(void)
{
	mavlink_gimbal_info.gimbal_active = TRUE;
}

void SetMavlinkGimbalDisabled(void)
{
	mavlink_gimbal_info.gimbal_active = FALSE;
}
