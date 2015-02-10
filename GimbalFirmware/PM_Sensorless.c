/* ==============================================================================
System Name:  	PM_Sensorless

File Name:	  	PM_Sensorless.C

Description:	Primary system file for the Real Implementation of Sensorless  
          		Field Orientation Control for Three Phase Permanent-Magnet
          		Synchronous Motor(s) (PMSM) 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DRV8412-EVM kit. 
=====================================================================================
 History: 04-9-2010	Version 1.1: Support F2803x 
=================================================================================  */

#include "PM_Sensorless.h"

// Include header files used in the main function
#include "PM_Sensorless-Settings.h"
#include "PeripheralHeaderIncludes.h"
#include "hardware/device_init.h"
#include "can/cand_BitFields.h"
#include "can/cand.h"
#include "can/cb.h"
#include "hardware/gyro.h"
#include "hardware/HWSpecific.h"
#include "control/gyro_kinematics_correction.h"
#include "control/PID.h"
#include "hardware/system_analyzer.h"
#include "control/average_power_filter.h"
#include "control/running_average_filter.h"
#include "hardware/uart.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "can/can_message_processor.h"
#include "version.h"
#include "gopro/gopro_interface.h"
#include "parameters/flash_params.h"
#include "motor/motor_drive_state_machine.h"
#include "control/rate_loops.h"
#include "parameters/load_axis_parms_state_machine.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define USE_SYS_ANALYZER

// Prototype statements for functions found within this file.
static void UpdateEncoderReadings(EncoderParms* encoder_parms, ControlBoardParms* cb_parms);
static void ProcessParamUpdates(ParamSet* param_set, ControlBoardParms* cb_parms, DebugData* debug_data, BalanceProcedureParms* balance_proc_parms);
void DeviceInit();
void MemCopy();
void InitFlash();

#define getTempSlope() (*(int (*)(void))0x3D7E82)();
#define getTempOffset() (*(int (*)(void))0x3D7E85)();
// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

int16	VTimer0[4];			// Virtual Timers slaved off CPU Timer 0 (A events)
int16	VTimer1[4]; 		// Virtual Timers slaved off CPU Timer 1 (B events)
int16	VTimer2[4]; 		// Virtual Timers slaved off CPU Timer 2 (C events)
int16	SerialCommsTimer;

// Global variables used in this system

int16 DegreesC;
int16 TempOffset;
int16 TempSlope;
Uint8 board_hw_id = 0;
int16 DcBusVoltage;

// Global timestamp counter.  Counts up by 1 every 100uS
Uint32 global_timestamp_counter = 0;

#define T (0.001/ISR_FREQUENCY)    // Samping period (sec), see parameter.h

Uint8 feedback_decimator;

Uint32 IsrTicker = 0;
Uint32 GyroISRTicker = 0;
Uint32 ISRStartTimestamp = 0;
Uint32 ISREndTimestamp = 0;
Uint32 ISRElapsedTime = 0;
Uint32 RateLoopStartTimestamp = 0;
Uint32 RateLoopEndTimestamp = 0;
Uint32 RateLoopElapsedTime = 0;
Uint32 MaxISRElapsedTime = 0;
Uint32 MaxGyroReadElapsedTime = 0;
Uint32 TorqueLoopEndTime = 0;
Uint32 TorqueLoopElapsedTime = 0;
Uint32 MaxTorqueLoopElapsedTime = 0;
Uint16 GyroISRTime = 0;
Uint8 GyroIntStatus = 0;
Uint8 GyroIntStatus2 = 0;
Uint16 BackTicker = 0;
Uint32 debug_output_decimation_count = 0;

int16* SysAnalyzerDataPtr = NULL;
float* SysAnalyzerDataFloatPtr = NULL;

Uint16 DRV_RESET = 0;

volatile Uint16 EnableCAN = FALSE;
volatile Uint8 GyroDataReadyFlag = FALSE;
Uint8 GyroDataOverflowLatch = FALSE;
Uint32 GyroDataOverflowCount = 0;

Uint8 gracefulstop = 0;
Uint8 gracefulstop_count = 0;

Uint16 IndexTimeOut = 0;

#define ANALOG_POT_MECH_DIVIDER 4096.0 // Resolution of 10-bit ADC
#define CURRENT_LIMIT_HOMING 0.5
#define CURRENT_LIMIT 0.25 // ADB - Roughly 0.5A at +/- 1.75A full scale

BalanceProcedureParms balance_proc_parms = {
    // Balance angle setpoints
    {
        //TODO: Temporarily limiting range of EL balance procedure to account for flex that constrains look down angle
        //{6389, 6944, 7500, 7917, 8333, 8750, 9167, 9583, 0, 416, 833, 1250},
        {8889, 9028, 9306, 9444, 9722, 9861, 0, 139, 417, 694, 972, 1250},
        {9306, 9444, 9583, 9722, 9861, 9931, 0, 139, 278, 417, 556, 694},
        {8889, 9028, 9306, 9444, 9722, 9861, 0, 139, 278, 556, 833, 1111}
    },
    0,      // Current balance angle index
    0,      // Current balance angle counter
    133,    // Balance angle counter max
    0,      // Current direction (0 = negative to positive, 1 = positive to negative)
    EL    // Balance axis
};

EncoderParms encoder_parms = {
    0,              // Raw theta
    0,              // Virtual counts
    0,              // Virtual counts offset
    0,              // Virtual counts accumulator
    0,              // Virtual counts accumulated
    {0},            // Encoder median history
    0.0,            // Mechanical theta
    0.0,            // Corrected mechanical theta
    0.0,            // Electrical theta
    0.0,            // Calibration mechanical theta Y0
    0.0,            // Calibration mechanical theta Y1
    0.0,            // Calibration slope
    0.0,            // Calibration intercept
};

AxisParms axis_parms = {
    BLINK_NO_COMM,          // Blink state
    FALSE,                  // Enable flag
    FALSE,                  // Run motor flag
    FALSE,                  // BIT Heartbeat enable
    0,                      // BIT Heartbeat decimate
    FALSE,                  // All init params received
    {FALSE, FALSE, FALSE},  // Other axis heartbeats received
    {FALSE, FALSE, FALSE},  // Other axis init params received
    0                       // Other axis init retry counter
};

ControlBoardParms control_board_parms = {
    {0, 0, 0},                                              // Gyro readings
    {0, 0, 0},                                              // Corrected gyro readings
    {0, 0, 0},                                              // Gyro offsets
    {0, 0, 0},                                              // Integrated raw gyro readings
    {0, 0, 0},                                              // Integrated raw accelerometer readings
    {0, 0, 0},                                              // Encoder readings
    {0, 0, 0},                                              // Motor torques
    {0, 0, 0},                                              // Unfiltered position errors
    {0, 0, 0},                                              // Filtered position errors
    {FALSE, FALSE, FALSE},                                  // Out of position deadband positive
    {FALSE, FALSE, FALSE},                                  // Out of position deadband negative
    {0, 0, 0},                                              // Position deadband hysteresis positive
    {0, 0, 0},                                              // Position deadband hysteresis negative
    {0, 0, 0},                                              // Axis errors
    {0, 0, 0},                                              // Angle targets,
    {CAND_FAULT_NONE, CAND_FAULT_NONE, CAND_FAULT_NONE},    // Last axis faults
    {FALSE, FALSE, FALSE},									// Encoder values received
    {FALSE, FALSE, FALSE},                                  // Axes homed
    0,                                                      // 2nd stage position loop decimation counter
    {0, 0, 0},                                              // Tuning rate inject
    {0, 0, 0},                                              // Rate command inject
    READ_GYRO_PASS,                                         // Rate loop pass
    FALSE,                                                  // Initialized
    FALSE                                                   // Enabled
};

LoadAxisParmsStateInfo load_ap_state_info = {
    0,                                          // Current param to load
    0,                                          // Total params to load (initialized in init function)
    REQUEST_RETRY_PERIOD,                       // Request retry counter
    0x0000,                                     // Init param received flags 1
    0x0000,                                     // Init param received flags 2
    0x0000,                                     // Init param received flags 3
    FALSE,                                      // Axis parms load complete
};

MavlinkGimbalInfo mavlink_gimbal_info = {
    MAV_STATE_UNINIT,                   // System status for heartbeat message
    MAV_MODE_GIMBAL_UNINITIALIZED       // Custom mode for heartbeat message
};

DebugData debug_data = {
    0,      // Debug 1
    0,      // Debug 2
    0       // Debug 3
};

AveragePowerFilterParms power_filter_parms = {
    0.0,        // Iq filter output
    0.0,        // Iq filter previous
    0.0,        // Alpha factor
    0.0,        // Current limit
    FALSE,      // Iq over current
};

// NOTE: The C2000 compiler doesn't support the {0} zero initializer syntax, so the sample
// history arrays here are not really zero initialized.  These arrays get properly initialized in
// the running average filter init routine
RunningAvgFilterParms pos_loop_filter_parms_stage_1 = {
    {0},        // Az sample history
    {0},        // El sample history
    {0},        // Rl sample history
    0,          // Az accumulator
    0,          // El accumulator
    0,          // Rl accumulator
    0,          // Az average
    0,          // El average
    0,          // Rl average
    0,          // Sample position
    FALSE       // Initialized
};

RunningAvgFilterParms pos_loop_filter_parms_stage_2 = {
    {0},        // Az sample history
    {0},        // El sample history
    {0},        // Rl sample history
    0,          // Az accumulator
    0,          // El accumulator
    0,          // Rl accumulator
    0,          // Az average
    0,          // El average
    0,          // Rl average
    0,          // Sample position
    FALSE       // Initialized
};

MotorDriveParms motor_drive_parms = {
    STATE_INIT,                     // Motor drive state
    PARK_DEFAULTS,                  // Park transform parameters
    CLARKE_DEFAULTS,                // Clarke transform parameters
    IPARK_DEFAULTS,                 // Inverse Park transform parameters
    // ID PID controller parameters
    {
        PID_TERM_DEFAULTS,
        PID_PARAM_DEFAULTS,
        PID_DATA_DEFAULTS,
    },
    // IQ PID controller parameters
    {
        PID_TERM_DEFAULTS,
        PID_PARAM_DEFAULTS,
        PID_DATA_DEFAULTS
    },
    SVGENDQ_DEFAULTS,               // Space vector generator parameters
    PWMGEN_DEFAULTS,                // PWM generator parameters
    RAMPGEN_DEFAULTS,               // Ramp generator 1 parameters
    _IQ15(0.5),                     // Cal offset A. Current calibration offset done on power up as part of system init sequence, so set to midscale for uncalibrated at init.
    _IQ15(0.5),                     // Cal offset B. Same as above
    _IQ15(T/(T+TC_CAL)),            // Phase current offset calibration filter gain
    _IQ(0.0),                       // Iq setpoint
    0,                              // Current calibration timer
    0,                              // Pre-init timer
    0,                              // Fault revive counter
    FALSE                           // Motor drive initialized
};

Uint8 unused = FALSE;
Uint8 current_flag = FALSE;
Uint8 rate_pid_el_p_flag = FALSE;
Uint8 rate_pid_el_i_flag = FALSE;
Uint8 rate_pid_el_d_flag = FALSE;
Uint8 rate_pid_el_windup_flag = FALSE;
Uint8 rate_pid_az_p_flag = FALSE;
Uint8 rate_pid_az_i_flag = FALSE;
Uint8 rate_pid_az_d_flag = FALSE;
Uint8 rate_pid_az_windup_flag = FALSE;
Uint8 rate_pid_rl_p_flag = FALSE;
Uint8 rate_pid_rl_i_flag = FALSE;
Uint8 rate_pid_rl_d_flag = FALSE;
Uint8 rate_pid_rl_windup_flag = FALSE;
Uint8 debug_1_flag = FALSE;
Uint8 debug_2_flag = FALSE;
Uint8 debug_3_flag = FALSE;
Uint8 pos_pid_el_p_flag = FALSE;
Uint8 pos_pid_el_i_flag = FALSE;
Uint8 pos_pid_el_d_flag = FALSE;
Uint8 pos_pid_el_windup_flag = FALSE;
Uint8 pos_pid_az_p_flag = FALSE;
Uint8 pos_pid_az_i_flag = FALSE;
Uint8 pos_pid_az_d_flag = FALSE;
Uint8 pos_pid_az_windup_flag = FALSE;
Uint8 pos_pid_rl_p_flag = FALSE;
Uint8 pos_pid_rl_i_flag = FALSE;
Uint8 pos_pid_rl_d_flag = FALSE;
Uint8 pos_pid_rl_windup_flag = FALSE;
Uint8 gyro_offset_x_flag = FALSE;
Uint8 gyro_offset_y_flag = FALSE;
Uint8 gyro_offset_z_flag = FALSE;
Uint8 rate_cmd_az_flag = FALSE;
Uint8 rate_cmd_el_flag = FALSE;
Uint8 rate_cmd_rl_flag = FALSE;
Uint8 gp_cmd_flag = FALSE;
#ifdef ENABLE_BALANCE_PROCEDURE
Uint8 balance_axis_flag = FALSE;
Uint8 balance_step_duration_flag = FALSE;
#endif

ParamSet param_set[CAND_PID_LAST];

// Software version descriptor (for version reporting)
DavinciVersion our_version;

void init_param_set(void)
{
	int i;
	// Initialize parameter set to be empty
	for (i = 0; i < CAND_PID_LAST; i++) {
		param_set[i].param = 0;
		param_set[i].sema = &unused;
	}

	// Set up parameters we're using
	param_set[CAND_PID_TORQUE].sema = &current_flag;
	param_set[CAND_PID_RATE_EL_P].sema = &rate_pid_el_p_flag;
	param_set[CAND_PID_RATE_EL_I].sema = &rate_pid_el_i_flag;
	param_set[CAND_PID_RATE_EL_D].sema = &rate_pid_el_d_flag;
	param_set[CAND_PID_RATE_EL_WINDUP].sema = &rate_pid_el_windup_flag;
	param_set[CAND_PID_RATE_AZ_P].sema = &rate_pid_az_p_flag;
    param_set[CAND_PID_RATE_AZ_I].sema = &rate_pid_az_i_flag;
    param_set[CAND_PID_RATE_AZ_D].sema = &rate_pid_az_d_flag;
    param_set[CAND_PID_RATE_AZ_WINDUP].sema = &rate_pid_az_windup_flag;
    param_set[CAND_PID_RATE_RL_P].sema = &rate_pid_rl_p_flag;
    param_set[CAND_PID_RATE_RL_I].sema = &rate_pid_rl_i_flag;
    param_set[CAND_PID_RATE_RL_D].sema = &rate_pid_rl_d_flag;
    param_set[CAND_PID_RATE_RL_WINDUP].sema = &rate_pid_rl_windup_flag;
    param_set[CAND_PID_DEBUG_1].sema = &debug_1_flag;
    param_set[CAND_PID_DEBUG_2].sema = &debug_2_flag;
    param_set[CAND_PID_DEBUG_3].sema = &debug_3_flag;
    param_set[CAND_PID_POS_AZ_P].sema = &pos_pid_az_p_flag;
    param_set[CAND_PID_POS_AZ_I].sema = &pos_pid_az_i_flag;
    param_set[CAND_PID_POS_AZ_D].sema = &pos_pid_az_d_flag;
    param_set[CAND_PID_POS_AZ_WINDUP].sema = &pos_pid_az_windup_flag;
    param_set[CAND_PID_POS_EL_P].sema = &pos_pid_el_p_flag;
    param_set[CAND_PID_POS_EL_I].sema = &pos_pid_el_i_flag;
    param_set[CAND_PID_POS_EL_D].sema = &pos_pid_el_d_flag;
    param_set[CAND_PID_POS_EL_WINDUP].sema = &pos_pid_el_windup_flag;
    param_set[CAND_PID_POS_RL_P].sema = &pos_pid_rl_p_flag;
    param_set[CAND_PID_POS_RL_I].sema = &pos_pid_rl_i_flag;
    param_set[CAND_PID_POS_RL_D].sema = &pos_pid_rl_d_flag;
    param_set[CAND_PID_POS_RL_WINDUP].sema = &pos_pid_rl_windup_flag;
    param_set[CAND_PID_GYRO_OFFSET_X_AXIS].sema = &gyro_offset_x_flag;
    param_set[CAND_PID_GYRO_OFFSET_Y_AXIS].sema = &gyro_offset_y_flag;
    param_set[CAND_PID_GYRO_OFFSET_Z_AXIS].sema = &gyro_offset_z_flag;
    param_set[CAND_PID_RATE_CMD_AZ].sema = &rate_cmd_az_flag;
    param_set[CAND_PID_RATE_CMD_EL].sema = &rate_cmd_el_flag;
    param_set[CAND_PID_RATE_CMD_RL].sema = &rate_cmd_rl_flag;
    param_set[CAND_PID_GP_CMD].sema = &gp_cmd_flag;
#ifdef ENABLE_BALANCE_PROCEDURE
    param_set[CAND_PID_BALANCE_AXIS].sema = &balance_axis_flag;
    param_set[CAND_PID_BALANCE_STEP_DURATION].sema = &balance_step_duration_flag;
#endif
}

// TODO: For testing gopro stuff
Uint16 gp_cmd_sent = 0;
Uint16 gp_cmd_wait = 0;
Uint16 gp_cmd_num = 0;

static void MainISRwork(void);

Uint32 MissedInterrupts = 0;

void main(void)
{
	DeviceInit();	// Device Life support & GPIO

	// initialize flash
    board_hw_id = GetBoardHWID();
	if (board_hw_id == AZ) {
		int i;
		init_flash();
		for ( i = 0; i < 3; i++) {
			AxisCalibrationSlopes[i] = flash_params.AxisCalibrationSlopes[i];
			AxisCalibrationIntercepts[i] = flash_params.AxisCalibrationIntercepts[i];
			AxisHomePositions[i] = flash_params.AxisHomePositions[i];
		}
	}
#if 0
	write_flash();
#endif
	// Initialize CAN peripheral, and CAND backend
	ECanInit();
	cand_init();

	init_param_set();

	// Only El and Roll load parameters over CAN
	if ((board_hw_id == EL) || (board_hw_id == ROLL)) {
	    InitAxisParmsLoader(&load_ap_state_info);
	}
    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the compiler
    // (see TwoChannelBuck.pjt file)
#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)

    // Timing sync for slow background tasks
    // Timer period definitions found in device specific PeripheralHeaderIncludes.h
	CpuTimer0Regs.PRD.all =  mSec1;		// A tasks
	CpuTimer1Regs.PRD.all =  mSec5;		// B tasks
	CpuTimer2Regs.PRD.all =  mSec50;	// C tasks

    // Tasks State-machine init
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;

    // Initialize PWM module
	//add 1 as math in pwm macro does math and truncates
    motor_drive_parms.pwm_gen_parms.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2.5/2 + 1;  // Prescaler X1 (T1), ISR period = T x 1
	PWM_INIT_MACRO(motor_drive_parms.pwm_gen_parms);

    // Initialize ECAP Module for use as a timer
    ECap1Regs.CAP3 = SYSTEM_FREQUENCY*1000000*T;  //Set Period Value, Main ISR
	ECap1Regs.CTRPHS = 0x0;  //Set Phase to 0
	ECap1Regs.ECCTL2.bit.CAP_APWM = 0x1;  //Set APWM Mode
	ECap1Regs.ECCTL2.bit.APWMPOL = 0x0;  //Set Polarity Active HI
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 0x0;  //Disable Sync In
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0x3;  //Disable Sync Out
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0x1;  //Set to free run Mode
	ECap1Regs.CAP4 = SYSTEM_FREQUENCY*1000000*T/2;  //  Set Compare Value

    // Initialize ADC module
    //ADC_MACRO();
	init_adc();

    if (board_hw_id == EL) {
        // Initialize Gyro
        InitGyro();

        // Initialize the HeroBus interface
        init_gp_interface();
    }

    // If we're the AZ board, initialize UART for MAVLink communication
    // Also initialize the MAVLink subsystem
    if (board_hw_id == AZ) {
        init_uart();
        init_mavlink();
    }

    // If we're using the system analyzer, initialize it here
#ifdef USE_SYS_ANALYZER
    InitSystemAnalyzer();
    // Initialize the system analyzer input pointer here to an initial, valid value
    SysAnalyzerDataPtr = &(control_board_parms.corrected_gyro_readings[EL]);
    SysAnalyzerDataFloatPtr = &motor_drive_parms.iq_ref;
#endif

#ifdef ENABLE_BALANCE_PROCEDURE
    // If we're running the balance procedure, pre-populate the angle targets with the starting balance angles
    control_board_parms.angle_targets[AZ] = balance_proc_parms.balance_angles[AZ][0];
    control_board_parms.angle_targets[EL] = balance_proc_parms.balance_angles[EL][0];
    control_board_parms.angle_targets[ROLL] = balance_proc_parms.balance_angles[ROLL][0];
#endif

    // Initialize the average power filter
    // Current sample frequency is frequency of main ISR
    // Tau = 120 seconds per CW's calculations
    // Current limit = 0.093 Amps^2 per CW's calculations
    init_average_power_filter(&power_filter_parms, (ISR_FREQUENCY * 1000), 120, 0.093);

    // Initialize the encoder median history array with the 16-bit integer max value, so that the median
    // accumulation algorithm will work
    memset(&(encoder_parms.encoder_median_history[0]), INT16_MAX, ENCODER_MEDIAN_HISTORY_SIZE * sizeof(int16));

    // Initialize the RAMPGEN module
    motor_drive_parms.rg1.StepAngleMax = _IQ(BASE_FREQ*T);

    // Initialize the PID_GRANDO_CONTROLLER module for Id
    motor_drive_parms.pid_id.param.Kp = AxisTorqueLoopKp[board_hw_id];
    motor_drive_parms.pid_id.param.Kr = _IQ(1.0);
    motor_drive_parms.pid_id.param.Ki = AxisTorqueLoopKi[board_hw_id];
    motor_drive_parms.pid_id.param.Kd = AxisTorqueLoopKd[board_hw_id];
    motor_drive_parms.pid_id.param.Km = _IQ(1.0);
    motor_drive_parms.pid_id.param.Umax = _IQ(1.0);
    motor_drive_parms.pid_id.param.Umin = _IQ(-1.0);

// Initialize the PID_GRANDO_CONTROLLER module for Iq 
    motor_drive_parms.pid_iq.param.Kp = AxisTorqueLoopKp[board_hw_id];
    motor_drive_parms.pid_iq.param.Kr = _IQ(1.0);
    motor_drive_parms.pid_iq.param.Ki = AxisTorqueLoopKi[board_hw_id];
    motor_drive_parms.pid_iq.param.Kd = AxisTorqueLoopKd[board_hw_id];
    motor_drive_parms.pid_iq.param.Km = _IQ(1.0);
    motor_drive_parms.pid_iq.param.Umax = _IQ(1.0);
    motor_drive_parms.pid_iq.param.Umin = _IQ(-1.0);
	
	// Get temp sensor calibration coefficients
	TempOffset = getTempOffset();
	TempSlope = getTempSlope();

	InitInterrupts();

	// Parse version
	if (GitTag && *GitTag == 'v') {
		int in[3];
		char str[100];
		strncpy( str, GitTag, 50);
		sscanf( str, "v%d.%d.%d", &in[0], &in[1], &in[2]);
		our_version.major = in[0];
		our_version.minor = in[1];
		our_version.rev = in[2];
	} else {
		our_version.major = our_version.minor = our_version.rev = 0xff;
	}

	if (GitVersionString && *GitVersionString == 'v') {
		int len = strlen(GitVersionString);
		if( strcmp((GitVersionString+len-5),"dirty") == 0 ) {
			our_version.dirty = 1;
		} else {
			our_version.dirty = 0;
		}
	}
	our_version.branch = *GitBranch;

	if (board_hw_id == AZ) {
	    axis_parms.enable_flag = TRUE;
	}

	// IDLE loop. Just sit and loop forever:
	for(;;)  //infinite loop
	{
		// State machine entry & exit point
		//===========================================================
		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
		//===========================================================

		//Put the DRV chip in RESET if we want the power stage inactive
		if(DRV_RESET)
		{
			GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
			GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
			GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
		}
		else
		{
			GpioDataRegs.GPASET.bit.GPIO1 = 1;
			GpioDataRegs.GPASET.bit.GPIO3 = 1;
			GpioDataRegs.GPASET.bit.GPIO5 = 1;
		}

		// Process and respond to any waiting CAN messages
		if (EnableCAN) {
		    Process_CAN_Messages(&axis_parms, &motor_drive_parms, &control_board_parms, param_set, &load_ap_state_info);
		}

		// If we're the AZ board, we also have to process messages from the MAVLink interface
		if (board_hw_id == AZ) {
		    mavlink_state_machine();
		}
		{
			static Uint32 OldIsrTicker = 0;
			if (OldIsrTicker != IsrTicker) {
				if (OldIsrTicker != (IsrTicker-1)) MissedInterrupts++;
				OldIsrTicker = IsrTicker;
				MainISRwork();
			}
		}

		// Update any parameters that have changed due to CAN messages
		ProcessParamUpdates(param_set, &control_board_parms, &debug_data, &balance_proc_parms);
	}
} //END MAIN CODE

//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// loop rate synchronizer for A-tasks
	if(CpuTimer0Regs.TCR.bit.TIF == 1) {
		CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag
		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
		SerialCommsTimer++;
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void)
{
	// loop rate synchronizer for B-tasks
	if(CpuTimer1Regs.TCR.bit.TIF == 1) {
		CpuTimer1Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
	}

	Alpha_State_Ptr = &C0;		// Allow C state tasks
}

void C0(void)
{
	// loop rate synchronizer for C-tasks
	if(CpuTimer2Regs.TCR.bit.TIF == 1) {
		CpuTimer2Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*C_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;	// Back to State A0

	EnableCAN = TRUE;
}


//=================================================================================
//	A - TASKS (executed in every 1 msec)
//=================================================================================
//--------------------------------------------------------
void A1(void) // Used to enable and disable the motor
//--------------------------------------------------------
{
	if (axis_parms.enable_flag == FALSE) {

	    // Disable the motor driver
	    GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;

		axis_parms.run_motor = FALSE;
		
		EALLOW;
	 	EPwm1Regs.TZFRC.bit.OST=1;
		EPwm2Regs.TZFRC.bit.OST=1;
		EPwm3Regs.TZFRC.bit.OST=1;
	 	EDIS;
	} else if ((axis_parms.enable_flag == TRUE) && (axis_parms.run_motor == FALSE)) {
	    // Reset rampgen state
        motor_drive_parms.rg1.Freq=0;
        motor_drive_parms.rg1.Out=0;
        motor_drive_parms.rg1.Angle=0;

        // Reset ID and IQ pid controller state
        motor_drive_parms.pid_id.data.d1 = 0;
        motor_drive_parms.pid_id.data.d2 = 0;
        motor_drive_parms.pid_id.data.i1 = 0;
        motor_drive_parms.pid_id.data.ud = 0;
        motor_drive_parms.pid_id.data.ui = 0;
        motor_drive_parms.pid_id.data.up = 0;
        motor_drive_parms.pid_id.data.v1 = 0;
        motor_drive_parms.pid_id.data.w1 = 0;
        motor_drive_parms.pid_id.term.Out = 0;

        motor_drive_parms.pid_iq.data.d1 = 0;
        motor_drive_parms.pid_iq.data.d2 = 0;
        motor_drive_parms.pid_iq.data.i1 = 0;
        motor_drive_parms.pid_iq.data.ud = 0;
        motor_drive_parms.pid_iq.data.ui = 0;
        motor_drive_parms.pid_iq.data.up = 0;
        motor_drive_parms.pid_iq.data.v1 = 0;
        motor_drive_parms.pid_iq.data.w1 = 0;
        motor_drive_parms.pid_iq.term.Out = 0;

        // Enable the motor driver
        GpioDataRegs.GPBSET.bit.GPIO39 = 1;

        axis_parms.run_motor = TRUE;

        EALLOW;
        EPwm1Regs.TZCLR.bit.OST=1;
        EPwm2Regs.TZCLR.bit.OST=1;
        EPwm3Regs.TZCLR.bit.OST=1;
        EDIS;
	}

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A2
	A_Task_Ptr = &A2;
	//-------------------
}

#ifdef ENABLE_CURRENT_TOGGLE
int iq_toggle_limit = 100;

float ThetaMaxHistory[THETA_MAX_MIN_HISTORY_SIZE];
float ThetaMinHistory[THETA_MAX_MIN_HISTORY_SIZE];
float CurrentThetaMax = -100.0;
float CurrentThetaMin = 100.0;
int ThetaMaxMinHistoryIndex = 0;
#endif

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{
#ifdef ENABLE_CURRENT_TOGGLE
    static int iq_toggle_counter = 0;
    static int gpio_state = FALSE;
    static int toggle_state = 0;
    static float next_iq_ref = 0.0;
    if (++iq_toggle_counter >= iq_toggle_limit) {
    	if (toggle_state == 0) {
    		motor_drive_parms.iq_ref = next_iq_ref;
    		toggle_state++;
    	} else {
    		next_iq_ref = -motor_drive_parms.iq_ref;
    		motor_drive_parms.iq_ref = 0.0;
    		toggle_state = 0;
    	}

        iq_toggle_counter = 0;

        if (gpio_state == TRUE) {
			GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
			GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
			gpio_state = FALSE;
		} else {
			GpioDataRegs.GPASET.bit.GPIO28 = 1;
			GpioDataRegs.GPASET.bit.GPIO29 = 1;
			gpio_state = TRUE;
		}

        // If we still have room in the min max theta array, log the current mins and maxes,
        // then reset them for the next cycle
        if (ThetaMaxMinHistoryIndex < THETA_MAX_MIN_HISTORY_SIZE) {
            ThetaMaxHistory[ThetaMaxMinHistoryIndex] = CurrentThetaMax;
            CurrentThetaMax = -100.0;
            ThetaMinHistory[ThetaMaxMinHistoryIndex] = CurrentThetaMin;
            CurrentThetaMin = 100.0;
            ThetaMaxMinHistoryIndex++;
        }
    }
#endif

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A3
	A_Task_Ptr = &A3;
	//-------------------
}

int standalone_enable_counts = 0;
int standalone_enable_counts_max = 2667;
Uint16 standalone_enabled = FALSE;

int init_counts = 0;
int init_counts_max = 333;

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{
    // Need to call the gopro interface state machine periodically
    gp_interface_state_machine();

#ifdef STANDALONE_MODE
    // If we're operating in standalone mode, enable the gimbal after 5s
	if (board_hw_id == AZ) {
		if (!standalone_enabled) {
			if (standalone_enable_counts++ >= standalone_enable_counts_max) {
			    standalone_enabled = TRUE;
				standalone_enable_counts = 0;
				cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_ENABLE);
				EnableAZAxis();
			}
		}
	}
#endif

	//-----------------
	//the next time CpuTimer0 'counter' reaches Period value go to A1
	A_Task_Ptr = &A1;
	//-----------------
}


//=================================================================================
//	B - TASKS (executed in every 5 msec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // SPARE
//----------------------------------------
{
	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B2
	B_Task_Ptr = &B2;	
	//-----------------
}

//----------------------------------------
void B2(void) //  SPARE
//----------------------------------------
{
	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B3
	B_Task_Ptr = &B3;
	//-----------------
}

//----------------------------------------
void B3(void) //  SPARE
//----------------------------------------
{
	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B1
	B_Task_Ptr = &B1;	
	//-----------------
}


//=================================================================================
//	C - TASKS (executed in every 50 msec)
//=================================================================================

//--------------------------------- USER ------------------------------------------

#define STATUS_LED_ON() 	{GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;}
#define STATUS_LED_OFF() 	{GpioDataRegs.GPASET.bit.GPIO7 = 1;}

// Gimbal Beacon LEDs (off-board, operated by EL axis)
#define BEACON_RED_ON()     {GpioDataRegs.GPASET.bit.GPIO8 = 1;}
#define BEACON_RED_OFF()    {GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;}
#define BEACON_GREEN_ON()   {GpioDataRegs.GPASET.bit.GPIO9 = 1;}
#define BEACON_GREEN_OFF()  {GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;}
#define BEACON_BLUE_ON()    {GpioDataRegs.GPASET.bit.GPIO10 = 1;}
#define BEACON_BLUE_OFF()   {GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;}

#define CH1OTWn GpioDataRegs.GPBDAT.bit.GPIO50
#define CH1Faultn GpioDataRegs.GPADAT.bit.GPIO13

int led_cnt = 0;
BeaconState beacon_state = BEACON_RED;
Uint32 beacon_counter = 0;

//----------------------------------------
void C1(void) // Update Status LEDs
//----------------------------------------
{
	switch (axis_parms.blink_state) {
	case BLINK_NO_COMM:
		// fast, 3Hz
		if( led_cnt%2 ) {
			STATUS_LED_ON();
		} else {
			STATUS_LED_OFF();
		}
		break;

	case BLINK_INIT:
		// slow, .8Hz
		if( (led_cnt%6) < 3 ) {
			STATUS_LED_ON();
		} else {
			STATUS_LED_OFF();
		}
		break;

	case BLINK_READY:
		STATUS_LED_ON();
		break;

	case BLINK_ERROR:
		// fast, 3Hz, pause after 3 cycles
		if( (led_cnt%2) && (led_cnt%12)<=6 ) {
			STATUS_LED_ON();
		} else {
			STATUS_LED_OFF();
		}
		break;
	}

	led_cnt++;

	//TODO: Testing Gimbal Beacon
	if (board_hw_id == EL) {
        beacon_counter++;
        switch (beacon_state) {
        case BEACON_RED:
            if (beacon_counter >= 7) {
                beacon_counter = 0;
                BEACON_RED_OFF();
                BEACON_GREEN_ON();
                beacon_state = BEACON_GREEN;
            }
            break;

        case BEACON_GREEN:
            if (beacon_counter >= 13) {
                beacon_counter = 0;
                BEACON_GREEN_OFF();
                BEACON_BLUE_ON();
                beacon_state = BEACON_BLUE;
            }
            break;

        case BEACON_BLUE:
            if (beacon_counter >= 20) {
                beacon_counter = 0;
                BEACON_BLUE_OFF();
                BEACON_RED_ON();
                beacon_state = BEACON_RED;
            }
            break;
        }
	}

	//the next time CpuTimer2 'counter' reaches Period value go to C2
	C_Task_Ptr = &C2;	
	//-----------------
}

//----------------------------------------
void C2(void) // Send periodic BIT message and send fault messages if necessary
//----------------------------------------
{
	// Send the BIT message once every ~1sec
	if (axis_parms.BIT_heartbeat_enable && (axis_parms.BIT_heartbeat_decimate-- <= 0)) {
		CBSendStatus();
		axis_parms.BIT_heartbeat_decimate = 6;	// ~1 Hz
	}

	//TODO: Deal with over-temp and over-current for 3DR hardware
	// ROBBY SAYS THIS IS WRONG, DON'T USE
	/*
	//Fault is asserted low if: an over current, GVDD under voltage or overtemperature (>150 C die temp)
	if (CH1Faultn == 0 && Faultmsgsent == 0) {
	   cand_tx_fault(CAND_FLVL_ERR, CAND_FMOD_POWER, "501", 3);
	   Faultmsgsent = 1;
	} else if (CH1Faultn == 0) {
		Faultmsgsent = 0;
	}

	//OTW is asserted if die temp is >125C
	if (CH1OTWn == 0 && OTWmsgsent == 0) {
	   cand_tx_fault(CAND_FLVL_ERR, CAND_FMOD_TEMP, "501", 3);
	   OTWmsgsent = 1;
	} else if (CH1Faultn == 0) {
		OTWmsgsent = 0;
	}
	*/

	// If we're the EL board, periodically check if there are any new GoPro responses that we should send back to the AZ board
	if (board_hw_id == EL) {
        if (gp_get_new_response_available()) {
            // If there are, get them and package them up to be sent out over CAN.
            // NOTE: Unfortunately, there are 5 bytes of data I'd like to send, and the biggest parameters are 4 bytes,
            // so we have to send the data as 1 4-byte parameter and 1 1-byte parameter.  Both the parameters can be packed into the
            // same CAN message, so it doesn't increase the bus load, but it complicates the parsing a bit on the other side
            GPCmdResponse* response = gp_get_last_response();
            Uint32 response_buffer[2];
            response_buffer[0] = 0;
            response_buffer[0] |= ((((Uint32)response->cmd[0]) << 24) & 0xFF000000);
            response_buffer[0] |= ((((Uint32)response->cmd[1]) << 16) & 0x00FF0000);
            response_buffer[0] |= ((((Uint32)response->cmd_status) << 8) & 0x0000FF00);
            response_buffer[0] |= (((Uint32)response->cmd_response) & 0x000000FF);

            response_buffer[1] = 0;
            response_buffer[1] = response->cmd_result;

            CAND_ParameterID pids[2];
            pids[0] = CAND_PID_GP_CMD;
            pids[1] = CAND_PID_GP_LAST_CMD_RESULT;
            cand_tx_multi_response(CAND_ID_AZ, pids, response_buffer, 2);
        }
	}

	//the next time CpuTimer2 'counter' reaches Period value go to C3
	C_Task_Ptr = &C3;	
	//-----------------
}

#ifdef ENABLE_RATE_LOOP_TUNING
int rate_toggle_limit = 10;
#endif

int mavlink_heartbeat_counter = 0;

//-----------------------------------------
void C3(void) // Read temperature and handle stopping motor on receipt of fault messages
//-----------------------------------------
{
	DegreesC = ((((long)((AdcResult.ADCRESULT15 - (long)TempOffset*33/30) * (long)TempSlope*33/30))>>14) + 1)>>1;

	DcBusVoltage = AdcResult.ADCRESULT14; // DC Bus voltage meas.

	// software start of conversion for temperature measurement and Bus Voltage Measurement
	AdcRegs.ADCSOCFRC1.bit.SOC14 = 1;
	AdcRegs.ADCSOCFRC1.bit.SOC15 = 1;

	// TODO: Add a new graceful stop routine if it's necessary
	/*
	//Graceful stop routine when a fault from another board occurs.
	if (gracefulstop == 1 & gracefulstop_count < 34) {
		if (gracefulstop_count == 33) {
			mode = MODE_DEFAULT;
		} else {
			motor_drive_parms.iq_ref = 0;
			mode = MODE_CURRENT;
		}
		gracefulstop_count ++;
	}
	*/

	if (board_hw_id == AZ) {
		// If we're the AZ axis, send the MAVLink heartbeat message at (roughly) 1Hz
		if (++mavlink_heartbeat_counter > MAVLINK_HEARTBEAT_PERIOD) {
			send_mavlink_heartbeat(mavlink_gimbal_info.mav_state, mavlink_gimbal_info.mav_mode);
			mavlink_heartbeat_counter = 0;
		}
	}

#ifdef ENABLE_BALANCE_PROCEDURE
	if (axis_parms.run_motor) {
        if (balance_proc_parms.balance_angle_counter++ > balance_proc_parms.balance_angle_counter_max) {
            balance_proc_parms.balance_angle_counter = 0;
            if (balance_proc_parms.current_direction == 0) {
                balance_proc_parms.current_balance_angle_index++;
                if (balance_proc_parms.current_balance_angle_index >= BALANCE_PROCEDURE_ANGLE_COUNT) {
                    balance_proc_parms.current_direction = 1;
                    balance_proc_parms.current_balance_angle_index -= 2; // Subtract 2 so that we don't repeat the last angle when we're going the other direction
                }
            } else {
                balance_proc_parms.current_balance_angle_index--;
                if (balance_proc_parms.current_balance_angle_index < 0) {
                    balance_proc_parms.current_direction = 0;
                    balance_proc_parms.current_balance_angle_index += 2; // Add 2 so that we don't repeat the last angle when we're going the other direction
                }
            }
            control_board_parms.angle_targets[balance_proc_parms.balance_axis] = balance_proc_parms.balance_angles[balance_proc_parms.balance_axis][balance_proc_parms.current_balance_angle_index];
        }
	}
#endif

#ifdef ENABLE_RATE_LOOP_TUNING
	static int rate_toggle_counter = 0;
	static int gpio_state = FALSE;
	static int rate_toggle_state = 0;
	static int16 next_rate_inject[AXIS_CNT] = {0, 0, 0};
	if (board_hw_id == EL) { // EL axis is control board
        if (rate_toggle_counter++ >= rate_toggle_limit) {
        	if (rate_toggle_state == 0) {
        		control_board_parms.tuning_rate_inject[AZ] = next_rate_inject[AZ];
        		control_board_parms.tuning_rate_inject[EL] = next_rate_inject[EL];
        		control_board_parms.tuning_rate_inject[ROLL] = next_rate_inject[ROLL];
        		rate_toggle_state++;
        	} else {
        	    next_rate_inject[AZ] = -control_board_parms.tuning_rate_inject[AZ];
        	    next_rate_inject[EL] = -control_board_parms.tuning_rate_inject[EL];
        	    next_rate_inject[ROLL] = -control_board_parms.tuning_rate_inject[ROLL];
        	    control_board_parms.tuning_rate_inject[AZ] = 0;
        	    control_board_parms.tuning_rate_inject[EL] = 0;
        	    control_board_parms.tuning_rate_inject[ROLL] = 0;
        		rate_toggle_state = 0;
        	}

        	/*
            if (gpio_state == TRUE) {
            	GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
            	GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
            	gpio_state = FALSE;
            } else {
            	GpioDataRegs.GPASET.bit.GPIO28 = 1;
				GpioDataRegs.GPASET.bit.GPIO29 = 1;
            	gpio_state = TRUE;
            }
            */

            rate_toggle_counter = 0;
        }
	}
#endif

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C1
	C_Task_Ptr = &C1;	
	//-----------------
}

interrupt void GyroIntISR(void)
{
    // Notify the main loop that there is new gyro data to process
    // If the flag is already set, it means the previous gyro data has not yet been processed,
    // so set an overflow latch to flag the condition
    if (GyroDataReadyFlag == FALSE) {
        GyroDataReadyFlag = TRUE;
    } else {
        GyroDataOverflowLatch = TRUE;
        GyroDataOverflowCount++;
    }

    // Acknowledge interrupt to receive more interrupts from PIE group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

int position_loop_deadband_counts = 10;
int position_loop_deadband_hysteresis = 100;

int16 rate_cmds_received[3];

static void ProcessParamUpdates(ParamSet* param_set, ControlBoardParms* cb_parms, DebugData* debug_data, BalanceProcedureParms* balance_proc_parms)
{
    IntOrFloat float_converter;
    // Check for updated rate loop PID params
    if (*(param_set[CAND_PID_RATE_EL_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[EL].integralCumulative = 0.0;
        rate_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_EL_P].param;
        rate_pid_loop_float[EL].gainP = float_converter.float_val;
        *(param_set[CAND_PID_RATE_EL_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_EL_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[EL].integralCumulative = 0.0;
        rate_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_EL_I].param;
        rate_pid_loop_float[EL].gainI = float_converter.float_val;
        *(param_set[CAND_PID_RATE_EL_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_EL_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[EL].integralCumulative = 0.0;
        rate_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_EL_D].param;
        rate_pid_loop_float[EL].gainD = float_converter.float_val;
        *(param_set[CAND_PID_RATE_EL_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_EL_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[EL].integralCumulative = 0.0;
        rate_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_EL_WINDUP].param;
        rate_pid_loop_float[EL].integralMax = float_converter.float_val;
        rate_pid_loop_float[EL].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_RATE_EL_WINDUP].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_AZ_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[AZ].integralCumulative = 0.0;
        rate_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_P].param;
        rate_pid_loop_float[AZ].gainP = float_converter.float_val;
        *(param_set[CAND_PID_RATE_AZ_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_AZ_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[AZ].integralCumulative = 0.0;
        rate_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_I].param;
        rate_pid_loop_float[AZ].gainI = float_converter.float_val;
        *(param_set[CAND_PID_RATE_AZ_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_AZ_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[AZ].integralCumulative = 0.0;
        rate_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_D].param;
        rate_pid_loop_float[AZ].gainD = float_converter.float_val;
        *(param_set[CAND_PID_RATE_AZ_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_AZ_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[AZ].integralCumulative = 0.0;
        rate_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_WINDUP].param;
        rate_pid_loop_float[AZ].integralMax = float_converter.float_val;
        rate_pid_loop_float[AZ].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_RATE_AZ_WINDUP].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_RL_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[ROLL].integralCumulative = 0.0;
        rate_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_RL_P].param;
        rate_pid_loop_float[ROLL].gainP = float_converter.float_val;
        *(param_set[CAND_PID_RATE_RL_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_RL_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[ROLL].integralCumulative = 0.0;
        rate_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_RL_I].param;
        rate_pid_loop_float[ROLL].gainI = float_converter.float_val;
        *(param_set[CAND_PID_RATE_RL_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_RL_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[ROLL].integralCumulative = 0.0;
        rate_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_RL_D].param;
        rate_pid_loop_float[ROLL].gainD = float_converter.float_val;
        *(param_set[CAND_PID_RATE_RL_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_RL_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[ROLL].integralCumulative = 0.0;
        rate_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_RL_WINDUP].param;
        rate_pid_loop_float[ROLL].integralMax = float_converter.float_val;
        rate_pid_loop_float[ROLL].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_RATE_RL_WINDUP].sema) = FALSE;
    }

    // Check for updated position loop PID params
    if (*(param_set[CAND_PID_POS_EL_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[EL].integralCumulative = 0.0;
        pos_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_EL_P].param;
        pos_pid_loop_float[EL].gainP = float_converter.float_val;
        *(param_set[CAND_PID_POS_EL_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_EL_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[EL].integralCumulative = 0.0;
        pos_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_EL_I].param;
        pos_pid_loop_float[EL].gainI = float_converter.float_val;
        *(param_set[CAND_PID_POS_EL_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_EL_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[EL].integralCumulative = 0.0;
        pos_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_EL_D].param;
        pos_pid_loop_float[EL].gainD = float_converter.float_val;
        *(param_set[CAND_PID_POS_EL_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_EL_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[EL].integralCumulative = 0.0;
        pos_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_EL_WINDUP].param;
        pos_pid_loop_float[EL].integralMax = float_converter.float_val;
        pos_pid_loop_float[EL].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_POS_EL_WINDUP].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_AZ_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[AZ].integralCumulative = 0.0;
        pos_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_AZ_P].param;
        pos_pid_loop_float[AZ].gainP = float_converter.float_val;
        *(param_set[CAND_PID_POS_AZ_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_AZ_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[AZ].integralCumulative = 0.0;
        pos_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_AZ_I].param;
        pos_pid_loop_float[AZ].gainI = float_converter.float_val;
        *(param_set[CAND_PID_POS_AZ_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_AZ_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[AZ].integralCumulative = 0.0;
        pos_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_AZ_D].param;
        pos_pid_loop_float[AZ].gainD = float_converter.float_val;
        *(param_set[CAND_PID_POS_AZ_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_AZ_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[AZ].integralCumulative = 0.0;
        pos_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_AZ_WINDUP].param;
        pos_pid_loop_float[AZ].integralMax = float_converter.float_val;
        pos_pid_loop_float[AZ].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_POS_AZ_WINDUP].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_RL_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[ROLL].integralCumulative = 0.0;
        pos_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_RL_P].param;
        pos_pid_loop_float[ROLL].gainP = float_converter.float_val;
        *(param_set[CAND_PID_POS_RL_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_RL_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[ROLL].integralCumulative = 0.0;
        pos_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_RL_I].param;
        pos_pid_loop_float[ROLL].gainI = float_converter.float_val;
        *(param_set[CAND_PID_POS_RL_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_RL_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[ROLL].integralCumulative = 0.0;
        pos_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_RL_D].param;
        pos_pid_loop_float[ROLL].gainD = float_converter.float_val;
        *(param_set[CAND_PID_POS_RL_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_RL_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[ROLL].integralCumulative = 0.0;
        pos_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_RL_WINDUP].param;
        pos_pid_loop_float[ROLL].integralMax = float_converter.float_val;
        pos_pid_loop_float[ROLL].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_POS_RL_WINDUP].sema) = FALSE;
    }

    if ((*(param_set[CAND_PID_DEBUG_1].sema) == TRUE) || (*(param_set[CAND_PID_DEBUG_2].sema) == TRUE) || (*(param_set[CAND_PID_DEBUG_3].sema) == TRUE)) {
        if (*(param_set[CAND_PID_DEBUG_1].sema) == TRUE) {
            debug_data->debug_1 = param_set[CAND_PID_DEBUG_1].param;
            *(param_set[CAND_PID_DEBUG_1].sema) = FALSE;
        }

        if (*(param_set[CAND_PID_DEBUG_2].sema) == TRUE) {
            debug_data->debug_2 = param_set[CAND_PID_DEBUG_2].param;
            *(param_set[CAND_PID_DEBUG_2].sema) = FALSE;
        }

        if (*(param_set[CAND_PID_DEBUG_3].sema) == TRUE) {
            debug_data->debug_3 = param_set[CAND_PID_DEBUG_3].param;
            *(param_set[CAND_PID_DEBUG_3].sema) = FALSE;
        }

        // If any of the debug data changed, send the debug mavlink message
        if (debug_output_decimation_count++ > 9) {
            debug_output_decimation_count = 0;
            send_mavlink_debug_data(debug_data);
        }
    }

#ifdef ENABLE_BALANCE_PROCEDURE
    if (*(param_set[CAND_PID_BALANCE_AXIS].sema) == TRUE) {
        // Convert and set the new axis
        IntOrFloat float_converter;
        float_converter.uint32_val = param_set[CAND_PID_BALANCE_AXIS].param;
        GimbalAxis new_axis = (GimbalAxis)float_converter.float_val;
        balance_proc_parms->balance_axis = new_axis;

        // Also reset the direction, step, and time counters when we get a new axis, so we begin
        // at the start of the balance procedure for the new axis
        balance_proc_parms->balance_angle_counter = 0;
        balance_proc_parms->current_balance_angle_index = 0;
        balance_proc_parms->current_direction = 0;

        // Also update the target angle here so we immediately snap to the new position on the new axis
        cb_parms->angle_targets[balance_proc_parms->balance_axis] = balance_proc_parms->balance_angles[balance_proc_parms->balance_axis][balance_proc_parms->current_balance_angle_index];

        *(param_set[CAND_PID_BALANCE_AXIS].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_BALANCE_STEP_DURATION].sema) == TRUE) {
        // Convert and update the new counter max
        IntOrFloat float_converter;
        float_converter.uint32_val = param_set[CAND_PID_BALANCE_STEP_DURATION].param;
        balance_proc_parms->balance_angle_counter_max = (int)(float_converter.float_val / 150.0); // The counter gets incremented every 150ms, and the parameter comes in as ms

        *(param_set[CAND_PID_BALANCE_STEP_DURATION].sema) = FALSE;
    }
#endif

    // There are several sets of parameters that only make sense if we're the elevation board,
    // such as rate commands, gyro offsets, and gopro commands
    if (board_hw_id == EL) {
        // Check for new rate commands from the copter
        if ((*(param_set[CAND_PID_RATE_CMD_AZ].sema) == TRUE) || (*(param_set[CAND_PID_RATE_CMD_EL].sema) == TRUE) || (*(param_set[CAND_PID_RATE_CMD_RL].sema) == TRUE)) {
            if (*(param_set[CAND_PID_RATE_CMD_AZ].sema) == TRUE) {
                rate_cmds_received[AZ] = (int16)param_set[CAND_PID_RATE_CMD_AZ].param;
                *(param_set[CAND_PID_RATE_CMD_AZ].sema) = FALSE;
            }

            if (*(param_set[CAND_PID_RATE_CMD_EL].sema) == TRUE) {
                rate_cmds_received[EL] = (int16)param_set[CAND_PID_RATE_CMD_EL].param;
                *(param_set[CAND_PID_RATE_CMD_EL].sema) = FALSE;
            }

            if (*(param_set[CAND_PID_RATE_CMD_RL].sema) == TRUE) {
                rate_cmds_received[ROLL] = (int16)param_set[CAND_PID_RATE_CMD_RL].param;
                *(param_set[CAND_PID_RATE_CMD_RL].sema) = FALSE;
            }

            // If any of the rate commands have been updated, run the kinematics transform and update the transformed rate commands
            // (NOTE: in practice, all 3 rate commands should be updated at the same time, since the parameter updates come in the same message)
            do_gyro_correction(rate_cmds_received, cb_parms->encoder_readings, cb_parms->rate_cmd_inject);
        }

        // Check for new gyro offsets from the copter
        if (*(param_set[CAND_PID_GYRO_OFFSET_X_AXIS].sema) == TRUE) {
            cb_parms->gyro_offsets[X_AXIS] = (int16)param_set[CAND_PID_GYRO_OFFSET_X_AXIS].param;
            *(param_set[CAND_PID_GYRO_OFFSET_X_AXIS].sema) = FALSE;
        }

        if (*(param_set[CAND_PID_GYRO_OFFSET_Y_AXIS].sema) == TRUE) {
            cb_parms->gyro_offsets[Y_AXIS] = (int16)param_set[CAND_PID_GYRO_OFFSET_Y_AXIS].param;
            *(param_set[CAND_PID_GYRO_OFFSET_Y_AXIS].sema) = FALSE;
        }

        if (*(param_set[CAND_PID_GYRO_OFFSET_Z_AXIS].sema) == TRUE) {
            cb_parms->gyro_offsets[Z_AXIS] = (int16)param_set[CAND_PID_GYRO_OFFSET_Z_AXIS].param;
            *(param_set[CAND_PID_GYRO_OFFSET_Z_AXIS].sema) = FALSE;
        }

        // Check for any new GoPro commands
        if (*(param_set[CAND_PID_GP_CMD].sema) == TRUE) {
            // Extract the GoPro command and parameter from the CAN parameter
            GPCmd cmd;
            cmd.cmd[0] = (param_set[CAND_PID_GP_CMD].param >> 24) & 0x000000FF;
            cmd.cmd[1] = (param_set[CAND_PID_GP_CMD].param >> 16) & 0x000000FF;
            cmd.cmd_parm = (param_set[CAND_PID_GP_CMD].param >> 8) & 0x000000FF;
            gp_send_command(&cmd);

            *(param_set[CAND_PID_GP_CMD].sema) = FALSE;
        }
    }
}

static void UpdateEncoderReadings(EncoderParms* encoder_parms, ControlBoardParms* cb_parms)
{
    encoder_parms->raw_theta = AdcResult.ADCRESULT5;
    if (encoder_parms->raw_theta < 0) {
        encoder_parms->raw_theta += ANALOG_POT_MECH_DIVIDER;
    } else if (encoder_parms->raw_theta > ANALOG_POT_MECH_DIVIDER) {
        encoder_parms->raw_theta -= ANALOG_POT_MECH_DIVIDER;
    }

    // AZ axis motor is mounted opposite of the encoder relative to the other two axes, so we need to invert it here if we're AZ
#if (HW_REV == 1)
    if (board_hw_id == AZ) {
#elif (HW_REV == 2)
    // On new hardware, EL is also flipped relative to what it was on the old hardware
    if ((board_hw_id == AZ) || (board_hw_id == EL)) {
#endif
        encoder_parms->raw_theta = ANALOG_POT_MECH_DIVIDER - encoder_parms->raw_theta;
    }

    encoder_parms->mech_theta = ((float)encoder_parms->raw_theta) / ANALOG_POT_MECH_DIVIDER;
    encoder_parms->corrected_mech_theta = (encoder_parms->mech_theta - encoder_parms->calibration_intercept) / encoder_parms->calibration_slope;
    encoder_parms->elec_theta = encoder_parms->corrected_mech_theta - floor(encoder_parms->corrected_mech_theta);

#ifdef ENABLE_CURRENT_TOGGLE
    // Keep track of max and min corrected thetas seen
    if (encoder_parms->corrected_mech_theta > CurrentThetaMax) {
        CurrentThetaMax = encoder_parms->corrected_mech_theta;
    }
    if (encoder_parms->corrected_mech_theta < CurrentThetaMin) {
        CurrentThetaMin = encoder_parms->corrected_mech_theta;
    }
#endif

    // Calculate the emulated encoder value to communicate back to the control board
    encoder_parms->virtual_counts = (encoder_parms->mech_theta * ((float)ENCODER_COUNTS_PER_REV)) - encoder_parms->virtual_counts_offset;

    // Invert the encoder reading if necessary to make sure it counts up in the right direction
    // This is necessary for the kinematics math to work properly
    if (EncoderSignMap[board_hw_id] < 0) {
        encoder_parms->virtual_counts = ENCODER_COUNTS_PER_REV - encoder_parms->virtual_counts;
    }

    // Convert the virtual counts to be symmetric around 0
    if (encoder_parms->virtual_counts < -(ENCODER_COUNTS_PER_REV / 2)) {
        encoder_parms->virtual_counts += ENCODER_COUNTS_PER_REV;
    } else if (encoder_parms->virtual_counts >= (ENCODER_COUNTS_PER_REV / 2)) {
        encoder_parms->virtual_counts -= ENCODER_COUNTS_PER_REV;
    }

    // Accumulate the virtual counts at the torque loop rate (10kHz), which will then be averaged to be sent out
    // at the rate loop rate (1kHz)
    encoder_parms->virtual_counts_accumulator += encoder_parms->virtual_counts;
    encoder_parms->virtual_counts_accumulated++;

    /*
    // Run a median filter on the encoder values.
    int i;
    int j;
    for (i = 0; i < ENCODER_MEDIAN_HISTORY_SIZE; i++) {
        // Iterate over the median history until we find a value that's larger than the current value we're trying to insert
        // When we find a larger value in the median history, that's the position the new value should be put into to keep the
        // median history sorted from smallest to largest.  Then, we need to shift the entire median history past the insertion index
        // left by one to keep the array sorted.  We only keep half of the expected median history, because there's no point storing the
        // upper half of the list because the median can't be there
        if (encoder_parms->virtual_counts < encoder_parms->encoder_median_history[i]) {
            int16 old_value = 0;
            int16 new_value = encoder_parms->virtual_counts;
            for (j = i; j < ENCODER_MEDIAN_HISTORY_SIZE; j++) {
                old_value = encoder_parms->encoder_median_history[j];
                encoder_parms->encoder_median_history[j] = new_value;
                new_value = old_value;
            }

            // Once we've inserted the new value and shifted the list, we can break out of the outer loop
            break;
        }
    }

    // Keep track of how many virtual encoder values we've accumulated since the last request for virtual encoder values,
    // so we can pick the right index in the median history array
    encoder_parms->virtual_counts_accumulated++;
    */

    // We've received our own encoder value, so indicate as such
    if (!cb_parms->encoder_value_received[EL]) {
        cb_parms->encoder_value_received[EL] = TRUE;
    }
}

// MainISR
interrupt void MainISR(void)
{
    // Verifying the ISR
    IsrTicker++;

#if (DSP2803x_DEVICE_H==1)||(DSP280x_DEVICE_H==1)||(F2806x_DEVICE_H==1)
    // Enable more interrupts from this timer
    // KRK Changed to ECAP1 interrupt
    ECap1Regs.ECCLR.bit.CTR_EQ_PRD1 = 0x1;
    ECap1Regs.ECCLR.bit.INT = 0x1;

    // Acknowledge interrupt to receive more interrupts from PIE group 3
    // KRK Changed to Group 4 to use ECAP interrupt.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
#endif

}

static void MainISRwork(void)
{
    // TODO: Measuring timing
    GpioDataRegs.GPASET.bit.GPIO28 = 1;
	GpioDataRegs.GPASET.bit.GPIO29 = 1;

	// Increment the global timestamp counter
	global_timestamp_counter++;

    ISRStartTimestamp = CpuTimer2Regs.TIM.all;

    if (axis_parms.run_motor) {
        // Do the encoder calculations no matter what state we're in (we care in a bunch of states, so no reason to duplicate the work)
        UpdateEncoderReadings(&encoder_parms, &control_board_parms);

        // Run the motor drive state machine to compute the correct inputs to the Park transform and Id and Iq PID controllers
        MotorDriveStateMachine(&axis_parms,
                &control_board_parms,
                &motor_drive_parms,
                &encoder_parms,
                param_set,
                &pos_loop_filter_parms_stage_1,
                &pos_loop_filter_parms_stage_2,
                &power_filter_parms,
                &load_ap_state_info);

        // ------------------------------------------------------------------------------
        //  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
        //  Connect inputs of the CLARKE module and call the clarke transformation macro
        // ------------------------------------------------------------------------------
#ifdef F2806x_DEVICE_H
        motor_drive_parms.clarke_xform_parms.As=(((AdcResult.ADCRESULT1)*0.00024414-motor_drive_parms.cal_offset_A)*2); // Phase A curr.
        motor_drive_parms.clarke_xform_parms.Bs=(((AdcResult.ADCRESULT3)*0.00024414-motor_drive_parms.cal_offset_B)*2); // Phase B curr.
#endif                                                         // ((ADCmeas(q12)/2^12)-0.5)*2

#ifdef DSP2803x_DEVICE_H
        motor_drive_parms.clarke_xform_parms.As=-(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-motor_drive_parms.cal_offset_A)<<1);
        motor_drive_parms.clarke_xform_parms.Bs=-(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-motor_drive_parms.cal_offset_B)<<1);
#endif

#ifdef USE_AVERAGE_POWER_FILTER
        // Run an iteration of the average power filter
        // Scale -1 to +1 current to +/- full scale current, since power filter expects current in amps
        run_average_power_filter(&power_filter_parms, motor_drive_parms.pid_iq.term.Ref * MAX_CURRENT);

        // If the average power has exceeded the preset limit on either phase a or b, error out this axis
        if (check_average_power_over_limit(&power_filter_parms)) {
            AxisFault(CAND_FAULT_OVER_CURRENT);
        }
#endif

        CLARKE_MACRO(motor_drive_parms.clarke_xform_parms)

        // ------------------------------------------------------------------------------
        //  Connect inputs of the PARK module and call the park trans. macro
        // ------------------------------------------------------------------------------
        motor_drive_parms.park_xform_parms.Alpha = motor_drive_parms.clarke_xform_parms.Alpha;
        motor_drive_parms.park_xform_parms.Beta = motor_drive_parms.clarke_xform_parms.Beta;
        // Park transformation angle is set in MotorDriveStateMachine, according to the current motor state
        motor_drive_parms.park_xform_parms.Sine = _IQsinPU(motor_drive_parms.park_xform_parms.Angle);
        motor_drive_parms.park_xform_parms.Cosine = _IQcosPU(motor_drive_parms.park_xform_parms.Angle);

        PARK_MACRO(motor_drive_parms.park_xform_parms)

        // ------------------------------------------------------------------------------
        //    Connect inputs of the id PID controller and call the PID controller macro
        // ------------------------------------------------------------------------------
        // Limit the requested current to prevent burning up the motor
#ifndef USE_AVERAGE_POWER_FILTER
        if (motor_drive_parms.motor_drive_state == STATE_HOMING) {
            // TODO: Temp for testing, allow higher currents during homing routine
            if (motor_drive_parms.pid_id.term.Ref > CURRENT_LIMIT_HOMING) {
                motor_drive_parms.pid_id.term.Ref = CURRENT_LIMIT_HOMING;
            } else if (motor_drive_parms.pid_id.term.Ref < -CURRENT_LIMIT_HOMING) {
                motor_drive_parms.pid_id.term.Ref = -CURRENT_LIMIT_HOMING;
            }
        } else {
            if (motor_drive_parms.pid_id.term.Ref > CURRENT_LIMIT) {
                motor_drive_parms.pid_id.term.Ref = CURRENT_LIMIT;
            } else if (motor_drive_parms.pid_id.term.Ref < -CURRENT_LIMIT) {
                motor_drive_parms.pid_id.term.Ref = -CURRENT_LIMIT;
            }
        }
#endif
        motor_drive_parms.pid_id.term.Fbk = motor_drive_parms.park_xform_parms.Ds;
        PID_GR_MACRO(motor_drive_parms.pid_id)

        // ------------------------------------------------------------------------------
        //    Connect inputs of the iq PID controller and call the PID controller macro
        // ------------------------------------------------------------------------------
        // Limit the requested current to prevent burning up the motor
#ifndef USE_AVERAGE_POWER_FILTER
        if (motor_drive_parms.motor_drive_state == STATE_HOMING) {
            // TODO: Temp for testing, allow higher currents during homing routine
            if (motor_drive_parms.pid_iq.term.Ref > CURRENT_LIMIT_HOMING) {
                motor_drive_parms.pid_iq.term.Ref = CURRENT_LIMIT_HOMING;
            } else if (motor_drive_parms.pid_iq.term.Ref < -CURRENT_LIMIT_HOMING) {
                motor_drive_parms.pid_iq.term.Ref = -CURRENT_LIMIT_HOMING;
            }
        } else {
            if (motor_drive_parms.pid_iq.term.Ref > CURRENT_LIMIT) {
                motor_drive_parms.pid_iq.term.Ref = CURRENT_LIMIT;
            } else if (motor_drive_parms.pid_iq.term.Ref < -CURRENT_LIMIT) {
                motor_drive_parms.pid_iq.term.Ref = -CURRENT_LIMIT;
            }
        }
#endif
        motor_drive_parms.pid_iq.term.Fbk = motor_drive_parms.park_xform_parms.Qs;
        PID_GR_MACRO(motor_drive_parms.pid_iq)

        // ------------------------------------------------------------------------------
        //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
        // ------------------------------------------------------------------------------
        motor_drive_parms.ipark_xform_parms.Qs = motor_drive_parms.pid_iq.term.Out;
        motor_drive_parms.ipark_xform_parms.Ds = motor_drive_parms.pid_id.term.Out;
        motor_drive_parms.ipark_xform_parms.Sine = motor_drive_parms.park_xform_parms.Sine;
        motor_drive_parms.ipark_xform_parms.Cosine = motor_drive_parms.park_xform_parms.Cosine;
        IPARK_MACRO(motor_drive_parms.ipark_xform_parms)

        // ------------------------------------------------------------------------------
        //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
        // ------------------------------------------------------------------------------
        motor_drive_parms.svgen_parms.Ualpha = motor_drive_parms.ipark_xform_parms.Alpha;
        motor_drive_parms.svgen_parms.Ubeta = motor_drive_parms.ipark_xform_parms.Beta;
        SVGEN_MACRO(motor_drive_parms.svgen_parms)

        // ------------------------------------------------------------------------------
        //  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
        // ------------------------------------------------------------------------------
        motor_drive_parms.pwm_gen_parms.MfuncC1 = _IQtoQ15(motor_drive_parms.svgen_parms.Ta);
        motor_drive_parms.pwm_gen_parms.MfuncC2 = _IQtoQ15(motor_drive_parms.svgen_parms.Tb);
        motor_drive_parms.pwm_gen_parms.MfuncC3 = _IQtoQ15(motor_drive_parms.svgen_parms.Tc);
        PWM_MACRO(motor_drive_parms.pwm_gen_parms);

        // Calculate the new PWM compare values

        //Check that the return is not negative
        if (motor_drive_parms.pwm_gen_parms.PWM1out < 0) {
            motor_drive_parms.pwm_gen_parms.PWM1out = 0;
        }
        if (motor_drive_parms.pwm_gen_parms.PWM2out < 0) {
            motor_drive_parms.pwm_gen_parms.PWM2out = 0;
        }
        if (motor_drive_parms.pwm_gen_parms.PWM3out < 0) {
            motor_drive_parms.pwm_gen_parms.PWM3out = 0;
        }

        //Set ADC sample point on pwm1 output to avoid switching transients
        if (motor_drive_parms.pwm_gen_parms.PWM1out < motor_drive_parms.pwm_gen_parms.PeriodMax/4) {
            EPwm1Regs.CMPB = motor_drive_parms.pwm_gen_parms.PeriodMax/2;
        } else if (motor_drive_parms.pwm_gen_parms.PWM1out > motor_drive_parms.pwm_gen_parms.PeriodMax/4 + 10) {
            EPwm1Regs.CMPB = motor_drive_parms.pwm_gen_parms.PeriodMax/8;
        }

        //Set ADC sample point on pwm2 output to avoid switching transients
        if (motor_drive_parms.pwm_gen_parms.PWM2out < motor_drive_parms.pwm_gen_parms.PeriodMax/4) {
            EPwm2Regs.CMPB = motor_drive_parms.pwm_gen_parms.PeriodMax/2;
        } else if (motor_drive_parms.pwm_gen_parms.PWM2out > motor_drive_parms.pwm_gen_parms.PeriodMax/4 + 10) {
            EPwm2Regs.CMPB = motor_drive_parms.pwm_gen_parms.PeriodMax/8;
        }


        if (motor_drive_parms.motor_drive_state == STATE_CALIBRATING_CURRENT_MEASUREMENTS) { // SPECIAL CASE: set all PWM outputs to 0 for Current measurement offset calibration
            EPwm1Regs.CMPA.half.CMPA=0; // PWM 1A - PhaseA
            EPwm2Regs.CMPA.half.CMPA=0; // PWM 2A - PhaseB
            EPwm3Regs.CMPA.half.CMPA=0; // PWM 3A - PhaseC
        } else if (0) { // TODO: For testing, disable motor outputs on EL and ROLL
            EPwm1Regs.CMPA.half.CMPA=0; // PWM 1A - PhaseA
            EPwm2Regs.CMPA.half.CMPA=0; // PWM 2A - PhaseB
            EPwm3Regs.CMPA.half.CMPA=0; // PWM 3A - PhaseC
        } else { // Otherwise, set PWM outputs appropriately
            EPwm1Regs.CMPA.half.CMPA=motor_drive_parms.pwm_gen_parms.PWM1out;  // PWM 1A - PhaseA
            EPwm2Regs.CMPA.half.CMPA=motor_drive_parms.pwm_gen_parms.PWM2out;  // PWM 2A - PhaseB
            EPwm3Regs.CMPA.half.CMPA=motor_drive_parms.pwm_gen_parms.PWM3out;  // PWM 3A - PhaseC
        }

#ifdef ENABLE_CURRENT_TOGGLE
#ifdef USE_SYS_ANALYZER
        SystemAnalyzerSendReceive((int16)(*SysAnalyzerDataFloatPtr * 32768.0));
#endif
#endif
    }

    TorqueLoopEndTime = CpuTimer2Regs.TIM.all;

    if (TorqueLoopEndTime < ISRStartTimestamp) {
		TorqueLoopElapsedTime = ISRStartTimestamp - TorqueLoopEndTime;
	} else {
		TorqueLoopElapsedTime = (mSec50 - TorqueLoopEndTime) + ISRStartTimestamp;
	}

    if (TorqueLoopElapsedTime > MaxTorqueLoopElapsedTime) {
    	MaxTorqueLoopElapsedTime = TorqueLoopElapsedTime;
    }

    // If there is new gyro data to be processed, and all axes have been homed (the rate loop has been enabled), run the rate loops
    if ((GyroDataReadyFlag == TRUE) && (control_board_parms.enabled == TRUE)) {

    	RateLoopStartTimestamp = CpuTimer2Regs.TIM.all;

        RunRateLoops(&control_board_parms, param_set, &pos_loop_filter_parms_stage_1, &pos_loop_filter_parms_stage_2, &balance_proc_parms);

        // Only reset the gyro data ready flag if we've made it through a complete rate loop pipeline cycle
        if (control_board_parms.rate_loop_pass == READ_GYRO_PASS) {
        	GyroDataReadyFlag = FALSE;
        }

        RateLoopEndTimestamp = CpuTimer2Regs.TIM.all;

        if (RateLoopEndTimestamp < RateLoopStartTimestamp) {
        	RateLoopElapsedTime = RateLoopStartTimestamp - RateLoopEndTimestamp;
        } else {
        	RateLoopElapsedTime = (mSec50 - RateLoopEndTimestamp) + RateLoopStartTimestamp;
        }
    }

    ISREndTimestamp = CpuTimer2Regs.TIM.all;

    if (ISREndTimestamp < ISRStartTimestamp) {
    	ISRElapsedTime = ISRStartTimestamp - ISREndTimestamp;
    } else {
    	ISRElapsedTime = (mSec50 - ISREndTimestamp) + ISRStartTimestamp;
    }

    if (ISRElapsedTime > MaxISRElapsedTime) {
        MaxISRElapsedTime = ISRElapsedTime;
    }

    static int debug_output_decimation = 0;
    static int max_time_reset_counter = 0;
    if (board_hw_id == EL) {
        if (++debug_output_decimation >= 10000) {
            debug_output_decimation = 0;
            Uint8 debug_info[4];
            debug_info[0] = (MaxISRElapsedTime >> 8) & 0x000000FF;
            debug_info[1] = (MaxISRElapsedTime & 0x000000FF);
            debug_info[2] = (MissedInterrupts >> 8) & 0x000000FF;
            debug_info[3] = (MissedInterrupts & 0x000000FF);
            //cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ARBITRARY_DEBUG, debug_info, 4);

            if (++max_time_reset_counter >= 5) {
                max_time_reset_counter = 0;
                MaxISRElapsedTime = 0;
            }
        }
    }

    // TODO: Testing timing
    GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
}

void power_down_motor()
{
    EPwm1Regs.CMPA.half.CMPA=0; // PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA=0; // PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA=0; // PWM 3A - PhaseC
}

int16 CorrectEncoderError(int16 raw_error)
{
    if (raw_error < -(ENCODER_COUNTS_PER_REV / 2)) {
        return raw_error + ENCODER_COUNTS_PER_REV;
    } else if (raw_error > (ENCODER_COUNTS_PER_REV / 2)) {
        return raw_error - ENCODER_COUNTS_PER_REV;
    } else {
        return raw_error;
    }
}

void AxisFault(CAND_FaultCode fault_code)
{
    // Remember our own last fault code
    control_board_parms.last_axis_fault[board_hw_id] = fault_code;

    // Put ourselves into fault mode (this stops driving current to the motor)
    motor_drive_parms.motor_drive_state = STATE_FAULT;

    // Indicate the error with the blink state
    axis_parms.blink_state = BLINK_ERROR;

    // Transmit this fault to the rest of the system
    cand_tx_fault(fault_code);
}

int GetIndexTimeOut(void)
{
    return IndexTimeOut;
}

int GetAxisHomed(void)
{
    return control_board_parms.axes_homed[board_hw_id];
}

Uint16 GetEnableFlag(void)
{
    return axis_parms.enable_flag;
}

Uint16 GetAxisParmsLoaded(void)
{
    return axis_parms.all_init_params_recvd;
}

void EnableAZAxis(void)
{
    if (motor_drive_parms.md_initialized) {
        motor_drive_parms.motor_drive_state = STATE_RUNNING;
    }
}

void RelaxAZAxis(void)
{
    if (motor_drive_parms.md_initialized) {
        motor_drive_parms.motor_drive_state = STATE_DISABLED;
    }
}

//===========================================================================
// No more.
//===========================================================================