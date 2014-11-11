
/* 	@description: 
				This is a multi-threaded embedded application to achieve a wireless RF control of a Stellaris Evalbot MCU 
				from another Evalbot by using 2.4 GHz IEEE 802.15.4 communication.

				Each Transmitter and Receiver Evalbot has been connected to Texas Instruments CC2520-CC2591EMK RF transceiver module via SPI communication.
				The Receiver can be controlled from the Transmitter by pressing push buttons and bump sensors on the Transmitter.
				
				To configure an Evalbot as a Transmitter, the code should be conditionally compiled by setting the "CC2520_IS_TRANSMITTER_MODE"
				constant from "cc2520_stellaris_porting.h" file, which was also written by me, as 1. To configure the Evalbot as a Receiver, 
				CC2520_IS_TRANSMITTER_MODE should be set to 0.
	
				Each Evalbot runs with Micrium's uC/OS-III Real-time OS.
				
				Note that this is NOT the full source code for this project, because some of the other code that comes with 
				the kernel and the CC2520-CC2591EMK chip cannot be distributed due to the licensing regulations.
	
	@disclaimer:
				Note that this software is NOT intended for sale or any commercial application. Only for demo purpose.
				Modified example code by Hong Moon.
				The original version of this 'app.c' file is from "http://micrium.com/download/micrium-book_lm3s9b92_osiii-exe/"
				The original version does not support any sort of wireless control. Also there is a non-trivial change in the motor control logic.

	@name:	 	Hong Moon (hsm5xw@gmail.com)
	@date: 		2014-Oct
	@platform: 	Texas Instruments Stellaris Evalbot (lm3s9b92)

	You must retain the above header and disclaimer in this file. 
*/


/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*                          (c) Copyright 2009-2012; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : JJL
*                 EHS
*                 KWN
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <includes.h>

/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/

typedef  enum {                                                 /* States for the control task.                         */
    CONTROL_IDLE = 0,
    CONTROL_DRIVE_FORWARD,
    CONTROL_DRIVING_FORWARD,
    CONTROL_TURN_LEFT,
    CONTROL_TURN_RIGHT,
    CONTROL_IDLE_TURN_LEFT,    // @@@@@
    CONTROL_IDLE_TURN_RIGHT,   // @@@@@
    CONTROL_TURNING
} tControlTaskState;

typedef  enum {                                                 /* States for the PID task.                             */
    PID_IDLE = 0,
    PID_START,
    PID_RUNNING
} tPIDTaskState;

typedef  enum {                                                 /* Motor drive message types.                           */
    MSG_TYPE_MOTOR_DRIVE_START = 0,
    MSG_TYPE_MOTOR_STOP,
    MSG_TYPE_MOTOR_DRIVE
} tMotorMsgType;

typedef  enum {                                                 /* Motor drive message contents.                        */
    MOTOR_DRIVE_FORWARD = 0,
    MOTOR_DRIVE_REVERSE
} tMotorMsgContent;

                                                                /* Other defines.                                       */
#define INIT_DRIVE_TIME_WINDOW    (OSCfg_TmrTaskRate_Hz)               
#define MAX_RPM                 87
#define RIGHT_SIDE_SENSOR       SENSOR_A
#define LEFT_SIDE_SENSOR        SENSOR_A
#define RIGHT_SIDE_SENSOR_PORT  RIGHT_IR_SENSOR_A_PORT
#define RIGHT_SIDE_SENSOR_PIN   RIGHT_IR_SENSOR_A_PIN
#define LEFT_SIDE_SENSOR_PORT    LEFT_IR_SENSOR_A_PORT
#define LEFT_SIDE_SENSOR_PIN     LEFT_IR_SENSOR_A_PIN

                                                                /* Robot Control Task Flag Definitions.                 */
#define FLAG_PUSH_BUTTON          (OS_FLAGS)0x0001u
#define FLAG_RIGHT_BUMP_SENSOR    (OS_FLAGS)0x0002u
#define FLAG_LEFT_BUMP_SENSOR     (OS_FLAGS)0x0004u
#define FLAG_TIMER_EXPIRATION     (OS_FLAGS)0x0008u
#define FLAG_RF_RIGHT_BUMP_SENSOR    (OS_FLAGS)0x0010u			// Right bump sensor press from the RF Transmitter side @@@@@@@@@@@@@@@@@@@@@
#define FLAG_RF_LEFT_BUMP_SENSOR     (OS_FLAGS)0x0020u			// Left  bump sensor press from the RF Transmitter side @@@@@@@@@@@@@@@@@@@@@

                                                                /* Robot Motor PID Task Flag Definitions.               */
#define FLAG_PID_START            (OS_FLAGS)0x0001u
#define FLAG_PID_STOP             (OS_FLAGS)0x0002u


// Flag definitions for RF Transmitter Flag Group @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

#define FLAG_RF_TRANS_PUSH_BUTTON          	(OS_FLAGS)0x0001u
#define FLAG_RF_TRANS_RIGHT_BUMP_SENSOR    	(OS_FLAGS)0x0002u
#define FLAG_RF_TRANS_LEFT_BUMP_SENSOR     	(OS_FLAGS)0x0004u

/*
*********************************************************************************************************
*                                            LOCAL VARIABLES
*********************************************************************************************************
*/

static  OS_TCB        AppTaskRobotStartTCB;
static  OS_TCB        AppTaskRobotControlTCB;
static  OS_TCB        AppTaskRobotLeftMotorDriveTCB;
static  OS_TCB        AppTaskRobotRightMotorDriveTCB;
static  OS_TCB        AppTaskRobotLeftMotorPIDTCB;
static  OS_TCB        AppTaskRobotRightMotorPIDTCB;
static  OS_TCB        AppTaskRobotInputMonitorTCB;
static  OS_TCB        AppTaskRobotDisplayTCB;

#if CC2520_IS_TRANSMITTER_MODE == 1                     // If in RF Transmitter mode
static  OS_TCB        AppTaskRobotRF_TransmitTCB;       // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#else													// If in RF Receiver mode
static  OS_TCB        AppTaskRobotRF_ReceiveTCB;
#endif

static  CPU_STK       AppTaskRobotStartStk[APP_TASK_ROBOT_START_STK_SIZE];
static  CPU_STK       AppTaskRobotControlStk[APP_TASK_ROBOT_CONTROL_STK_SIZE];
static  CPU_STK       AppTaskRobotLeftMotorDriveStk[APP_TASK_ROBOT_MOTOR_DRIVE_STK_SIZE];
static  CPU_STK       AppTaskRobotRightMotorDriveStk[APP_TASK_ROBOT_MOTOR_DRIVE_STK_SIZE];
static  CPU_STK       AppTaskRobotLeftMotorPIDStk[APP_TASK_ROBOT_MOTOR_PID_STK_SIZE];
static  CPU_STK       AppTaskRobotRightMotorPIDStk[APP_TASK_ROBOT_MOTOR_PID_STK_SIZE];
static  CPU_STK       AppTaskRobotInputMonitorStk[APP_TASK_ROBOT_SWITCH_MONITOR_STK_SIZE];
static  CPU_STK       AppTaskRobotDisplayStk[APP_TASK_ROBOT_DISPLAY_STK_SIZE];

#if CC2520_IS_TRANSMITTER_MODE == 1                     // If in RF Transmitter mode
static  CPU_STK       AppTaskRobotRF_TransmitStk[APP_TASK_ROBOT_RF_TRANSMIT_STK_SIZE];  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#else													// If in RF Receiver mode
static  CPU_STK       AppTaskRobotRF_ReceiveStk[APP_TASK_ROBOT_RF_RECEIVE_STK_SIZE]; 
#endif

#if 0 // Unused Reference
static  OS_TMR        AppRobotDriveTmr;
#endif // End of Unused Reference

static  OS_TMR        AppRobotTurnTmr;
static  OS_FLAG_GRP   AppRobotControlFlagGroup;
static  OS_FLAG_GRP   AppRobotPIDLeftFlagGroup;
static  OS_FLAG_GRP   AppRobotPIDRightFlagGroup;

#if CC2520_IS_TRANSMITTER_MODE == 1                     // If in RF Transmitter mode
static  OS_FLAG_GRP   RF_TransmitterFlagGroup;          // RF Trasmitter Flag Group  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#else													// If in RF Receiver mode
/*
	no separate Flag Group needed since I'm going to 
	reuse the "AppRobotControlFlagGroup" when in Receiver mode
*/	
#endif

static  CPU_BOOLEAN   AppRobotLeftWheelFirstEdge;
static  CPU_BOOLEAN   AppRobotRightWheelFirstEdge;

static  const  tSide  AppRobotLeftSide  = LEFT_SIDE;
static  const  tSide  AppRobotRightSide = RIGHT_SIDE;

                                                                /* Variables used by uC/Probe for monitoring            */
static  CPU_BOOLEAN   bLED[2]        = {DEF_TRUE, DEF_FALSE};
//static  CPU_BOOLEAN   bForward       = DEF_FALSE;
//static  CPU_BOOLEAN   bLeft          = DEF_FALSE;
//static  CPU_BOOLEAN   bRight         = DEF_FALSE;
//static  CPU_INT16U    usLeftBumpCnt  = 0;
//static  CPU_INT16U    usRightBumpCnt = 0;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void        AppTaskRobotStart                 (void        *p_arg);
static  void        AppTaskRobotMotorDrive            (void        *p_arg);
static  void        AppTaskRobotMotorPID              (void        *p_arg);
static  void        AppTaskRobotControl               (void        *p_arg);
static  void        AppTaskRobotInputMonitor          (void        *p_arg);

static  void        AppTaskRobotDisplay               (void        *p_arg);

#if CC2520_IS_TRANSMITTER_MODE == 1                    // If in RF Transmitter mode   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
static  void        AppTask_RF_Transmit               (void        *p_arg);
#else												   // If in RF Receiver mode
static  void        AppTask_RF_Receive	              (void        *p_arg);
#endif

static  void        AppRobotDriveForward              (CPU_INT08U   ucLeftMotorSpeed,
                                                       CPU_INT08U   ucRightMotorSpeed);
static  void        AppRobotTasksCreate               (void);
static  void        AppRobotTurnLeft                  (void);
static  void        AppRobotTurnRight                 (void);
static  void        AppRobotStop                      (void);

#if 0	// Unused Reference
static  void        AppRobotDriveTmrInit              (CPU_INT16U   usDrive);
static  void        AppRobotDriveTmrDel               (void);
#endif  // End of Unused Reference

static  void        AppRobotTurnTmrInit               (CPU_INT16U   usDriveTimeWindow);
static  void        AppRobotTurnTmrDel                (void);
static  void        AppRobotMotorDriveSensorEnable    (tSide        eSide);
static  void        AppRobotMotorDriveSensorDisable   (tSide        eSide);
static  void        AppRobotWheelSensorIntHandler     (void);
static  void        AppRobotMotorPIDTaskTargetSpeedSet(tSide        eSide,
                                                       CPU_INT08U   ucSpeed);
static  void        AppRobotMotorPIDTaskActualSpeedSet(tSide        eSide,
                                                       CPU_INT08U   ucSpeed);
static  CPU_INT08U  AppRobotMotorPIDTaskTargetSpeedGet(tSide        eSide);
static  CPU_INT08U  AppRobotMotorPIDTaskActualSpeedGet(tSide        eSide);
static  void        AppRobotMotorPIDTaskFlagCreate    (tSide        eSide);
static  void        AppRobotMotorPIDTaskStartFlagPend (tSide        eSide);
static  OS_ERR      AppRobotMotorPIDTaskStopFlagPend  (tSide        eSide);
static  void        AppRobotMotorPIDTaskStart         (tSide        eSide);
static  void        AppRobotMotorPIDTaskStop          (tSide        eSide);

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

int  main (void)
{
    OS_ERR  err;


    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */

    OSTaskCreate((OS_TCB     *)&AppTaskRobotStartTCB,           /* Create the start task                                */
                 (CPU_CHAR   *)"App Task Start",
                 (OS_TASK_PTR ) AppTaskRobotStart,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_ROBOT_START_PRIO,
                 (CPU_STK    *)&AppTaskRobotStartStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_START_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_START_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskRobotStart (void  *p_arg)
{
    CPU_INT32U  clk_freq;
    CPU_INT32U  ulPHYMR0;
    CPU_INT32U  cnts;
    OS_ERR      err;


   (void)&p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);                  /* Enable and Reset the Ethernet Controller.            */
    SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);

    ulPHYMR0 = EthernetPHYRead(ETH_BASE, PHY_MR0);              /* Power Down PHY                                       */
    EthernetPHYWrite(ETH_BASE, PHY_MR0, ulPHYMR0 | PHY_MR0_PWRDN);
    SysCtlPeripheralDeepSleepDisable(SYSCTL_PERIPH_ETH);


    clk_freq = BSP_CPUClkFreq();                                /* Determine SysTick reference freq.                    */
    cnts     = clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */
    CPU_TS_TmrFreqSet(clk_freq);

#if (OS_CFG_STAT_TASK_EN > 0u)
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

    CPU_IntDisMeasMaxCurReset();

    App_ProbeInit();

    AppRobotTasksCreate();

    BSP_LED_On(1);
    BSP_LED_Off(2);

    while (DEF_ON) {                                            /* Task body, always written as an infinite loop.       */
        OSTimeDlyHMSM(0u, 0u, 1u, 0u,                           /* Delay one second.                                    */
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        BSP_LED_Toggle(0);                                      /* Toggle both LEDs every second.                       */

        bLED[0] ^= DEF_TRUE;
        bLED[1] ^= DEF_TRUE;
    }
}


static  void  AppTaskRobotControl (void  *p_arg)
{
    CPU_INT16U         usDriveTimeWindow;
    //CPU_INT08U         pucMotorSpeed[2];
    
    tControlTaskState  eState;  // current state
    tControlTaskState  pState;  // previous state    
    
    OS_FLAGS           flags;
    OS_ERR             err;
    CPU_TS             ts;


    eState = CONTROL_IDLE;                                      /* Initially start in the IDLE state.                   */
    pState = CONTROL_IDLE;										// previous state

    usDriveTimeWindow = INIT_DRIVE_TIME_WINDOW;
    
    while (DEF_ON) {
        switch (eState) {                                       /* Current control state.                               */
            case CONTROL_IDLE:

                 OSFlagPend(&AppRobotControlFlagGroup,          /* Pend until a push button is pressed.                 */
                            FLAG_PUSH_BUTTON      +
                            FLAG_LEFT_BUMP_SENSOR +
                            FLAG_RIGHT_BUMP_SENSOR +
                            FLAG_RF_LEFT_BUMP_SENSOR +
                            FLAG_RF_RIGHT_BUMP_SENSOR,							
                            0u,
                            OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                            &ts,
                            &err);

                 flags = OSFlagPendGetFlagsRdy(&err);           /* Get the flag that was ready.                         */

                 if ( (flags & FLAG_LEFT_BUMP_SENSOR) || (flags & FLAG_RF_LEFT_BUMP_SENSOR ) ) {
                        eState = CONTROL_IDLE_TURN_LEFT;             // Switch to turn idle left state.  @@@@@   
			
                 } else if ( (flags & FLAG_RIGHT_BUMP_SENSOR) || (flags & FLAG_RF_RIGHT_BUMP_SENSOR) ){
                        eState = CONTROL_IDLE_TURN_RIGHT;		// Switch to turn idle right state. @@@@@
				} else{
                        eState = CONTROL_DRIVE_FORWARD;         // Switch to the drive state.                           			 
                 }
				
                 pState = CONTROL_IDLE;
                 break;

            case CONTROL_DRIVE_FORWARD:

                 AppRobotDriveForward(65u, 65u);                /* Drive straight forward.                              */
  // @           AppRobotDriveTmrInit(usDriveTimeWindow);       /* Create and Start the drive timer.                    */
                 eState = CONTROL_DRIVING_FORWARD;              /* Switch to the driving state.                         */
				 pState = CONTROL_DRIVE_FORWARD;
                 break;


            case CONTROL_DRIVING_FORWARD:
                 OSFlagPend(&AppRobotControlFlagGroup,          /* Pend until push button or bump sensor is pressed,... */
                            FLAG_PUSH_BUTTON       +            /* ... or until the timer expires.                      */
                            FLAG_RIGHT_BUMP_SENSOR +
                            FLAG_LEFT_BUMP_SENSOR  +
                            FLAG_RF_LEFT_BUMP_SENSOR +
                            FLAG_RF_RIGHT_BUMP_SENSOR,							
  // @                      FLAG_TIMER_EXPIRATION
                            0u,
                            OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                            &ts,
                            &err);

  // @           AppRobotDriveTmrDel();                         /* Delete drive timer.                                  */
                 AppRobotStop();                                /* Stop robot.                                          */

                 flags = OSFlagPendGetFlagsRdy(&err);           /* Get the flag that was ready.                         */

                 if (flags & FLAG_PUSH_BUTTON) {                /* Check for push button event.                         */
                     eState = CONTROL_IDLE;                     /* Switch to idle state.                                */
                 } 

                 else if (flags & FLAG_RIGHT_BUMP_SENSOR) {
                     eState = CONTROL_IDLE;

                     // If both bump sensor flags are on, consume one of them. @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                     OSFlagPend(&AppRobotControlFlagGroup,      
                                  FLAG_LEFT_BUMP_SENSOR +
                                  FLAG_RIGHT_BUMP_SENSOR,
                                  5u,
                                  OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                                  &ts,
                                  &err);  
                     
                 } else if (flags & FLAG_LEFT_BUMP_SENSOR) {
                     eState = CONTROL_IDLE;
                     
                     // If both bump sensor flags are on, consume one of them. @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                     OSFlagPend(&AppRobotControlFlagGroup,      
                                  FLAG_LEFT_BUMP_SENSOR +
                                  FLAG_RIGHT_BUMP_SENSOR,
                                  5u,                                  
                                  OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                                  &ts,
                                  &err);     
								  
                 } else if( flags & FLAG_RF_LEFT_BUMP_SENSOR ){
                      eState = CONTROL_TURN_LEFT;               // turn left upon bump sensor presses from the RF Transmitter
                 }				 
                 else if( flags & FLAG_RF_RIGHT_BUMP_SENSOR ){
                      eState = CONTROL_TURN_RIGHT;             // turn right upon bump sensor presses from the RF Transmitter
                 }					 
                 
                 pState = CONTROL_DRIVING_FORWARD;
                 break;


            case CONTROL_TURN_LEFT:
			
                 AppRobotTurnLeft();                            /* Turn left.                                           */
                 AppRobotTurnTmrInit(usDriveTimeWindow);	/* Create and start turn timer.                         */

                 eState = CONTROL_TURNING;                      /* Switch to turning state.                             */
                 pState = CONTROL_TURN_LEFT;
                 break;


            case CONTROL_TURN_RIGHT:

                 AppRobotTurnRight();                           /* Turn right.                                          */
                 AppRobotTurnTmrInit(usDriveTimeWindow);	/* Create and start turn timer.                         */

                 eState = CONTROL_TURNING;                      /* Switch to turning state.                             */
                 pState = CONTROL_TURN_RIGHT;
                 break;

            case CONTROL_IDLE_TURN_LEFT:	// @@@@@
			
                 AppRobotTurnLeft();                            /* Turn left.                                           */
                 AppRobotTurnTmrInit(usDriveTimeWindow);	/* Create and start turn timer.                         */

                 eState = CONTROL_TURNING;                      /* Switch to turning state.                             */
                 pState = CONTROL_IDLE_TURN_LEFT;
                 break;


            case CONTROL_IDLE_TURN_RIGHT:	// @@@@@

                 AppRobotTurnRight();                           /* Turn right.                                          */
                 AppRobotTurnTmrInit(usDriveTimeWindow);	/* Create and start turn timer.                         */

                 eState = CONTROL_TURNING;                      /* Switch to turning state.                             */
                 pState = CONTROL_IDLE_TURN_RIGHT;
                 break;				 
				 

            case CONTROL_TURNING:
                 OSFlagPend(&AppRobotControlFlagGroup,          /* Pend until push button or bump sensor is pressed,... */
                            FLAG_PUSH_BUTTON       +            /* ... or until the timer expires.                      */
                            FLAG_LEFT_BUMP_SENSOR  +
                            FLAG_RIGHT_BUMP_SENSOR +
                            FLAG_TIMER_EXPIRATION,
                            0u,
                            OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                            &ts,
                            &err);

                 flags = OSFlagPendGetFlagsRdy(&err);           /* Get the flag that was ready.                         */

                 if ((flags & FLAG_LEFT_BUMP_SENSOR) ||
                     (flags & FLAG_RIGHT_BUMP_SENSOR) ||
					 (flags & FLAG_RF_LEFT_BUMP_SENSOR) ||
                     (flags & FLAG_RF_RIGHT_BUMP_SENSOR) ) {
                                                                /* Do nothing: just break out and come right back ...   */
                                                                /* ... to the same state. The bump sensors should ...   */
                                                                /* ... be ignored while turning, but we want the  ...   */
                                                                /* ... flags to be consumed if the event occurs.        */
                       pState = CONTROL_TURNING;											
                       break;
                 }

                AppRobotTurnTmrDel();                          /* Delete drive timer.                                  */
                AppRobotStop();                                /* Stop robot.                                          */

                 if (flags & FLAG_PUSH_BUTTON) {                /* Check for push button event.                         */
                     eState = CONTROL_IDLE;                     /* Switch to idle state.                                */
                 } 
                 else if (flags & FLAG_TIMER_EXPIRATION) {    	/* Check for timer expired.                             */
                   
                    if( (pState == CONTROL_IDLE_TURN_LEFT) || (pState == CONTROL_IDLE_TURN_RIGHT) ) 
                          eState = CONTROL_IDLE;   	        // switch to idle state         
                    else  if( pState != CONTROL_TURNING )
                          eState = CONTROL_DRIVE_FORWARD;   	// switch to drive forward state
                    else
                          eState = CONTROL_IDLE;
                 }
                 
                 pState = CONTROL_TURNING;
                 break;
            }
     }
}


static  void  AppTaskRobotMotorDrive (void  *p_arg)
{
    CPU_INT08U   *pucMsg;
    OS_MSG_SIZE   msg_size;
    CPU_INT16U    usPercent;
    tSide         eSide;
    OS_ERR        err;
    CPU_TS        ts;


    eSide = *(tSide *)p_arg;                                    /* p_arg specifies which motor this task is for.        */

    while (DEF_ON) {
        pucMsg = (CPU_INT08U *)OSTaskQPend(0u,
                                           OS_OPT_PEND_BLOCKING,
                                           &msg_size,
                                           &ts,
                                           &err);

        switch (pucMsg[0]) {
            case MSG_TYPE_MOTOR_DRIVE_START:
                 if (pucMsg[1] == MOTOR_DRIVE_FORWARD) {        /* Get direction to drive from message.                 */
                     BSP_MotorDir(eSide, FORWARD);
                 } else {
                     BSP_MotorDir(eSide, REVERSE);
                 }

                 BSP_MotorSpeed(eSide, 0u);                     /* Initially set motor speed to be 0%. This will be ... */
                                                                /* ... updated by messages sent from the PID task.      */

                 BSP_MotorRun(eSide);                           /* Run motor.                                           */

                 AppRobotMotorDriveSensorEnable(eSide);         /* Reset and enable edge detection capture for ...      */
                                                                /* ... motor drive sensor.                              */

                 AppRobotMotorPIDTaskActualSpeedSet(eSide, 0u); /* Clear actual speed for PID task.                     */
                                                                /* Set desired speed for PID task.                      */
                 AppRobotMotorPIDTaskTargetSpeedSet(eSide, pucMsg[2]);

                 AppRobotMotorPIDTaskStart(eSide);              /* Start PID task.                                      */
                 break;


            case MSG_TYPE_MOTOR_STOP:
                 BSP_MotorStop(eSide);                          /* Stop motor.                                          */

                 AppRobotMotorDriveSensorDisable(eSide);        /* Disable edge detection capture of motor drive sensor */
                 AppRobotMotorPIDTaskStop(eSide);               /* Stop PID task.                                       */
                 break;


            case MSG_TYPE_MOTOR_DRIVE:
                 usPercent = (pucMsg[1] << 8u) | (pucMsg[2]);

                 BSP_MotorSpeed(eSide, usPercent);              /* Update motor speed.                                  */
                 break;


            default:
                 break;
        }
    }
}


static  void  AppTaskRobotMotorPID (void  *p_arg)
{
    CPU_INT32S     lDelta;
    CPU_INT32U     ulPercent;
    CPU_INT08U     ucTargetSpeed;
    CPU_INT08U     ucActualSpeed;
    CPU_INT08U     pucMotorMsg[3];
    tPIDState      sSpeedState;
    tPIDTaskState  eState;
    tSide          eSide; 
    OS_ERR         err;


    eSide = *(tSide *)p_arg;                                    /* p_arg specifies which motor this task is for.        */

    AppRobotMotorPIDTaskFlagCreate(eSide);

    eState = PID_IDLE;                                          /* Initially start in IDLE state.                       */

    while (DEF_ON) {
        switch (eState) {
            case PID_IDLE:
                 AppRobotMotorPIDTaskStartFlagPend(eSide);      /* Pend until start flag is set.                        */

                 eState = PID_START;                            /* Switch to start state.                               */
                 break;


            case PID_START:
                                                                /* Integrator max = 100, min = 0 (16.16 format)         */
                                                                /* P = 0,                                               */
                                                                /* I = 750/65536 = 0.05,                                */
                                                                /* D = 0   ((100 << 16) / 1)                            */
                 PIDInitialize(&sSpeedState,
                               (100u << 16u) / 750u,
                               0u,
                               0u,
                               750u << 16u,
                               0u);

                 eState = PID_RUNNING;                          /* Switch to running state.                             */
                 break;


            case PID_RUNNING:
                                                                /* Check for stop flag. Timeout after 10ms and run ...  */
                                                                /* PID controller if stop flag not set.                 */
                 if (AppRobotMotorPIDTaskStopFlagPend(eSide) == OS_ERR_TIMEOUT) {
                                                                /* Get target speed.                                    */
                     ucTargetSpeed = AppRobotMotorPIDTaskTargetSpeedGet(eSide);

                                                                /* Get actual speed.                                    */
                     ucActualSpeed = AppRobotMotorPIDTaskActualSpeedGet(eSide);

                                                                /* Compute difference in speeds.                        */
                     lDelta = ((CPU_INT32S)ucTargetSpeed) - ((CPU_INT32S)ucActualSpeed);

                     // Run the difference through the PID controller. The output
                     // is the percentage of the total voltage to drive to the motor.
                     // The percentage will be in 16.16 format. Need to reduce to 8.8.
                     // The integer portion is limited to 100, so only need to adjust
                     // the fractional portion.
                     ulPercent = PIDUpdate(&sSpeedState, lDelta);

                     if (ulPercent > (100u << 16u)) {
                         ulPercent =  100u << 16u;
                     }

                     pucMotorMsg[0] = MSG_TYPE_MOTOR_DRIVE;
                     pucMotorMsg[1] =   ulPercent >> 16u;       /* Upper byte of 8.8 percentage.                        */
                     pucMotorMsg[2] = ((ulPercent & 0xFF00u) >> 8u);

                     OSTaskQPost((eSide == LEFT_SIDE) ?
                                 (OS_TCB    *)&AppTaskRobotLeftMotorDriveTCB :
                                 (OS_TCB    *)&AppTaskRobotRightMotorDriveTCB,
                                 (void      *)&pucMotorMsg[0],
                                 (OS_MSG_SIZE) 3u,
                                 (OS_OPT     ) OS_OPT_POST_FIFO,
                                 (OS_ERR    *)&err);
                 } else {                                       /* Stop flag was posted.                                */
                     eState = PID_IDLE;                         /* Switch to IDLE state.                                */
                 }
                 break;


            default:
                 break;
        }
    }
}


static  void  AppTaskRobotInputMonitor (void  *p_arg)
{
    CPU_INT08U  ucData;
    OS_ERR      err;

    // The debounced state of the 4 switches. The bit positions
    // correspond to:
    //
    //     0 - Right Push Button
    //     1 - Left  Push Button
    //     2 - Right Bump Sensor
    //     3 - Left  Bump Sensor
    CPU_INT08U  ucSwitches;

    // The swithches that just changed state. The bit positions
    // are the same as g_ucPushButtons.
    CPU_INT08U  ucDelta;

    // The vertical counter used to debounce the switches.  The
    // bit positions are the same as g_ucPushButtons.
    CPU_INT08U  ucSwitchesClockA;
    CPU_INT08U  ucSwitchesClockB;

   (void)&p_arg;

                                                                /* Initialize the variables                             */
    ucSwitches       = 0x0Fu;
    ucSwitchesClockA =    0u;
    ucSwitchesClockB =    0u;

    while (DEF_ON) {
        OSTimeDlyHMSM(0u, 0u, 0u, 5u,                           /* Delay for 5 milliseconds.                            */
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

                                                                /* Read the state of the switches.                      */
        ucData =  BSP_PushButtonGetStatus(1u)        |          /*    Right Push Button                                 */
                 (BSP_PushButtonGetStatus(2u) << 1u) |          /*    Left  Push Button                                 */
                 (BSP_BumpSensorGetStatus(1u) << 2u) |          /*    Right Bump Sensor                                 */
                 (BSP_BumpSensorGetStatus(2u) << 3u);           /*    Left  Bump Sensor                                 */

                                                                /* Determine the switches at a different state than ... */
        ucDelta = ucData ^ ucSwitches;                          /* ... the debounced state.                             */

        ucSwitchesClockA ^=  ucSwitchesClockB;                  /* Increment the clocks by one.                         */
        ucSwitchesClockB  = ~ucSwitchesClockB;

        ucSwitchesClockA &= ucDelta;                            /* Reset the clocks corresponding to switches that ...  */
        ucSwitchesClockB &= ucDelta;                            /* ... have not changed state.                          */

                                                                /* Get the new debounced switch state.                  */
        ucSwitches &=    ucSwitchesClockA | ucSwitchesClockB;
        ucSwitches |= (~(ucSwitchesClockA | ucSwitchesClockB)) & ucData;

        ucDelta ^= ucSwitchesClockA | ucSwitchesClockB;         /* Determine switches that changed debounced state.     */

        if ((ucDelta & 0x03u) && (~ucSwitches & 0x03u)) {       /* Right or left push button.                           */
 
              #if CC2520_IS_TRANSMITTER_MODE == 1   // If in RF Transmitter mode                                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                   
                  // When in transmitter mode, prevent the wheels from turning (to save some processing time)
                  /*
                  OSFlagPost(&AppRobotControlFlagGroup,   
                              FLAG_PUSH_BUTTON,
                              OS_OPT_POST_FLAG_SET,
                              &err);
                  */
          
                  OSFlagPost(&RF_TransmitterFlagGroup,
                              FLAG_RF_TRANS_PUSH_BUTTON,
                              OS_OPT_POST_FLAG_SET,
                              &err);        
              #else                                 // If in RF Receiver mode                                           
                  OSFlagPost(&AppRobotControlFlagGroup,   
                              FLAG_PUSH_BUTTON,
                              OS_OPT_POST_FLAG_SET,
                              &err);
              #endif    
        }

        if ((ucDelta & 0x04u) && (~ucSwitches & 0x04u)) {       /* Right bump sensor.  */
          
               #if CC2520_IS_TRANSMITTER_MODE == 1   // If in RF Transmitter mode                                       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                   
                  // When in transmitter mode, prevent the wheels from turning (to save some processing time)
                  /*
                  OSFlagPost(&AppRobotControlFlagGroup,   
                              FLAG_RIGHT_BUMP_SENSOR,
                              OS_OPT_POST_FLAG_SET,
                              &err);
                  */
          
                  OSFlagPost(&RF_TransmitterFlagGroup,
                              FLAG_RF_TRANS_RIGHT_BUMP_SENSOR,
                              OS_OPT_POST_FLAG_SET,
                              &err);        
              #else                                 // If in RF Receiver mode
                  OSFlagPost(&AppRobotControlFlagGroup,   
                              FLAG_RIGHT_BUMP_SENSOR,
                              OS_OPT_POST_FLAG_SET,
                              &err);
              #endif    
        }
   
        if ((ucDelta & 0x08u) && (~ucSwitches & 0x08u)) {       /* Left bump sensor. */

              #if CC2520_IS_TRANSMITTER_MODE == 1   // If in RF Transmitter mode                                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                   
                  // When in transmitter mode, prevent the wheels from turning (to save some processing time)
                  /*
                  OSFlagPost(&AppRobotControlFlagGroup,   
                              FLAG_LEFT_BUMP_SENSOR,
                              OS_OPT_POST_FLAG_SET,
                              &err);
                  */
                  OSFlagPost(&RF_TransmitterFlagGroup,
                              FLAG_RF_TRANS_LEFT_BUMP_SENSOR,
                              OS_OPT_POST_FLAG_SET,
                              &err);        
              #else                                 // If in RF Receiver mode
                  OSFlagPost(&AppRobotControlFlagGroup,   
                              FLAG_LEFT_BUMP_SENSOR,
                              OS_OPT_POST_FLAG_SET,
                              &err);
              #endif    
        }
    }
}


/*
*********************************************************************************************************
*                                         ROBOT DISPLAY TASK
*
* Description : This is a task to display messages in the OLED display.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskRobotDisplay()' by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskRobotDisplay (void  *p_arg)
{
    CPU_INT08U  state;
    CPU_INT16U  sec;
    OS_ERR      err;
    
    //CPU_INT32U  clockspeed;        // ************ DEBUG ****************************************************************************
    CPU_INT08U  chip_id;
    CPU_INT08U  chip_ver;
    
    //uint8 pTxData[APP_PAYLOAD_LENGTH];          // **
    //uint8 pRxData[APP_PAYLOAD_LENGTH];          // **

   (void)&p_arg;

    sec   = 1u;
    state = 0u;

    while (DEF_ON) {
        OSTimeDlyHMSM(0u, 0u, sec, 300u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

        switch(state) {
            case 0:
              
                 //clockspeed = SysCtlClockGet();      // ************ DEBUG *****************************************************
                 chip_id  = halRfGetChipId();
                 chip_ver = halRfGetChipVer();
                 
                #if CC2520_IS_TRANSMITTER_MODE == 1                   	
                    BSP_DisplayStringDraw( "Sending Data" , 17u, 0u); 
                #else
                    BSP_DisplayStringDraw( "Receiving Data" , 13u, 0u); 
                    
                    // DEBUGGING MODE USING THE OLED DISPLAY
                    /*
                    //while(!basicRfPacketIsReady());

                    // New Packet available
                    if( basicRfPacketIsReady() == TRUE )
                    {
                        if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0)            // RECEIVE DATA
                        {	       
                            BSP_DisplayStringDraw( (const CPU_INT08S *) pRxData , 29u, 0u);
                            BSP_DisplayStringDraw("TRUE", 31u, 1u);
                            
                            // Decode packet message here (move it to the Receiver task later !!!! @@
                            // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                  
                            // left
                            if( strncmp( (char const *)&pRxData[0],"l",APP_PAYLOAD_LENGTH) == 0) {
                                       OSFlagPost(&AppRobotControlFlagGroup,
                                                FLAG_RF_LEFT_BUMP_SENSOR,
                                                OS_OPT_POST_FLAG_SET,
                                                &err);
                            }
                             // right
                            if( strncmp( (char const *)&pRxData[0],"r",APP_PAYLOAD_LENGTH) == 0) {
                                      OSFlagPost(&AppRobotControlFlagGroup,
                                                FLAG_RF_RIGHT_BUMP_SENSOR,
                                                OS_OPT_POST_FLAG_SET,
                                                &err);
                            }                           
                             // push button
                            if( strncmp( (char const *)&pRxData[0],"p",APP_PAYLOAD_LENGTH) == 0) {
                                      OSFlagPost(&AppRobotControlFlagGroup,   
                                                FLAG_PUSH_BUTTON,
                                                OS_OPT_POST_FLAG_SET,
                                                &err);
                            }                           
                            // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                          
                        }
                    }
                    // No new packet available
                    else
                    {
                        if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0)            // RECEIVE DATA
                        {	       
                            BSP_DisplayStringDraw( (const CPU_INT08S *) pRxData , 29u, 0u);
                            BSP_DisplayStringDraw("FALSE", 31u, 1u);
                        }
                        else
                        {
                            BSP_DisplayStringDraw( "No Data F" , 29u, 0u); 
                        }   
                     }
		*/	 
                #endif

                 sec   = 1u;	
                 state = 1u;   
                 break;


            case 1:
                 BSP_DisplayClear();
                 sec   = 1u;
                 state = 2u;
                 break;


            case 2:
                #if CC2520_IS_TRANSMITTER_MODE == 1
                    BSP_DisplayStringDraw("TRANSMITTER", 21u, 0u);
                    
                #else
                    BSP_DisplayStringDraw("RECEIVER", 21u, 0u);                  
                #endif   
                 
                    BSP_DisplayStringDraw("MODE", 31u, 1u);
                    
                 sec   = 1u;	
                 state = 3u;
                 break;


            case 3:
                 BSP_DisplayClear();
                 sec   = 1u;
				 state = 0u;	
                 break;


            default:
                 sec   = 1u;
                 state = 0u;
                 break;
        }
    }
}

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

/*
    RF Transmit Task
*/

#if CC2520_IS_TRANSMITTER_MODE == 1     // If in transmitter mode

static  void    AppTask_RF_Transmit (void  *p_arg)
{
    OS_FLAGS    flags;
    OS_ERR      err;
    CPU_TS      ts;
  
    uint8 status;                              
    uint8 pTxData[APP_PAYLOAD_LENGTH];                  
    
    while( DEF_ON ){
        
        OSFlagPend(&RF_TransmitterFlagGroup,          // Pend until a push button is pressed.                
                    FLAG_RF_TRANS_PUSH_BUTTON      +
                    FLAG_RF_TRANS_LEFT_BUMP_SENSOR +
                    FLAG_RF_TRANS_RIGHT_BUMP_SENSOR,
                    0u,
                    OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                    &ts,
                    &err);

        flags = OSFlagPendGetFlagsRdy(&err);           // Get the flag that was ready.                        
        
        // Upon push button press
        if( flags & FLAG_RF_TRANS_PUSH_BUTTON ){
                    pTxData[0] = 'p';
                    status = basicRfSendPacket(RX_ADDR, pTxData, APP_PAYLOAD_LENGTH);	// TRANSMIT DATA 
        }
        // Upon left bump sensor 
        else if( flags & FLAG_RF_TRANS_LEFT_BUMP_SENSOR) {
                    pTxData[0] = 'l';
                    status = basicRfSendPacket(RX_ADDR, pTxData, APP_PAYLOAD_LENGTH);	// TRANSMIT DATA       
        }
        // Upon right bump sensor
        else if( flags & FLAG_RF_TRANS_RIGHT_BUMP_SENSOR){
                    pTxData[0] = 'r';
                    status = basicRfSendPacket(RX_ADDR, pTxData, APP_PAYLOAD_LENGTH);	// TRANSMIT DATA 
        }
        
    }	
	
}
#else	// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/*
    RF Receiver Task
*/
static  void    AppTask_RF_Receive (void  *p_arg)
{
        OS_ERR  err;
	uint8 pRxData[APP_PAYLOAD_LENGTH];          // **  
  
        while (DEF_ON) {
            OSTimeDlyHMSM(0u, 0u, 0u, 50u,			
                          OS_OPT_TIME_HMSM_STRICT,
                          &err);
	
            // New Packet available
            if( basicRfPacketIsReady() == TRUE )
            {
                if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0)            // RECEIVE DATA
                {	       
                    //BSP_DisplayStringDraw( (const CPU_INT08S *) pRxData , 29u, 0u);
                    //BSP_DisplayStringDraw("TRUE", 31u, 1u);

					// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                    
                    // Decode packet message here

                    // left
                    if( strncmp( (char const *)&pRxData[0],"l",APP_PAYLOAD_LENGTH) == 0) {
                            OSFlagPost(&AppRobotControlFlagGroup,
                                        FLAG_RF_LEFT_BUMP_SENSOR,
                                        OS_OPT_POST_FLAG_SET,
                                        &err);
                    }
                    // right
                    if( strncmp( (char const *)&pRxData[0],"r",APP_PAYLOAD_LENGTH) == 0) {
                            OSFlagPost(&AppRobotControlFlagGroup,
                                        FLAG_RF_RIGHT_BUMP_SENSOR,
                                        OS_OPT_POST_FLAG_SET,
                                        &err);
                    }                           
                    // push button
                    if( strncmp( (char const *)&pRxData[0],"p",APP_PAYLOAD_LENGTH) == 0) {
                            OSFlagPost(&AppRobotControlFlagGroup,   
                                        FLAG_PUSH_BUTTON,
                                        OS_OPT_POST_FLAG_SET,
                                        &err);
                    }                           
                    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                          
                }
            }
            // No new packet available
            else
            {
                if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0)            // RECEIVE DATA
                {	       
                    //BSP_DisplayStringDraw( (const CPU_INT08S *) pRxData , 29u, 0u);
                    //BSP_DisplayStringDraw("FALSE", 31u, 1u);
                }
                else
                {
                    //BSP_DisplayStringDraw( "No Data F" , 29u, 0u); 
                }   
            }						  
							
        }            
}

#endif
          
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


static  void  AppRobotTimerCallback (OS_TMR  *p_tmr,
                                     void    *p_arg)
{
    OS_ERR  err;


   (void)&p_tmr;
   (void)&p_arg;

    OSFlagPost(&AppRobotControlFlagGroup,
               FLAG_TIMER_EXPIRATION,
               OS_OPT_POST_FLAG_SET,
               &err);
}


static  void  AppRobotDriveForward (CPU_INT08U  ucLeftMotorSpeed,
                                    CPU_INT08U  ucRightMotorSpeed)
{
    CPU_INT08U  pucMotorMsg[3];
    OS_ERR      err;


                                                                /* Send message to right and left wheel tasks to ...    */
                                                                /* ... start driving motors. Bound speed by max rpm.    */
    pucMotorMsg[0] = MSG_TYPE_MOTOR_DRIVE_START;                /* Speed control mode.                                  */
    pucMotorMsg[1] = MOTOR_DRIVE_FORWARD;                       /* Drive forward.                                       */

    pucMotorMsg[2] = (ucLeftMotorSpeed < MAX_RPM) ?
                      ucLeftMotorSpeed : MAX_RPM;
    OSTaskQPost((OS_TCB    *)&AppTaskRobotLeftMotorDriveTCB,
                (void      *)&pucMotorMsg[0],
                (OS_MSG_SIZE) 3u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);

    pucMotorMsg[2] = (ucRightMotorSpeed < MAX_RPM) ?
                      ucRightMotorSpeed : MAX_RPM;
    OSTaskQPost((OS_TCB    *)&AppTaskRobotRightMotorDriveTCB,
                (void      *)&pucMotorMsg[0],
                (OS_MSG_SIZE) 3u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);
}


static  void  AppRobotTurnLeft (void)
{
    CPU_INT08U  pucMsg[3];
    OS_ERR      err;


                                                                /* Send messages to cause to robot to turn left.        */
    pucMsg[0] = MSG_TYPE_MOTOR_DRIVE_START;                     /* Speed control mode.                                  */
    pucMsg[2] = 75;                                             /* 75 RPM                                               */

    pucMsg[1] = MOTOR_DRIVE_REVERSE;                            /* Reverse.                                             */
    OSTaskQPost((OS_TCB    *)&AppTaskRobotLeftMotorDriveTCB,
                (void      *)&pucMsg[0],
                (OS_MSG_SIZE) 3u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);

                                                                /* Just change direction.                               */
    pucMsg[1] = MOTOR_DRIVE_FORWARD;                            /* Forward.                                             */
    
    OSTaskQPost((OS_TCB    *)&AppTaskRobotRightMotorDriveTCB,
                (void      *)&pucMsg[0],
                (OS_MSG_SIZE) 3u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);
}


static  void  AppRobotTurnRight (void)
{
    CPU_INT08U  pucMsg[3];
    OS_ERR      err;


                                                                /* Send messages to cause to robot to turn right.       */
    pucMsg[0] = MSG_TYPE_MOTOR_DRIVE_START;                     /* Speed control mode.                                  */
    pucMsg[2] = 75;                                             /* 75 RPM                                               */

    pucMsg[1] = MOTOR_DRIVE_REVERSE;                            /* Reverse.                                             */
    OSTaskQPost((OS_TCB    *)&AppTaskRobotRightMotorDriveTCB,
                (void      *)&pucMsg[0],
                (OS_MSG_SIZE) 3u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);

    pucMsg[1] = MOTOR_DRIVE_FORWARD;                            /* Forward.                                             */
    
    OSTaskQPost((OS_TCB    *)&AppTaskRobotLeftMotorDriveTCB,
                (void      *)&pucMsg[0],
                (OS_MSG_SIZE) 3u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);
}


static  void  AppRobotStop (void)
{
    CPU_INT08U  ucMsg;
    OS_ERR      err;


                                                                /* Signal both motors to stop.                          */
    ucMsg = MSG_TYPE_MOTOR_STOP;
    OSTaskQPost((OS_TCB    *)&AppTaskRobotLeftMotorDriveTCB,
                (void      *)&ucMsg,
                (OS_MSG_SIZE) 1u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);
    OSTaskQPost((OS_TCB    *)&AppTaskRobotRightMotorDriveTCB,
                (void      *)&ucMsg,
                (OS_MSG_SIZE) 1u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);

    OSTimeDlyHMSM(0u, 0u, 0u, 100u,                             /* Delay for 100ms to allow motor to stop turning.      */
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
}


#if 0 // Unused Reference
void  AppRobotDriveTmrInit (CPU_INT16U  usDriveTimeWindow)
{
    OS_ERR      err;

    OSTmrCreate((OS_TMR            *)&AppRobotDriveTmr,         /* Create drive timer with specified timeout.           */
                (CPU_CHAR          *)"Drive Timer",
                (OS_TICK            ) usDriveTimeWindow,
                (OS_TICK            ) 0u,
                (OS_OPT             ) OS_OPT_TMR_ONE_SHOT,
                (OS_TMR_CALLBACK_PTR) AppRobotTimerCallback,
                (void              *) 0,
                (OS_ERR            *)&err);

    OSTmrStart(&AppRobotDriveTmr, &err);                        /* Start drive timer.                                   */
}

void  AppRobotDriveTmrDel (void)
{
    OS_ERR  err;


    OSTmrDel(&AppRobotDriveTmr, &err);                          /* Delete drive timer.                                  */
}
#endif	// end of Unused Reference

static  void  AppRobotTurnTmrInit ( CPU_INT16U   usDriveTimeWindow )
{
    OS_ERR      err;

    OSTmrCreate((OS_TMR            *)&AppRobotTurnTmr,          /* Create turn timer with specified timeout.            */
                (CPU_CHAR          *)"Turn Timer",
                (OS_TICK            ) usDriveTimeWindow,
                (OS_TICK            ) 0u,
                (OS_OPT             ) OS_OPT_TMR_ONE_SHOT,
                (OS_TMR_CALLBACK_PTR) AppRobotTimerCallback,
                (void              *) 0,
                (OS_ERR            *)&err);

    OSTmrStart(&AppRobotTurnTmr, &err);                         /* Start turn timer.                                    */
}


static  void  AppRobotTurnTmrDel (void)
{
    OS_ERR  err;


    OSTmrDel(&AppRobotTurnTmr, &err);                           /* Delete turn timer.                                   */
}


static  void  AppRobotWheelSensorIntHandler (void)
{
    static CPU_INT32U  ulRightEdgePrevTime;
    static CPU_INT32U  ulLeftEdgePrevTime;
    CPU_INT32U         ulRightEdgeDiff;
    CPU_INT32U         ulLeftEdgeDiff;
    CPU_INT32U         ulRightEdgeTime;
    CPU_INT32U         ulLeftEdgeTime;
    CPU_INT08U         ucRightRPM;
    CPU_INT08U         ucLeftRPM;
    CPU_INT32U         ulStatus;
    CPU_INT08U         ucCnt;
    OS_ERR             err;


    ulStatus = GPIOPinIntStatus(LEFT_SIDE_SENSOR_PORT, DEF_TRUE);
    if (ulStatus & LEFT_SIDE_SENSOR_PIN) {

        GPIOPinIntClear(LEFT_SIDE_SENSOR_PORT,                  /* Clear interrupt.                                     */
                        LEFT_SIDE_SENSOR_PIN);

                                                                /* {Workaround}                                         */
        for (ucCnt = 0u; ucCnt < 100u; ucCnt++) {               /* Check for pin high (rising edge).                    */

            if (!GPIOPinRead(LEFT_SIDE_SENSOR_PORT, LEFT_SIDE_SENSOR_PIN)) {
                return;
            }
        }

                                                                /* Check to make sure that the time since the last ...  */
                                                                /* ... makes sense. This only matters if it is not ...  */
                                                                /* ... the first edge.                                  */
        if (!AppRobotLeftWheelFirstEdge) {

            ulLeftEdgeTime = OSTimeGet(&err);
            if(ulLeftEdgeTime <= ulLeftEdgePrevTime) {
                                                                /* Account for rollover.                                */
                ulLeftEdgeDiff = ulLeftEdgeTime + (0xFFFFFFFFu - ulLeftEdgePrevTime);
            } else {
                ulLeftEdgeDiff = ulLeftEdgeTime - ulLeftEdgePrevTime; 
            }

                                                                /* Is the time difference less than expected for ...    */
                                                                /* ... the MAX_RPM + some buffer.                       */
            if ((ulLeftEdgeDiff) < (OSCfg_TickRate_Hz / (((MAX_RPM + 10u) * 8u) / 60u))) {
                return;
            }
        }
                                                                /* {Workaround End}                                     */

        if (AppRobotLeftWheelFirstEdge) {
            ulLeftEdgePrevTime = OSTimeGet(&err);
            AppRobotLeftWheelFirstEdge = DEF_FALSE;
        } else {
            ulLeftEdgeTime = OSTimeGet(&err);
            if (ulLeftEdgeTime <= ulLeftEdgePrevTime) {
                                                                /* Account for rollover.                                */
                ulLeftEdgeDiff  = ulLeftEdgeTime +
                                 (0xFFFFFFFFu - ulLeftEdgePrevTime);
            } else {
                ulLeftEdgeDiff  = ulLeftEdgeTime - ulLeftEdgePrevTime; 
            }

            ucLeftRPM = (60u * OSCfg_TickRate_Hz) /
                        ( 8u * ulLeftEdgeDiff);

            ulLeftEdgePrevTime = ulLeftEdgeTime;

                                                                /* Pass actual speed to PID task.                       */
            AppRobotMotorPIDTaskActualSpeedSet(LEFT_SIDE, ucLeftRPM);
        }
    }

    ulStatus = GPIOPinIntStatus(RIGHT_SIDE_SENSOR_PORT, DEF_TRUE);
    if (ulStatus & RIGHT_SIDE_SENSOR_PIN) {

        GPIOPinIntClear(RIGHT_SIDE_SENSOR_PORT,                 /* Clear interrupt.                                     */
                        RIGHT_SIDE_SENSOR_PIN);

                                                                /* {Workaround}                                         */
        for (ucCnt = 0u; ucCnt < 100u; ucCnt++) {               /* Check for pin high (rising edge).                    */
            if (!GPIOPinRead(RIGHT_SIDE_SENSOR_PORT, RIGHT_SIDE_SENSOR_PIN)) {
                return;
            }
        }

                                                                /* Check to make sure that the time since the last ...  */
                                                                /* ... makes sense. This only matters if it is not ...  */
                                                                /* ... the first edge.                                  */
        if (!AppRobotRightWheelFirstEdge) {
            ulRightEdgeTime = OSTimeGet(&err);
            if (ulRightEdgeTime <= ulRightEdgePrevTime) {
                                                                /* Account for rollover.                                */
                ulRightEdgeDiff  = ulRightEdgeTime + 
                                  (0xFFFFFFFFu - ulRightEdgePrevTime);
            } else {
                ulRightEdgeDiff  = ulRightEdgeTime - ulRightEdgePrevTime; 
            }

                                                                /* Is the time difference less than expected for ...    */
                                                                /* ... the MAX_RPM + some buffer.                       */
            if((ulRightEdgeDiff) < (OSCfg_TickRate_Hz / (((MAX_RPM + 10u) * 8u) / 60u))) {
                return;
            }
        }
                                                                /* {Workaround End}                                     */

        if (AppRobotRightWheelFirstEdge) {
            ulRightEdgePrevTime = OSTimeGet(&err);
            AppRobotRightWheelFirstEdge = DEF_FALSE;

        } else {

            ulRightEdgeTime = OSTimeGet(&err);
            if (ulRightEdgeTime <= ulRightEdgePrevTime) {
                                                                /* Account for rollover.                                */
                ulRightEdgeDiff  = ulRightEdgeTime +
                                  (0xFFFFFFFFu - ulRightEdgePrevTime) + 1u;

            } else {

                ulRightEdgeDiff = ulRightEdgeTime - ulRightEdgePrevTime; 
            }

            ucRightRPM = (60u * OSCfg_TickRate_Hz) /
                         ( 8u * ulRightEdgeDiff);

            ulRightEdgePrevTime = ulRightEdgeTime;

                                                                /* Pass actual speed to PID task.                       */
            AppRobotMotorPIDTaskActualSpeedSet(RIGHT_SIDE, ucRightRPM);
        }
    }
}


static  void  AppRobotMotorDriveSensorEnable (tSide  eSide)
{
    if (eSide == AppRobotLeftSide) {

        AppRobotLeftWheelFirstEdge = DEF_TRUE;                  /* Indicate the first edge has yet to occur.            */

        BSP_WheelSensorEnable();                                /* Enable wheel sensors.                                */

        BSP_WheelSensorIntEnable(LEFT_SIDE, LEFT_SIDE_SENSOR,   /* Enable wheel sensor interrupts.                      */
                                 AppRobotWheelSensorIntHandler);

    } else {

        AppRobotRightWheelFirstEdge = DEF_TRUE;                 /* Indicate the first edge has yet to occur.            */

        BSP_WheelSensorEnable();                                /* Enable wheel sensors.                                */

        BSP_WheelSensorIntEnable(RIGHT_SIDE, RIGHT_SIDE_SENSOR, /* Enable wheel sensor interrupts.                      */
                                 AppRobotWheelSensorIntHandler);
    }
}


static  void  AppRobotMotorDriveSensorDisable (tSide  eSide)
{
    if (eSide == AppRobotLeftSide) {

        BSP_WheelSensorDisable();                               /* Disable wheel sensors.                               */

        BSP_WheelSensorIntDisable(LEFT_SIDE, LEFT_SIDE_SENSOR); /* Enable wheel sensor interrupts.                      */

    } else {

        BSP_WheelSensorDisable();                               /* Disable wheel sensors.                               */

        BSP_WheelSensorIntDisable(RIGHT_SIDE,                   /* Enable wheel sensor interrupts.                      */
                                  RIGHT_SIDE_SENSOR);
    }
}


static  void  AppRobotMotorPIDTaskTargetSpeedSet (tSide       eSide,
                                                  CPU_INT08U  ucSpeed)
{
    OS_REG  reg;                                                /* 32-bits task register. Second byte is target speed.  */
    OS_ERR  err;


    if (eSide == LEFT_SIDE) {
        reg  =  OSTaskRegGet(&AppTaskRobotLeftMotorPIDTCB, 0, &err);
        reg &=  0xFFFF00FFu;
        reg |= (ucSpeed << 8u);
        OSTaskRegSet(&AppTaskRobotLeftMotorPIDTCB, 0, reg, &err);

    } else {

        reg  =  OSTaskRegGet(&AppTaskRobotRightMotorPIDTCB, 0, &err);
        reg &=  0xFFFF00FFu;
        reg |= (ucSpeed << 8u);
        OSTaskRegSet(&AppTaskRobotRightMotorPIDTCB, 0, reg, &err);
    }
}


static  void  AppRobotMotorPIDTaskActualSpeedSet (tSide       eSide,
                                                  CPU_INT08U  ucSpeed)
{
    OS_REG  reg;                                                /* 32-bits task register. First byte is actual speed.   */
    OS_ERR  err;


    if (eSide == LEFT_SIDE) {
        reg  = OSTaskRegGet(&AppTaskRobotLeftMotorPIDTCB, 0, &err);
        reg &= 0xFFFFFF00u;
        reg |= ucSpeed;
        OSTaskRegSet(&AppTaskRobotLeftMotorPIDTCB, 0, reg, &err);

    } else {

        reg  = OSTaskRegGet(&AppTaskRobotRightMotorPIDTCB, 0, &err);
        reg &= 0xFFFFFF00u;
        reg |= ucSpeed;
        OSTaskRegSet(&AppTaskRobotRightMotorPIDTCB, 0, reg, &err);
    }
}


static CPU_INT08U  AppRobotMotorPIDTaskTargetSpeedGet (tSide  eSide)
{
    OS_REG  reg;                                                /* 32-bits task register. Second byte is target speed.  */
    OS_ERR  err;


    if (eSide == LEFT_SIDE) {
        reg = OSTaskRegGet(&AppTaskRobotLeftMotorPIDTCB, 0, &err);
        return (CPU_INT08U)((reg & 0xFF00u) >> 8u);

    } else {

        reg = OSTaskRegGet(&AppTaskRobotRightMotorPIDTCB, 0, &err);
        return (CPU_INT08U)((reg & 0xFF00u) >> 8u);
    }
}


static  CPU_INT08U  AppRobotMotorPIDTaskActualSpeedGet (tSide  eSide)
{
    OS_REG  reg;                                                /* 32-bits task register. First byte is actual speed.   */
    OS_ERR  err;


    if (eSide == LEFT_SIDE) {
        reg = OSTaskRegGet(&AppTaskRobotLeftMotorPIDTCB, 0u, &err);
        return (CPU_INT08U)(reg & 0xFFu);

    } else {

        reg = OSTaskRegGet(&AppTaskRobotRightMotorPIDTCB, 0u, &err);
        return (CPU_INT08U)(reg & 0xFFu);
    }
}


void  AppRobotMotorPIDTaskStart (tSide  eSide)
{
    OS_ERR  err;


                                                                /* Post the start flag to the PID control task.         */
    if (eSide == LEFT_SIDE) {
        OSFlagPost(&AppRobotPIDLeftFlagGroup,
                   FLAG_PID_START,
                   OS_OPT_POST_FLAG_SET,
                   &err);

    } else {

        OSFlagPost(&AppRobotPIDRightFlagGroup,
                   FLAG_PID_START,
                   OS_OPT_POST_FLAG_SET,
                   &err);
    }
}


void  AppRobotMotorPIDTaskStop (tSide  eSide)
{
    OS_ERR  err;


                                                                /* Post the start flag to the PID control task.         */
    if (eSide == LEFT_SIDE) {
        OSFlagPost(&AppRobotPIDLeftFlagGroup,
                    FLAG_PID_STOP,
                    OS_OPT_POST_FLAG_SET,
                   &err);

    } else {

        OSFlagPost(&AppRobotPIDRightFlagGroup,
                    FLAG_PID_STOP,
                    OS_OPT_POST_FLAG_SET,
                   &err);
    }
}


static  void  AppRobotMotorPIDTaskFlagCreate (tSide  eSide)
{
    OS_ERR  err;


                                                                /* Create the flag group used by the task.              */
    if (eSide == LEFT_SIDE) {
        OSFlagCreate(&AppRobotPIDLeftFlagGroup,
                     "Robot Control Task Flag Group",
                     (OS_FLAGS)0,
                     &err);

    } else {

        OSFlagCreate(&AppRobotPIDRightFlagGroup,
                     "Robot Control Task Flag Group",
                     (OS_FLAGS)0,
                     &err);
    }
}


static  void  AppRobotMotorPIDTaskStartFlagPend (tSide  eSide)
{
    OS_ERR  err;
    CPU_TS  ts;


                                                                /* Look for the stop flag. Timeout after 10ms and ...   */
                                                                /* ... run the PID controller if stop flag not set.     */
    if (eSide == LEFT_SIDE) {
        OSFlagPend(&AppRobotPIDLeftFlagGroup,
                   FLAG_PID_START,
                   0u,
                   OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                   &ts,
                   &err);

    } else {

        OSFlagPend(&AppRobotPIDRightFlagGroup,
                   FLAG_PID_START,
                   0u,
                   OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                   &ts,
                   &err);
    }
}


static  OS_ERR  AppRobotMotorPIDTaskStopFlagPend (tSide  eSide)
{
    OS_ERR  err;
    CPU_TS  ts;


                                                                /* Look for the stop flag. Timeout after 10ms and ...   */
                                                                /* ... run the PID controller if stop flag not set.     */
    if (eSide == LEFT_SIDE) {
        OSFlagPend(&AppRobotPIDLeftFlagGroup,
                   FLAG_PID_STOP,
                   5u,
                   OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                   &ts,
                   &err);

    } else {

        OSFlagPend(&AppRobotPIDRightFlagGroup,
                   FLAG_PID_STOP,
                   5u,
                   OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                   &ts,
                   &err);
    }

    return err;
}


static  void  AppRobotTasksCreate (void)
{
    OS_ERR  err;


    OSTaskCreate((OS_TCB     *)&AppTaskRobotControlTCB,         /* Main control task.                                   */
                 (CPU_CHAR   *)"Control Task",
                 (OS_TASK_PTR ) AppTaskRobotControl,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_ROBOT_CONTROL_PRIO,
                 (CPU_STK    *)&AppTaskRobotControlStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_CONTROL_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_CONTROL_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskRobotLeftMotorDriveTCB,  /* Motor control and motor PID tasks.                   */
                 (CPU_CHAR   *)"Left Motor Drive Task",
                 (OS_TASK_PTR ) AppTaskRobotMotorDrive,
                 (void       *)&AppRobotLeftSide,
                 (OS_PRIO     ) APP_TASK_ROBOT_MOTOR_DRIVE_PRIO,
                 (CPU_STK    *)&AppTaskRobotLeftMotorDriveStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_MOTOR_DRIVE_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_MOTOR_DRIVE_STK_SIZE,
                 (OS_MSG_QTY  ) 32u,
                 (OS_TICK     )  0u,
                 (void       *)  0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskRobotRightMotorDriveTCB,
                 (CPU_CHAR   *)"Right Motor Drive Task",
                 (OS_TASK_PTR ) AppTaskRobotMotorDrive,
                 (void       *)&AppRobotRightSide,
                 (OS_PRIO     ) APP_TASK_ROBOT_MOTOR_DRIVE_PRIO,
                 (CPU_STK    *)&AppTaskRobotRightMotorDriveStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_MOTOR_DRIVE_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_MOTOR_DRIVE_STK_SIZE,
                 (OS_MSG_QTY  ) 32u,
                 (OS_TICK     )  0u,
                 (void       *)  0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskRobotLeftMotorPIDTCB,
                 (CPU_CHAR   *)"Left Motor PID Task",
                 (OS_TASK_PTR ) AppTaskRobotMotorPID,
                 (void       *)&AppRobotLeftSide,
                 (OS_PRIO     ) APP_TASK_ROBOT_MOTOR_PID_PRIO,
                 (CPU_STK    *)&AppTaskRobotLeftMotorPIDStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_MOTOR_PID_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_MOTOR_PID_STK_SIZE,
                 (OS_MSG_QTY  ) 10u,
                 (OS_TICK     )  0u,
                 (void       *)  0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskRobotRightMotorPIDTCB,
                 (CPU_CHAR   *)"Right Motor PID Task",
                 (OS_TASK_PTR ) AppTaskRobotMotorPID,
                 (void       *)&AppRobotRightSide,
                 (OS_PRIO     ) APP_TASK_ROBOT_MOTOR_PID_PRIO,
                 (CPU_STK    *)&AppTaskRobotRightMotorPIDStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_MOTOR_PID_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_MOTOR_PID_STK_SIZE,
                 (OS_MSG_QTY  ) 10u,
                 (OS_TICK     )  0u,
                 (void       *)  0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskRobotInputMonitorTCB,    /* Switch monitor task.                                 */
                 (CPU_CHAR   *)"Switch/Sensor Monitor Task",
                 (OS_TASK_PTR ) AppTaskRobotInputMonitor,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_ROBOT_SWITCH_MONITOR_PRIO,
                 (CPU_STK    *)&AppTaskRobotInputMonitorStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_SWITCH_MONITOR_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_SWITCH_MONITOR_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskRobotDisplayTCB,         /* Display task.                                        */
                 (CPU_CHAR   *)"Display Task",
                 (OS_TASK_PTR ) AppTaskRobotDisplay,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_ROBOT_DISPLAY_PRIO,
                 (CPU_STK    *)&AppTaskRobotDisplayStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_DISPLAY_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_DISPLAY_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    #if CC2520_IS_TRANSMITTER_MODE == 1         // If in RF Transmitter mode       
        // Create an RF Transmit Task
        OSTaskCreate((OS_TCB *)&AppTaskRobotRF_TransmitTCB,     // RF Transmit task.   
                 (CPU_CHAR   *)"RF Transmit Task",
                 (OS_TASK_PTR ) AppTask_RF_Transmit,            
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_ROBOT_RF_TRANSMIT_PRIO,
                 (CPU_STK    *)&AppTaskRobotRF_TransmitStk[0],     
                 (CPU_STK_SIZE) APP_TASK_ROBOT_RF_TRANSMIT_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_RF_TRANSMIT_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
				 
	#else					// If in RF Receiver mode			 
        // Create an RF Receiver Task      
        OSTaskCreate((OS_TCB *)&AppTaskRobotRF_ReceiveTCB,     // RF Receiver task. 
                 (CPU_CHAR   *)"RF Receive Task",
                 (OS_TASK_PTR ) AppTask_RF_Receive,            
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_ROBOT_RF_RECEIVE_PRIO,
                 (CPU_STK    *)&AppTaskRobotRF_ReceiveStk[0],     
                 (CPU_STK_SIZE) APP_TASK_ROBOT_RF_RECEIVE_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_RF_RECEIVE_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
        
    #endif				 
				 			 
    OSFlagCreate(&AppRobotControlFlagGroup,                     /* Create flag group used by the application.           */
                 "Robot Control Task Flag Group",
                 (OS_FLAGS)0,
                 &err);

    #if CC2520_IS_TRANSMITTER_MODE == 1             // If in RF Transmitter mode
        // Create a Flag Group for an RF Transmitter 
        OSFlagCreate(&RF_TransmitterFlagGroup,      /* Create flag group for RF Transmitter. @@@@@@@@@@@@@@@@@@@ */
                    "RF Transmitter Flag Group",
                    (OS_FLAGS)0,
                    &err);
    #endif				 
}
