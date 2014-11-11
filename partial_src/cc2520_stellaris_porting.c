
/* 	This work is licensed under a Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
*
* 	Copyright(c) 2014 Hong Moon 	All Rights Reserved
*/

/* 	@description: This file is intended to port the CC2520-CC2591EMK module on the Stellaris Evalbot MCU with SPI communication.

				 Note that this is NOT the full source code needed to port the CC2520-CC2591EMK. Sadly, due to the licensing 
				 requirements from the chip vendor, the full source code cannot be distributed in anyway. However, this "cc2520_stellaris_porting"
				 header file and the source file, written by me, show about 70 percent of the job needed.  
				 
				 The rest involves changing the code from the chip vendor. For example, stopping irrelevant function calls from being called, 
				 fixing the erroneous implementation of the ISR, and adding some unavoidable board specific changes in the following files:
				 
					basic_rf.c, hal_board.c, hal_board.h, hal_cc2520.c, hal_cc2520.h, hal_mcu.c, hal_mcu.h, hal_rf.h, hal_int.c, hal_int.h, hal_rf_security.c
				 
	@disclaimer:
				Note that this software is NOT intended for sale or any commercial application. Only for demo purpose.

	@name:	 	Hong Moon (hsm5xw@gmail.com)
	@date: 		2014-Oct
	@platform: 	Texas Instruments Stellaris Evalbot (lm3s9b92)
	
	You must retain the above header, disclaimer, and copyright notice in this file. 		
*/


/***********************************************************************************
* INCLUDES
*/
#include "cc2520_stellaris_porting.h"

/***********************************************************************************
 * LOCAL VARIABLES
 */

//static uint8 pTxData[APP_PAYLOAD_LENGTH];
//static uint8 pRxData[APP_PAYLOAD_LENGTH];
static basicRfCfg_t basicRfConfig;

#ifdef SECURITY_CCM
// Security key
static uint8 key[]= {
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
};
#endif

/***********************************************************************************
* LOCAL FUNCTIONS
*/

static uint8 CC2520_Transmitter_Config( void );
static uint8 CC2520_Receiver_Config( void );

// *************************** NEW ****************************************************************************************

/*
	Controls the GPIO_RF_RESET output pin
	GPIO_RF_RESET_PIN	(GPIO Port C Pin 5 on Stellaris Evalbot) ***
*/
void CC2520_RESET_OPIN(uint8 data)
{
    if(data == 0)	// Reset
		GPIOPinWrite( GPIO_PORTC_BASE, GPIO_PIN_5, 0 );
    else			// Set
		GPIOPinWrite( GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5 );
}

/* 
	Controls the GPIO_RF_VREG_EN output pin
	GPIO_RF_VREG_EN_PIN  (PH2: GPIO Port H Pin 2 on Stellaris Evalbot) ***
*/
void CC2520_VREG_EN_OPIN(uint8 data)
{
    if(data == 0)	// Reset	
		GPIOPinWrite( GPIO_PORTH_BASE, GPIO_PIN_2, 0 );
    else			// Set
		GPIOPinWrite( GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_PIN_2 );
}

// ****************************** RF_GPIO PINS ********************************************************************************

/* 
	Controls the CC2520_CSN output pin
	(PH5: GPIO Port H Pin 5 on Stellaris Evalbot) ***
*/
void CC2520_CSN_OPIN(uint8 data)
{
    if(data == 0)	// Reset	
		GPIOPinWrite( GPIO_PORTH_BASE, GPIO_PIN_5, 0 );
    else			// Set
		GPIOPinWrite( GPIO_PORTH_BASE, GPIO_PIN_5, GPIO_PIN_5 );
}

/* 
	Controls the CC2520_SCLK output pin
	(PH4: GPIO Port H Pin 4 on Stellaris Evalbot) ***              
*/            
void CC2520_SCLK_OPIN(uint8 data)
{
    if(data == 0)	// Reset	
		GPIOPinWrite( GPIO_PORTH_BASE, GPIO_PIN_4, 0 );
    else			// Set
		GPIOPinWrite( GPIO_PORTH_BASE, GPIO_PIN_4, GPIO_PIN_4 );
}

/* 
	Controls the CC2520_MOSI output pin
	(PH7: GPIO Port H Pin 7 on Stellaris Evalbot) ***            
*/        
void CC2520_MOSI_OPIN(uint8 data)  
{
    if(data == 0)	// Reset	
		GPIOPinWrite( GPIO_PORTH_BASE, GPIO_PIN_7, 0 );
    else			// Set
		GPIOPinWrite( GPIO_PORTH_BASE, GPIO_PIN_7, GPIO_PIN_7 );
}  

// ************************************************************ SPI **************************************************************

#ifndef SW_SPI_MODE			// Actually not using them since I now use software SPI !!!!!! reserved for compatibility
// transmit
void CC2520_SPI_TX(uint8 data)
{
	SSIDataPut( SSI1_BASE, (unsigned long)data );
}

// wait
void CC2520_SPI_WAIT_RXRDY(void)
{
	// wait until SSI1 is done transferring all the data in the transmit FIFO
	while( SSIBusy(SSI1_BASE) ){}
	
}

// receive
uint8 CC2520_SPI_RX(void)
{
	unsigned long data = 0xFFFFFFFF;
	uint8 result = 0;
	
	SSIDataGet( SSI1_BASE, &data);
	result = (uint8) ( data & 0xFF );

	return result;
}
#endif

// *********************************************************** GPIO INTERRUPT *************************************************************

// GPIO 0 Interrupt Enable
void CC2520_GPIO0_IntEnable(void)
{
	GPIOPinIntClear(  CC2520_GPIO_RF_GPIO0_PORT, CC2520_GPIO_RF_GPIO0_PIN);	
	BSP_IntVectSet(	GPIO_RF_GPIO0_INT, CC2520_GPIO0_IntHandler);			// ISR handler
	GPIOPinIntEnable( CC2520_GPIO_RF_GPIO0_PORT, CC2520_GPIO_RF_GPIO0_PIN);
}

// GPIO 0 Interrupt Disable
void CC2520_GPIO0_IntDisable(void)
{
	GPIOPinIntDisable( CC2520_GPIO_RF_GPIO0_PORT, CC2520_GPIO_RF_GPIO0_PIN);
}

// GPIO 0 Interrupt Clear
void CC2520_GPIO0_IntClear(void)
{
	GPIOPinIntClear(  CC2520_GPIO_RF_GPIO0_PORT, CC2520_GPIO_RF_GPIO0_PIN);	
}

// GPIO 0 Interrupt Handler
void CC2520_GPIO0_IntHandler(void)
{
	CPU_INT32U  ulStatus;
	
	ulStatus = GPIOPinIntStatus( CC2520_GPIO_RF_GPIO0_PORT, true); // get status for a masked interrupt
	
	if( ulStatus & CC2520_GPIO_RF_GPIO0_PIN ){
			GPIOPinIntClear( CC2520_GPIO_RF_GPIO0_PORT, CC2520_GPIO_RF_GPIO0_PIN );
			basicRfRxFrmDoneIsr();
	}
}

// ************************************* BSP_CC2520 ********************************************************************************************

/*	IMPORTANT FUNCTION

	Configures the hardware peripherals related to CC2520.
	Configures the RF setting and sets the MCU as either a transmitter or a receiver.
	
	This function should be called at the last line of the BSP_Init() function in "bsp.c" file,
    where all the hardware peripherals and devices are initialized.	
*/
void BSP_CC2520Init(void)
{
    uint8 config_status = CONFIG_FAILED;   // added for debugging purpose

    /* Config basicRF */
	
	/* Frequency Range: 	2405-2480MHz
	   Selected  Frequency: 2405 + 5( basicRfConfig.channel - 11) MHz   
	*/
    basicRfConfig.panId = PAN_ID;
    basicRfConfig.channel = RF_CHANNEL;	// must be between MIN_CHANNEL and MAX_CHANNEL
    basicRfConfig.ackRequest = TRUE;
	
	#ifdef SECURITY_CCM
    basicRfConfig.securityKey = key;
	#endif

	// **************************************************************
    CC2520_SPI_Init();		// SPI  connection set-up
    CC2520_GPIO_Init();		// GPIO connection set-up
		
    halRfInit();
	
    // MAKE SURE to choose the right mode in "cc2520_stellaris_porting.h" file !!!
    #if CC2520_IS_TRANSMITTER_MODE == 1	 
	config_status = CC2520_Transmitter_Config();
    #else
	config_status = CC2520_Receiver_Config();
    #endif
}

/* 
	GPIO Configuration to interface with CC2520.
	
	Also registers an interrupt handler for the CC2520's GPIO0 pin, which is for the RX_FRM_DONE exception.
	See the data-sheet for more information.

*/
void CC2520_GPIO_Init(void)
{
	// Register the port-level interrupt handler
	GPIOPortIntRegister( CC2520_GPIO_RF_GPIO0_PORT, CC2520_GPIO0_IntHandler ); // ISR handler for GPIO_PORTC

	// GPIO Port C (OUTPUT):	Configure CC2520_RESET_O_PIN
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, 	CC2520_RESET_OPIN_NUMBER );
        GPIOPadConfigSet(GPIO_PORTC_BASE, 	CC2520_RESET_OPIN_NUMBER, GPIO_STRENGTH_2MA,
					GPIO_PIN_TYPE_STD_WPU );
	  
	// GPIO Port H (OUTPUT):	Configure CC2520_VREG_EN_OPIN
	GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, 	CC2520_VREG_EN_OPIN_NUMBER );
        GPIOPadConfigSet(GPIO_PORTH_BASE, 	CC2520_VREG_EN_OPIN_NUMBER, GPIO_STRENGTH_2MA,
					GPIO_PIN_TYPE_STD_WPU );	
	
    // ************************************************************************************************************************************
        
	// GPIO Port C (INPUT):		 Configure  CC2520_RX_FRM_DONE_PIN
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, 	CC2520_RX_FRM_DONE_PIN_NUMBER);
        
	// GPIO Port B (INPUT):		Configure CC2520_SAMPLED_CCA_PIN, CC2520_RSSI_VALID_PIN, CC2520_TX_FRM_DONE_PIN
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, 	CC2520_SAMPLED_CCA_PIN_NUMBER |  CC2520_RSSI_VALID_PIN_NUMBER |  CC2520_TX_FRM_DONE_PIN_NUMBER );

    
    // Disable pin interrupts (CC2520_RX_FRM_DONE_PIN, which is GPIO0 interrupt)
    GPIOPinIntDisable(CC2520_GPIO_RF_GPIO0_PORT, CC2520_GPIO_RF_GPIO0_PIN);	// CC2520_RX_FRM_DONE_PIN
	GPIOPinIntDisable(GPIO_PORTC_BASE, CC2520_RESET_OPIN_NUMBER);			// CC2520_RESET_O_PIN
       
    // Set the interrupt as rising-edge triggered
    GPIOIntTypeSet(CC2520_GPIO_RF_GPIO0_PORT, CC2520_GPIO_RF_GPIO0_PIN, GPIO_RISING_EDGE);

	// Enable Interrupt for GPIO_RF_GPIO0_PIN
	// The interrupts for the individual pins still need to be enabled by CC2520_GPIO0_IntEnable().
	BSP_IntEn( GPIO_RF_GPIO0_INT );      // uC/OS-III RTOS support **
}


/* 
	Software SPI Init
*/
void CC2520_SPI_Init(void)
{
	/* Configure SPI MISO signal (Input) */
    GPIOPinTypeGPIOInput(SSI1_RX_PORT, SSI1_RX_PIN );
    GPIOPadConfigSet(SSI1_RX_PORT, 	SSI1_RX_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU ); 
			
    /* Configure SPI CS signal (Output) */
    GPIOPinTypeGPIOOutput(SSI1_CS_PORT, SSI1_CS_PIN );
    GPIOPadConfigSet(SSI1_CS_PORT, 	SSI1_CS_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU );  

	/* Configure SPI SCLK signal (Output) */
	GPIOPinTypeGPIOOutput(SSI1_SCLK_PORT, SSI1_SCLK_PIN );
    GPIOPadConfigSet(SSI1_SCLK_PORT, SSI1_SCLK_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU ); 
	
	/* Configure SPI MOSI signal (Output) */
    GPIOPinTypeGPIOOutput(SSI1_TX_PORT, SSI1_TX_PIN );
    GPIOPadConfigSet(SSI1_TX_PORT, SSI1_TX_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU ); 	
	
	SW_SPI_CLK_Init();
	
	CC2520_RESET_OPIN(1);
    CC2520_VREG_EN_OPIN(1);
}

// ********** Software SPI *******************************************************************************************************************************

void TIMER_Wait_us( uint32 nCount)
{
	uint32 count;
	for( count = nCount; count != 0; count-- );
}

// SPI MOSI High
void SW_SPI_MOSI_H(void)	
{
	GPIOPinWrite( SSI1_TX_PORT, SSI1_TX_PIN , SSI1_TX_PIN);
}

// SPI MOSI Low
void SW_SPI_MOSI_L(void)	
{
	GPIOPinWrite( SSI1_TX_PORT, SSI1_TX_PIN , 0);
}

// SPI CLK  High
void SW_SPI_CLK_H(void)	
{
	GPIOPinWrite( SSI1_SCLK_PORT, SSI1_SCLK_PIN , SSI1_SCLK_PIN);
}

// SPI CLK  Low
void SW_SPI_CLK_L(void)	
{
	GPIOPinWrite( SSI1_SCLK_PORT, SSI1_SCLK_PIN , 0);
}

// MISO Read
uint8 SW_SPI_ReadVal_MISO(void)
{
	return (uint8) GPIOPinRead( SSI1_RX_PORT, SSI1_RX_PIN );
}

// MOSI Write
void  SW_SPI_MOSI_OUT(uint8 out)
{
	if( out)
		SW_SPI_MOSI_H();
	else
		SW_SPI_MOSI_L();
		
	TIMER_Wait_us( SW_SPI_WAIT_TIME );	
}

// SPI CLK Init
void SW_SPI_CLK_Init(void)
{
	if( SW_SPI_CPOL_val == 0)
		SW_SPI_CLK_L();
	else
		SW_SPI_CLK_H();
		
	TIMER_Wait_us( SW_SPI_WAIT_TIME );
}

// SPI CLK Toggle
void SW_SPI_CLK_Toggle(void)
{
	GPIOPinWrite( SSI1_SCLK_PORT, SSI1_SCLK_PIN, (SSI1_SCLK_PIN - GPIOPinRead(SSI1_SCLK_PORT, SSI1_SCLK_PIN)) );
	TIMER_Wait_us( SW_SPI_WAIT_TIME );
}

/*
	SPI TXRX	
	Software SPI TXRX routine.
*/
uint8 SW_SPI_TXRX(uint8 tx_data)
{
	uint8 rx_data = 0;
	uint32 i = 0;
	uint8 bitCheckVal = 0x80;
	
	if( SW_SPI_CPHA_val == 1){
		SW_SPI_CLK_Toggle();
	}
	
	for( i=0; i<8; i++)
	{
		SW_SPI_MOSI_OUT( tx_data & bitCheckVal);
		bitCheckVal >>= 1;
		SW_SPI_CLK_Toggle();		// clock toggle
		
		rx_data <<= 1;
		
		// if bit set
		if( SW_SPI_ReadVal_MISO() != 0)
			rx_data |= 0x1;
			
		// In last loop, if CPHA is 1, should skip CLK Toggle
		if( (i == 7) && (SW_SPI_CPHA_val == 1) )
			break;
			
		SW_SPI_CLK_Toggle();		// clock toggle
	}
	TIMER_Wait_us( SW_SPI_WAIT_TIME );

	return rx_data;
}


// ********** RF Communication Config **********************************************************************************************************

/*  RECEIVER MODE

	Configures the MCU as an RF receiver
*/
static uint8 CC2520_Receiver_Config(void)
{
    // Initialize BasicRF
    basicRfConfig.myAddr = RX_ADDR;		// RECEIVER ADDRESS
	
    if(basicRfInit(&basicRfConfig) == FAILED) {
		return CONFIG_FAILED;
    }
    basicRfReceiveOn();					// RECEIVER ON
	
    return CONFIG_SUCCESS;
}

/*  TRANSMITTER MODE

	Configures the MCU as an RF transmitter.
*/
static uint8 CC2520_Transmitter_Config( void)
{
    // Initialize BasicRF
    basicRfConfig.myAddr = TX_ADDR;		// TRANSMIT ADDRESS
	
    if(basicRfInit(&basicRfConfig)==FAILED) {
		return CONFIG_FAILED;
    }

    // Keep Receiver off when not needed to save power
    basicRfReceiveOff();				// RECEIVER OFF
	
    return CONFIG_SUCCESS;
}


