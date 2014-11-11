
/* 	This work is licensed under a Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
*
* 	Copyright(c) 2014 Hong Moon 		All Rights Reserved
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


#ifndef CC2520_STELLARIS_PORTING_H
#define CC2520_STELLARIS_PORTING_H

/*************************************************7**********************************
* INCLUDES
*/

#include "hal_types.h"
#include "lm3s9b92.h"
#include <gpio.h>
#include <ssi.h>
#include <hw_memmap.h>
#include "basic_rf.h"
#include "hal_rf.h"
#include "hal_cc2520.h"
#include "bsp_int.h"	// interrupt

#include "os.h"			// uC/OS-III includes
#include "cpu.h"		// uC/OS-III includes
#include <hw_types.h>	// type tBooelan 

/***********************************************************************************
* DEFS
*/
#define SW_SPI_MODE		        1	// USE SOFTWARE SPI MODE !!!

#define CC2520_IS_TRANSMITTER_MODE     0 	// 1 if in Transmitter mode, 0 if in Receiver mode


#ifndef CONFIG_SUCCESS
#define CONFIG_SUCCESS  1
#endif

#ifndef CONFIG_FAILED
#define CONFIG_FAILED   0
#endif


/***********************************************************************************
* MACROS
*/

/* MCU Pin Access */
//#ifndef MCU_PIN_DIR_OUT																		// unnecessary *
//#define MCU_PIN_DIR_OUT(port,bit)       st( P##port##DIR |= BV(bit); )						// unnecessary *
//#endif																					
//#ifndef MCU_PIN_DIR_IN																		// unnecessary *
//#define MCU_PIN_DIR_IN(port,bit)        st( P##port##DIR &= ~BV(bit); )						// unnecessary *
//#endif

// CC2520 I/O Definitions

/* Basic I/O pin setup */
//#define CC2520_BASIC_IO_DIR_INIT()      st( MCU_PIN_DIR_OUT(5,7); MCU_PIN_DIR_OUT(1,0); )		// unnecessary *

/* MCU port control for SPI interface */
//#define CC2520_DISABLE_SPI_FUNC()       st( P5SEL &= ~(BV(1) | BV(2) | BV(3)); )				// unnecessary *
//#define CC2520_ENABLE_SPI_FUNC()        st( P5SEL |= BV(1) | BV(2) | BV(3); )					// unnecessary *


// *********************************************** GPIO **********************************************************************************************

/* GPIO Pin numbers on Stellaris Evalbot */


/* Output GPIO Pins */
#define CC2520_RESET_OPIN_NUMBER			GPIO_PIN_5				// CC2520_RESET_OPIN   	OUTPUT (PC5: GPIO Port C, PIN_5 on Stellaris Evalbot)
#define CC2520_VREG_EN_OPIN_NUMBER			GPIO_PIN_2				// CC2520_VREG_EN_OPIN 	OUTPUT (PH2: GPIO Port H, PIN_2 on Stellaris Evalbot)

/* Input GPIO Pins */
#define CC2520_RX_FRM_DONE_PIN_NUMBER		GPIO_PIN_4				// GPIO_RF_GPIO0_PIN **	INPUT  (PC4: GPIO Port C, PIN_4 on Stellaris Evalbot ) ***
#define CC2520_SAMPLED_CCA_PIN_NUMBER		GPIO_PIN_5				// GPIO_RF_GPIO1_PIN 	INPUT  (PB5: GPIO Port B, PIN_5 on Stellaris Evalbot )
#define CC2520_RSSI_VALID_PIN_NUMBER		GPIO_PIN_4				// GPIO_RF_GPIO2_PIN 	INPUT  (PB4: GPIO Port B, PIN_4 on Stellaris Evalbot )
#define CC2520_TX_FRM_DONE_PIN_NUMBER		GPIO_PIN_4				// GPIO_RF_GPIO2_PIN 	INPUT  (PB4: GPIO Port B, Pin 4 on Stellaris Evalbot ) (same pin reused)

#define CC2520_GPIO_RF_GPIO0_PORT			GPIO_PORTC_BASE			// GPIO_RF_GPIO0_PORT** 	   (PC4: GPIO Port C on Stellaris Evalbot)
#define CC2520_GPIO_RF_GPIO0_PIN			GPIO_PIN_4				// GPIO_RF_GPIO0_PIN ** 	   (PC4: GPIO Pin  4 on Stellaris Evalbot)


/* Interrupts */
#define GPIO_RF_GPIO0_INT					BSP_INT_ID_GPIOC		// GPIO_RF_GPIO0_PIN interrupt


// GPIO pin direction control
/*
#define CC2520_GPIO_DIR_OUT(pin) \
    st( \
        if (pin == 0) CC2520_GPIO0_DIR_OUT(); \													// unnecessary *
            if (pin == 1) CC2520_GPIO1_DIR_OUT(); \												// unnecessary *
                if (pin == 2) CC2520_GPIO2_DIR_OUT(); \											// unnecessary *
                    if (pin == 3) CC2520_GPIO3_DIR_OUT(); \										// unnecessary *
                        if (pin == 4) CC2520_GPIO4_DIR_OUT(); \									// unnecessary *
                            if (pin == 5) CC2520_GPIO5_DIR_OUT(); \								// unnecessary *
                                )
#define CC2520_GPIO0_DIR_OUT()          MCU_PIN_DIR_IN(1,3)										// unnecessary *
#define CC2520_GPIO1_DIR_OUT()          MCU_PIN_DIR_IN(1,5)										// unnecessary *
#define CC2520_GPIO2_DIR_OUT()          MCU_PIN_DIR_IN(1,6)										// unnecessary *
#define CC2520_GPIO3_DIR_OUT()          // not connected to MCU in CC2520_CC2591EM				// unnecessary *
#define CC2520_GPIO4_DIR_OUT()          // not connected to MCU in CC2520_CC2591EM				// unnecessary *
#define CC2520_GPIO5_DIR_OUT()          // not connected to MCU in CC2520_CC2591EM				// unnecessary *
*/

//#define CC2520_GPIO_DIR_IN(pin) \																// unnecessary *
//    st( \
//        if (pin == 0) CC2520_GPIO0_DIR_IN(); \												// unnecessary *
//            if (pin == 1) CC2520_GPIO1_DIR_IN(); \											// unnecessary *
//                if (pin == 2) CC2520_GPIO2_DIR_IN(); \										// unnecessary *
//                    if (pin == 3) CC2520_GPIO3_DIR_IN(); \									// unnecessary *
//                        if (pin == 4) CC2520_GPIO4_DIR_IN(); \								// unnecessary *
//                            if (pin == 5) CC2520_GPIO5_DIR_IN(); \							// unnecessary *
//                                )
//#define CC2520_GPIO0_DIR_IN()           MCU_PIN_DIR_OUT(1,3)									// unnecessary *
//#define CC2520_GPIO1_DIR_IN()           MCU_PIN_DIR_OUT(1,5)									// unnecessary *
//#define CC2520_GPIO2_DIR_IN()           MCU_PIN_DIR_OUT(1,6)									// unnecessary *
//#define CC2520_GPIO3_DIR_IN()           // not connected to MCU in CC2520_CC2591EM			// unnecessary *
//#define CC2520_GPIO4_DIR_IN()           // not connected to MCU in CC2520_CC2591EM			// unnecessary *
//#define CC2520_GPIO5_DIR_IN()           // not connected to MCU in CC2520_CC2591EM			// unnecessary *

/* Outputs: Power and reset control */

void CC2520_RESET_OPIN(uint8 data);																// (PC5, GPIO Port C Pin 5 on Stellaris Evalbot)  ***
void CC2520_VREG_EN_OPIN(uint8 data);															// (PH2, GPIO Port C Pin 2 on Stellaris Evalbot)  ***

/* Outputs: GPIO */

// *** These are actually input pins (see the datasheet for more, including the temp sensor)
//#define CC2520_GPIO0_OPIN(v)			MCU_IO_SET(1,3,v)			// output pin				// unnecessary *
//#define CC2520_GPIO1_OPIN(v)			MCU_IO_SET(1,5,v)										// unnecessary *
//#define CC2520_GPIO2_OPIN(v)			MCU_IO_SET(1,6,v)										// unnecessary *
//#define CC2520_GPIO3_OPIN(v)			MCU_IO_SET(1,1,v)										// unnecessary *
//#define CC2520_GPIO4_OPIN(v)			MCU_IO_SET(1,2,v)										// unnecessary *
//#define CC2520_GPIO5_OPIN(v)			MCU_IO_SET(1,7,v)										// unnecessary *


/* Inputs: GPIO */
#define CC2520_GPIO0_IPIN				GPIOPinRead( GPIO_PORTC_BASE, GPIO_PIN_4 )				// (PC4: GPIO Port C Pin 4 on Stellaris Evalbot) *** (see hal_cc2520.h)
#define CC2520_GPIO1_IPIN				GPIOPinRead( GPIO_PORTB_BASE, GPIO_PIN_5 )				// (PB5: GPIO Port B Pin 5 on Stellaris Evalbot) ***
#define CC2520_GPIO2_IPIN				GPIOPinRead( GPIO_PORTB_BASE, GPIO_PIN_4 )				// (PB4: GPIO Port B Pin 4 on Stellaris Evalbot) ***
#define CC2520_GPIO3_IPIN				0														// not connected to MCU in CC2520_CC2591EM
#define CC2520_GPIO4_IPIN				0														// not connected to MCU in CC2520_CC2591EM
#define CC2520_GPIO5_IPIN				0														// not connected to MCU in CC2520_CC2591EM

// Platform specific definitions
/* IRQ on GPIO0 */
//#define CC2520_GPIO0_IRQ_INIT()         st( P1IES &= ~0x08; CC2520_GPIO0_IRQ_CLEAR(); )		// unnecessary *
//#define CC2520_GPIO0_IRQ_ENABLE()       st( P1IE  |=  0x08; )
//#define CC2520_GPIO0_IRQ_DISABLE()      st( P1IE  &= ~0x08; )
//#define CC2520_GPIO0_IRQ_CLEAR()        st( P1IFG &= ~0x08; )


void CC2520_GPIO0_IntEnable(void);
void CC2520_GPIO0_IntDisable(void);
void CC2520_GPIO0_IntClear(void);
void CC2520_GPIO0_IntHandler(void);

/* IRQ on GPIO1 */																				// unnecessary *
//#define CC2520_GPIO1_IRQ_INIT()         st( P1IES &= ~0x20; CC2520_GPIO1_IRQ_CLEAR(); )		// unnecessary *
//#define CC2520_GPIO1_IRQ_ENABLE()       st( P1IE  |=  0x20; )									// unnecessary *
//#define CC2520_GPIO1_IRQ_DISABLE()      st( P1IE  &= ~0x20; )									// unnecessary *
//#define CC2520_GPIO1_IRQ_CLEAR()        st( P1IFG &= ~0x20; )									// unnecessary *

// ***************************************** SPI **************************************************************************************************

/* SPI Pin numbers (SSI1 module) on Stellaris Evalbot */

#define SSI1_SCLK_PERIPH				(SYSCTL_PERIPH_GPIOH)									// RF_SPI_CLK (PH4, port H pin 4)	
#define SSI1_SCLK_PORT					(GPIO_PORTH_BASE)
#define SSI1_SCLK_PIN					(GPIO_PIN_4)

#define SSI1_CS_PERIPH					(SYSCTL_PERIPH_GPIOH)									// RF_SPI_CS (PH5, port H pin 5)  
#define SSI1_CS_PORT					(GPIO_PORTH_BASE)
#define SSI1_CS_PIN						(GPIO_PIN_5)

#define SSI1_RX_PERIPH					(SYSCTL_PERIPH_GPIOH)									// RF_SPI_MISO (PH6, port H pin 6)
#define SSI1_RX_PORT					(GPIO_PORTH_BASE)
#define SSI1_RX_PIN 					(GPIO_PIN_6)

#define SSI1_TX_PERIPH					(SYSCTL_PERIPH_GPIOH)									// RF_SPI_MOSI (PH7, port H pin 7)
#define SSI1_TX_PORT					(GPIO_PORTH_BASE)
#define SSI1_TX_PIN						(GPIO_PIN_7)

/* Outputs: SPI interface */

//#define CC2520_CSN_OPIN(v)              MCU_IO_SET(5,0,v)												// unnecessary *
//#define CC2520_SCLK_OPIN(v)             MCU_IO_SET(5,3,v)												// unnecessary *
//#define CC2520_MOSI_OPIN(v)             MCU_IO_SET(5,1,v)												// unnecessary *

void CC2520_CSN_OPIN(uint8 data); 		  																// (PH5: GPIO Port H Pin 5 on Stellaris Evalbot)       

//void CC2520_SCLK_OPIN(uint8 data);      // (PH4: GPIO Port H Pin 4 on Stellaris Evalbot)       		// unnecessary *
//void CC2520_MOSI_OPIN(uint8 data);      // (PH7: GPIO Port H Pin 7 on Stellaris Evalbot)       		// unnecessary *

/* Inputs: SPI interface */
//#define CC2520_MISO_IPIN                MCU_IO_GET(5,2)												// unnecessary *
//#define CC2520_MISO_OPIN(v)             MCU_IO_SET(5,2,v) // For use in LPM							// unnecessary *
//#define CC2520_MISO_DIR_IN()            MCU_PIN_DIR_OUT(5,2)											// unnecessary *
//#define CC2520_MISO_DIR_OUT()           MCU_PIN_DIR_IN(5,2)											// unnecessary *

/* SPI register definitions */
//#define CC2520_SPI_TX_REG               (UCB1TXBUF)													// unnecessary *
//#define CC2520_SPI_RX_REG               (UCB1RXBUF)													// unnecessary *

//#define CC2520_SPI_RX_IS_READY()        (UC1IFG & UCB1RXIFG)											// unnecessary *
//#define CC2520_SPI_RX_NOT_READY()       (UC1IFG &= ~UCB1RXIFG)										// unnecessary *


// SPI access macros
//#define CC2520_SPI_BEGIN()              st( CC2520_CSN_OPIN(0); )										// redefine them
//#define CC2520_SPI_TX(x)                st( CC2520_SPI_RX_NOT_READY(); CC2520_SPI_TX_REG = x; )		// redefine them
//#define CC2520_SPI_RX()                 (CC2520_SPI_RX_REG)											// redefine them
//#define CC2520_SPI_WAIT_RXRDY()         st( while (!CC2520_SPI_RX_IS_READY()); )						// redefine them
//#define CC2520_SPI_END()                st( CC2520_CSN_OPIN(1); )										// redefine them

#define CC2520_SPI_BEGIN()                st( CC2520_CSN_OPIN(0); )										// SPI begin			
#define CC2520_SPI_END()                  st( CC2520_CSN_OPIN(1); )										// SPI end				


#ifndef SW_SPI_MODE
void 	CC2520_SPI_TX(uint8 data);																		// SPI transmit
void    CC2520_SPI_WAIT_RXRDY(void);																	// SPI wait
uint8 	CC2520_SPI_RX(void);																			// SPI receive
#endif

// ********** Miscellaneous HAL_INT *********************************************************************************************************************

#define HAL_INT_ON(x)      st( )
#define HAL_INT_OFF(x)     st( )
#define HAL_INT_LOCK(x)    st( )
#define HAL_INT_UNLOCK(x)  st( )


// ********** Software SPI *******************************************************************************************************************************

#define SW_SPI_WAIT_TIME	100			// wait time

#define  SW_SPI_CPHA_val        0
#define  SW_SPI_CPOL_val        0 

void TIMER_Wait_us( uint32 nCount);

void  SW_SPI_MOSI_H(void);	// SPI MOSI High
void  SW_SPI_MOSI_L(void);	// SPI MOSI Low

void  SW_SPI_CLK_H(void);	// SPI CLK  High
void  SW_SPI_CLK_L(void);	// SPI CLK  Low

uint8 SW_SPI_ReadVal_MISO(void);	// SPI MISO Read
void  SW_SPI_MOSI_OUT(uint8 out);	// SPI MOSI Write

void  SW_SPI_CLK_Init(void);		// SPI CLK Init
void  SW_SPI_CLK_Toggle(void);		// SPI CLK Toggle

uint8 SW_SPI_TXRX(uint8 tx_data);	// SPI TXRX


// ********** RF Communication Config ****************************************************************************************************************************

// Application parameters
#define RF_CHANNEL           25      // 2.4 GHz RF channel

// BasicRF address definitions
#define PAN_ID               0x2007
#define TX_ADDR              0x2520
#define RX_ADDR              0xBEEF

// transmit data length
#define APP_PAYLOAD_LENGTH    1

/***********************************************************************************
* BSP INIT FUNCTIONS
*/
void BSP_CC2520Init(void);

void CC2520_GPIO_Init(void);
void CC2520_SPI_Init(void);

#endif
