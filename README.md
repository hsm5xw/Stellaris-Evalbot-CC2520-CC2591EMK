Stellaris-Evalbot-CC2520-CC2591EMK
==================================

a multi-threaded mobile RTOS robot with TI CC2520-CC2591EMK (IEEE 802.15.4) RF transceiver chips

-----------------------------------------------------------------------------------------------------------------

![alt tag](https://raw.github.com/hsm5xw/Stellaris-Evalbot-CC2520-CC2591EMK/master/pic.PNG)

This is a multi-threaded embedded application to achieve a wireless RF control of a Stellaris Evalbot MCU 
from another Evalbot by using 2.4 GHz IEEE 802.15.4 communication.

Each Transmitter and Receiver Evalbot has been connected to Texas Instruments CC2520-CC2591EMK RF transceiver module via SPI communication.
The Receiver can be controlled from the Transmitter by pressing push buttons and bump sensors on the Transmitter.
See "ROBOT_BEHAVIOR.txt" file for more information.
				
To configure an Evalbot as a Transmitter, the code should be conditionally compiled by setting the "CC2520_IS_TRANSMITTER_MODE"
constant from "cc2520_stellaris_porting.h" file, which was also written by me, as 1. To configure the Evalbot as a Receiver, 
CC2520_IS_TRANSMITTER_MODE should be set to 0.
	
Each Evalbot runs with Micrium's uC/OS-III Real-time OS.
				
Note that this is NOT the full source code for this project.

Since the last time I released my Evalbot project, I've received quite a few requests to share the last project's source and also 
implement a wireless version to control the Evalbot. I try to be as open as possible, but some of the other code that comes with 
the kernel and the CC2520-CC2591EMK chip cannot be distributed due to the licensing regulations. 

----------------------------------------------------------------------------------------------------------------------------------

References

[1] J. Labrosse and The Stellaris Team, uC/OS-III. Weston, FL: Micrium Press, 2010, pp. 765-786.

[2] Micrium, Micrium-Book_LM3S9B92_OSIII.exe. Weston, FL: Micrium, 2012.

[3] Texas Instruments, CC2520 Software Examples (Rev. B). Dallas, TX: Texas Instruments, 2009.
