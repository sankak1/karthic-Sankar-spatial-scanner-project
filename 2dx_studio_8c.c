/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/// karthic vasan sankar, 400390308, sankak1
//bus speed is 30mhz measurement led PN0 and additional status D0
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"                                           // Including all of the necessary files
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
uint16_t motion_movements = 0, status = 0, tot_measurement, dev = 0x29, currently_running = 0, data_true = 1;  // Variables such as total distance, address of sensor, # of movements, data
void I2C_Init(){    																							 // Function for I2C communication
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           								 // Activates the I2C0				
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          							 // Activates Port B on the MCU
  GPIO_PORTB_AFSEL_R |= 0b1100;           											   // Enables the alt funct on PB2 and PB3 pins
	GPIO_PORTB_DEN_R |= 0b1100;             											   // Enables the digital I/O for the PB2 and PB3 pins
  GPIO_PORTB_ODR_R |= 0b1000;             											   // Enables the open drain on the PB3 pin only
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R& 0xFFFF00FF)+ 0x00002200; 
  I2C0_MCR_R = I2C_MCR_MFE;                      									 // Enabling the master functionSS
  I2C0_MTPR_R = 0b0000000000000101000000000111011;                 // Configuring it for a 100kbps clock (added 8 clocks of glitch suppression ~50ns)        
}
void PortH_All_Init(){                                             // Port H used as output pins for the stepper motor
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                         // Activating clock for Port H
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};              // Giving time for the clock to stabilize fully
	GPIO_PORTH_DEN_R |= 0xFF;                                        // Enabling the digital I/O on PH0-PH3
  GPIO_PORTH_DIR_R |= 0b1111;                                      // Assigning PH0-PH3 to be outputs for the stepper motor   
}
void PortF_Init() {  //Turns on D3, D4 corresponding to my status LED for student number 
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                         // Activates the clockvfor Port N
	GPIO_PORTN_DEN_R = 0b00000011;													         // Enabling PN0 (pin 0) and PN1 (pin 1)
  GPIO_PORTN_DIR_R = 0b00000011; 	// Make PN0 and PN1 output, corresponding to my student number
}
void PortN_Init(){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
		GPIO_PORTN_DIR_R=0b00000011;
		GPIO_PORTN_DEN_R=0b00000011;
		return;
}
void PortM_Init(){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                        // Activating clock for Port M
	GPIO_PORTM_DEN_R = 0b1; 																	       // Enabling the digital I/O on PM0
	GPIO_PORTM_DIR_R = 0b0;       								    				       // Making PM0 input by setting direction register to 0
	GPIO_PORTM_PUR_R |= 0b01;													               //	Enabling a pull-up resistor on PM0 to assist wirh active-low functionality
}
void PortL_Init(){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;                        // Activating clock for Port L
	GPIO_PORTL_DEN_R = 0b1; 																	       // Enabling the digital I/O on PL0
	GPIO_PORTL_DIR_R = 0b0;       								    				       // Making PL0 input by setting direction register to 0
	GPIO_PORTL_PUR_R |= 0b01;													               //	Enabling a pull-up resistor on PM0 to assist wirh active-low functionality
}
void PortE_Init() {
	// Activate clock
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4)==0){};
	GPIO_PORTE_DIR_R = 0b00001111;
	GPIO_PORTE_DEN_R = 0b00001111;
	return;
	// We need Port E for the busSpeed function
}
void busSpeed(){
	while(1){
		// set PE0 as high
		GPIO_PORTE_DATA_R = 0b00000001;
		// 1 ms delay
		SysTick_Wait10ms(1);
		// set PE0 as low
		GPIO_PORTE_DATA_R = 0b00000000;
		// 10 ms delay
		SysTick_Wait10ms(1);
	}
}
void EnableInt()                                                   // Enable interrupts
{    __asm("    cpsie   i\n");}
void DisableInt()                                                  // Disable interrupts
{    __asm("    cpsid   i\n");}
void WaitForInt()    																							 // Low power wait
{    __asm("    wfi\n");}
void PortM_Interrupt_Init(){                                       // Initializing interrupt parameters for GPIO Port M, IRQ is 72
		GPIO_PORTM_IS_R = 0;    																			 // Port M is edge-sensitive 
		GPIO_PORTM_IBE_R = 0;   																			 // Port M is not both of the clock edges 
		GPIO_PORTM_IEV_R = 0;    																		 	 // Falling-edge event is required
		GPIO_PORTM_IM_R = 0b01;    																		 // Arming the interrupt by setting necessary bit in IM register
		NVIC_EN2_R = 0x00000100;                                       // Enabling the interrupt for IRQ 72 in NVIC
		NVIC_PRI18_R = 0b10100000; 																		 // Setting the interrupt priority
		EnableInt();           																				 // Enabling the global interrupts
}
void GPIOM_IRQHandler(){ 																					 // IRQ Handler (Interrupt Service Routine for Port M 
	currently_running ^= 1;
	GPIO_PORTM_ICR_R = 0b01;     																		 // Acknowledgin the interrupt by setting bit in the ICR register
}
void PortL_Interrupt_Init(){                                       // Initializing interrupt parameters for GPIO Port L, IRQ is 53
		GPIO_PORTL_IS_R = 0;    																			 // Port L is edge-sensitive 
		GPIO_PORTL_IBE_R = 0;   																			 // Port L is not both of the clock edges 
		GPIO_PORTL_IEV_R = 0;    																		 	 // Falling-edge event is required
		GPIO_PORTL_IM_R = 0b01;    																		 // Arming the interrupt by setting necessary bit in IM register
		NVIC_EN1_R = 0x200000;                                         // Enabling the interrupt for IRQ 53 in NVIC
		NVIC_PRI13_R = 0xC000; 																		     // Setting the interrupt priority
		EnableInt();           																				 // Enabling the global interrupts
}
void GPIOL_IRQHandler(){ 																					 // IRQ Handler (Interrupt Service Routine for Port L 
	SysTick_Wait(1045000);
	data_true ^= 1;
	GPIO_PORTL_ICR_R = 0b01;     																		 // Acknowledging the interrupt by setting bit in the ICR register
	if (data_true == 1) { GPIO_PORTN_DATA_R = 0b00000010;}                                  // Initialling turnning D1 on
	else { GPIO_PORTN_DATA_R = 0b00000000; }                         // Initialling turnning D3 on
}
int main() {
	PLL_Init();	SysTick_Init(); PortH_All_Init(); I2C_Init(); UART_Init(); PortL_Init(); PortL_Interrupt_Init();    // Initializing the functions
	PortF_Init();					                                           	
	PortM_Init();		// Initializing this port so push-button can be connected
	PortM_Interrupt_Init();                                          // Initializing the function for the interrupt parameters
	PortN_Init();
	PortE_Init();
	//busSpeed();
	while(1) {
		if (currently_running) {
			status = VL53L1X_ClearInterrupt(dev);                        // Clearing the interrupt so the next one can be called later
			status = VL53L1X_SensorInit(dev);                            // Initializes the ToF sensor with the primary settings
			status = VL53L1X_StartRanging(dev);                          // Called to enable the ranging of the ToF sensor
			while(motion_movements++ < 32) {                             // The stepper motor has 32 individual motion movements
				status = VL53L1X_GetDistance(dev, &tot_measurement);       // Reads ToF sensor data and gets the measured distance value
				status = VL53L1X_ClearInterrupt(dev);                      // Clearing the interrupt so the next one can be called later
				if (data_true == 1) {sprintf(printf_buffer,"%u\r\n", tot_measurement); } // Putting the measurement in a buffer with the correct formatting                     
				else {sprintf(printf_buffer, "%u\r\n", data_true);}				 // Case for when the 2nd button is toggled and data_true is LO
				UART_printf(printf_buffer);                                // Transmitting the results through UART
				for(int step_val = 0; step_val < 16; step_val++){          // Rotating the stepper motor 11.25deg with a delay in between the states
					GPIO_PORTH_DATA_R = 0b00001100;
					SysTick_Wait(145000);
					GPIO_PORTH_DATA_R = 0b00000110;
					SysTick_Wait(145000);
					GPIO_PORTH_DATA_R = 0b00000011;
					SysTick_Wait(145000);
					GPIO_PORTH_DATA_R = 0b00001001;
					SysTick_Wait(145000);
				}
				GPIO_PORTN_DATA_R ^= 0b00001;     				                 // Toggles D1 (Pn0, pin 0) from on to off
				SysTick_Wait(9500000);                                     // Delay in between
				GPIO_PORTN_DATA_R ^= 0b00001;      					               // Toggles D1 (Pn0, pin 0) from off to 
				if (!currently_running) {break;}                           // If button is pressed in between, then break out of the loop
			}
			if (currently_running) {                                     // Case for when a full revolution is done
				currently_running = 0;                                     // Sets to 0 so it wont start rotating again
				motion_movements = 0;                                      // Setting counter to 0 as motor will go to home position
				SysTick_Wait(2250000);                                     // Delay 
				for(int total_step_val = 0; total_step_val < 512; total_step_val++){  // Reversing back to home position, all the way, 360deg
					GPIO_PORTH_DATA_R = 0b00001001;
					SysTick_Wait(98000);
					GPIO_PORTH_DATA_R = 0b00000011;
					SysTick_Wait(98000);
					GPIO_PORTH_DATA_R = 0b00000110;
					SysTick_Wait(98000);
					GPIO_PORTH_DATA_R = 0b00001100;
					SysTick_Wait(98000);
				}
			}
		}
	}
	VL53L1X_StopRanging(dev);                                       // Method gets called to tell the ToF sensor to stop the ranging
}