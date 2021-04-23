/**
*
* @file ece544ip_test.c
*
* @author Roy Kravitz (roy.kravitz@pdx.edu)
* @copyright Portland State University, 2016-2020
*
* This file implements a test program for the Nexys4IO and Digilent Pmod peripherals
* used in ECE 544. The peripherals provides access to the pushbuttons
* and slide switches, the LEDs, the RGB LEDs (only Nexys A7), and the Seven Segment display
* on the Digilent Nexys A7 and Basys 3 boards and the PmodOLEDrgb (94 x 64 RGB graphics display)
* and the PmodENC (rotary encoder + slide switch + pushbutton).
*
* The test is basic but covers all of the API functions:
*	o initialize the Nexys4IO, Pmod drivers and all the other peripherals
*	o Set the LED's to different values
*	o Check that the duty cycles can be set for both RGB LEDs
*	o Write character codes to the digits of the seven segment display banks
*	o Check that all of the switches and pushbuttons on the Nexys4 can be read
*	o Performs a basic test on the rotary encoder and pmodOLEDrgb
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a	rhk	02-Jul-2016		First release of test program.  Builds on the ece544 peripheral test used
*							to check the functionality of Nexys4IO and PMod544IOR2
* 2.00a sy  14-Oct-2016		Modified the code to include different initialize function for other peripherals 
*							connected to the system.
* 3.00	rk	05-Apr-2018		Modified for Digilent PmodENC and PmodOLEDrgb.  Replaced MB_Sleep() w/ usleep.
* 4.00	rk	30-Mar-2020		Modified to support both the Nexys A7 and Basys 3 FPGA platforms
* </pre>
*
* @note
* The minimal hardware configuration for this test is a Microblaze-based system with at least 32KB of memory,
* an instance of Nexys4IO, an instance of the pmodOLEDrgb AXI slave peripheral, and instance of the pmodENC AXI
* slave peripheral, an instance of AXI GPIO, an instance of AXI timer and an instance of the AXI UARTLite 
* (used for xil_printf() console output)
*
* @note
* The driver code and test application(s) for the pmodOLDrgb and pmodENC are
* based on code provided by Digilent, Inc.
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "microblaze_sleep.h"
#include "nexys4IO.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"

/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0


// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_0_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
// Microblaze peripheral instances
uint64_t 	timestamp = 0L;
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
XGpio		GPIOInst0;					// GPIO instance
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance


// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler


volatile uint32_t			gpio_in;			// GPIO input port


/************************** Function Prototypes *****************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void FIT_Handler(void);										// fixed interval timer interrupt handler
int AXI_Timer_initialize(void);

void RunTest1(void);
void RunTest2(void);
void RunTest3(void);
void RunTest4(void);
void RunTest5(void);
void HSVOLEDDISPLAY(void);
uint16_t HSV2RGB (int h, int s, int v);
void clearRectangle (void);

/************************** MAIN PROGRAM ************************************/
int main(void)
{
    init_platform();

	uint32_t sts;
	uint32_t state, laststate;

	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	microblaze_enable_interrupts();
	
	

	xil_printf("ECE 544 Nexys4io test program\n\r");
	xil_printf("By Roy Kravitz. 31-Mar-2020\n\n\r");


	// TEST 1 - Test the LD15..LD0 on the Nexys4
	//RunTest1();
	// TEST 2 - Test RGB1 (LD16) and RGB2 (LD17) on the Nexys4
	//RunTest2();
	// TEST 3 - test the seven segment display banks
	//RunTest3();
	// TEST 4 - test the rotary encoder (PmodENC) and display (PmodOLEDrgb)
	HSVOLEDDISPLAY();

	// TEST 5 - test the switches and pushbuttons
	// We will do this in a busy-wait loop
	// pressing BTN_C (the center button will
	// cause the loop to terminate
	timestamp = 0;

	xil_printf("Starting Test 5...the buttons and switch test\n\r");
	xil_printf("Press the rotary encoder shaft to exit\n\r");

	// blank the display digits and turn off the decimal points
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	// loop the test until the user presses the center button
	laststate = ENC_getState(&pmodENC_inst);
	while (1)
	{
		// Run an iteration of the test
		RunTest5();
		// check for positive edge on the Rotary Encoder shaft.  exit the loop if it's pressed
		state = ENC_getState(&pmodENC_inst);
		if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate))//only check on button posedge
		{
			// show the timestamp on the seven segment display and quit the loop
			NX4IO_SSEG_putU32Dec((uint32_t) timestamp, true);
			break;
		}
		else
		{
			// increment the timestamp and delay 100 msecs
			timestamp += 100;
			usleep(100 * 1000);
		}
		laststate = state;
	}

	// announce that we're done
	xil_printf("\nThat's All Folks!\n\n\r");

	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 2);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(0, 0, 255));  // blue font
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"BYE BYE");

	usleep(5000 * 1000);
	
	// clear the displays and power down the pmodOLEDrbg
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);
	
	// cleanup and exit
    cleanup_platform();
    exit(0);
}

/************************ TEST FUNCTIONS ************************************/

/****************************************************************************/
/**
* Test 1 - Test the LEDS (LD15..LD0)
*
* Checks the functionality of the LEDs API with some constant patterns and
* a sliding patter.  Determine Pass/Fail by observing the LEDs on the Nexys4
*
* @param	*NONE*
*
* @return	*NONE*
*
*****************************************************************************/
void RunTest1(void)
{
	uint16_t ledvalue;

	xil_printf("Starting Test 1...the LED test\n\r");
	// test the LEDS (LD15..LD0) with some constant patterns
	NX4IO_setLEDs(0x00005555);
	usleep(1000 * 1000);
	NX4IO_setLEDs(0x0000AAAA);
	usleep(1000 * 1000);
	NX4IO_setLEDs(0x0000FF00);
	usleep(1000 * 1000);
	NX4IO_setLEDs(0x000000FF);
	usleep(1000 * 1000);

	// shift a 1 through all of the leds
	ledvalue = 0x0001;
	do
	{
		NX4IO_setLEDs(ledvalue);
		usleep(500 * 1000);
		ledvalue = ledvalue << 1;
	} while (ledvalue != 0);
	return;
}


/****************************************************************************/
/**
* Test 2 - Test the RGB LEDS (LD17..LD16)
*
* Checks the functionality of the RGB LEDs API with a fixed duty cycle.
* Determine Pass/Fail by observing the RGB LEDs on the Nexys4.
*
* @param	*NONE*
*
* @return	*NONE*
*
*****************************************************************************/
void RunTest2(void)
{
	// Test the RGB LEDS (LD17..LD16)
	xil_printf("Starting Test 2...the RGB LED test\n\r");

	// For RGB1 turn on only the blue LED (e.g. Red and Green duty cycles
	// are set to 0 but enable all three PWM channels
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
	NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 16);
	usleep(1500 * 1000);

	// For RGB2, only write a non-zero duty cycle to the green channel
	NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
	NX4IO_RGBLED_setDutyCycle(RGB2, 0, 32, 0);
	usleep(1500 * 1000);

	// Next make RGB1 red. This time we'll only enable the red PWM channel
	NX4IO_RGBLED_setChnlEn(RGB1, true, false, false);
	NX4IO_RGBLED_setDutyCycle(RGB1, 64, 64, 64);
	usleep(1500 * 1000);
	
	// Enable all three channels of both RGB LEDs
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
	NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);

	// Next make both RGB1 and RGB2 BRIGHT purple-ish by changing the duty cycle
	NX4IO_RGBLED_setDutyCycle(RGB1, 255, 255, 255);
	NX4IO_RGBLED_setDutyCycle(RGB2, 255, 255, 255);
	usleep(1500 * 1000);

	// Finish by turning the both LEDs off
	// We'll do this by disabling all of the channels without changing
	// the duty cycles
	NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0);
	NX4IO_RGBLED_setDutyCycle(RGB2, 0, 0, 0);

	return;
}

/****************************************************************************/
/**
* Test 3 - Test the seven segment display
*
* Checks the seven segment display by displaying DEADBEEF and lighting all of
* the decimal points. Determine Pass/Fail by observing the seven segment
* display on the Nexys4.
*
* @param	*NONE*
*
* @return	*NONE*
*
*****************************************************************************/
void RunTest3(void)
{
	xil_printf("Starting Test 3...The seven segment display test\n\r");

	NX4IO_SSEG_putU32Hex(0xBEEFDEAD);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, true);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT6, true);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT5, true);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT4, true);
	usleep(2500 * 1000);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT6, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT5, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT4, false);

	NX4IO_SSEG_putU32Hex(0xDEADBEEF);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT3, true);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT2, true);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, true);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT0, true);
	usleep(2500 * 1000);
	return;
}


/****************************************************************************/
/**
* Test 4 - Test the PmodENC and PmodOLEDrgb
*
* The rotary encoder portion of this test is taken from the Digilent PmodENC driver example
*
* Performs some basic tests on the PmodENC and PmodOLEDrgb.  Includes the following tests
* 	1.	check the rotary encoder by displaying the rotary encoder
* 		count in decimal and hex on the LCD display.  Rotate the knob
* 		to change the values up or down.  The pushbuttons can be used
* 		as follows:
* 			o 	press the rotary encoder pushbutton to exit
* 			o 	press BtnUp to clear the count
*
*	6.	display the string "357#&CFsw" on the LCD display.  These values
* 		were chosen to check that the bit order is correct.  The screen will
* 		clear in about 5 seconds.
* 	7.	display "Looks Good" on the display.  The screen will clear
* 		in about 5 seconds.
*
*
* @param	*NONE*
*
* @return	*NONE*
*
*****************************************************************************/
void RunTest4(void)
{
	uint32_t state, laststate; //comparing current and previous state to detect edges on GPIO pins.
	int ticks = 0, lastticks = 1;
	char s[] = " End Test 4 ";

	xil_printf("Starting Test 4...The PmodOLEDrgb and PmodENC Test\n\r");
	xil_printf("Turn PmodENC shaft.  Rotary Encoder count is displayed\n\r");
	xil_printf("Press BTNUP to clear the count");
	xil_printf("Press Rotary encoder shaft to exit\n\r");

	// turn off all of the decimal points
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT6, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT5, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT4, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT3, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT2, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT0, false);

	// Set up the display output
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Enc:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Hex:");

	// get the previous state
	laststate = ENC_getState(&pmodENC_inst);
	while(1) {
		// get the PmodENC state
		state = ENC_getState(&pmodENC_inst);

		// check if the rotary encoder pushbutton is pressed
		// exit the loop if either one is pressed.
		if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate))//only check on button posedge
		{
			break;
		}


		// check BTNU and clear count if it's pressed
		// update the count if it is not
		if (NX4IO_isPressed(BTNU))
		{
			ticks = 0;
		} 
		else
		{
			// BTNU is not pressed so increment count
			ticks += ENC_getRotation(state, laststate);
		}
		
		
		// update the display with the new count if the count has changed
		if (ticks != lastticks) {
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"         ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
			PMDIO_putnum(&pmodOLEDrgb_inst, ticks, 10);

			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"         ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);
			PMDIO_puthex(&pmodOLEDrgb_inst, ticks);

			// display the count on the LEDs and seven segment display while we're at it
			NX4IO_setLEDs(ticks);
			NX4IO_SSEG_putU32Dec(ticks, true);
		}

		laststate = state;
		lastticks = ticks;
		usleep(1000);
	} // rotary button has been pressed - exit the loop
	xil_printf("\nPmodENC test completed\n\r");

	// Write some character to the screen to check the ASCII translation
 	OLEDrgb_Clear(&pmodOLEDrgb_inst);
 	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, '3');
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, '5');
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, '7');
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, '#');
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, '&');
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, 'C');
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, 'F');
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, 's');
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, 'w');
	usleep(2500 * 1000);

	// Write one final string
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst, s);

	return;
}



/****************************************************************************/
/**
* Test 5 - Test the pushbuttons and switches
*
* Copies the slide switch values to LEDs and the pushbuttons to the decimal
* points in SSEGLO.  Also shows shows the value of the switches on SSEGLO.
* Doing this not only tests the putU16HEX() function but also lets the user
* try all of the character codes (they are displayed on DIGIT3. Determine
* Pass/Fail by flipping switches and pressing
* buttons and seeing if the results are reflected in the LEDs and decimal points
*
* @param	*NONE*
*
* @return	*NONE*
*
* @note
* This function does a single iteration. It should be inclosed in a loop
* if you want to repeat the test
*
*****************************************************************************/
void RunTest5()
{
	uint16_t ledvalue;
	uint8_t  btnvalue;
	uint32_t regvalue;
	uint32_t ssegreg;

	// read the switches and write them to the LEDs and SSEGLO
	ledvalue = NX4IO_getSwitches();
	NX4IO_setLEDs(ledvalue);
	NX4IO_SSEG_putU16Hex(SSEGLO, ledvalue);

	// write sw[4:0] as a character code to digit 3 so we can
	// check that all of the characters are displayed correctly
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, (ledvalue & 0x1F));

	// read the buttons and write them to the decimal points on SSEGLO
	// use the raw get and put functions for the seven segment display
	// to test them and to keep the functionality simple.  We want to
	// ignore the center button so mask that out while we're at it.
	btnvalue = NX4IO_getBtns() & 0x01F;
	ssegreg = NX4IO_SSEG_getSSEG_DATA(SSEGLO);

	// shift the button value to bits 27:24 of the SSEG_DATA register
	// these are the bits that light the decimal points.
	regvalue = ssegreg & ~NEXYS4IO_SSEG_DECPTS_MASK;
	regvalue  |=  btnvalue << 24;

	// write the SSEG_DATA register with the new decimal point values
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, regvalue);
	return;
}

/**************************** HELPER FUNCTIONS ******************************/

/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/

int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO0 channel 1 is an 8-bit input port.
	// GPIO0 channel 2 is an 8-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);


	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}
	
	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}
/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	uint32_t		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion, 
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}
	
  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;
	
	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;
	
  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*       
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;
  
  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;
    
    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];
  
  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}


/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
*
* @note
* ECE 544 students - When you implement your software solution for pulse width detection in
* Project 1 this could be a reasonable place to do that processing.
 *****************************************************************************/

void FIT_Handler(void)
{
	// Read the GPIO port to read back the generated PWM signal for RGB led's
	gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);
}


/**************************** TIFFANI'S FUNCTIONS ******************************/

void HSVOLEDDISPLAY (void)
{
	uint32_t state;
	uint32_t laststate; //comparing current and previous state to detect edges on GPIO pins.
	char s[] = " End application. Goodbye! ";
	int hue = 0;
	int lastHue = 0;
	int saturation = 0;
    int lastSaturation = 0;
	int value = 0;
    int lastValue = 0;
    uint16_t RGB = 0;
   // uint16_t lastRGB;

    //RGB = HSV2RGB(hue, saturation, value);

	xil_printf("Entered HSV function.");

	// Set up the display output
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"H:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"S:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"V:");
	OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 48, 0, 95, 63, 0xFFFF, 0xFF, 0xFFFF);

	// get the previous state
	laststate = ENC_getState(&pmodENC_inst);
	while(1) {
		// get the PmodENC state
		state = ENC_getState(&pmodENC_inst);

		// check if the rotary encoder pushbutton is pressed
		// exit the app if pressed.
		if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate))//only check on button posedge
		{
			break;
		}

		hue += ENC_getRotation(state, laststate);
		if(hue == 360)
		{
			hue = 0;
		}
		else if (hue == -1)
		{
			hue = 359;
		}

		//xil_printf("H: %d \n", hue);

		// check BTNR and increase saturation if it has been pressed
		if (NX4IO_isPressed(BTNR))
		{
			saturation += 1;
			if (saturation == 100)
			{
				saturation = 0;
			}
		}

		// check BTNL and decrease saturation if it has been pressed
	    if (NX4IO_isPressed(BTNL))
		{
			saturation -= 1;
			if (saturation == -1)
			{
				saturation = 99;
			}
		}

	    //xil_printf("S: %d \n", saturation);

		// check BTNU and increase value if it has been pressed
		if (NX4IO_isPressed(BTNU))
		{
			value += 1;
			if (value == 100)
			{
				value = 0;
			}
		}

		// check BTND and decrease value if it has been pressed
		if (NX4IO_isPressed(BTND))
		{
			value -= 1;
			if (value == -1)
			{
				value = 99;
			}
		}

        RGB = HSV2RGB(hue, saturation, value);

		//xil_printf("V: %d \n", value);

		// update the display with the new count if the count has changed
		if (hue != lastHue)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);
			PMDIO_putnum(&pmodOLEDrgb_inst, hue, 10);
			clearRectangle();
			OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 48, 0, 95, 63, RGB, 0xFF, RGB);

		}
		else if (saturation != lastSaturation)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
			PMDIO_putnum(&pmodOLEDrgb_inst, saturation, 10);
			clearRectangle();
			OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 48, 0, 95, 63, RGB, 0xFF, RGB);
		}
		else if (value != lastValue)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);
			PMDIO_putnum(&pmodOLEDrgb_inst, value, 10);
			clearRectangle();
			OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 48, 0, 95, 63, RGB, 0xFF, RGB);
		}

		laststate = state;
		lastHue = hue;
		lastSaturation = saturation;
		lastValue = value;
		usleep(1000);
	}
    // rotary button has been pressed - exit the loop
	xil_printf("\nLeaving HSV function\n\r");

	// Write one final string
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst, s);

	return;
}

uint16_t HSV2RGB (int h, int s, int v)
{
	int i;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	float RGB_min, RGB_max;
	RGB_max = v * 2.55f;
	RGB_min = RGB_max * (100 - s) / 100.0f;

	i = h / 60;
	int difs = h % 60;

	float RGB_adj = (RGB_max - RGB_min) * difs / 60.0f;

	switch (i) {
	case 0:
		r = RGB_max;
		g = RGB_min + RGB_adj;
		b = RGB_min;
		break;
	case 1:
		r = RGB_max - RGB_adj;
		g = RGB_max;
		b = RGB_min;
		break;
	case 2:
		r = RGB_min;
		g = RGB_max;
		b = RGB_min + RGB_adj;
		break;
	case 3:
		r = RGB_min;
		g = RGB_max - RGB_adj;
		b = RGB_max;
		break;
	case 4:
		r = RGB_min + RGB_adj;
		g = RGB_min;
		b = RGB_max;
		break;
	default:
		r = RGB_max;
		g = RGB_min;
		b = RGB_max - RGB_adj;
		break;
	}
	return OLEDrgb_BuildRGB(r, g, b);
}

void clearRectangle (void)
{
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 0);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 2);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 6);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");

	return;
}
