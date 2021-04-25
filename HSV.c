/**
*
* @file ece544ip_test.c
*
* @author Tiffani Shilts (tshilts2@pdx.edu)
* @copyright Portland State University, 2021
*
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
float    rHigh = 0;
float    rLow  = 0;
float    gHigh = 0;
float    gLow  = 0;
float    bHigh = 0;
float    bLow  = 0;
int    interruptCount = 0;
_Bool   decPtToggle = false;


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

void HSVOLEDDISPLAY(void);
uint16_t HSV2RGB (int h, int s, int v);
void clearRectangle (void);
void writeDisplay(u32 word);
void bin2hex(u32 bin, u8 *hex);
/************************** MAIN PROGRAM ************************************/
int main(void)
{
    init_platform();

	uint32_t sts;
	//uint32_t state, laststate;

	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	microblaze_enable_interrupts();
	
	HSVOLEDDISPLAY();

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
	NX4IO_RGBLED_setChnlEn(RGB1, false, false, false);
	
	// cleanup and exit
    cleanup_platform();
    exit(0);
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
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 2498);
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
	if ((decPtToggle == false) && (interruptCount == 5000))
	{
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, true);
	decPtToggle = true;
	}
	else if ((decPtToggle == true) &&  (interruptCount == 5000))
	{
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, false);
	decPtToggle = false;
	}

	// Read the GPIO port to read back the generated PWM signal for RGB led's
	gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);

	if (interruptCount == 5000)
	{
	    rHigh = 0;
		rLow  = 0;
		gHigh = 0;
		gLow  = 0;
		bHigh = 0;
		bLow  = 0;
		interruptCount = 0;
	}

	switch (gpio_in) {
	case 0:
		rLow += 1;
		bLow += 1;
		gLow += 1;
		interruptCount++;
		break;
	case 1:
		rLow  += 1;
		bLow  += 1;
		gHigh += 1;
		interruptCount++;
		break;
	case 2:
		rLow  += 1;
		bHigh += 1;
		gLow  += 1;
		interruptCount++;
		break;
	case 3:
		rLow  += 1;
		bHigh += 1;
		gHigh += 1;
		interruptCount++;
		break;
	case 4:
		rHigh += 1;
		bLow  += 1;
		gLow  += 1;
		interruptCount++;
		break;
	case 5:
		rHigh += 1;
		bLow  += 1;
		gHigh += 1;
		interruptCount++;
		break;
	case 6:
		rHigh += 1;
		bHigh += 1;
		gLow  += 1;
		interruptCount++;
		break;
	case 7:
		rHigh += 1;
		bHigh += 1;
		gHigh += 1;
		interruptCount++;
		break;
	default:
	    break;
	}
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
    u8 r = 0;   // pwm values generated from HSV
    u8 g = 0;
    u8 b = 0;
    float redSWPWM = 0;   // raw pwm values generated from software PWD
    float blueSWPWM = 0;
    float greenSWPWM = 0;
    u32 rPWM = 0;   // software values cast to 32 bits for display on the 7 seg display
    u32 bPWM = 0;
    u32 gPWM = 0;
    int test1;
    int test2;


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

	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
	NX4IO_RGBLED_setDutyCycle(RGB1, r, g, b);
	//usleep(1500 * 1000);

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

		// check BTNR and increase saturation if it has been pressed
		if (NX4IO_isPressed(BTNR))
		{
			saturation += 1;
			if (saturation == 101)
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
				saturation = 100;
			}
		}

		// check BTNU and increase value if it has been pressed
		if (NX4IO_isPressed(BTNU))
		{
			value += 1;
			if (value == 101)
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
				value = 100;
			}
		}

        RGB = HSV2RGB(hue, saturation, value);
        r = OLEDrgb_ExtractRFromRGB(RGB);
        g = OLEDrgb_ExtractGFromRGB(RGB);
        b = OLEDrgb_ExtractBFromRGB(RGB);

        if ((interruptCount > 500) && (interruptCount % 10 == 0))
        {
          redSWPWM = (rHigh/(rLow + rHigh)) * 256;
          blueSWPWM = (bHigh/(bLow + bHigh)) * 256;
          greenSWPWM = (gHigh/(gLow + gHigh)) * 256;
          rPWM = (u32)redSWPWM << 1; // multiply by 2 for correction
          bPWM = (u32)blueSWPWM << 1;
          gPWM = (u32)greenSWPWM << 1;
          //test1 = (int)redSWPWM * 2;
          //test2 = (int)rPWM;

          rPWM = rPWM << 24;  // shift red to digits 7 & 6
          gPWM = gPWM << 12;  // shift green to digits 4 & 3

          rPWM = (rPWM | gPWM) | bPWM;   // or with blue which is in place for digits 1 & 0
          rPWM = rPWM | 0x00F00F00;      // Put a blank character in digits 2 & 5
          writeDisplay(rPWM);   // display on seven seg display
        }

        xil_printf("\nActual PWM: %d %d %d\n", r, g, b);

        //xil_printf("Software PWM: %d %d\n", test1, test2);

		// update the display with the new count if the count has changed
		if (hue != lastHue)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);
			PMDIO_putnum(&pmodOLEDrgb_inst, hue, 10);
			clearRectangle();
			OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 48, 0, 95, 63, RGB, 0xFF, RGB);
			NX4IO_RGBLED_setDutyCycle(RGB1, r, g, b);
		}
		else if (saturation != lastSaturation)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
			PMDIO_putnum(&pmodOLEDrgb_inst, saturation, 10);
			clearRectangle();
			OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 48, 0, 95, 63, RGB, 0xFF, RGB);
			NX4IO_RGBLED_setDutyCycle(RGB1, r, g, b);
		}
		else if (value != lastValue)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);
			PMDIO_putnum(&pmodOLEDrgb_inst, value, 10);
			clearRectangle();
			OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 48, 0, 95, 63, RGB, 0xFF, RGB);
			NX4IO_RGBLED_setDutyCycle(RGB1, r, g, b);
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

void writeDisplay(u32 word)
{
	u8 cc[8];		// character codes for each of the nibbles in data
	u8 dplo, dphi;	// current decimal points

	// get the existing current decimal points.  We don't want to change them
	dplo = (u8) (NX4IO_SSEG_getSSEG_DATA(SSEGLO)  >> 24);
	dphi = (u8) (NX4IO_SSEG_getSSEG_DATA(SSEGHI)  >> 24);

	// convert data to hex and display it on all 8 digits of the display
	bin2hex(word, cc);
	NX410_SSEG_setAllDigits(SSEGHI, cc[7], cc[6], CC_BLANK, cc[4], dphi);
	NX410_SSEG_setAllDigits(SSEGLO, cc[3], CC_BLANK, cc[1], cc[0], dplo);

	return;
}
