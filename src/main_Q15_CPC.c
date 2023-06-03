/**********************************************************************
* © 2008 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
*******************************************************************************/
#include "p33FJ16GS502.h"
#include "Functions.h"
#include "dsp.h"
int FBS __attribute__((space(prog), address(0xF80000))) = 0xF ;
//_FBS(
//    BWRP_WRPROTECT_OFF & // Boot Segment Write Protect (Boot Segment may be written)
//    BSS_NO_FLASH         // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
//);
 int FGS __attribute__((space(prog), address(0xF80004))) = 0x7 ;
//_FGS(
//    GWRP_OFF &           // General Code Segment Write Protect (General Segment may be written)
//    GSS_OFF              // General Segment Code Protection (User program memory is not code-protected)
//);
 int FOSCSEL __attribute__((space(prog), address(0xF80006))) = 0x80 ;
//_FOSCSEL(
//    FNOSC_FRC &          // Oscillator Source Selection (Internal Fast RC (FRC) oscillator)
//    IESO_ON              // Internal External Switch Over Mode (Start up device with FRC, then automatically switch to user-selected oscillator source)
//);
 int FOSC __attribute__((space(prog), address(0xF80008))) = 0x63 ;
//_FOSC(
//    POSCMD_NONE &        // Primary Oscillator Source (Primary oscillator disabled)
//    OSCIOFNC_ON &        // OSC2 Pin Function (OSC2 is general purpose digital I/O pin)
//    IOL1WAY_ON &         // Peripheral Pin Select Configuration (Allow only one reconfiguration)
//    FCKSM_CSECMD         // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is Disabled)
//);
 int FWDT __attribute__((space(prog), address(0xF8000A))) = 0x5F ;
//_FWDT(
//    WDTPOST_PS32768 &    // Watchdog Timer Postscaler (1:32,768)
//    WDTPRE_PR128 &       // WDT Prescaler (1:128)
//    WINDIS_OFF &         // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
//    FWDTEN_OFF           // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)
//);
 int FPOR __attribute__((space(prog), address(0xF8000C))) = 0xFF ;
//_FPOR(
//    FPWRT_PWR128         // POR Timer Value (128ms)
//);

int FICD __attribute__((space(prog), address(0xF8000E))) = 0xC1 ;


#define INPUTUNDERVOLTAGE 391					/* Input voltage <7V --> 2.2k/(10k+2.2k)*7V = 1.2623V
												Now calculate the ADC expected value = 1.2623/3.3*1023 = 391 */
#define INPUTOVERVOLTAGE 839						/* Input voltage >15V --> 2.2k/ (10k+2.2k)*15 = 2.70492V
												Now calculate the ADC expected value  = 2.70492/3.3*1023 = 839 */
extern unsigned int  FlybackReferenceNew,FlybackReferenceOld;
//extern char OnOffFuente;			
extern tPID FlybackVoltagePID;
//extern void CompensarRampa(void);

int main(void)
{
	int InputVoltage;
	/* Configure Oscillator to operate the device at 40Mhz
	   Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
 	   Fosc= 7.37*(43)/(2*2)=80Mhz for Fosc, Fcy = 40Mhz */

	/* Configure PLL prescaler, PLL postscaler, PLL divisor */
	PLLFBD=41; 				/* M = PLLFBD + 2 */
	CLKDIVbits.PLLPOST=0;   /* N1 = 2 */
	CLKDIVbits.PLLPRE=0;    /* N2 = 2 */

    __builtin_write_OSCCONH(0x01);			/* New Oscillator selection FRC w/ PLL */
    __builtin_write_OSCCONL(0x01);  		/* Enable Switch */
      
while(OSCCONbits.COSC != 0b001);		/* Wait for Oscillator to switch to FRC w/ PLL */  
while(OSCCONbits.LOCK != 1);			/* Wait for Pll to Lock */

	/* Now setup the ADC and PWM clock for 120MHz
	   ((FRC * 16) / APSTSCLR ) = (7.37MHz * 16) / 1 = 117.9MHz*/
	
	ACLKCONbits.FRCSEL = 1;					/* FRC provides input for Auxiliary PLL (x16) */
	ACLKCONbits.SELACLK = 1;				/* Auxiliary Ocillator provides clock source for PWM & ADC */
	ACLKCONbits.APSTSCLR = 7;				/* Divide Auxiliary clock by 1 */
	ACLKCONbits.ENAPLL = 1;					/* Enable Auxiliary PLL */
	
	while(ACLKCONbits.APLLCK != 1);			/* Wait for Auxiliary PLL to Lock */
   //El valor de la frecuencia se asigna en case ON porque pueden ser 300kHz Buck2 o 100 kHz flyback
    //PTPER = 3155;                         /* PTPER = ((1 / 300kHz) / 1.04ns) = 3155, where 300kHz 
											/* is the desired switching frequency and 1.04ns is PWM resolution. */
    PTPER = 9075;                         /* PTPER = ((1 / 100kHz) / 1.04ns) = 9075, where 100kHz 
											 /*is the desired switching frequency and 1.04ns is PWM resolution. */
/* For the 2nd buck stage Jumpers J12 and J13 must be populated while J14 and J15 are not. */

				
	FlybackDriveCPC();				    	/* PWM Setup for 3.3V Buck2 */
    CurrentandVoltageMeasurements();		/* ADC Setup for bucks and boost */
	FlybackVoltageLoop();						/* Initialize Buck2 PID */
    //InitUART1();
    FlybackRefVoltValInit();

   
    PTCONbits.PTEN = 1;						/* Enable the PWM */ 
    ADCONbits.ADON = 1;						/* Enable the ADC */
    //Buck2ReferenceRoutine();				/* Initiate Buck 2 soft start to 3.3V */ 
  
//OnOffFuente=0; //la fuente esta apagada    
FlybackVoltagePID.controlReference=0x4D20; //valor inicial de la referencia
    while(1){
/*
if (Buck2ReferenceOld!=Buck2ReferenceNew)
{
	Buck2ReferenceOld=Buck2ReferenceNew;
	Buck2ReferenceRoutine(); //actualiza el valor de la referencia
}	
*/
/* Initiate Buck 2 soft start to 3.3V 


	
					if (ADSTATbits.P2RDY ==1)
			
					{
						InputVoltage = ADCBUF4;						// Read input Voltage 
						ADSTATbits.P2RDY = 0;						//Clear the ADC pair ready bit
						
                    }
*/
	/*
					if ((InputVoltage <= INPUTUNDERVOLTAGE) || (InputVoltage >= INPUTOVERVOLTAGE))  if input voltage is less than 
																				    		underVoltage or greater than over 
																							voltage limitshut down all PWM output 

					{
						
						IOCON2bits.OVRENH = 1;							// Over ride the PWM2H to inactive state 
						IOCON2bits.OVRENL = 1;							// Over ride the PWM2L to inactive state 
						
					}
*/
			

}
		
}			
					


