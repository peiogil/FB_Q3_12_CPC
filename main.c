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

_FOSCSEL(FNOSC_FRC)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON)
_FWDT(FWDTEN_OFF)
_FPOR(FPWRT_PWR128)
_FICD(ICS_PGD2 & JTAGEN_OFF)

#define INPUTUNDERVOLTAGE 391					/* Input voltage <7V --> 2.2k/(10k+2.2k)*7V = 1.2623V
												Now calculate the ADC expected value = 1.2623/3.3*1023 = 391 */
#define INPUTOVERVOLTAGE 839						/* Input voltage >15V --> 2.2k/ (10k+2.2k)*15 = 2.70492V
												Now calculate the ADC expected value  = 2.70492/3.3*1023 = 839 */
unsigned int  Buck2ReferenceNew,Buck2ReferenceOld;
//char OnOffFuente=OFF;			

extern tPID Buck2VoltagePID;
//extern void CompensarRampa(void);

int main(void)
{
	//int InputVoltage;
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
   //PTPER = 3155;                         /* PTPER = ((1 / 300kHz) / 1.04ns) = 3155, where 300kHz 
											/* is the desired switching frequency and 1.04ns is PWM resolution. */
    PTPER = 9075;                         /* PTPER = ((1 / 100kHz) / 1.04ns) = 9075, where 100kHz 
											 /*is the desired switching frequency and 1.04ns is PWM resolution. */

/* For the 2nd buck stage Jumpers J12 and J13 must be populated while J14 and J15 are not. */

				
	Buck2DriveCPC();				    	/* PWM Setup for 3.3V Buck2 */
    CurrentandVoltageMeasurements();		/* ADC Setup for bucks and boost */
	Buck2VoltageLoop();						/* Initialize Buck2 PID */
    InitUART1();
    Buck2RefVoltValInit();

   
    PTCONbits.PTEN = 0;						/* Enable the PWM */ 
    ADCONbits.ADON = 1;						/* Enable the ADC */
    Buck2ReferenceRoutine();				/* Initiate Buck 2 soft start to 3.3V */ 
   

    while(1){

if (Buck2ReferenceOld != Buck2ReferenceNew)
{
	Buck2ReferenceOld=Buck2ReferenceNew;
	Buck2ReferenceRoutine(); //actualiza el valor de la referencia
}	
/* Para hacer que la UART1 funcione por polling
if(U1STAbits.OERR == 1)
{
U1STAbits.OERR = 0;
continue;
}

//check for receive errors 
if(U1STAbits.FERR == 1)
{
continue;
}

    if(U1STAbits.URXDA == 1)
ReceivedChar();
*/



/* Initiate Buck 2 soft start to 3.3V */

/*
	
					if (ADSTATbits.P2RDY ==1)
			
					{
						InputVoltage = ADCBUF4;						// Read input Voltage 
						ADSTATbits.P2RDY = 0;						//Clear the ADC pair ready bit 
						
					}
	
	
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
					


