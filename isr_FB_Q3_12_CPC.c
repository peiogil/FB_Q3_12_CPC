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
#include "dsp.h"
#include "Functions.h"
//#include "CNI_cambio.h"

//extern char OnOffFuente;
unsigned int TimerInterruptCount = 0; 
//extern tPID FlybackVoltagePID;
extern unsigned int FlybackReferenceNew;
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt()
{
  TimerInterruptCount ++; 	/* Increment interrupt counter */
  IFS0bits.T1IF = 0; 		/* Clear Interrupt Flag */
}
/* Esta rutina de interrupción deja de usarse porque el pin al que está
 * conectado el interrupor SW1 pasa a usarse como salida del regulador para
 * hacer el bode
 * 
 */
/*
void  __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt()
{
    
int VPotRefence;
while (ADSTATbits.P2RDY ==0);
VPotRefence=ADCBUF5;
//FlybackReferenceNew=VPotRefence<<5; Q0.15
FlybackReferenceNew=VPotRefence<<2; //Q3.12 ajuste al decimal
//FlybackVoltagePID.controlReference=VPotRefence;
 
    //CNI sw1 to start and stop the supply
    //With saturation problems at on and off are solved

    switch (CNI_EST)
    {
        case CNI_EST_ON1:
        {
            CNI_EST=CNI_EST_ON2;
            PTCONbits.PTEN = 1;
        }
                break;
        case CNI_EST_ON2:
        {
             CNI_EST=CNI_EST_OFF1;
            PTCONbits.PTEN = 0;
        }
                break;
        case CNI_EST_OFF1:
        {
            CNI_EST=CNI_EST_OFF2;
            PTCONbits.PTEN = 0;
        }
                break;
        case CNI_EST_OFF2:
        {
            CNI_EST=CNI_EST_ON1;
            PTCONbits.PTEN = 1;
        }
                break;
    }

IFS1bits.CNIF=0; 	//Habilita interrupcon de sw1
}
*/