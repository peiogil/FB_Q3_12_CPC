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
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~  PID Variable Definitions  ~~~~~~~~~~~~~~~~~~~~~~~ */

/* Variable Declaration required for each PID controller in the application. */
//void PIDInitFlyback(fractional *);
tPID FlybackVoltagePID;
//void Delay_ms(unsigned int);




/* These data structures contain a pointer to derived coefficients in X-space and 
   pointer to controler state (history) samples in Y-space. So declare variables 
   for the derived coefficients and the controller history samples */

/*1) C attributes, designated by the __attribute__ keyword, provide a *
 * means to specify various characteristics of a variable or *
 * function, such as where a particular variable should be placed *
 * in memory, whether the variable should be aligned to a certain *
 * address boundary, whether a function is an Interrupt Service *
 * Routine (ISR), etc. If no special characteristics need to be *
 * specified for a variable or function, then attributes are not *
 * required. For more information about attributes, refer to the *
 * C30 User's Guide. *
 * *
 * 2) The __section__(".xbss") and __section__(".ybss") attributes are *
 * used to place a variable in X data space and Y data space, *
 * respectively. Variables accessed by dual-source DSP instructions *
 * must be defined using these attributes.
 * 3) The aligned(k) attribute, used in variable definitions, is used *
 * to align a variable to the nearest higher 'k'-byte address *
 * boundary. 'k' must be substituted with a suitable constant *
 * number when the ModBuf_X(k) or ModBuf_Y(k) macro is invoked. *
 * In most cases, variables are aligned either to avoid potential *
 * misaligned memory accesses, or to configure a modulo buffer. *
 * *
 * 4) The __interrupt__ attribute is used to qualify a function as an *
 * interrupt service routine. An interrupt routine can be further *
 * configured to save certain variables on the stack, using the *
 * __save__(var-list) directive. *
 * *
 * 5) The __shadow__ attribute is used to set up any function to *
 * perform a fast context save using shadow registers. *
 * *
 * 6) Note the use of double-underscores (__) at the start and end of *
 * all the keywords mentioned above. *
 * *
 **********************************************************************/



/*reserva de memoria en zona x, donde se ubicaran los coeficientes de la fdt*/
fractional FlybackVoltageABC[5] __attribute__((section(".xbss, bss, xmemory")));
/*reserva de memoria en zona y, donde se ubicaran error y d actual y anteriores*/
fractional FlybackVoltageHistory[5] __attribute__((section(".ybss, bss, ymemory")));
/*el tamaño depende del tipo de regulador a implementar, en este caso tipo3*/



/*coeficientes del denominador de la FDT,
 en formato Q3.12 expresada en decimal 
 (parte entera+fracción) OJO: CON SIGNO CAMBIADO!!!*/

#define PID_BUCK2_A2 -3923  //-A2=-0.8775 (decimal)-> binario Q3.12 1111000111110110-> 2'C -3594
#define PID_BUCK2_A1 7688   //-A1=1.877 (decimal)-> binario Q3.12 0001111000001000-> 2'C 7688
//coeficientes del numerador de la FDT, parte entera+decimal enn Q3.12
#define PID_BUCK2_B0 4776   //B0=1.166 (decimal)-> binario Q3.12 0001001010101000-> 2'C 4776
#define PID_BUCK2_B1 142     //B1=0.03476 (decimal)-> binario Q3.12 0000000010001110-> 2'C 142
#define PID_BUCK2_B2 -4637  //B2=- 1.132 (decimal)-> binario Q3.12 1110110111100011-> 2'C -4637

/*
// Regulador Frecuencia de cruce fc=1000 Hz
 
 1.166 z^2 + 0.03476 z - 1.132
 ----------------------------(Ts=100kHz)
      z^2 - 1.877 z + 0.8775
 */

#define ACMP1 0b100111                                  /*ACMP1 asociado a pin 2 current sense*/
#define PID_FLYBACK_VOLTAGE_REFERENCE 0x9A4				/* Reference voltage is from resistor divider circuit R29 & R30
													    	Voltage FB2 = (5kOhm / (5kOhm + 3.3kOhm)) * 3.3V = 1.988V
														    Now calculate expected ADC value (1.988V * 1024)/3.3V = 617 
															Then left shift by 2 for Q3_12 format (617 * 4) = 2468 = 0x9A4 */

#define PID_BUCK2_VOLTAGE_REF_MIN 	  0x90			/* Minimum reference voltage is total dead time (72) left shifted by 2 bits*/

/* This is increment rate to give us desired PID_BUCK1VOLTAGE_REFERENCE. The sofstart takes 50ms */
#define FLYBACK_SOFTSTART_INCREMENT	 (PID_FLYBACK_VOLTAGE_REFERENCE  / 50 )

/*referencia1=3.3v, del sensor (divisor de tension)*/
/*5/(5+3.3)*3.3=1.988v--->(1.988*1024)/3.3v=617*/
/*ADCres=10, Q15res=15--->617*32=19744 = 0x4D20*/


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern unsigned int TimerInterruptCount;

extern unsigned int FlybackReferenceNew, FlybackReferenceOld;

void FlybackDriveCPC(void) {
    //comparator configuration para control pico de corriente
    CMPCON1 = 0x8101;
    //CMPON ? CMPSIDL ? ? ? ? DACOE INSEL1 INSEL0 EXTREF ? CMPSTAT ? CMPPOL RANGE
    //  1   0    0    0 0 0 0   1     0      0      0     0   0     0   0     1 
    //INSEL1 INSEL0
    // 0        0  LA ENTRADA ISENSE ES EL PIN 2=> CMP1A=comparador 1 A (pin2)
    //En U-0 idle  hyst flter  X   X  CMP1
    // PTCON: PWM Time Base Control Register PWM no habilitado
    PTCONbits.PTEN = 0;
    // PTCON2: PWM Clock Divider Select Register
    PTCON2 = 0; //Divide by 1, maximum PWM timing resolution
    //Esta instrucción "INTCON1bits.NSTDIS=1" es la que permite
    //que la comunicación serie, y que con ella la USB
    //no se bloquee.
    //Con anidamiento  nstdis=1 la Change Int no funciona
    // su gemela con control en modo tensión Q3.12 yla UART no la necesita 
    //INTCON1bits.NSTDIS=0; // aunque es por defecto;
    INTCON1bits.NSTDIS = 1; //deshabilitado anidamiento de interrupciones

    /* FlyBack converter setup to output 3.3V */

    IOCON2bits.PENH = 1; /* PWM2H is controlled by PWM module */
    IOCON2bits.PENL = 1; /* PWM2L is controlled by PWM module */
    IOCON2bits.PMOD = 0; /* Complementary Output mode*/
    IOCON2bits.POLH = 1; /* Drive signals are active-low */
    IOCON2bits.POLL = 0; /* Drive signals are active-high */
    IOCON2bits.OVRENH = 0; /* Disable Override feature for shutdown PWM */
    IOCON2bits.OVRENL = 0; /* Disable Override feature for shutdown PWM */
    //IOCON2bits.OVRDAT = 0b00;	/* Shut down PWM with Over ride 0 on PWMH and PWML */	
    IOCON2bits.FLTDAT = 0b01; 
    /* As IFLTMOD=0 (modo normal) when fault is active PWM2H = high 
    /* because FLTDAT<1>=0 means PWM2H no-active when fault
    /* so as POLH=1 PWM2H=High */
    /*and FLTDAT<0>=1 means PWM2L active when fault, and 
    /* in this case as POLL=0 PWM2L implies PWM2L=high*/
    /*PWML alta en falta para resetear el Cap de la rampa*/
    /*PWMH actve low porque la salida del driver es NOT*/
    /*El driver TC4428A invierte PWM2H y no invierte PWM2L*/
    DTR2 = 0x60; /* DTR = (100ns / 1.04ns), where desired dead time is 25ns. 
									     Mask upper two bits since DTR<13:0> */
    ALTDTR2 = 0x60; /* ALTDTR = (50ns / 1.04ns), where desired dead time is 50ns. 
									     Mask upper two bits since ALTDTR<13:0> */
    PWMCON2bits.FLTIEN = 1; /*Fault interrupt is enabled*/
    PWMCON2bits.ITB = 0; /* PTPER based period Select Primary Timebase mode */
    PWMCON2bits.MDCS = 0; /*duty cycle lo pone cada PDC*/
    PWMCON2bits.DTC = 0;
    PWMCON2bits.IUE = 1; /* Enable Immediate duty cycle updates */
//RPOR16bits.RP33R = 0b100111; 	/* Analog Comparator Output 1 is Assigned to RP33 Output Pin*/
    RPOR16bits.RP33R = ACMP1;
    RPINR29bits.FLT1R = 33; //Assign PWM Fault Input 1 (FLT1) to the RP33 Pin 
    
    LEBCON2bits.PHR = 1; /* Rising edge of PWMxH will trigger LEB counter */
    LEBCON2bits.PHF = 1; /* Falling edge of PWMxH is ignored by LEB counter */
    LEBCON2bits.FLTLEBEN = 1; /* Enable fault LEB for selected source */
    LEBCON2bits.CLLEBEN = 0; /* Disable current-limit LEB for selected source */
    LEBCON2bits.LEB = 30; /* Blanking period of 8.32*30=249,6ns */

    FCLCON2bits.IFLTMOD = 0; /*Normal Fault mode*/
    FCLCON2bits.CLMOD = 0; /* Current-limit function is disabled */
    FCLCON2bits.FLTPOL = 0; /* fault source is active high*/
    FCLCON2bits.FLTSRC = 0; /*Fault 1 Selected for PWM Generator 2 Control Signal Source*/
    FCLCON2bits.FLTMOD = 1; /* Fault cycle by cycle */
    TRGCON2bits.TRGDIV = 0; /* Trigger interrupt generated every PWM cycle*/
    TRGCON2bits.TRGSTRT = 0; /* Trigger generated inmediatly*/
    PDC2 = 0x2100; /*0x6C0 Dmax=0,5 0x2100=8848/9075=>Dmax=0,97                                   
   //PDC2 = ( PWM_PERIOD*0.9 );      /* Initial pulse-width = minimum deadtime required (DTR2 + ALDTR2)*/
    TRIG2 = 100; /* Trigger generated almost at beginning of PWM active period */
    /*Inicializa ChangeONInterrupt para on/off con sw1 de la DB*/
    //CNI_EST=CNI_EST_OFF1;
    /*Inicializa el Timer1 para los delays*/
    PR1 = 0x9C40; //(1ms / 25ns) = 40,000 = 0x9C40 
    IPC0bits.T1IP = 4; //Set Interrupt Priority lower then ADC
    IEC0bits.T1IE = 1; //Enable Timer1 interrupts 
    /*Fin de inicializacion del Timer1*/
    // TRISBbits.TRISB13=0; 	/*salida digital PIN 23 PWM1L*/ 
    //AHORA PIN2H&PWM2L PERO TRIS NO HACE FALTA, vale con IOCON2bits.PENH&PENL 
    //TRISBbits.TRISB14=0 ;  /*salida digital PIN 24 PWM1H*/  //AHORA PIN24 PWM2H                                
}

void CurrentandVoltageMeasurements(void) {
    ADCONbits.FORM = 0; /* Integer data format */
    ADCONbits.EIE = 0; /* Early Interrupt disabled */
    ADCONbits.ORDER = 1; /* Convert odd channel first */
    ADCONbits.SEQSAMP = 0; /* Select simultaneous sampling */
    ADCONbits.ADCS = 2; /* ADC clock = FADC/6 = 120MHz / 6 = 20MHz, 
											12*Tad = 1.6 MSPS, two SARs = 3.2 MSPS */

    IEC6bits.ADCP0IE = 1; /* Enable the ADC Pair 0 interrupt*/
    IFS6bits.ADCP0IF = 0; /* Clear ADC interrupt flag */
    IPC27bits.ADCP0IP = 7; /* Set ADC interrupt priority */


    ADPCFGbits.PCFG0 = 0; /* PinConCFigAnalog Current Measurement for Buck 2 */
    ADPCFGbits.PCFG1 = 0; /* PinConCFigAnalog Voltage Measurement for Buck 2 */

    ADPCFGbits.PCFG4 = 0; /* PinConCFigAnalog Voltage Measurement for input voltage source */



    ADSTATbits.P0RDY = 0; /* Clear Pair 0, AN0&AN1, data ready bit */
    ADCPC0bits.IRQEN0 = 1; /* Enable ADC Interrupt for Buck 2 control loop: current&voltage */
    ADCPC0bits.TRGSRC0 = 5; /* ADC Pair 0 triggered by PWM Generator 2 ¿1? primary trigger */

    /*lo que se entiende de las hojas es para PWM generator 2 ADCPC0bits.TRGSRC1 = 5 ¿?*/

    ADSTATbits.P2RDY = 0; /* Clear Pair 2 data ready bit */
    ADCPC1bits.IRQEN2 = 0; /* Enable ADC Interrupt for input voltage measurement */
    ADCPC1bits.TRGSRC2 = 5; /* ADC Pair 2 triggered by PWM Generator 2 ¿1 con trgsrc1=4? primary trigger */


}

void FlybackVoltageLoop() {
    FlybackVoltagePID.abcCoefficients = FlybackVoltageABC; /* Set up pointer to derived coefficients */
    FlybackVoltagePID.controlHistory = FlybackVoltageHistory; /* Set up pointer to controller history samples */

    PIDInitFlyback(&FlybackVoltagePID); /*pone todas las variables de error y anteriores de control a cero*/


    /* se llama a funcion pidinit, se le pasan las zonas de memoria x e y para inicializarlas 
    if (
    (PID_BUCK2_B0 == 0x7FFF || PID_BUCK2_B0 == 0x8000) ||
    (PID_BUCK2_B1 == 0x7FFF || PID_BUCK2_B1 == 0x8000))
    {
    while(1); //comprobacion de coeficientes en q15, si alguno es q15 entra en bucle infinito 
    } 
     */
    /* ubica los coeficientes en sus posiciones de memoria */

    FlybackVoltagePID.abcCoefficients[0] = PID_BUCK2_B0;
    FlybackVoltagePID.abcCoefficients[1] = PID_BUCK2_B1;
    FlybackVoltagePID.abcCoefficients[2] = PID_BUCK2_B2;
    FlybackVoltagePID.abcCoefficients[3] = PID_BUCK2_A1;
    FlybackVoltagePID.abcCoefficients[4] = PID_BUCK2_A2;

    FlybackVoltagePID.controlReference = PID_BUCK2_VOLTAGE_REF_MIN;

    FlybackVoltagePID.measuredOutput = 0;
    FlybackReferenceOld = PID_FLYBACK_VOLTAGE_REFERENCE;
    FlybackReferenceNew = PID_FLYBACK_VOLTAGE_REFERENCE;
}

void FlybackReferenceRoutine(void) {
    /* This routine increments the control reference until the reference reaches 
       the desired output voltage reference. In this case we have a softstart of 50ms 
  also in this case controReference stars from 0.
  */
  if (FlybackVoltagePID.controlReference<FlybackReferenceOld)
  {
      while (FlybackVoltagePID.controlReference <= FlybackReferenceOld)
      {
          Delay_ms(1);
          FlybackVoltagePID.controlReference += FLYBACK_SOFTSTART_INCREMENT;
      }
  }
  else
  {
      while (FlybackVoltagePID.controlReference >= FlybackReferenceOld)
      {
          Delay_ms(1);
          FlybackVoltagePID.controlReference -= FLYBACK_SOFTSTART_INCREMENT;
      }

	
  } 
    FlybackVoltagePID.controlReference = FlybackReferenceOld;
}
/*No se usa FlybackRefVoltValInit porque el pin 14 donde iba
 * SW1 de la placa de desarrollo está ocupado ahora por DCOUT
 */
/*
void FlybackRefVoltValInit(void) {
    //Inicializa la interrupcion por cambio en el pin 14/RB para poder cambiar la referencia
    //con el potenciometro
    TRISBbits.TRISB8 = 1; //Pin 14 conectado a sw1 es una entrada digital.
    IEC1bits.CNIE = 1; //Permite las interrupciones por Change Notification//
    CNEN1bits.CN8IE = 1; //Cuando se detecte una transicion en 8 se genera la interrupcion CN
    CNPU1bits.CN8PUE = 1; //PullUP activado (no hace falta con sw1)
    IPC4bits.CNIP = 4; //Prioridad 4, inferior a la de PMW y conv AD
    IFS1bits.CNIF = 0; //Habilita interrupciones tipo Change Notification
}
*/
void Delay_ms(unsigned int delay) {
    TimerInterruptCount = 0; //Clear Interrupt counter flag 
    //PR1 = 0x9C40;						//(1ms / 25ns) = 40,000 = 0x9C40 
    //IPC0bits.T1IP = 4;				 	//Set Interrupt Priority lower then ADC
    //IEC0bits.T1IE = 1;					//Enable Timer1 interrupts 

    T1CONbits.TON = 1; //Enable Timer1 

    while (TimerInterruptCount < delay); //Wait for Interrupt counts to equal delay

    T1CONbits.TON = 0; //Disable the Timer 
}

