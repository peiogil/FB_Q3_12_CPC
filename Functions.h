/* 
 * File:   dspcommon.h
 * Author: peio.gil
 *
 * Created on 29 de mayo de 2023, 11:07
 */
/****************************************************************************
*
* functions.h
* Interface to the DSP Library for the dsPIC30F.
*
****************************************************************************/

#include "dsp.h"
#define	G_PWM_FREQUENCY	100.0e3
#define	PWM_PERIOD	(1.0e9/(1.04 * G_PWM_FREQUENCY))
void Flyback2Drive(void);
void FlybackDriveCPC(void);
void CurrentandVoltageMeasurements(void);
void FlybackVoltageLoop(void);
void Delay_ms(unsigned int);
void PIDInitFlyback(tPID *);
void Buck2RefVoltValPotIntr(void);
void FlybackReferenceRoutine(void);
void InitUART1(void);
void ReceivedChar(void);
