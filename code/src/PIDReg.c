/**
  ******************************************************************************
  * @file    PIDReg.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2012-02-01
  * @brief
  ******************************************************************************
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "IQmathLib.h"         	// include header for IQmath library
#include "PIDReg.h"		// include header for PIREG module
#include "TypesDef.h"

extern int16_t ExtVar[];

//###########################################################################
// void PIController(PIDREG *)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine - an universal PID controller
// Data format change form 32Q0 to 16Q16 !!!
// Example: IF Input is in 32Q0 bit format, Output will be in 16Q16 bit format
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
void PIDController(PIDREG *temp)
{	
sQW LongLongTemp;

    temp->Err = temp->Ref - temp->Fdbk; 			// 32Q0 bit format. Compute error
    LongLongTemp.qw = (int64_t)temp->Err*temp->Kp; 		// 32Q0*16Q16 = 48Q16 bit format. Compute proportional part.

    if(LongLongTemp.qw < -2147483647)	LongLongTemp.dw[0] = -2147483647; else
    if(LongLongTemp.qw >  2147483647)	LongLongTemp.dw[0] =  2147483647;
    
    LongLongTemp.dw[0] >>= 1;					// 16Q16 -> 17Q15 bit format

    temp->DPart = LongLongTemp.dw[0] - temp->PPart;		// 17Q15 bit format
    temp->PPart = LongLongTemp.dw[0];				// 17Q15 bit format
    
    LongLongTemp.qw = (int64_t)temp->DPart*temp->Kd;		// 17Q15*16Q16 = 33Q31 bit format. Compute the Differential part.
    if(LongLongTemp.dw[1] < -16384)	LongLongTemp.dw[1] = -16384; else
    if(LongLongTemp.dw[1] >  16383)	LongLongTemp.dw[1] =  16383;

    LongLongTemp.w[0] = LongLongTemp.w[1];			// 17Q15 bit format

    LongLongTemp.w[1] = LongLongTemp.w[2];
    
    temp->DPart = LongLongTemp.dw[0] + temp->PPart;		// 17Q15 bit format

    if(temp->DPart < -temp->SatOut)	temp->DPart = -temp->SatOut; else
    if(temp->DPart >  temp->SatOut)	temp->DPart =  temp->SatOut;

    LongLongTemp.qw = (int64_t)temp->PPart*temp->Ki;		// 17Q15*16Q16 = 33Q31 bit format. Compute the current integral part.
    if(LongLongTemp.dw[1] < -16384)	LongLongTemp.dw[1] = -16384; else
    if(LongLongTemp.dw[1] >  16383)	LongLongTemp.dw[1] =  16383;
    
    LongLongTemp.w[0] = LongLongTemp.w[1];			// 17Q15 bit format.
    LongLongTemp.w[1] = LongLongTemp.w[2];

    temp->IPart += LongLongTemp.dw[0];				// 17Q15 bit format. Compute the integral part
    
    LongLongTemp.dw[0] = _IQabs(temp->DPart);			// 17Q15 bit format.
    LongLongTemp.dw[0] = temp->SatOut - LongLongTemp.dw[0];	// Negative limit

     
    if(temp->IPart < -LongLongTemp.dw[0])	temp->IPart = -LongLongTemp.dw[0]; else
    if(temp->IPart >  LongLongTemp.dw[0])	temp->IPart =  LongLongTemp.dw[0];

    LongLongTemp.dw[0] = temp->IPart + temp->DPart;	 	// 17Q15 bit format; Compute command
    temp->Out = LongLongTemp.dw[0] << 1;			// 17Q15 -> 16Q16 bit format.
}
//###########################################################################
//===========================================================================
// No more.
//===========================================================================
