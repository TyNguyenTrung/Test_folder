/**
  ******************************************************************************
  * @file    PIReg.c 
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
#include "PIReg.h"			// include header for PIREG module
#include "TypesDef.h"

extern int16_t ExtVar[];
/*****************************************************************************
  * @brief - This routine is an universal PI controller
  Data format change form 32Q0 to 16Q16 !!!
  Example: IF Input is in 32Q0 bit format, Output will be in 16Q16 bit format
  * @param  None
  * @retval None
*****************************************************************************/
void PIController(PIREG *temp)
{	
sQW LongLongTemp;

    //temp->Err = temp->Ref - temp->Fdbk; 			//32Q0 bit format. Compute error

    LongLongTemp.qw = (int64_t)temp->Err*temp->Kp; 		//32Q0*16Q16 = 48Q16 bit format. Compute proportional part.

    if(LongLongTemp.qw < -temp->SatOut)	LongLongTemp.dw[0] = -temp->SatOut; else
    if(LongLongTemp.qw >  temp->SatOut)	LongLongTemp.dw[0] =  temp->SatOut;

    temp->PPart = LongLongTemp.dw[0];				//16Q16 bit format
    
    LongLongTemp.qw = (int64_t)LongLongTemp.dw[0]*temp->Ki;		//16Q16*16Q16 = 32Q32 bit format. Compute the current integral part.
    if(LongLongTemp.dw[1] > 32767)	LongLongTemp.dw[1] = 32767;	else
    if(LongLongTemp.dw[1] < -32768)	LongLongTemp.dw[1] = -32768;
    
    LongLongTemp.w[0] = LongLongTemp.w[1];
    LongLongTemp.w[1] = LongLongTemp.w[2];
    LongLongTemp.dw[0] >>= 1;					//16Q16 -> 17Q15 bit format.

    temp->IPart = temp->IPart >> 1;				//16Q16 -> 17Q15 bit format.
    temp->IPart += LongLongTemp.dw[0];				//Compute the integral part
    
    temp->SatOut >>= 1;												//16Q16 -> 17Q15 bit format.
    LongLongTemp.dw[0] = _IQabs(temp->PPart) >> 1;		//16Q16 -> 17Q15 bit format.
    LongLongTemp.dw[0] = temp->SatOut - LongLongTemp.dw[0];     //Negative limit
    temp->SatOut <<= 1;												//Restore SatOut value
     
    if(temp->IPart < -LongLongTemp.dw[0])	temp->IPart = -LongLongTemp.dw[0]; else
    if(temp->IPart >  LongLongTemp.dw[0])	temp->IPart =  LongLongTemp.dw[0];
    temp->IPart <<= 1;												//17Q15 -> 16Q16 bit format.

    temp->Out = temp->IPart + temp->PPart;	 		//16Q16 bit format.
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
