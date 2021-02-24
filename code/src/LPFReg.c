//==============================================================================
// System Name:  	
//
// File Name:		LPFReg.c
//
// Description:	 
//          		 
//
// Originator:	Fastech Ltd.	
//
//###########################################################################
//
//  Ver | dd mmmm yyyy | Who  | Description of changes
// =====|==============|======|===============================================
//  0.01| xx xxxx xxxx | A.A. | Alpha Release
//###########################################################################

/*******************************************************************************************************
	Include Files
*******************************************************************************************************/
#include "IQmathLib.h"         	// include header for IQmath library
#include "LPFReg.h"		// include header for LPFREG module
#include "TypesDef.h"


//###########################################################################
// void LPF_Init(int32_t fc, int32_t T0, LPFREG *ptr)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine
//
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
void LPF_Init(int32_t fc, int32_t T0,LPFREG *ptr)
{
int32_t A0, Omega, Temp1, Temp2;

    //------------------ Omega= fc*T0/2 ------------------
    Temp1 = T0;						// 1Q31 bit format
    Temp1 *= fc;					// 1Q31*32Q0 = 1Q31 bit format
    Temp1++;						// Rounding up
    Omega = Temp1 >> 2;					// 1Q31 -> 2Q30 bit format and division by 2

    //-------------------- sin(omega) --------------------
    // sin input is scaled by (2*pi/2)!
    Temp1 = _IQ30sinPU(Omega) >> 6;			// Input 2Q30, output 2Q30 -> 8Q24 bit format

    //-------------------- cos(omega) --------------------
    // cos input is scaled by (2*pi/2)!
//	Temp2 = _IQ30cosPU(Omega) >> 6;			// Input 2Q30, output 2Q30 -> 8Q24 bit format
    Temp2 = _IQ30sinPU(Omega+268435456)>>6;	// Input 2Q30, output 2Q30 -> 8Q24 bit format

    //-------------------- Cotangent ---------------------
    Temp1 = _IQ24div(Temp2,Temp1);		// 8Q24/8Q24 = 8Q24 bit format;

    //--------------- Coefficient A0 (a0=1+cotan(Omega);) -----------------
    A0 = 16777216 + Temp1;			// 8Q24+8Q24 = 8Q24 bit format;

    //--------------- Coefficient A1 (a1=1-cotan(Omega);) -----------------
    //Temp2 = 16777216 - Temp1;			// 8Q24+8Q24 = 8Q24 bit format;
    //Temp1 = _IQ24div(Temp2,A0);		// 8Q24/8Q24 = 8Q24 bit format; Coefficient Scaling
    //Temp1 >>= 8;				// 8Q24 -> 16Q16 bit format;
    //ptr->A1 = -Temp1;				// Change the Denumerator coefficients' signs!!!

    //-------------------- Coefficient B0 (b0=1/a0;) -----------------
    Temp1 = _IQ24div(16777216,A0);		// 8Q24/8Q24 = 8Q24 bit format;
    ptr->B0 = Temp1 >> 8;			// 8Q24 -> 16Q16 bit format;

    //----- A1 = 1 - 2*B0 ------------
    ptr->A1 = 65536L - 2*ptr->B0;		// 16Q16 bit format;

    //-------------------- Coefficient B1 (b1=b0;) -----------------
    //ptr->B1 = ptr->B0;			// 16Q16 bit format;

    //---- Change the Denumerator coefficients' signs!!! ---
    //ptr->A1 = -ptr->A1;

    //--------------- Clean Filter State ---------------
    //ptr->U1.dw = 0;
    //ptr->Y1.dw = 0;
}
//###########################################################################




//###########################################################################
// int32_t LPF(int32_t, LPFREG *)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
// Input 16Q16 bit format; Output 16Q16 bit format
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
int32_t LPF(int32_t U0, LPFREG *ptr)
{
sQW Temp2;

  Temp2.qw = (int64_t)U0*ptr->B0 + (int64_t)ptr->U1.dw*ptr->B0 +\
  			 (int64_t)ptr->Y1.dw*ptr->A1; // 16Q16x16Q16 = 32Q32 bit format
  Temp2.w[0] = Temp2.w[1];
  Temp2.w[1] = Temp2.w[2];
  ptr->Y1.dw = Temp2.dw[0];			// Save output in 16Q16 bit format
  ptr->U1.dw = U0;				// Save input in 16Q16 bit format

  return(ptr->Y1.dw);				// 16Q16 bit format
}
//###########################################################################
//===========================================================================
// No more.
//===========================================================================
