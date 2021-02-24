/**
  ******************************************************************************
  * @file    Command_functions.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2011-12-02
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
#include "stm32f4xx.h"
#include "Command_functions.h"
#include "Parameters.h"
#include "USART_functions.h"
#include "USART_functions_2.h"
#include "GPIO_functions.h"
#include "PIReg.h"
#include "CurrentControl.h"
#include "PositionVelocityControl.h"
#include "REFGenFunctions.h"
#include "ring_buff.h"
#include "VERSION.H"
#include "TorqueMode.h"
//===========================================================================

struct CommStruct Comm;

uint16_t RAM_Address;
uint16_t ROM_MDB_Ptr;

extern uint16_t USBSendEn;
extern s16 UqRef;
extern PIREG IqFBController;
extern int32_t Dist_Buff;	// command distance (from RS232)
extern int32_t Speed_Buff; 	// command maximum speed (from RS232)
extern int32_t VelocityFBControllerOut;
extern int32_t CheckSum;

union VERSION Version;

extern uint8_t RecDataBuff[];
uint8_t PTCCmd;
uint16_t PTCAddr;
uint32_t PTCData;


uint8_t rCmd;
uint16_t rAddr;
uint32_t rData;

int16_t NewCommand;

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t Protocol_RecvPacket(void)
{
	int16_t Err = 0;
	int8_t wChecksum;
	uint16_t i;
	uint32_t temp = 0;

	// Check packat.
	// Checksum calculation
	wChecksum = 0;
	for (i=0; i<PACKET_RECV_SIZE - 1; i++){
		wChecksum += RecDataBuff[i];
	}
	// Checksum Inversion
	wChecksum = ~wChecksum;

	if((int8_t)RecDataBuff[PACKET_RECV_SIZE - 1] == (int8_t)wChecksum){ // if PASS
		// copy CMD
		PTCCmd = RecDataBuff[0];

		// Combine PTCAddr
		temp = RecDataBuff[2] & 0xFF;
		temp <<= 8;
		temp |= RecDataBuff[1] & 0xFF;
		PTCAddr = temp;

		// Combine Data
		temp = RecDataBuff[6] & 0xFF;
		temp <<= 8;
		temp |= RecDataBuff[5] & 0xFF;
		temp <<= 8;
		temp |= RecDataBuff[4] & 0xFF;
		temp <<= 8;
		temp |= RecDataBuff[3] & 0xFF;
		PTCData = temp;
	}
	else Err = -1;

	return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void Protocol_SendPacket(int16_t Err)
{
	uint16_t wChecksum;
	uint16_t i;

	if(Err)
	{
		PTCCmd = -1;
		PTCAddr = 0;
		PTCData = 0;
	}
	// PACKET FRAME -------------------------------------------------

	// Frame Type
	RecDataBuff[0] = PTCCmd;

	// Addr
	RecDataBuff[1] = (uint8_t)(PTCAddr & 0xFF);
	RecDataBuff[2] = (uint8_t)((PTCAddr >> 8) & 0xFF);

	// Data
	RecDataBuff[3] = (uint8_t)(PTCData & 0xFF);
	RecDataBuff[4] = (uint8_t)((PTCData >> 8) & 0xFF);
	RecDataBuff[5] = (uint8_t)((PTCData >> 16) & 0xFF);
	RecDataBuff[6] = (uint8_t)((PTCData >> 24) & 0xFF);

	// Checksum calculation
	wChecksum = 0;
	for (i=0; i<(PACKET_RECV_SIZE - 1); i++)
		wChecksum += RecDataBuff[i];

	RecDataBuff[7] = (uint8_t)(~wChecksum);
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void CommandManagement(void)
{
	int16_t Err;

        if(NewCommand == 0) return;
        NewCommand = 0;
        
	Err = Protocol_RecvPacket();
rCmd = PTCCmd;
rAddr = PTCAddr;
rData = PTCData;
	if (Err == 0)
	{
		if(PTCCmd == DST)              DriveStatus(&PTCAddr,&PTCData);      // #10 Drive STatus
                else if(PTCCmd == NDST)        DriveStatus2(&PTCAddr,&PTCData);     // #12 New Drive STatus
		else if(PTCCmd == WMA)  Err |= ParameterSet(PTCAddr,PTCData);       // #1 Parameter write
		else if(PTCCmd == RMA)  Err |= ParameterGet(PTCAddr,&PTCData);      // #2 Parameter read
		//else if(PTCCmd == TME)         TorqueModeEnDis(PTCAddr,PTCData);    // #11 Torque Mode Enable/Disable
		else if(PTCCmd == VDP)  Err |= DataVerify(DriverParam);             // #3 Verifing of Driver parameters
		//else if(PTCCmd == VMP)  Err |= DataVerify(MotorParam);              // #4 Verifing of Motor parameters
		else if(PTCCmd == SDP)  Err |= ParametersSave(DriverParam);         // #5 Save Driver Parameters; Returns "0" on success
		//else if(PTCCmd == SMP)  Err |= ParametersSave(MotorParam);          // #6 Save Motor Parameters; Returns "0" on success
		else if(PTCCmd == LPR){ Err |= ParametersLoad(DriverParam);         // #7 Load Driver Parameter
					/*Err |= ParametersLoad(MotorParam);*/}         //    Load Motor Parameter
		//else if(PTCCmd == CER)  Err |= FB_Res_Calc(PTCData);                // #8 Change Encoder Resolution
		else if(PTCCmd == VER)  Err |= VERsion(&PTCAddr,&PTCData);          // #9 Firmware VERsion
		else Err |= -1;
	}

	Protocol_SendPacket(Err);
        
        USART3_DMA_TransmitReInit();
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t VERsion(uint16_t *Data1, uint32_t *Data2)
{
	Version.bit.EquipmentType =	VER_ET_VAL;
	Version.bit.ScriptVer =		VER_SC_VAL;
	Version.bit.ProtocolVer =	VER_PR_VAL;
	Version.bit.FirmVer =		VER_FW_VAL;
	Version.bit.HardVer =		VER_HW_VAL;

	Version.bit.Year =		VER_YEAR_VAL;
	Version.bit.Month =		VER_MON_VAL;
	Version.bit.Day =		VER_DAY_VAL;

	*Data2 = Version.all & 0xFFFFFFFF;
	*Data1 = (Version.all >> 32)& 0xFFFF;

	return 0;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
void CommandFunctions_Init(void)
{
int8_t i;
int8_t *p;

    //	Clear memory
    p = (int8_t*) &Comm;
    for(i = 0; i < sizeof(Comm); i++){
      *p = 0;
      p++;
    }

    Comm.FIFOInputPtr = 1;
    
    ROM_MDB_Ptr = 0;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Receive command string and save it in a FIFO
  * @param  None
  * @retval None
  ***************************************************************************/
void ReceiveCommand(void)
{
uint8_t RXByte;

  #ifdef INTERFECE_USART2
    RXByte = (USART_ReceiveData(USART2) & 0x7F); //	Receive a character from USART
  #else
    //while(VCP_get_char(&RXByte))	// Receive a character from USB
  #endif
  {
    //	If EoS
    if(RXByte == '\r'){
      //	Processing of error (int32_t) string
      if(Comm.FIFOPosPtr >= FIFOColumn){
        // End of String
        CmdEoS();
        CmdEoS();
        // Error String
        CmdError(1);
        // End of String
        CmdEoS();
      }
      else{
        //  Addition EoS to string
        Comm.FIFO[Comm.FIFOInputPtr][Comm.FIFOPosPtr] = '\0';
        //  Going to new line
        if(Comm.FIFOPosPtr > 0)
          Comm.FIFOInputPtr++;
        //  Check new value
        if(Comm.FIFOInputPtr >= FIFORow)
          Comm.FIFOInputPtr = 0;
        //  End of String
        CmdEoS();
      }
      //  Going to start of string
      Comm.FIFOPosPtr = 0;
    }
    //	If not EoS
    else{
      //	Check new position
      if(Comm.FIFOPosPtr > FIFOColumn - 1)
         Comm.FIFOPosPtr = FIFOColumn - 1;

      if(((RXByte >= 'A') && (RXByte <= 'Z')) || \
        ((RXByte >= 'a') && (RXByte <= 'z')) || \
        ((RXByte >= '0') && (RXByte <= '9')) || \
	((RXByte == '-')) ){
	//	Save character in a buffer
	Comm.FIFO[Comm.FIFOInputPtr][Comm.FIFOPosPtr] = RXByte;
	// Send data byte
          #ifdef INTERFECE_USART2
            USART_WaitCharDataSend(USART2, RXByte);
          #else
            //VCP_send_char(RXByte);
          #endif
        
	// Increment position number
	Comm.FIFOPosPtr++;
	// Processing of error (int32_t) string
	if(Comm.FIFOPosPtr >= FIFOColumn){
	  Comm.FIFOPosPtr = 0;
	  // End of String
	  CmdEoS();
	  CmdEoS();
	  // Error String
	  CmdError(1);
	  // End of String
	  CmdEoS();
	}
      }
      else if((RXByte == '\b') && (Comm.FIFOPosPtr > 0)){		//	Backspace
        // Send data byte
       #ifdef INTERFECE_USART2
          USART_WaitCharDataSend(USART2, RXByte);
          USART_WaitCharDataSend(USART2, ' ');
          USART_WaitCharDataSend(USART2, RXByte);
       #else
          //VCP_send_char(RXByte);
          //VCP_send_char(' ');
          //VCP_send_char(RXByte);
        #endif

        // Decrement position number
        Comm.FIFOPosPtr--;
        // Delete character from a buffer
        Comm.FIFO[Comm.FIFOInputPtr][Comm.FIFOPosPtr] = '\0';
      }
    }
  }
}
/*****************************************************************************/




//###########################################################################
// void CommandsFunctions_Init(void)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//	Routine Name: ReadCommand
//	Purpose		: Called from the main function
//	Description	: Read command string and load command functions
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
void CommandRead(void)
{
  int8_t CmdStr[3];
  int16_t Err = 0;

    while(!((Comm.FIFOOutputPtr == Comm.FIFOInputPtr - 1) || \
	   ((Comm.FIFOOutputPtr >= FIFORow - 1) && (Comm.FIFOInputPtr == 0))) ){
      // Increment FIFO Output Pointer
      Comm.FIFOOutputPtr++;
      // Check new value
      if(Comm.FIFOOutputPtr >= FIFORow)
        Comm.FIFOOutputPtr = 0;

      // Read Command
      CmdStr[0] = Comm.FIFO[Comm.FIFOOutputPtr][0];
      CmdStr[1] = Comm.FIFO[Comm.FIFOOutputPtr][1];
      CmdStr[2] = Comm.FIFO[Comm.FIFOOutputPtr][2];
      
      // End of String
      CmdEoS();
		
      // Load function
      if(Err == 0){
	if(CmdStrCmp(CmdStr, "\0\0")) Err = 0;
	else if	(CmdStrCmp(CmdStr, "LPR")) Err = CmdLPR(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Load Parameters
	//else if	(CmdStrCmp(CmdStr, "LDF")) Err = CmdLDF(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Load Default parameters
        else if	(CmdStrCmp(CmdStr, "SPR")) Err = CmdSPR(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Save Parameters
	//else if	(CmdStrCmp(CmdStr, "SDP")) Err = CmdSDP(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Save Driver Parameters
	//else if	(CmdStrCmp(CmdStr, "SMP")) Err = CmdSMP(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   /=/ Save Motor Parameters
        else if	(CmdStrCmp(CmdStr, "VPR")) Err = CmdVPR(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Verify Parameters
	//else if	(CmdStrCmp(CmdStr, "VDP")) Err = CmdVDP(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Verify Driver Parameters
	//else if	(CmdStrCmp(CmdStr, "VMP")) Err = CmdVMP(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Verify Motor Parameters
        else if	(CmdStrCmp(CmdStr, "CSC")) Err = CmdCSC(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Parameter's Check Sum calculation
	else if	(CmdStrCmp(CmdStr, "VER")) Err = CmdVER(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Firmware VERsion
	//else if	(CmdStrCmp(CmdStr, "ERE")) Err = CmdERE(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Recalculate Encoder Resolution
        //else if	(CmdStrCmp(CmdStr, "MDB")) Err = CmdMDB(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Select MDB in ROM
        
        else if	(CmdStrCmp(CmdStr, "MDS")) Err = CmdMDS(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Motion Distance Set
        else if	(CmdStrCmp(CmdStr, "MVS")) Err = CmdMVS(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Motion Velocity Set
        else if	(CmdStrCmp(CmdStr, "MAS")) Err = CmdMAS(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Motion Acceleration Set

        
	else if	(CmdStrCmp(CmdStr, "MAD")) Err = CmdMAD(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Set Start Address
	else if	(CmdStrCmp(CmdStr, "MDA")) Err = CmdMDA(&Comm.FIFO[Comm.FIFOOutputPtr][3]);   // Write/Read data to/from address "Address"
	else if	(CmdStrCmp(CmdStr, "END")) Err = 0;											// String "END" of Data Downloading

	else	Err = 2;	//	Incorrect command
      }

      if(Err != 0) CmdError(Err);

      //	End of String
      //CmdEoS();
    }
}
//###########################################################################




//###########################################################################
// int CmdStrCmp(const int8_t, const int8_t)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
// 
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
int16_t CmdStrCmp(const int8_t Str1[3], const int8_t Str2[3])
{
    if(((Str1[0] == Str2[0]) || (Str1[0] == (Str2[0] + 0x20))) && \
       ((Str1[1] == Str2[1]) || (Str1[1] == (Str2[1] + 0x20))) && \
       ((Str1[2] == Str2[2]) || (Str1[2] == (Str2[2] + 0x20))))
      return 1;
    else
      return 0;
}
//###########################################################################




//###########################################################################
// void CmdEoS(void)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
// 
//	End of String
// ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
void CmdEoS(void)
{
  #ifdef INTERFECE_USART2
    USART_WaitCharDataSend(USART2, 0x0D);     // send carriage return
    USART_WaitCharDataSend(USART2, 0x0A);     // send line feed
  #else
    //VCP_send_char(0x0D);     // send carriage return
    //VCP_send_char(0x0A);     // send line feed	
  #endif
}
//###########################################################################




//###########################################################################
// void CmdError(int)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
// 
//	Command Errors
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
void CmdError(int16_t Err)
{
    int8_t Err1[] = "ERR String is very long\0";
    int8_t Err2[] = "ERR Incorrect command\0";
    int8_t Err3[] = "ERR Incorrect command parameter\0";

    switch(Err){
      case 1:
        CmdSendString(&Err1[0]);
      break;
  
      case 2:
        CmdSendString(&Err2[0]);
      break;
  
      case 3:
        CmdSendString(&Err3[0]);
      break;
  
      default:
        //	End of String
      CmdEoS();
    }

    //	End of String
    CmdEoS();

    return;
}
//###########################################################################




//###########################################################################
// void CmdSendString(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
// 
//	Send String
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
void CmdSendString(int8_t* Str)
{
    while (*Str != '\0'){
      #ifdef INTERFECE_USART2
        USART_WaitCharDataSend(USART2, *Str);
      #else
        //VCP_send_char(*Str);	
      #endif
      Str++;
    }
}
//###########################################################################




//###########################################################################
// void CmdSendStrLong(int32_t, unsigned int)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
// 
//	Send Decimal Digit 
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
void CmdSendStrLong(int32_t Value, uint16_t IntegerPart)
{
    int32_t Divisor = 10, Temp;
    int16_t i = 0, Minus = 0;

    if(Value < 0){
      Minus = 1;
      // Absolute value
      Value ^= 0xFFFFFFFF;
      Value++;
    }

	//	Calculate divisor
	if(Value <= 9){
		Divisor = 1;
		i--;
	}
	else
		while(((Value / Divisor) > 9) && (i < 10)){
			Divisor *= 10;
			i++;
		}

	//	Send ' '
/*	Spaces = (int16_t) IntegerPart - i - 2 - Minus;
	while(Spaces > 0){
		SCIA_WaitSendChar(' ');
		Spaces--;
	}*/
	
	//	Send '-'
	if(Minus > 0)
          #ifdef INTERFECE_USART2
            USART_WaitCharDataSend(USART2, '-');
          #else
            //VCP_send_char('-');
          #endif

	//	Send Digit
	while(Divisor > 1){
		Temp = Value;
		Value = Value / Divisor;
                #ifdef INTERFECE_USART2
                  USART_WaitCharDataSend(USART2, Value + 0x30);
                #else
                  //VCP_send_char(Value + 0x30);
                #endif
		Value = Temp - (Value * Divisor);
		Divisor = Divisor / 10;
	}

	//	Send Last Digit
        #ifdef INTERFECE_USART2
          USART_WaitCharDataSend(USART2, Value + 0x30);
        #else
          //VCP_send_char(Value + 0x30);
        #endif

}
//###########################################################################




//###########################################################################
// int32_t CmdReadStrLong(int8_t*, int16_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
// 
//	Read Decimal Digit 
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int32_t CmdReadStrLong(int8_t* Str, int16_t* Err)
{
  int32_t Value = 0;
  int16_t Mult = 10, Minus = 0, i;

  *Err = 0;

    if(*Str == '-'){
      Minus = 1;
      Str++;
    }

    for(i = 0; i < FIFOColumn - 2; i++){
      if((*Str >= '0') && (*Str <= '9')){
	Value *= Mult;
	Value += ((*Str) - 0x30);
	Str++;
      }
      else if(*Str == '\0'){
	i = FIFOColumn + 1;
      }
      else{
	i = FIFOColumn + 1;
	*Err = 3;
      }
    }

    if(Minus == 1)
      Value *= -1;

    return Value;
}
//###########################################################################




//###########################################################################
// int16_t CmdLPR(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//  Load Parameters
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdLPR(int8_t* StrParam)
{
//int8_t Str[] = "Loading parameters...  \0";
int8_t StrOK[] = "OKY\0";
int8_t StrERR[] = "ERR\0";
int16_t Err;

    if(*StrParam == '\0'){
      //CmdSendString(Str);
      
      Err =  ParametersLoad(DriverParam);
      Err |= ParametersLoad(MotorParam);

      if(Err) CmdSendString(StrERR);
      else    CmdSendString(StrOK);
      
      CmdEoS();

      return 0;
    }
    else
      return 3;
}
//###########################################################################




//###########################################################################
// int16_t CmdLD(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//	Load Default parameters
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdLDF(int8_t* StrParam)
{
/*	int8_t Str[] = "Loading default parameters...  \0";
	int8_t StrDone[] = "Done\0";

	if(*StrParam == '\0')
	{
		CmdSendString(Str);

//		EEPROMLoadDefault();

		CmdSendString(StrDone);
		CmdEoS();

		return 0;
	}
	else*/
		return 3;
}
//###########################################################################




//###########################################################################
// int16_t CmdSPR(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//  Save Parameters
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdSPR(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int8_t StrERR[] = "ERR\0";
int16_t Err;

    if(*StrParam == '\0'){
      
      Err = ParametersSave(0);  // Returns "0" on success
	
      if(Err) CmdSendString(StrERR);
      else    CmdSendString(StrOK);
                
      CmdEoS();

      return 0;
    }
    else
      return 3;
}
//###########################################################################




//###########################################################################
// int16_t CmdSDP(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//  Save Driver Parameters
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
/*int16_t CmdSDP(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int8_t StrERR[] = "ERR\0";
int16_t Err;

    if(*StrParam == '\0'){
      
      Err = ParametersSave(DriverParam);  // Returns "0" on success
	
      if(Err) CmdSendString(StrERR);
      else    CmdSendString(StrOK);
                
      CmdEoS();

      return 0;
    }
    else
      return 3;
}*/
//###########################################################################




//###########################################################################
// int16_t CmdSMP(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//  Save Motor Parameters
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdSMP(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int8_t StrERR[] = "ERR\0";
int16_t Err;

    if(*StrParam == '\0'){

      Err = ParametersSave(MotorParam); // Returns "0" on success

      if(Err) CmdSendString(StrERR);
      else    CmdSendString(StrOK);
      
      CmdEoS();

      return 0;
    }
    else
      return 3;
}
//###########################################################################




//###########################################################################
// int16_t CmdVPR(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//	Verify Parameters
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdVPR(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int8_t StrERR[] = "ERR\0";
int16_t Err;

    if(*StrParam == '\0'){
      // Verifing of the parameters
      Err = DataVerify(0);
      
      if(Err) CmdSendString(StrERR);
      else    CmdSendString(StrOK);
      
      // String Printing
      //CmdSendStrLong(Value, 6);

      CmdEoS();

      return 0;
    }
    else
      return 3;
}
//###########################################################################




//###########################################################################
// int16_t CmdVDP(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//	Verify Driver Parameters
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
/*int16_t CmdVDP(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int8_t StrERR[] = "ERR\0";
int16_t Err;

    if(*StrParam == '\0'){
      // Verifing of the parameters
      Err = DataVerify(DriverParam);
      
      if(Err) CmdSendString(StrERR);
      else    CmdSendString(StrOK);
      
      // String Printing
      //CmdSendStrLong(Value, 6);

      CmdEoS();

      return 0;
    }
    else
      return 3;
}*/
//###########################################################################




//###########################################################################
// int16_t CmdVMP(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//	Verify Motor Parameters
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdVMP(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int8_t StrERR[] = "ERR\0";
int16_t Err;

    if(*StrParam == '\0'){
      // Verifing of the parameters
      Err = DataVerify(MotorParam);
      
      if(Err) CmdSendString(StrERR);
      else    CmdSendString(StrOK);
      
      // String Printing
      //CmdSendStrLong(Value, 6);

      CmdEoS();

      return 0;
    }
    else
      return 3;
}
//###########################################################################




//###########################################################################
// int16_t CmdCSC(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//  
//	Parameter's Check Sum Calculation
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdCSC(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int8_t StrERR[] = "ERR\0";
int16_t Err;

    if(*StrParam == '\0'){
      // Verifing of the parameters
      Err = CheckSumCalculate(MotorParam);
      
      if(Err) CmdSendString(StrERR);
      else    CmdSendString(StrOK);
      
      // String Printing
      CmdSendStrLong(CheckSum, 6);

      CmdEoS();

      return 0;
    }
    else
      return 3;
}
//###########################################################################




//###########################################################################
// int16_t CmdVR(int8_t*)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//
//	Firmware VERsion
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdVER(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
//int8_t Str[] = "A-Servo Bipolar ver.01.01.02.07/03/2012.11.16\0";

int8_t Str1[]= "Servo BLDC ver.01.01.01.01/01/2016.11.04\0";
int8_t Str2[] = "S-Servo Bipolar ver.01.01.02.08/03/2014.11.04\0";


  if(*StrParam == '\0'){
    CmdSendString(StrOK);
    if (VERSION())
      CmdSendString(Str1);
    else
      CmdSendString(Str2);
    CmdEoS();
    return 0;
  }
  else  return 3; 
}
//###########################################################################




//###########################################################################
// int16_t CmdMAD(int8_t* StrParam)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//
//	Set Address Pointer
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdMAD(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int16_t Err = 0;
int32_t Value;

    // If parameter exist then
    if(*StrParam != '\0'){
      // Reading of the parameter
      Value = CmdReadStrLong(StrParam, &Err);
      // If parameter is real then set it
      if(!Err){
	if(Value < 0)		Value = 0;
	if(Value > 65535)	Value = 65535;
	// Saving the parameter
	RAM_Address = Value;
      }
    }

    // If error is zero then
    if(!Err){
      // Reading of the parameter
      Value = RAM_Address;
      // String Printing
      CmdSendString(StrOK);
      CmdSendStrLong(Value, 6);
      CmdEoS();
    }

    return Err;
}
//###########################################################################




//###########################################################################
// int16_t CmdMDA(int8_t* StrParam)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//
// Send data to RAM memory at Address "Address"
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdMDA(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int16_t Err = 0;
uint32_t Value;

    // If parameter exist then
    if(*StrParam != '\0'){
      // Reading of the parameter
      Value = CmdReadStrLong(StrParam, &Err);
      // If parameter is real then set it
      if(!Err){
        //if(Value < 0) Value = 0;
	//if(Value > 1) Value = 1;
	// Parameter Writing
	ParameterSet(RAM_Address, Value);
      }
    }

    // If error is zero then
    if(!Err){
      // Parameter Reading
      ParameterGet(RAM_Address, &Value);
      // String Printing
      CmdSendString(StrOK);
      CmdSendStrLong(Value, 6);
      CmdEoS();
    }

    return Err;
}
//###########################################################################




/*****************************************************************************
  * @brief  Change Encoder Resolution
  * @param  None
  * @retval None
*****************************************************************************/
int16_t CmdERE(int8_t* StrParam)
{
    int8_t StrERR[] = "ERR\0";
    int8_t StrOK[] = "OKY\0";
    int16_t Err = 0;
    int32_t Value;

    // If parameter exist then
    if(*StrParam != '\0'){
      // Reading of the parameter
      Value = CmdReadStrLong(StrParam, &Err);
      // If parameter is real then set it
      if(!Err){
        //if(Value < 0)		Value = 0;
        //if(Value > 65536)	Value = 65536;
  
      // Parameter Recalculation
        //Err = FB_Res_Calc(Value);
        //UqRef = Value;
        //IqFBController.Ref = Value;
        VelocityFBController.Ref = Value;
      }
    }

    // If error
    if(Err) CmdSendString(StrERR);
    else{
      // Parameter Reading
      //Value = FB.EncRes;
      //Value = UqRef;
      //Value = IqFBController.Ref;
      Value = VelocityFBController.Ref;
        
      // String Printing
      CmdSendString(StrOK);
      CmdSendStrLong(Value, 6);
      CmdEoS();
    }

    return Err;
}
/*****************************************************************************/




//###########################################################################
// int16_t CmdMDB(int8_t* StrParam)
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// This routine 
//
// Select MDB in ROM
//––––––––––––––––––––––––––––––––––––––––––––––-----------------------------
int16_t CmdMDB(int8_t* StrParam)
{
int8_t StrOK[] = "OKY\0";
int16_t Err = 0;
uint32_t Value;

    // If parameter exist then
    if(*StrParam != '\0'){
      // Reading of the parameter
      Value = CmdReadStrLong(StrParam, &Err);
      // If parameter is real then set it
      if(!Err){
        //if(Value < 0)  Value = 0;
	if(Value > 15) Value = 15;

        ROM_MDB_Ptr = Value;
      }
    }

    // If error is zero then
    if(!Err){
      // Reading of the parameter
      Value = ROM_MDB_Ptr;
      // String Printing
      CmdSendString(StrOK);
      CmdSendStrLong(Value, 6);
      CmdEoS();
    }

    return Err;
}
//###########################################################################




/*****************************************************************************
  * @brief  Change RS232 Chanal A Baud Rate
  * @param  None
  * @retval None
*****************************************************************************/
int16_t CmdBRT(int8_t* StrParam)
{
//	int8_t Str[] = "Baud Rate Chanal 'A' ...............................(BRT)...\0";

	int16_t Err = 0;
/*	int32_t Value;

	// If parameter exist then
	if(*StrParam != '\0'){
		// Reading of the parameter
		Value = CmdReadStrLong(StrParam, &Err);
		// If parameter is real then set it
		if(!Err){
			//if(Value < 0) Value = 0;
			//if(Value > 1) Value = 1;

			// String Printing
			CmdSendString(Str);
			CmdSendStrLong(Value, 6);
			CmdEoS();
			
			DELAY_US(100000);						// Approximately 100 mS Delay

			Value = (250000000/(Value <<= 3)) - 2; 	// Baud Rate Calculation
			Value = (++Value) >> 1;					// Rounding

			SciaRegs.SCILBAUD = Value;
			SciaRegs.SCIHBAUD = Value >> 8;
		}
	}

	// If error is zero then
	if(!Err){
		Value = SciaRegs.SCIHBAUD;
		Value <<= 8;
		Value |= SciaRegs.SCILBAUD;
		Value++;
		Value <<= 3;
		Value = 125000000/Value;

		// String Printing
		CmdSendString(Str);
		CmdSendStrLong(Value, 6);
		CmdEoS();
	}*/

	return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Motion Distance Set
  * @param  None
  * @retval None
*****************************************************************************/
int16_t CmdMDS(int8_t* StrParam)
{
    int8_t StrERR[] = "ERR\0";
    int8_t StrOK[] = "OKY\0";
    int16_t Err = 0;
    int32_t Value;

    // If parameter exist then
    if(*StrParam != '\0'){
      // Reading of the parameter
      Value = CmdReadStrLong(StrParam, &Err);
      // If parameter is real then set it
      if(!Err){
        if(Value < -21400000) 	   Value = -21400000;
	else if(Value >  21400000) Value =  21400000;
  
      // Parameter Recalculation
        //Err = FB_Res_Calc(Value);
        //UqRef = Value;
        //IqFBController.Ref = Value;
        //Dist_Buff = Value;		// New Reference Position
      }
    }

    // If error
    if(Err) CmdSendString(StrERR);
    else{
      // Parameter Reading
      //Value = FB.EncRes;
      //Value = UqRef;
      //Value = IqFBController.Ref;
      //Value = Dist_Buff;
        
      // String Printing
      CmdSendString(StrOK);
      CmdSendStrLong(Value, 6);
      CmdEoS();
    }

    return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Motion Velocity Set
  * @param  None
  * @retval None
*****************************************************************************/
int16_t CmdMVS(int8_t* StrParam)
{
    int8_t StrERR[] = "ERR\0";
    int8_t StrOK[] = "OKY\0";
    int16_t Err = 0;
    int32_t Value;

    // If parameter exist then
    if(*StrParam != '\0'){
      // Reading of the parameter
      Value = CmdReadStrLong(StrParam, &Err);
      // If parameter is real then set it
      if(!Err){
        if(Value < -500000) 	 Value = -500000;
	else if(Value >  500000) Value =  500000;
  
      // Parameter Recalculation
        //Err = FB_Res_Calc(Value);
        //Speed_Buff = Value;
        //VelocityFBControllerOut/*IqFBController.Ref*/ = Value;
        //En = Value;
      }
    }

    // If error
    if(Err) CmdSendString(StrERR);
    else{
      // Parameter Reading
      //Value = FB.EncRes;
      //Value = UqRef;
      //Value = VelocityFBControllerOut;//IqFBController.Ref;
      Value = USBSendEn;
        
      // String Printing
      CmdSendString(StrOK);
      CmdSendStrLong(Value, 6);
      CmdEoS();
    }

    return Err;
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Motion Accelaration Set
  * @param  None
  * @retval None
*****************************************************************************/
int16_t CmdMAS(int8_t* StrParam)
{
    int8_t StrERR[] = "ERR\0";
    int8_t StrOK[] = "OKY\0";
    int16_t Err = 0;
    int32_t Value;

    // If parameter exist then
    if(*StrParam != '\0'){
      // Reading of the parameter
      Value = CmdReadStrLong(StrParam, &Err);
      // If parameter is real then set it
      if(!Err){
        if(Value < 0) 	Value = 0;
	if(Value > 500) Value = 500;
  
      // Parameter Recalculation
        //Err = FB_Res_Calc(Value);
        //UqRef = Value;
        //IqFBController.Ref = Value;
        //RLPF1.AccDec_time = Value;
      }
    }

    // If error
    if(Err) CmdSendString(StrERR);
    else{
      // Parameter Reading
      //Value = FB.EncRes;
      //Value = UqRef;
      //Value = IqFBController.Ref;
      //Value = RLPF1.AccDec_time;
        
      // String Printing
      CmdSendString(StrOK);
      CmdSendStrLong(Value, 6);
      CmdEoS();
    }

    return Err;
}
/*****************************************************************************/




//------------------------------------------------------------------------------
// USART 3 - RS485 interrupt service routines
//------------------------------------------------------------------------------
/*****************************************************************************
  * @brief  Transmit Data Register empty ISR
  * @param  None
  * @retval None
*****************************************************************************/
void USART3_isr_txe(void){
  // Transmition Complete - disable TX interrupt
  if( USART_GetFlagStatus(EZ_COM3, USART_FLAG_TC) == SET ){
    USART_ITConfig(EZ_COM3, USART_IT_TXE, DISABLE); // Disable TX interrupt
  }
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Transmission complete ISR
  * @param  None
  * @retval None
*****************************************************************************/
void USART3_isr_tc(void){
    // Disable the USART3 Transmit Complete interrupt
    //USART_ITConfig(COM_USART[COM3], USART_IT_TC, DISABLE);
  
    // Clear USART3 TC pending bit
    USART_ClearFlag(COM_USART[COM3], USART_FLAG_TC);
    
    EZ_COMStatus[COM3] |= USART_STATUS_FLAG_TC;  // Set TC status flag
    EZ_COMStatus[COM3] &= ~USART_STATUS_FLAG_TIP;  // Clear TIP status flag
    
    // RS485 Set DIR pin to LOW level
    //-------------------------------
    RS485_RxMode; //RX485 driver to RX direction
    //LED_PlusR1_status &= ~LED_PLUSR_TX; //clear PlusR Tx flag
    //LEDOff(LED_RJ45Y); //(Tx)Yellow LED on Plus-R socket turn Off
}
/*****************************************************************************/




/*****************************************************************************
  * @brief  Receive Data register not empty ISR
  * @param  None
  * @retval None
*****************************************************************************/
void USART3_isr_rxne(void){
  uint16_t data16, flags;
  
  do{ 
    //LED_PlusR1_status &= ~LED_PLUSR_TX; //clear PlusR Tx flag
    
    //LEDOn(LED_RJ45G); //(Rx)Green LED on Plus-R socket turn On
    
    flags = 0;
    
    if (USART_GetFlagStatus(COM_USART[COM3], USART_FLAG_ORE)) //OverRun Error flag
    {
      //USART_ClearFlag(COM_USART[COM3], USART_FLAG_ORE); //Clear OverRun Error flag
      flags |= USART_RXRB_FLAG_ORE; //Set Ring Buffer Rx OverRun Error flag
    }
    if (USART_GetFlagStatus(COM_USART[COM3], USART_FLAG_NE)) //Noise Error flag
    {
      flags |= USART_RXRB_FLAG_NE;
    }
    if (USART_GetFlagStatus(COM_USART[COM3], USART_FLAG_FE)) //Framing Error flag
    {
      flags |= USART_RXRB_FLAG_FE;
    }
    if (USART_GetFlagStatus(COM_USART[COM3], USART_FLAG_PE)) //Parity Error flag
    {
      flags |= USART_RXRB_FLAG_PE;
    }

    // PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun 
    // error) and IDLE (Idle line detected) flags are cleared by software 
    // sequence: a read operation to USART_SR register (USART_GetFlagStatus()) 
    // followed by a read operation to USART_DR register (USART_ReceiveData()).    
    data16 = flags | USART_ReceiveData(COM_USART[COM3]);
    
    // Save Rx data to ring buffer
    RingBuf_PutData((RingBuffTypeDef *)&USART_Rx_RingBuff[COM3], data16);
    
  }while(USART_GetFlagStatus(COM_USART[COM3], USART_FLAG_RXNE));
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
