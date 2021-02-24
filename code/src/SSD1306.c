/**
******************************************************************************
* @file    SSD1306.c 
* @author  Waveshare Team
* @version 
* @date    13-October-2014
* @brief   This file includes the OLED driver for SSD1306 display moudle
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "LIB_Config.h"
#include "SSD1306.h"
#include "Fonts.h"
#include "font.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SSD1306_CMD    0
#define SSD1306_DAT    1

#define SSD1306_WIDTH    0x60 /*96*/
#define SSD1306_HEIGHT   0x0F /*16*/

#define WRITE_ADDRESS 0x78/*slave addresses with write*/
#define READ_ADDRESS 0x79/*slave addresses with read*/




/* Private macro -------------------------------------------------------------*/

#if !defined(SH1106) && !defined(SSD1306)
#warning Please select first the target OLED device(SH1106 or SSD1306) in your application!
#define SSD1306  //define SSD1306 by default	
#endif

#if defined(SSD1306)
#define __SET_COL_START_ADDR() ssd1306_set_page_address(0); ssd1306_set_column_address(0)
#if 0
#define __SET_COL_START_ADDR() 	do { \
ssd1306_write_byte(0x10/*0x00*/, SSD1306_CMD); \
  ssd1306_write_byte(0x00/*0x10*/, SSD1306_CMD); \
				} while(false)
#endif                                  
                                  
#elif defined(SH1106)
#define __SET_COL_START_ADDR() 	do { \
                                  ssd1306_write_byte(0x02, SSD1306_CMD); \
                                    ssd1306_write_byte(0x10, SSD1306_CMD); \
				} while(false)
#endif	
                                  
                                  /* Private variables ---------------------------------------------------------*/
                                  static uint8_t s_chDispalyBuffer[128][8];

static uint8_t PosX, PosY, Page;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
* @brief  Writes an byte to the display data ram or the command register
*         
* @param  chData: Data to be writen to the display data ram or the command register
* @param chCmd:  
*                           0: Writes to the command register
*                           1: Writes to the display data ram
* @retval None
**/
static void ssd1306_write_byte(uint8_t chData, uint8_t chCmd) 
{
#ifdef INTERFACE_4WIRE_SPI
  
  __SSD1306_CS_CLR();
  
  if (chCmd) {
    __SSD1306_DC_SET();
  } else {
    __SSD1306_DC_CLR();
  }	
  __SSD1306_WRITE_BYTE(chData);
  
  __SSD1306_DC_SET();
  __SSD1306_CS_SET();
  
#elif defined(INTERFACE_3WIRE_SPI)
  
  uint8_t i;
  uint16_t hwData = 0;
  
  if (chCmd) {
    hwData = (uint16_t)chData | 0x0100;
  } else {
    hwData = (uint16_t)chData & ~0x0100;
  }
  
  __SSD1306_CS_CLR();
  for(i = 0; i < 9; i ++) {
    __SSD1306_CLK_CLR();
    
    if(hwData & 0x0100) {
      __SSD1306_DIN_SET();
    } else {
      __SSD1306_DIN_CLR();
    }
    __SSD1306_CLK_SET();
    hwData <<= 1;
  }
  __SSD1306_CS_SET();
  
#elif defined(INTERFACE_IIC)
  
  iic_start();
  iic_write_byte(WRITE_ADDRESS);
  iic_wait_for_ack();
  if (chCmd) {
    iic_write_byte(0x40);
    //iic_write_byte(0x80);
    iic_wait_for_ack();
  } else {
    iic_write_byte(0x00);
    iic_wait_for_ack();
  }
  iic_write_byte(chData);
  iic_wait_for_ack();
  
  iic_stop();
  
#endif
}   	  

/**
* @brief  OLED turns on 
*         
* @param  None
*         
* @retval None
**/ 
void ssd1306_display_on(void)
{
  ssd1306_write_byte(0x8D, SSD1306_CMD);  
  ssd1306_write_byte(0x14, SSD1306_CMD);  
  ssd1306_write_byte(0xAF, SSD1306_CMD);  
}

/**
* @brief  OLED turns off
*         
* @param  None
*         
* @retval  None
**/
void ssd1306_display_off(void)
{
  ssd1306_write_byte(0x8D, SSD1306_CMD);  
  ssd1306_write_byte(0x10, SSD1306_CMD); 
  ssd1306_write_byte(0xAE, SSD1306_CMD);  
}

/**
* @brief  Refreshs the graphic ram
*         
* @param  None
*         
* @retval  None
**/

void ssd1306_refresh_gram(void)
{
  uint8_t i, j;
  
  for (i = 0; i < 8; i ++) 
  {  
    ssd1306_write_byte(0xB0 + i, SSD1306_CMD);    
    __SET_COL_START_ADDR();  
    
    for (j = 0; j < 128; j ++) {
      ssd1306_write_byte(s_chDispalyBuffer[j][i], SSD1306_DAT); 
    }
  }   
}


/**
* @brief   Clears the screen
*         
* @param  None
*         
* @retval  None
**/

void ssd1306_clear_screen(uint8_t chFill)  
{ 
  uint8_t i, j;
  
  for (i = 0; i < 8; i ++) 
  {
    ssd1306_write_byte(0xB0 + i, SSD1306_CMD);
    __SET_COL_START_ADDR();
    
    for (j = 0; j < 128; j ++) {
      s_chDispalyBuffer[j][i] = chFill;
    }
  }
  
  ssd1306_refresh_gram();
}

/**
* @brief  Draws a piont on the screen
*         
* @param  chXpos: Specifies the X position
* @param  chYpos: Specifies the Y position
* @param  chPoint: 0: the point turns off    1: the piont turns on 
*         
* @retval None
**/

void ssd1306_draw_point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint)
{
  uint8_t chPos, chBx, chTemp = 0;
  
  if (chXpos > 127 || chYpos > 63) {
    return;
  }
  chPos = 7 - chYpos / 8; // 
  chBx = chYpos % 8;
  chTemp = 1 << (7 - chBx);
  
  if (chPoint) {
    s_chDispalyBuffer[chXpos][chPos] |= chTemp;
  } else {
    s_chDispalyBuffer[chXpos][chPos] &= ~chTemp;
  }
}

/**
* @brief  Fills a rectangle
*         
* @param  chXpos1: Specifies the X position 1 (X top left position)
* @param  chYpos1: Specifies the Y position 1 (Y top left position)
* @param  chXpos2: Specifies the X position 2 (X bottom right position)
* @param  chYpos3: Specifies the Y position 2 (Y bottom right position)
*         
* @retval 
**/

void ssd1306_fill_screen(uint8_t chXpos1, uint8_t chYpos1, uint8_t chXpos2, uint8_t chYpos2, uint8_t chDot)  
{  
  uint8_t chXpos, chYpos; 
  
  for (chXpos = chXpos1; chXpos <= chXpos2; chXpos ++) {
    for (chYpos = chYpos1; chYpos <= chYpos2; chYpos ++) {
      ssd1306_draw_point(chXpos, chYpos, chDot);
    }
  }	
  
  ssd1306_refresh_gram();
}


/**
* @brief Displays one character at the specified position    
*         
* @param  chXpos: Specifies the X position
* @param  chYpos: Specifies the Y position
* @param  chSize: 
* @param  chMode
* @retval 
**/
void ssd1306_display_char(uint8_t chXpos, uint8_t chYpos, uint8_t chChr, uint8_t chSize, uint8_t chMode)
{      	
  uint8_t i, j;
  uint8_t chTemp, chYpos0 = chYpos;
  
  chChr = chChr - ' ';				   
  for (i = 0; i < chSize; i ++) 
  {   
    if (chSize == 12) 
    {
      if (chMode) {
        chTemp = c_chFont1206[chChr][i];
      } else {
        chTemp = ~c_chFont1206[chChr][i];
      }
    } else 
    {
      if (chMode) {
        chTemp = c_chFont1608[chChr][i];
      } else {
        chTemp = ~c_chFont1608[chChr][i];
      }
    }
    
    for (j = 0; j < 8; j ++) 
    {
      if (chTemp & 0x80) {
        ssd1306_draw_point(chXpos, chYpos, 1);
      } else {
        ssd1306_draw_point(chXpos, chYpos, 0);
      }
      chTemp <<= 1;
      chYpos ++;
      
      if ((chYpos - chYpos0) == chSize) {
        chYpos = chYpos0;
        chXpos ++;
        break;
      }
    }  	 
  } 
}

static uint32_t pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;	 
  while(n --) result *= m;    
  return result;
}	


void ssd1306_display_num(uint8_t chXpos, uint8_t chYpos, uint32_t chNum, uint8_t chLen, uint8_t chSize)
{         	
  uint8_t i;
  uint8_t chTemp, chShow = 0;
  
  for(i = 0; i < chLen; i ++) {
    chTemp = (chNum / pow(10, chLen - i - 1)) % 10;
    if(chShow == 0 && i < (chLen - 1)) {
      if(chTemp == 0) {
        ssd1306_display_char(chXpos + (chSize / 2) * i, chYpos, ' ', chSize, 1);
        continue;
      } else {
        chShow = 1;
      }	 
    }
    ssd1306_display_char(chXpos + (chSize / 2) * i, chYpos, chTemp + '0', chSize, 1); 
  }
} 


/**
* @brief  Displays a string on the screen
*         
* @param  chXpos: Specifies the X position
* @param  chYpos: Specifies the Y position
* @param  pchString: Pointer to a string to display on the screen 
*         
* @retval  None
**/
void ssd1306_display_string(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString, uint8_t chSize, uint8_t chMode)
{
  while (*pchString != '\0') {       
    if (chXpos > (SSD1306_WIDTH - chSize / 2)) {
      chXpos = 0;
      chYpos += chSize;
      if (chYpos > (SSD1306_HEIGHT - chSize)) {
        chYpos = chXpos = 0;
        ssd1306_clear_screen(0x00);
      }
    }
    
    ssd1306_display_char(chXpos, chYpos, *pchString, chSize, chMode);
    chXpos += chSize / 2;
    pchString ++;
  }
}

void ssd1306_draw_1616char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
  uint8_t i, j;
  uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;
  
  for (i = 0; i < 32; i ++) {
    chTemp = c_chFont1612[chChar - 0x30][i];
    for (j = 0; j < 8; j ++) {
      chMode = chTemp & 0x80? 1 : 0; 
      ssd1306_draw_point(chXpos, chYpos, chMode);
      chTemp <<= 1;
      chYpos ++;
      if ((chYpos - chYpos0) == 16) {
        chYpos = chYpos0;
        chXpos ++;
        break;
      }
    }
  }
}

void ssd1306_draw_3216char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
  uint8_t i, j;
  uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;
  
  for (i = 0; i < 64; i ++) {
    chTemp = c_chFont3216[chChar - 0x30][i];
    for (j = 0; j < 8; j ++) {
      chMode = chTemp & 0x80? 1 : 0; 
      ssd1306_draw_point(chXpos, chYpos, chMode);
      chTemp <<= 1;
      chYpos ++;
      if ((chYpos - chYpos0) == 32) {
        chYpos = chYpos0;
        chXpos ++;
        break;
      }
    }
  }
}

void ssd1306_draw_bitmap(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight)
{
  uint16_t i, j, byteWidth = (chWidth + 7) / 8;
  
  for(j = 0; j < chHeight; j ++){
    for(i = 0; i < chWidth; i ++ ) {
      if(*(pchBmp + j * byteWidth + i / 8) & (128 >> (i & 7))) {
        ssd1306_draw_point(chXpos + i, chYpos + j, 1);
      }
    }
  }
}



/**
* @brief  SSd1306 initialization
*         
* @param  None
*         
* @retval None
**/
void ssd1306_init(void)
{
  
  // Perform a hard reset of the OLED controller
  // This functions will reset the OLED controller by setting the reset pin low.
#ifdef INTERFACE_4WIRE_SPI	  
  __SSD1306_CS_SET();   //CS set
  __SSD1306_DC_CLR();   //D/C reset
  __SSD1306_RES_SET();  //RES set
#elif defined(INTERFACE_3WIRE_SPI)	
  __SSD1306_CS_CLR();   //CS reset
  __SSD1306_DC_CLR();   //D/C reset
  __SSD1306_RES_SET();  //RES set	
#elif defined(INTERFACE_IIC)	  
  //__SSD1306_RES_CLR();  //RES clr
  //delay_us(SSD1306_LATENCY_US); // At least 3us
  //__SSD1306_RES_SET();  //RES set
  //delay_us(SSD1306_LATENCY_US); // At least 3us
#endif
  
  // ATMEL INIT
  //----------------------------------------------------------------------
  // Do a hard reset of the OLED display controller
  
  // Display off
  ssd1306_write_byte(SSD1306_CMD_SET_DISPLAY_OFF, SSD1306_CMD);
  
  // Initialize the interface
  //	ssd1306_interface_init();
  
  // 1/32 Duty (0x0F~0x3F)
  ssd1306_write_byte(SSD1306_CMD_SET_MULTIPLEX_RATIO, SSD1306_CMD);
  //ssd1306_write_byte(0x1F, SSD1306_CMD);
  ssd1306_write_byte(0x0F, SSD1306_CMD);
  
  // Shift Mapping RAM Counter (0x00~0x3F)
  ssd1306_write_byte(SSD1306_CMD_SET_DISPLAY_OFFSET, SSD1306_CMD);
  ssd1306_write_byte(0x00, SSD1306_CMD);
  
  // Set Mapping RAM Display Start Line (0x00~0x3F)
  ssd1306_write_byte(SSD1306_CMD_SET_START_LINE(0x00), SSD1306_CMD);
  
  // Set Column Address 0 Mapped to SEG0
  ssd1306_write_byte(SSD1306_CMD_SET_SEGMENT_RE_MAP_COL127_SEG0, SSD1306_CMD);
  
  // Set COM/Row Scan Scan from COM63 to 0
  ssd1306_write_byte(SSD1306_CMD_SET_COM_OUTPUT_SCAN_DOWN, SSD1306_CMD);
  
  // Set COM Pins hardware configuration
  ssd1306_write_byte(SSD1306_CMD_SET_COM_PINS, SSD1306_CMD);
  ssd1306_write_byte(0x02, SSD1306_CMD);
  
  // Set contrast
  ssd1306_write_byte(SSD1306_CMD_SET_CONTRAST_CONTROL_FOR_BANK0, SSD1306_CMD);
  ssd1306_write_byte(SSD1306_DEFAULT_CONTRAST, SSD1306_CMD); // contrast
  
  // Disable Entire display On
  ssd1306_write_byte(SSD1306_CMD_ENTIRE_DISPLAY_AND_GDDRAM_ON, SSD1306_CMD);
  
  // Display invert disable
  ssd1306_write_byte(SSD1306_CMD_SET_NORMAL_DISPLAY, SSD1306_CMD);
  
  // Display invert enable
  //ssd1306_write_byte(SSD1306_CMD_SET_INVERSE_DISPLAY, SSD1306_CMD);
  
  // Set Display Clock Divide Ratio / Oscillator Frequency (Default => 0x80)
  ssd1306_write_byte(SSD1306_CMD_SET_DISPLAY_CLOCK_DIVIDE_RATIO, SSD1306_CMD);
  ssd1306_write_byte(0x80, SSD1306_CMD);
  
  // Enable charge pump regulator
  ssd1306_write_byte(SSD1306_CMD_SET_CHARGE_PUMP_SETTING, SSD1306_CMD);
  ssd1306_write_byte(0x14, SSD1306_CMD);
  
  // Set VCOMH Deselect Level
  ssd1306_write_byte(SSD1306_CMD_SET_VCOMH_DESELECT_LEVEL, SSD1306_CMD);
  ssd1306_write_byte(0x40, SSD1306_CMD); // Default => 0x20 (0.77*VCC)
  
  // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  ssd1306_write_byte(SSD1306_CMD_SET_PRE_CHARGE_PERIOD, SSD1306_CMD);
  ssd1306_write_byte(0xF1, SSD1306_CMD);
  
  // Display on
  ssd1306_write_byte(SSD1306_CMD_SET_DISPLAY_ON, SSD1306_CMD);
}

// Set current page in display RAM
// This command is usually followed by the configuration of the column address
// because this scheme will provide access to all locations in the display RAM.
void ssd1306_set_page_address(uint8_t address)
{
  // Make sure that the address is 4 bits (only 8 pages)
  address &= 0x0F;
  ssd1306_write_byte(SSD1306_CMD_SET_PAGE_START_ADDRESS(address), SSD1306_CMD);
  
  Page = address;
}

// Set current column in display RAM
void ssd1306_set_column_address(uint8_t address)
{
  // Make sure the address is 7 bits
  address &= SSD1306_WIDTH; //0x7F;
  
  ssd1306_write_byte(SSD1306_CMD_SET_HIGH_COL(address >> 4), SSD1306_CMD);
  ssd1306_write_byte(SSD1306_CMD_SET_LOW_COL(address & 0x0F), SSD1306_CMD);
  
  PosX = address;
}

// Set the display start draw line address
// This function will set which line should be the start draw line for the OLED.
void ssd1306_set_display_start_line_address(uint8_t address)
{ 
  // Make sure address is 6 bits
  address &= 0x3F;
  
  ssd1306_write_byte(SSD1306_CMD_SET_START_LINE(address), SSD1306_CMD);
  
  PosY = address;
}

void ssd1306_set_location (uint8_t x, uint8_t y)
{
  uint8_t page;
  
  page = (y>>3)&0x2F;                  // Page = Y/8 (Y 0..63)
  ssd1306_set_page_address(page);      // Page (0..7)
  ssd1306_set_column_address(x&0x8F);  // X    (0..128), visible (0..95) 
}

void ssd1306_write_text(const char *string)
{
  uint8_t *char_ptr;
  uint8_t i;
  
  while (*string != 0) 
  {
    if (*string < 0x7F) 
    {
      char_ptr = font_table[*string - 32];
      
      for (i = 1; i <= char_ptr[0]; i++) 
      {
        ssd1306_write_byte(char_ptr[i], SSD1306_DAT);
      }
      
      ssd1306_write_byte(0x00, SSD1306_DAT);
    }
    string++;
  }
}

void ssd1306_write_bitmap(const char *bitmap, uint8_t SizeX, uint8_t SizeY)
{
  uint8_t x,y;
  
  for(y = 0; y < (SizeY >> 3); y++)
  {
    ssd1306_set_page_address(y);
    ssd1306_set_column_address(0);
    for(x = 0; x < SizeX; x++)
    {
      ssd1306_write_byte(*bitmap, SSD1306_DAT);
      bitmap++;
    }
    ssd1306_write_byte(0x00, SSD1306_DAT);
  }
}

void ssd1306_write_text_scaled(const char *string, uint8_t ScaleX, uint8_t ScaleY)
{
  uint8_t *char_ptr;
  uint8_t i, CntX;
  /* 
  uint64_t tmp64;
  uint8_t CurPage, CurPosX, CntY, j, k;
  
  union
  {
  uint64_t YBits64;
  uint8_t  Ybytes[8];
}uYScale;
  */ 
  
  while ((*string != 0) && (PosX <= SSD1306_WIDTH))
  {
    if (*string < 0x7F) 
    {
      /*      
      if (ScaleY == 1)
      */        
      {
        
        //======================================================================
        // X-scale only (No Y-scale)
        //======================================================================
        char_ptr = font_table[*string - 32];
        for (i = 1; i <= char_ptr[0]; i++) 
        {
          // X-scaling
          //--------------------------------------------------------------------
          CntX = 0;
          do
          {
            if(PosX <= SSD1306_WIDTH) ssd1306_write_byte(char_ptr[i], SSD1306_DAT);
            CntX++;
            PosX++;
          } while(CntX < ScaleX);
          //--------------------------------------------------------------------
        }
        if(PosX <= SSD1306_WIDTH) ssd1306_write_byte(0x00, SSD1306_DAT);
        //======================================================================
        /*        
      }
        else
        {
        char_ptr = font_table[*string - 32];
        for (i = 1; i <= char_ptr[0]; i++) 
        {
        // X-scaling
        //--------------------------------------------------------------------
        CntX = 0;
        do
        {
        if(PosX <= SSD1306_WIDTH) 
        {
        // Y-scaling
        //----------------------------------------------------------------
        tmp64 = char_ptr[i];
        uYScale.YBits64 = 0;
        for (j=0; j < 8; j++) // bits counter loop
        {
        for (k=0; k < ScaleY; k++) // Y-scaler loop counter
        { 
        if (tmp64 & 0x1ULL) uYScale.YBits64 |= (1ULL << (j*ScaleY + k));
        else                uYScale.YBits64 &= ~(1ULL << (j*ScaleY + k));
      }
        tmp64 = tmp64 >> 1;
      }
        
        CurPage = Page; // Get current page
        CurPosX = PosX; // Get current column
        for (k=0; k < ScaleY; k++)
        {
        //ssd1306_set_page_address(CurPage + k);
        ssd1306_set_column_address(CurPosX);
        ssd1306_write_byte(uYScale.Ybytes[k], SSD1306_DAT);
      }
        Page = CurPage; // Restore current Page
        PosX = CurPosX; // Restore current column
      }
        CntX++;
        PosX++;
      }while(CntX < ScaleX);
        //----------------------------------------------------------------------
      }
        
        CurPage = Page; // Get current page
        CurPosX = PosX; // Get current column
        for (k=0; k < ScaleY; k++)
        {
        ssd1306_set_page_address(CurPage + k);
        ssd1306_set_column_address(CurPosX);
        if(PosX <= SSD1306_WIDTH) ssd1306_write_byte(0x00, SSD1306_DAT);
      }
        Page = CurPage; // Restore current Page
        PosX = CurPosX; // Restore current column
        */         
      }
    }
    string++;
  }
}



/*-------------------------------END OF FILE-------------------------------*/
