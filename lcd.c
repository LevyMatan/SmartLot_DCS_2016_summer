//**********************************************************************//*****
//  MSP-EXP430F5438 Experimenter's Board - User Experience Demo
//
//  Main Project File: UserExperience.c
//
//  D. Dang
//  Texas Instruments Inc.
//  Ver 1.00 - May 2008
//  Built with Code Composer Essentials CCE v3.2
//*****************************************************************************
#include  <msp430x54xA.h>

#ifndef HAL_MSP_EXP430F5438_H
#define HAL_MSP_EXP430F5438_H

/*************************************************************************************/
/*******************************************************************************
    Filename: hal_lcd.h

    Copyright 2008 Texas Instruments, Inc.
***************************************************************************/
#ifndef HAL_LCD_H
#define HAL_LCD_H

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif

#define LCD_COMM_OUT        P8OUT
#define LCD_COMM_DIR        P8DIR
#define LCD_COMM_SEL        P8SEL
#define LCD_BACKLIGHT_PIN   BIT3

#ifdef REV_02
  #define LCD_CS_RST_DIR      LCD_COMM_DIR
  #define LCD_CS_RST_OUT      LCD_COMM_OUT
  #define LCD_CS_PIN          BIT1
  #define LCD_RESET_PIN       BIT2
#else 
  #define LCD_CS_RST_DIR      P9DIR
  #define LCD_CS_RST_OUT      P9OUT  
  #define LCD_CS_PIN          BIT6 
  #define LCD_RESET_PIN       BIT7
#endif 

#define LCD_ROW                 110
#define LCD_COL                 138
#define LCD_Size                3505
#define LCD_MEM_Size            110*17
#define LCD_Max_Column_Offset   0x10  
 
#define LCD_Last_Pixel          3505

#define LCD_MEM_Row             0x11
#define LCD_Row                 0x20

// Grayscale level definitions
#define PIXEL_OFF               0
#define PIXEL_LIGHT             1
#define PIXEL_DARK              2
#define PIXEL_ON                3

#define INVERT_TEXT             BIT0
#define OVERWRITE_TEXT          BIT2

/*-------------------------------------------------------------
 *                  Function Prototypes 
 * ------------------------------------------------------------*/ 
void halLcdInit(void);                   
void halLcdShutDown(void);
void halLcdBackLightInit(void);
void halLcdSetBackLight(unsigned char BackLightLevel);
unsigned int halLcdGetBackLight(void);
void halLcdShutDownBackLight(void);
void halLcdSendCommand(unsigned char Data[]) ;
void halLcdSetContrast(unsigned char ContrastLevel);
unsigned char halLcdGetContrast(void);
void halLcdStandby(void);
void halLcdActive(void);

//Move to specified LCD address
void halLcdSetAddress(int Address);          

//Draw at current segment location
void halLcdDrawCurrentBlock(unsigned int Value);           

//Draw at specified location by calling
//LCD_Set_Address(Address) & LCD_Draw_Current_Block( value )
void halLcdDrawBlock(unsigned int Address, unsigned int Value); 

//Read value from LCD CGRAM
int halLcdReadBlock(unsigned int Address);

//Clear LCD Screen  
void halLcdClearScreen(void);                    

//Invert black to white and vice versa
void halLcdReverse(void);

// Draw a Pixel @ (x,y) with GrayScale level
void halLcdPixel(  int x,  int y, unsigned char GrayScale);
//Draw Line from (x1,y1) to (x2,y2) with GrayScale level
void halLcdLine(  int x1,  int y1,  int x2,  int y2, unsigned char GrayScale); 

void halLcdCircle(int x, int y, int Radius, int GrayScale);

void halLcdImage(const unsigned int Image[], int Columns, int Rows, int x, int y);
void halLcdClearImage(int Columns, int Rows,  int x, int y);

//Print String of Length starting at current LCD location
void halLcdPrint( char String[], unsigned char TextStyle) ;

//Print String of Length starting at (x,y)
void halLcdPrintXY(char String[], int x, int y, unsigned char TextStyle);  

//Print String of Length starting at (x,y)
void halLcdPrintLine(char String[], unsigned char Line, unsigned char TextStyle);  
void halLcdPrintLineCol(char String[], unsigned char Line, unsigned char Col, unsigned char TextStyle);  

void halLcdCursor(void);
void halLcdCursorOff(void);
//Scroll a single row of pixels
void halLcdScrollRow(int y);
//Scroll a number of consecutive rows from yStart to yEnd
void halLcdHScroll(int yStart, int yEnd);
//Scroll a line of text
void halLcdScrollLine(int Line);

#endif

/*************************************************************************************/



/*************************************************************************************/
/**********************************************************************//**
    Filename: hal_board.h

    Copyright 2008 Texas Instruments, Inc.
***************************************************************************/
#ifndef HAL_BOARD_H
#define HAL_BOARD_H

#define LED_PORT_DIR      P1DIR
#define LED_PORT_OUT      P1OUT
#define LED_1             BIT0
#define LED_2             BIT1

#define CLK_PORT_DIR      P11DIR
#define CLK_PORT_OUT      P11OUT
#define CLK_PORT_SEL      P11SEL

#define ACLK_PIN          BIT0
#define MCLK_PIN          BIT1
#define SMCLK_PIN         BIT2

#define XT1_XTAL_DIR      P7DIR
#define XT1_XTAL_SEL      P7SEL
#define XT1_XTAL_OUT      P7OUT

#define SYSCLK_1MHZ             0
#define SYSCLK_4MHZ             1
#define SYSCLK_8MHZ             2
#define SYSCLK_12MHZ            3
#define SYSCLK_16MHZ            4
#define SYSCLK_20MHZ            5
#define SYSCLK_25MHZ            6

#define DCO_MULT_1MHZ           30
#define DCO_MULT_4MHZ           122
#define DCO_MULT_8MHZ           244
#define DCO_MULT_12MHZ          366
#define DCO_MULT_16MHZ          488
#define DCO_MULT_20MHZ          610
#define DCO_MULT_25MHZ          763

#define DCORSEL_1MHZ            DCORSEL_2
#define DCORSEL_4MHZ            DCORSEL_4
#define DCORSEL_8MHZ            DCORSEL_4
#define DCORSEL_12MHZ           DCORSEL_5
#define DCORSEL_16MHZ           DCORSEL_5
#define DCORSEL_20MHZ           DCORSEL_6
#define DCORSEL_25MHZ           DCORSEL_7

#define VCORE_1MHZ              PMMCOREV_0
#define VCORE_4MHZ              PMMCOREV_0
#define VCORE_8MHZ              PMMCOREV_0
#define VCORE_12MHZ             PMMCOREV_1
#define VCORE_16MHZ             PMMCOREV_1
#define VCORE_20MHZ             PMMCOREV_2
#define VCORE_25MHZ             PMMCOREV_3

#define VCORE_1_35V             PMMCOREV_0
#define VCORE_1_55V             PMMCOREV_1
#define VCORE_1_75V             PMMCOREV_2
#define VCORE_1_85V             PMMCOREV_3

/*----------------------------------------------------------------
 *                  Function Prototypes
 *----------------------------------------------------------------
 */
static void halBoardSetVCoreUp(unsigned char level);
static void halBoardSetVCoreDown(unsigned char level);
static void halBoardGetSystemClockSettings(unsigned char systemClockSpeed, 
                                           unsigned char *setDcoRange,
                                           unsigned char *setVCore,
                                           unsigned int  *setMultiplier);

void halBoardSetVCore(unsigned char level);
void halBoardDisableSVS(void);
void halBoardEnableSVS(void);
void halBoardStartXT1(void);
void halBoardSetSystemClock(unsigned char systemClockSpeed);
void halBoardOutputSystemClock(void);
void halBoardStopOutputSystemClock(void);
void halBoardInit(void);

#endif
/*************************************************************************************/


#endif
/*************************************************************************************/


//#include "UserExperienceGraphics.h"



/**********************************************************************//**
    Filename: flashUtils.h

    Copyright 2008 Texas Instruments, Inc.
***************************************************************************/
#ifndef FLASH_UTILS_H
#define FLASH_UTILS_H
#include <msp430x54xA.h>

/*-------------------------------------------------------------
 *                       Macros 
 * ------------------------------------------------------------*/ 
#define FLASH_UNLOCK    FCTL3 = FWKEY; FCTL1 = FWKEY + WRT; 
#define FLASH_LOCK      FCTL1 = FWKEY; FCTL3 = FWKEY +  LOCK; 
#define FLASH_WRITE_INFOB(destination, source) st( FLASH_UNLOCK_INFOB;  destination = source; while (FCTL3 & BUSY ); FLASH_LOCK_INFOB; )  

/*-------------------------------------------------------------
 *                  Function Prototypes 
 * ------------------------------------------------------------*/ 
void flashEraseBank(unsigned long FarPtr);
void flashEraseSegment(unsigned long FarPtr);
void flashErase(unsigned long Memstart, unsigned long Memend);

#endif

/*************************************************************************************/







/** 
 * @file  hal_board.c
 * 
 * Copyright 2008 Texas Instruments, Inc.
******************************************************************************/
//#include "MSP430x54x.h"
//#include "hal_MSP-EXP430F5438.h"

/*-------------------------------------------------------------------------*/
/**********************************************************************//**
 * @brief  Increments the VCore setting.
 * 
 * @param  level The target VCore setting
 * 
 * @return none
 *************************************************************************/
static void halBoardSetVCoreUp (unsigned char level)
{
  // Open PMM module registers for write access 
  PMMCTL0_H = 0xA5;                         
  
  // Set SVS/M high side to new level
  SVSMHCTL = (SVSMHCTL & ~(SVSHRVL0*3 + SVSMHRRL0)) | \
             (SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level); 
  
  // Set SVM new Level    
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;       
  // Set SVS/M low side to new level
  SVSMLCTL = (SVSMLCTL & ~(SVSMLRRL_3)) | (SVMLE + SVSMLRRL0 * level);
   
  while ((PMMIFG & SVSMLDLYIFG) == 0);      // Wait till SVM is settled (Delay)
  PMMCTL0_L = PMMCOREV0 * level;            // Set VCore to x
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);        // Clear already set flags
  
  if ((PMMIFG & SVMLIFG))
    while ((PMMIFG & SVMLVLRIFG) == 0);     // Wait till level is reached
  
  // Set SVS/M Low side to new level
  SVSMLCTL = (SVSMLCTL & ~(SVSLRVL0*3 + SVSMLRRL_3)) | \
             (SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level);
  
  // Lock PMM module registers from write access
  PMMCTL0_H = 0x00;                         
}

/**********************************************************************//**
 * @brief  Decrements the VCore setting.
 * 
 * @param  level The target VCore.  
 * 
 * @return none
 *************************************************************************/
static void halBoardSetVCoreDown(unsigned char level)
{
  // Open PMM module registers for write access
  PMMCTL0_H = 0xA5;                         
  
  // Set SVS/M low side to new level
  SVSMLCTL = (SVSMLCTL & ~(SVSLRVL0*3 + SVSMLRRL_3)) | \
             (SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level); 
  
  while ((PMMIFG & SVSMLDLYIFG) == 0);      // Wait till SVM is settled (Delay)
  PMMCTL0_L = (level * PMMCOREV0);          // Set VCore to new level 
  // Lock PMM module registers for write access
  
  PMMCTL0_H = 0x00;                         
}

/**********************************************************************//**
 * @brief  Get function for the DCORSEL, VCORE, and DCO multiplier settings 
 *         that map to a given clock speed. 
 * 
 * @param  systemClockSpeed Target DCO frequency - SYSCLK_xxMHZ.
 * 
 * @param  setDcoRange      Pointer to the DCO range select bits.
 * 
 * @param  setVCore         Pointer to the VCore level bits. 
 * 
 * @param  setMultiplier    Pointer to the DCO multiplier bits. 
 *
 * @return none
 ************************************************************************/
static void halBoardGetSystemClockSettings(unsigned char systemClockSpeed, 
                                    unsigned char *setDcoRange,
                                    unsigned char *setVCore,
                                    unsigned int  *setMultiplier)
{
  switch (systemClockSpeed)
  {
  case SYSCLK_1MHZ: 
    *setDcoRange = DCORSEL_1MHZ;
    *setVCore = VCORE_1MHZ;
    *setMultiplier = DCO_MULT_1MHZ;
    break;
  case SYSCLK_4MHZ: 
    *setDcoRange = DCORSEL_4MHZ;
    *setVCore = VCORE_4MHZ;
    *setMultiplier = DCO_MULT_4MHZ;
    break;
  case SYSCLK_8MHZ: 
    *setDcoRange = DCORSEL_8MHZ;
    *setVCore = VCORE_8MHZ;
    *setMultiplier = DCO_MULT_8MHZ;
    break;
  case SYSCLK_12MHZ: 
    *setDcoRange = DCORSEL_12MHZ;
    *setVCore = VCORE_12MHZ;
    *setMultiplier = DCO_MULT_12MHZ;
    break;
  case SYSCLK_16MHZ: 
    *setDcoRange = DCORSEL_16MHZ;
    *setVCore = VCORE_16MHZ;
    *setMultiplier = DCO_MULT_16MHZ;
    break;
/*------------------------------------- 
 * Commented out because fmax = 18 MHz 
 * ------------------------------------
  case SYSCLK_20MHZ: 
    *setDcoRange = DCORSEL_20MHZ;
    *setVCore = VCORE_20MHZ;
    *setMultiplier = DCO_MULT_20MHZ;
    break;
  case SYSCLK_25MHZ: 
    *setDcoRange = DCORSEL_25MHZ;
    *setVCore = VCORE_25MHZ;
    *setMultiplier = DCO_MULT_25MHZ;
    break;
 *-------------------------------------*/	     
  }	
}

/*-------------------------------------------------------------------------*/
/**********************************************************************//**
 * @brief  Set function for the PMM core voltage (PMMCOREV) setting
 * 
 * @param  level Target VCore setting 
 * 
 * @return none
 *************************************************************************/
void halBoardSetVCore(unsigned char level)
{
  unsigned int currentVCore;

  currentVCore = PMMCTL0 & PMMCOREV_3;      // Get actual VCore  
  // Change VCore step by step 
  while (level != currentVCore) 
  {   
    if (level > currentVCore) 
      halBoardSetVCoreUp(++currentVCore);
    else
      halBoardSetVCoreDown(--currentVCore);
  }
}

/**********************************************************************//**
 * @brief  Disables all supply voltage supervision and monitoring. 
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halBoardDisableSVS(void)
{
  // Open PMM module registers for write access
  PMMCTL0_H = 0xA5;              
             
  SVSMLCTL &= ~( SVMLE + SVSLE + SVSLFP + SVMLFP );   // Disable Low side SVM
  SVSMHCTL &= ~( SVMHE + SVSHE + SVSHFP + SVMHFP );   // Disable High side SVM
  PMMCTL1 = PMMREFMD;
  
  // Lock PMM module registers for write access
  PMMCTL0_H = 0x00;                         	
}

/**********************************************************************//**
 * @brief  Enables all supply voltage supervision and monitoring
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halBoardEnableSVS(void)
{
  // Open PMM module registers for write access
  PMMCTL0_H = 0xA5;                         
  
  /*-----------
   * NOTE: To attain the expected < 6 us wakeup from LPM modes, the following
   * two lines must be commented out due to the fact that the PMM will hold
   * the CPU until the reference is fully settled.  
   *----------*/
  SVSMHCTL &= ~(SVSHFP+SVMHFP);             // Disable full-performance mode 
  SVSMLCTL &= ~(SVSLFP+SVMLFP);             // Disable full-performance mode
  SVSMLCTL |= ( SVMLE + SVSLE);             // Enable Low side SVM
  SVSMHCTL |= ( SVMHE + SVSHE);             // Enable High side SVM
  PMMCTL1 &= ~PMMREFMD; 
  
  // Lock PMM module registers for write access  
  PMMCTL0_H = 0x00;                         
}

/**********************************************************************//**
 * @brief  Initialization routine for XT1. 
 * 
 * Sets the necessary internal capacitor values and loops until all 
 * ocillator fault flags remain cleared. 
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halBoardStartXT1(void)
{
  // Set up XT1 Pins to analog function, and to lowest drive	
  P7SEL |= 0x03;                            
  UCSCTL6 |= XCAP_3 ;                       // Set internal cap values
  
  while(SFRIFG1 & OFIFG) {                  // Check OFIFG fault flag
    while ( (SFRIFG1 & OFIFG))              // Check OFIFG fault flag
    {    
      // Clear OSC fault flags 
      UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT1HFOFFG + XT2OFFG);
      SFRIFG1 &= ~OFIFG;                    // Clear OFIFG fault flag
    }
    UCSCTL6 &= ~(XT1DRIVE1_L+XT1DRIVE0);    // Reduce the drive strength 
  }
}

/**********************************************************************//**
 * @brief  Set function for MCLK frequency.
 * 
 * @param  systemClockSpeed Intended frequency of operation - SYSCLK_xxMHZ.
 * 
 * @return none
 *************************************************************************/
void halBoardSetSystemClock(unsigned char systemClockSpeed)
{
  unsigned char setDcoRange, setVCore;
  unsigned int  setMultiplier;

  halBoardGetSystemClockSettings( systemClockSpeed, &setDcoRange,  \
                                  &setVCore, &setMultiplier);
  	
  if (setVCore > (PMMCTL0 & PMMCOREV_3))	// Only change VCore if necessary
    halBoardSetVCore( setVCore );   
    
  UCSCTL0 = 0x00;                           // Set lowest possible DCOx, MODx
  UCSCTL1 = setDcoRange;                    // Select suitable range
  
  UCSCTL2 = setMultiplier + FLLD_1;         // Set DCO Multiplier
  UCSCTL4 = SELA__XT1CLK | SELS__DCOCLKDIV  |  SELM__DCOCLKDIV ;
  
  // Worst-case settling time for the DCO when the DCO range bits have been 
  // changed is n x 32 x 32 x f_FLL_reference. See UCS chapter in 5xx UG 
  // for optimization.
  // 32 x 32 x 1 / f_FLL_reference (32,768 Hz) = .03125 = t_DCO_settle
  // t_DCO_settle / (1 / 18 MHz) = 562500 = counts_DCO_settle
  __delay_cycles(562500);  
}

/**********************************************************************//**
 * @brief  Initializes ACLK, MCLK, SMCLK outputs on P11.0, P11.1, 
 *         and P11.2, respectively.
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halBoardOutputSystemClock(void)
{
  P11DIR |= 0x07;
  P11SEL |= 0x07;                           
}

/**********************************************************************//**
 * @brief  Stops the output of ACLK, MCLK, SMCLK on P11.0, P11.1, and P11.2.
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halBoardStopOutputSystemClock(void)
{  
  P11OUT &= ~0x07;
  P11DIR |= 0x07;	
  P11SEL &= ~0x07;                 
}

/**********************************************************************//**
 * @brief  Initializes all GPIO configurations. 
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halBoardInit(void)
{  
  //Tie unused ports
  PAOUT  = 0;
  PADIR  = 0xFFFF;  
  PASEL  = 0;
  PBOUT  = 0;  
  PBDIR  = 0xFFFF;
  PBSEL  = 0;
  PCOUT  = 0;    
  PCDIR  = 0xFFFF;
  PCSEL  = 0;  
  PDOUT  = 0;  
  PDDIR  = 0xFFFF;
  PDSEL  = 0;  
  PEOUT  = 0;  
  PEDIR  = 0xFEFF;                          // P10.0 to USB RST pin, 
                                            // ...if enabled with J5
  PESEL  = 0;  
  P11OUT = 0;
  P11DIR = 0xFF;
  PJOUT  = 0;    
  PJDIR  = 0xFF;
  P11SEL = 0;
     
 // AUDIO_PORT_OUT = AUDIO_OUT_PWR_PIN ;
  //USB_PORT_DIR &= ~USB_PIN_RXD;             // USB RX Pin, Input with 
                                            // ...pulled down Resistor
  //USB_PORT_OUT &= ~USB_PIN_RXD;
  //USB_PORT_REN |= USB_PIN_RXD;
}

/****************************************************************************/


/**********************************************************************//**
 *  @file flashUtils.c
 * 
 * Copyright 2008 Texas Instruments, Inc.
 **************************************************************************/

//#include "msp430x54x.h"
//#include "flashUtils.h"

/**********************************************************************//**
 * @brief  Erases a bank in flash memory.
 * 
 * @param  FarPtr The pointer to the location in memory within the bank to
 *                be erased.
 * 
 * @return none
 **************************************************************************/
void flashEraseBank( unsigned long FarPtr)
{
  FCTL3 = FWKEY;
  while (FCTL3 & BUSY );
  FCTL1 = FWKEY + MERAS;
  
  __data20_write_char(FarPtr, 0x00);        // Dummy write to start erase

  while (FCTL3 & BUSY );
  FCTL1 = FWKEY;                
  FCTL3 = FWKEY +  LOCK;
}

/**********************************************************************//**
 * @brief  Erases a single segment of memory containing the address FarPtr. 
 * 
 * @param  FarPtr The address location within the segment of memory to be 
 *                erased. 
 * 
 * @return none
 **************************************************************************/
void flashEraseSegment(unsigned long FarPtr)
{
  FCTL3 = FWKEY;
  FCTL1 = FWKEY + ERASE;
  
  __data20_write_char(FarPtr, 0x00);        // Dummy write to start erase
  
  while (FCTL3 & BUSY );
  FCTL1 = FWKEY;                
  FCTL3 = FWKEY +  LOCK;
}

/**********************************************************************//**
 * @brief  Erases the (multiple) segment(s) of memory including address 
 *         locations Memstart through Memend.
 * 
 * @param  Memstart The starting address location identifying a segment of
 *                  memory to be erased
 * 
 * @param  Memend   The ending address location identifying a segment of
 *                  memory to be erased
 * 
 * @return none
 **************************************************************************/
void flashErase(unsigned long Memstart, unsigned long Memend)
{
  unsigned long FarPtr = Memstart;          // Start of record memory array
  
  FCTL3 = FWKEY;                            // Unlock Flash memory for write
  do
  {
    if ( (FarPtr & 0xFFFF) == 0x0000)       // Use bit 12 to toggle LED
      P1OUT ^= 0x01;

    FCTL1 = FWKEY + ERASE;
    
    __data20_write_char(FarPtr, 0x00);      // Dummy write to activate
    
    while (FCTL3 & BUSY );                  // Segment erase
    FarPtr += 0x0200;                       // Point to next segment
  } while (FarPtr < Memend);
  
  FCTL1 = FWKEY;                
  FCTL3 = FWKEY +  LOCK;
}








/****************************************************************************/


/*******************************************************************************/

/*			- lowinit.c -

  The function __low_level_init is called by the start-up code before doing
  the normal initialization of data segments. If the return value is zero,
  initialization is not performed.

  In the run-time library there is a dummy __low_level_init, which does
  nothing but return 1. This means that the start-up routine proceeds with
  initialization of data segments.

  To replace this dummy, compile a customized version (like the example
  below) and link it with the rest of your code.

 */

/*
 * $Revision: 4055 $
 */
//#include "msp430x54x.h"
int __low_level_init(void)
{
  WDTCTL = WDTPW + WDTHOLD; 

  /*==================================*/
  /* Choose if segment initialization */
  /* should be done or not.		*/
  /* Return: 0 to omit seg_init	*/
  /*	       1 to run seg_init	*/
  /*==================================*/
  return (1);
}







/*****************************************************************************/
/*****************************************************************************/
/** 
 * @file  hal_lcd.c
 * 
 * Copyright 2008 Texas Instruments, Inc.
***************************************************************************/

//#include  <msp430x54x.h>
//#include "hal_MSP-EXP430F5438.h"
//#include "hal_lcd_fonts.h"

/*******************************************************************************
    Filename: hal_lcd_fonts.h

    Copyright 2008 Texas Instruments, Inc.
***************************************************************************/
#ifndef FONTS_H
#define FONTS_H

#define FONT_HEIGHT		12                    // Each character has 13 lines 

const unsigned char fonts_lookup[]={
          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,              
          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,              
          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,              
          0x00,0x00,63,0x00,0x00,0x00,0x00,0x00,0x00,0x00,              
          64,65,0,69,0,68,67,0,0,1,         //'0' = 48 = 0x30
          2,3,4,5,6,7,8,9,66,0,             //'9' = 57 = 0x39
          0,70,0,62,0,10,11,12,13,14,       //'A' --> 'Z'
          15,16,17,18,19,20,21,22,23,24,
          25,26,27,28,29,30,31,32,33,34, 
          35,0,0,0,71,0,0,36,37,38,         //'a' = 97 
          39,40,41,42,43,44,45,46,47,48,
          49,50,51,52,53,54,55,56,57,58,
          59,60,61,62,0 ,0, 0, 72,73,74,
          75,76,77,78,79,80,81              //'z' = 122
          };            

const unsigned int fonts[]= {
          0x0000, 0x0ffc, 0x3c0f, 0x3f0f, 0x3fcf, 0x3ccf, 0x3cff, 0x3c3f, 
		  0x3c0f, 0x0ffc, 0x0000, 0x0000, 0x0000, 0x0000, 0x00c0, 0x00f0,        
          0x00ff, 0x00f0, 0x00f0, 0x00f0, 0x00f0, 0x00f0, 0x0fff, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f00, 0x03c0, 
          0x00f0, 0x003c, 0x0f0f, 0x0fff, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x03fc, 0x0f0f, 0x0f00, 0x0f00, 0x03f0, 0x0f00, 0x0f00, 0x0f0f, 
          0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f00, 0x0fc0, 0x0ff0, 
          0x0f3c, 0x0f0f, 0x3fff, 0x0f00, 0x0f00, 0x3fc0, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0fff, 0x000f, 0x000f, 0x000f, 0x03ff, 0x0f00, 
          0x0f00, 0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x03f0, 
          0x003c, 0x000f, 0x000f, 0x03ff, 0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x3fff, 0x3c0f, 0x3c0f, 0x3c00, 
          0x0f00, 0x03c0, 0x00f0, 0x00f0, 0x00f0, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f3f, 0x03fc, 0x0fcf, 0x0f0f, 
          0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 
          0x0f0f, 0x0f0f, 0x0ffc, 0x03c0, 0x03c0, 0x00f0, 0x00fc, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x00f0, 0x03fc, 0x0f0f, 0x0f0f, 0x0f0f, 
          0x0fff, 0x0f0f, 0x0f0f, 0x0f0f, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0fff, 0x3c3c, 0x3c3c, 0x3c3c, 0x0ffc, 0x3c3c, 0x3c3c, 0x3c3c, 
          0x0fff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0ff0, 0x3c3c, 0x3c0f, 
          0x000f, 0x000f, 0x000f, 0x3c0f, 0x3c3c, 0x0ff0, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x03ff, 0x0f3c, 0x3c3c, 0x3c3c, 0x3c3c, 0x3c3c, 
          0x3c3c, 0x0f3c, 0x03ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x3fff, 
          0x303c, 0x003c, 0x0c3c, 0x0ffc, 0x0c3c, 0x003c, 0x303c, 0x3fff, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x3fff, 0x3c3c, 0x303c, 0x0c3c, 
          0x0ffc, 0x0c3c, 0x003c, 0x003c, 0x00ff, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0ff0, 0x3c3c, 0x3c0f, 0x000f, 0x000f, 0x3f0f, 0x3c0f, 
          0x3c3c, 0x3ff0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f, 
          0x0f0f, 0x0f0f, 0x0fff, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x03fc, 0x00f0, 0x00f0, 0x00f0, 0x00f0, 
          0x00f0, 0x00f0, 0x00f0, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x3fc0, 0x0f00, 0x0f00, 0x0f00, 0x0f00, 0x0f0f, 0x0f0f, 0x0f0f, 
          0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x3c3f, 0x3c3c, 0x0f3c, 
          0x0f3c, 0x03fc, 0x0f3c, 0x0f3c, 0x3c3c, 0x3c3f, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x00ff, 0x003c, 0x003c, 0x003c, 0x003c, 0x303c, 
          0x3c3c, 0x3c3c, 0x3fff, 0x0000, 0x0000, 0x0000, 0x0000, 0x3c0f, 
          0x3f3f, 0x3fff, 0x3fff, 0x3ccf, 0x3c0f, 0x3c0f, 0x3c0f, 0x3c0f, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x3c0f, 0x3c0f, 0x3c3f, 0x3cff, 
          0x3fff, 0x3fcf, 0x3f0f, 0x3c0f, 0x3c0f, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x03f0, 0x0f3c, 0x3c0f, 0x3c0f, 0x3c0f, 0x3c0f, 0x3c0f, 
          0x0f3c, 0x03f0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0fff, 0x3c3c, 
          0x3c3c, 0x3c3c, 0x0ffc, 0x003c, 0x003c, 0x003c, 0x00ff, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x03f0, 0x0f3c, 0x3c0f, 0x3c0f, 0x3c0f, 
          0x3f0f, 0x3fcf, 0x0ffc, 0x0f00, 0x3fc0, 0x0000, 0x0000, 0x0000, 
          0x0fff, 0x3c3c, 0x3c3c, 0x3c3c, 0x0ffc, 0x0f3c, 0x3c3c, 0x3c3c, 
          0x3c3f, 0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 0x0f0f, 
          0x000f, 0x00fc, 0x03c0, 0x0f0f, 0x0f0f, 0x03fc, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0fff, 0x0cf3, 0x00f0, 0x00f0, 0x00f0, 0x00f0, 
          0x00f0, 0x00f0, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 
          0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 
          0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 0x00f0, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x3c0f, 0x3c0f, 0x3c0f, 0x3c0f, 0x3ccf, 0x3ccf, 0x0f3c, 
          0x0f3c, 0x0f3c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f, 
          0x0f0f, 0x03fc, 0x00f0, 0x03fc, 0x0f0f, 0x0f0f, 0x0f0f, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 
          0x00f0, 0x00f0, 0x00f0, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x3fff, 0x3f0f, 0x03c3, 0x03c0, 0x00f0, 0x003c, 0x303c, 0x3c0f, 
          0x3fff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x03fc, 0x0f00, 0x0ffc, 0x0f0f, 0x0f0f, 0x3cfc, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x003f, 0x003c, 0x003c, 0x0ffc, 0x3c3c, 0x3c3c, 
          0x3c3c, 0x3c3c, 0x0fcf, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x03fc, 0x0f0f, 0x000f, 0x000f, 0x0f0f, 0x03fc, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0fc0, 0x0f00, 0x0f00, 0x0ffc, 
          0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x3cfc, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 0x0fff, 0x000f, 
          0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x03f0, 0x0f3c, 
          0x003c, 0x003c, 0x03ff, 0x003c, 0x003c, 0x003c, 0x00ff, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3cfc, 0x0f0f, 
          0x0f0f, 0x0f0f, 0x0ffc, 0x0f00, 0x0f0f, 0x03fc, 0x0000, 0x0000, 
          0x003f, 0x003c, 0x003c, 0x0f3c, 0x3cfc, 0x3c3c, 0x3c3c, 0x3c3c, 
          0x3c3f, 0x0000, 0x0000, 0x0000, 0x0000, 0x03c0, 0x03c0, 0x0000, 
          0x03fc, 0x03c0, 0x03c0, 0x03c0, 0x03c0, 0x3ffc, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0f00, 0x0f00, 0x0000, 0x0ff0, 0x0f00, 0x0f00, 
          0x0f00, 0x0f00, 0x0f0f, 0x0f0f, 0x03fc, 0x0000, 0x0000, 0x003f, 
          0x003c, 0x003c, 0x3c3c, 0x0f3c, 0x03fc, 0x0f3c, 0x3c3c, 0x3c3f, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x03c0, 0x03c0, 0x03c0, 
          0x03c0, 0x03c0, 0x03c0, 0x03c0, 0x3ffc, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0fff, 0x3ccf, 0x3ccf, 0x3ccf, 
          0x3ccf, 0x3c0f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x03ff, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 
          0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0fcf, 0x3c3c, 0x3c3c, 0x3c3c, 0x3c3c, 
          0x0ffc, 0x003c, 0x00ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x3cfc, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0ffc, 0x0f00, 0x3fc0, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f3f, 0x3f3c, 0x3cfc, 
          0x003c, 0x003c, 0x00ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x03fc, 0x0f0f, 0x003c, 0x03c0, 0x0f0f, 0x03fc, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0030, 0x003c, 0x0fff, 
          0x003c, 0x003c, 0x003c, 0x0f3c, 0x03f0, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 
          0x0f0f, 0x3cfc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 0x00f0, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3c0f, 0x3c0f, 
          0x3ccf, 0x3ccf, 0x0f3c, 0x0f3c, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x3c0f, 0x0f3c, 0x03f0, 0x03f0, 0x0f3c, 
          0x3c0f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x3c3c, 0x3c3c, 0x3c3c, 0x3c3c, 0x0ff0, 0x0f00, 0x03c0, 0x00ff, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0fff, 0x0f03, 0x03c0, 
          0x003c, 0x0c0f, 0x0fff, 0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 
          0x0f0f, 0x0f00, 0x03c0, 0x00f0, 0x00f0, 0x0000, 0x00f0, 0x00f0, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0f00, 0x03c0, 0x00f0, 0x003c, 0x003c, 0x003c, 0x00f0, 
          0x03c0, 0x0f00, 0x0000, 0x0000, 0x0000, 0x0000, 0x003c, 0x00f0, 
          0x03c0, 0x0f00, 0x0f00, 0x0f00, 0x03c0, 0x00f0, 0x003c, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03f0, 0x03f0, 0x0000, 
          0x0000, 0x03f0, 0x03f0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03f0, 
          0x03f0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x3ffc, 0x3ffc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x03c0, 0x03c0, 0x3ffc, 0x3ffc, 
          0x03c0, 0x03c0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x3ffc, 0x0000, 0x0000, 0x3ffc, 0x0000, 0x0000, 
          0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f0f, 
          0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
//0---------------------------
          0x0000, 0x0ffc, 0x3c0f, 0x3f0f, 0x3fcf, 0x3ccf, 0x3cff, 0x3c3f, 
          0x3c0f, 0x0ffc, 0x0000, 0x0000, 0x0000, 
//1---------------------------        
          0x0000, 0x00c0, 0x00f0, 0x00ff, 0x00f0, 0x00f0, 0x00f0, 0x00f0, 
          0x00f0, 0x0fff, 0x0000, 0x0000, 0x0000, 
//2---------------------------          
          0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f00, 0x03c0, 0x00f0, 0x003c, 
          0x0f0f, 0x0fff, 0x0000, 0x0000, 0x0000, 
//3---------------------------        
          0x0000, 0x03fc, 0x0f0f, 0x0f00, 0x0f00, 0x03f0, 0x0f00, 0x0f00, 
          0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 
//4---------------------------
          0x0000, 0x0f00, 0x0fc0, 0x0ff0, 0x0f3c, 0x0f0f, 0x3fff, 0x0f00, 
          0x0f00, 0x3fc0, 0x0000, 0x0000, 0x0000, 
//5---------------------------          
          0x0000, 0x0fff, 0x000f, 0x000f, 0x000f, 0x03ff, 0x0f00, 0x0f00, 
          0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 
//6---------------------------          
          0x0000, 0x03f0, 0x003c, 0x000f, 0x000f, 0x03ff, 0x0f0f, 0x0f0f, 
          0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000,           
//7---------------------------          
          0x0000, 0x3fff, 0x3c0f, 0x3c0f, 0x3c00, 0x0f00, 0x03c0, 0x00f0, 
          0x00f0, 0x00f0, 0x0000, 0x0000, 0x0000, 
//8---------------------------          
          0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f3f, 0x03fc, 0x0fcf, 0x0f0f, 
          0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 
//9---------------------------          
          0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f0f, 0x0ffc, 0x03c0, 0x03c0, 
          0x00f0, 0x00fc, 0x0000, 0x0000, 0x0000,          
} ;
 
#endif        







/****************************************************************************/
unsigned char LcdInitMacro[]={
            0x74,0x00,0x00,0x76,0x00,0x01,  // R00 start oscillation
            0x74,0x00,0x01,0x76,0x00,0x0D,  // R01 driver output control
            0x74,0x00,0x02,0x76,0x00,0x4C,  // R02 LCD - driving waveform control
            0x74,0x00,0x03,0x76,0x12,0x14,  // R03 Power control
            0x74,0x00,0x04,0x76,0x04,0x66,  // R04 Contrast control
            0x74,0x00,0x05,0x76,0x00,0x10,  // R05 Entry mode
            0x74,0x00,0x06,0x76,0x00,0x00,  // R06 RAM data write mask
            0x74,0x00,0x07,0x76,0x00,0x15,  // R07 Display control
            0x74,0x00,0x08,0x76,0x00,0x03,  // R08 Cursor Control
            0x74,0x00,0x09,0x76,0x00,0x00,  // R09 RAM data write mask
            0x74,0x00,0x0A,0x76,0x00,0x15,  // R0A 
            0x74,0x00,0x0B,0x76,0x00,0x03,  // R0B Horizontal Cursor Position
            0x74,0x00,0x0C,0x76,0x00,0x03,  // R0C Vertical Cursor Position
            0x74,0x00,0x0D,0x76,0x00,0x00,  // R0D 
            0x74,0x00,0x0E,0x76,0x00,0x15,  // R0E 
            0x74,0x00,0x0F,0x76,0x00,0x03,  // R0F 
            0x74,0x00,0x10,0x76,0x00,0x15,  // R0E 
            0x74,0x00,0x11,0x76,0x00,0x03,  // R0F 
};

unsigned char Read_Block_Address_Macro[]= {0x74,0x00,0x12,0x77,0x00,0x00};                                     
unsigned char Draw_Block_Value_Macro[]={0x74,0x00,0x12,0x76,0xFF,0xFF};
unsigned char Draw_Block_Address_Macro[]={0x74,0x00,0x11,0x76,0x00,0x00};

unsigned int  LcdAddress = 0, LcdTableAddress = 0;
unsigned char contrast   = 0x66;
unsigned char backlight  = 8;
int LCD_MEM[110*17];

/**********************************************************************//**
 * @brief  Sends 3+3 bytes of data to the LCD using the format specified
 *         by the LCD Guide.
 * 
 * @param  Data[] Data array for transmission 
 * 
 * @return none
 *************************************************************************/
void halLcdSendCommand(unsigned char Data[]) 
{
  unsigned char i;

  LCD_CS_RST_OUT &= ~LCD_CS_PIN;            //CS = 0 --> Start Transfer
  for ( i = 0; i < 6; i++ )
  {
    while (!(UCB2IFG & UCTXIFG));           // Wait for TXIFG    
    UCB2TXBUF = Data[i];                    // Load data 
 
    while (UCB2STAT & UCBUSY);       

    if (i == 2)                             //Pull CS up after 3 bytes
    {
      LCD_CS_RST_OUT |= LCD_CS_PIN;         //CS = 1 --> Stop Transfer
      LCD_CS_RST_OUT &= ~LCD_CS_PIN;        //CS = 0 --> Start Transfer	 
    }
  }
  LCD_CS_RST_OUT |= LCD_CS_PIN;             //CS = 1 --> Stop Transfer       
}

/**********************************************************************//**
 * @brief  Initializes the USCI module, LCD device for communication. 
 *           
 * - Sets up the SPI2C Communication Module
 * - Performs Hitachi LCD Initialization Procedure
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdInit(void) 
{
  volatile unsigned int i=0;

  LCD_CS_RST_OUT |= LCD_CS_PIN | LCD_RESET_PIN ;
  LCD_CS_RST_DIR |= LCD_CS_PIN | LCD_RESET_PIN ;     
  
  LCD_COMM_SEL |= LCD_BACKLIGHT_PIN;
  
  LCD_CS_RST_OUT &= ~LCD_RESET_PIN;         // Reset LCD
  __delay_cycles(0x47FF);                   //Reset Pulse
  LCD_CS_RST_OUT |= LCD_RESET_PIN;        
    
  // UCLK,MOSI setup, SOMI cleared
  P9SEL |= BIT1 + BIT3;
  P9SEL &= ~BIT2;
  P9DIR |= BIT1 + BIT3;
  P9DIR &= ~BIT2;
  
  /* Initialize USCI state machine */ 
  UCB2CTL0 |= UCMST+UCSYNC+UCCKPL+UCMSB;    // 3-pin, 8-bit SPI master
  UCB2CTL1 |= UCSSEL_2;                     // SMCLK
  UCB2BR0 = 3; 
  UCB2BR1 = 0;
  UCB2CTL1 &= ~UCSWRST;                            
  UCB2IFG &= ~UCRXIFG;
   
  // LCD Initialization Routine Using Predefined Macros
  while (i < 8*6)
  {    
    halLcdSendCommand(&LcdInitMacro[i]);
    i += 6;
  }  
  halLcdActive();  
}

/**********************************************************************//**
 * @brief  Shuts down the LCD display and hdisables the USCI communication. 
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdShutDown(void)
{
  halLcdStandby();  

  LCD_CS_RST_DIR |= LCD_CS_PIN | LCD_RESET_PIN ;
  LCD_CS_RST_OUT &= ~(LCD_CS_PIN | LCD_RESET_PIN );  
  LCD_CS_RST_OUT &= ~LCD_RESET_PIN;                                         
  
  P9SEL &= ~(BIT1 + BIT3 + BIT2);
  P9DIR |= BIT1 + BIT3 + BIT2;
  P9OUT &= ~(BIT1 + BIT3 + BIT2);  
  
  UCB2CTL0 = UCSWRST; 
}  

/**********************************************************************//**
 * @brief  Initializes the LCD backlight PWM signal. 
 * 
 * @param  none
 * 
 * @return none
 * 
 *************************************************************************/
void halLcdBackLightInit(void)
{
  LCD_COMM_DIR |= LCD_BACKLIGHT_PIN;
  LCD_COMM_OUT |= LCD_BACKLIGHT_PIN;     
  
  LCD_COMM_SEL |= LCD_BACKLIGHT_PIN;
  TA0CCTL3 = OUTMOD_7;
  TA0CCR3 = TA0CCR0 >> 1 ;
  backlight = 8;
  
  TA0CCR0 = 400;
  TA0CTL = TASSEL_2+MC_1;   
}

/**********************************************************************//**
 * @brief  Get function for the backlight PWM's duty cycle. 
 * 
 * @param  none 
 * 
 * @return backlight One of the the 17 possible settings - valued 0 to 16. 
 *
 *************************************************************************/
unsigned int halLcdGetBackLight(void)
{  
  return backlight;
}

/**********************************************************************//**
 * @brief  Set function for the backlight PWM's duty cycle 
 * 
 * @param  BackLightLevel The target backlight duty cycle - valued 0 to 16.
 * 
 * @return none
 *************************************************************************/
void halLcdSetBackLight(unsigned char BackLightLevel)
{ 
  unsigned int dutyCycle = 0, i, dummy; 
  
  if (BackLightLevel > 0)
  {
    TA0CCTL3 = OUTMOD_7;
    dummy = (TA0CCR0 >> 4);
    
    for (i = 0; i < BackLightLevel; i++) 
      dutyCycle += dummy;
      
    TA0CCR3 = dutyCycle;
    
    // If the backlight was previously turned off, turn it on. 
    if (!backlight)                         
      TA0CTL |= MC0;  
  }
  else
  {   	
    TA0CCTL3 = 0;
    TA0CTL &= ~MC0;
  }  
  backlight = BackLightLevel;
}

/**********************************************************************//**
 * @brief  Turns off the backlight. 
 * 
 * Clears the respective GPIO and timer settings.
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdShutDownBackLight(void)
{
  LCD_COMM_DIR |= LCD_BACKLIGHT_PIN;
  LCD_COMM_OUT &= ~(LCD_BACKLIGHT_PIN);  
  LCD_COMM_SEL &= ~LCD_BACKLIGHT_PIN;
  
  TA0CCTL3 = 0;
  TA0CTL = 0;  
  
  backlight = 0;  
}

/**********************************************************************//**
 * @brief  Set function for the contrast level of the LCD. 
 * 
 * @param  ContrastLevel The target contrast level 
 * 
 * @return none 
 *************************************************************************/
void halLcdSetContrast(unsigned char ContrastLevel)
{
  if (ContrastLevel > 127) ContrastLevel = 127;
  if (ContrastLevel < 70) ContrastLevel = 70;
  LcdInitMacro[ 0x04 * 6 + 5 ] = ContrastLevel;
  halLcdSendCommand(&LcdInitMacro[ 0x04 * 6 ]);
} 

/**********************************************************************//**
 * @brief  Get function for the contrast level of the LCD. 
 * 
 * @param  none
 * 
 * @return ContrastLevel The LCD constrast level
 *************************************************************************/
unsigned char halLcdGetContrast(void)
{
  return LcdInitMacro[ 0x04 * 6 + 5 ] ;
}

/**********************************************************************//**
 * @brief  Turns the LCD cursor on at the current text position.
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdCursor()
{
  LcdInitMacro[  8 * 6 + 5 ] ^= BIT2;
  halLcdSendCommand(&LcdInitMacro[ 8 * 6 ]);
  
  LcdInitMacro[ 0x0B * 6 + 5 ] = ((LcdAddress & 0x1F) << 3) ;
  LcdInitMacro[ 0x0B * 6 + 4 ] = ( (LcdAddress & 0x1F) << 3 ) + 3;
  LcdInitMacro[ 0x0C * 6 + 5 ] = (LcdAddress >> 5);
  LcdInitMacro[ 0x0C * 6 + 4 ] = (LcdAddress >> 5) + 7;
  halLcdSendCommand(&LcdInitMacro[ 0x0B * 6 ]);
  halLcdSendCommand(&LcdInitMacro[ 0x0C * 6 ]);
  
  halLcdSetAddress(LcdAddress);
}

/**********************************************************************//**
 * @brief  Turns off the LCD cursor.
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdCursorOff()
{
  LcdInitMacro[  8 * 6 + 5 ] &= ~BIT2;
  halLcdSendCommand(&LcdInitMacro[ 8 * 6 ]);
}

/**********************************************************************//**
 * @brief  Inverts the grayscale values of the LCD display (Black <> white).  
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdReverse(void)
{
  LcdInitMacro[  7 * 6 + 5 ] ^= BIT1;
  halLcdSendCommand(&LcdInitMacro[ 7 * 6 ]);
}

/**********************************************************************//**
 * @brief  Sets the LCD in standby mode to reduce power consumption.
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdStandby(void)
{
  LcdInitMacro[ 3 * 6 + 5 ] &= (~BIT3) & (~BIT2);
  LcdInitMacro[ 3 * 6 + 5 ] |= BIT0;
  halLcdSendCommand(&LcdInitMacro[ 3 * 6 ]);
}

/**********************************************************************//**
 * @brief  Puts the LCD into active mode.
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdActive(void)
{
  halLcdSendCommand(LcdInitMacro);
  LcdInitMacro[ 3 * 6 + 5 ] |= BIT3 ;
  LcdInitMacro[ 3 * 6 + 5 ] &= ~BIT0;
  halLcdSendCommand(&LcdInitMacro[ 3 * 6 ]);
}

/**********************************************************************//**
 * @brief  Sets the pointer location in the LCD. 
 *         
 * - LcdAddress      = Address  					 
 * - LcdTableAddress = Correct Address Row + Column 
 *                   = (Address / 0x20)* 17 + Column  
 * 
 * @param  Address The target pointer location in the LCD.
 * 
 * @return none
 *************************************************************************/
void halLcdSetAddress(int Address)
{
  int temp;
  
  Draw_Block_Address_Macro[4] = Address >> 8;
  Draw_Block_Address_Macro[5] = Address & 0xFF;
  halLcdSendCommand(Draw_Block_Address_Macro);
  LcdAddress = Address;
  temp = Address >> 5;                      // Divided by 0x20
  temp = temp + (temp << 4); 
  //Multiplied by (1+16) and added by the offset
  LcdTableAddress = temp + (Address & 0x1F); 
}

/**********************************************************************//**
 * @brief  Draws a block at the specified LCD address. 
 * 
 * A block is the smallest addressable memory on the LCD and is
 * equivalent to 8 pixels, each of which is represented by 2 bits
 * that represent a grayscale value between 00b and 11b.   
 * 
 * @param  Address The address at which to draw the block.
 * 
 * @param  Value   The value of the block 
 * 
 * @return none
 *************************************************************************/
void halLcdDrawBlock(unsigned int Address, unsigned int Value)
{
  halLcdSetAddress(Address);
  halLcdDrawCurrentBlock(Value);
}

/**********************************************************************//**
 * @brief  Writes Value to LCD CGram and MSP430 internal LCD table. 
 * 
 * Also updates the LcdAddress and LcdTableAddress to the correct values.
 *  
 * @param  Value The value of the block to be written to the LCD. 
 * 
 * @return none
 *************************************************************************/
void halLcdDrawCurrentBlock(unsigned int Value)
{   
  int temp;

  Draw_Block_Value_Macro[4] = Value >> 8;
  Draw_Block_Value_Macro[5] = Value & 0xFF;
  LCD_MEM[ LcdTableAddress ] = Value;
  
  halLcdSendCommand(Draw_Block_Value_Macro);
  
  LcdAddress++;
  temp = LcdAddress >> 5;                   // Divided by 0x20
  temp = temp + (temp << 4); 
  // Multiplied by (1+16) and added by the offset
  LcdTableAddress = temp + (LcdAddress & 0x1F); 
  
  // If LcdAddress gets off the right edge, move to next line
  if ((LcdAddress & 0x1F) > 0x11)
    halLcdSetAddress( (LcdAddress & 0xFFE0) + 0x20 );
  if (LcdAddress == LCD_Size)
    halLcdSetAddress( 0 );  
}


/**********************************************************************//**
 * @brief  Returns the LCD CGRAM value at location Address.
 * 
 * @param  Address The address of the block to be read from the LCD. 
 * 
 * @return Value   The value held at the specified address.
 *************************************************************************/
int halLcdReadBlock(unsigned int Address)
{
  int i = 0, Value = 0, ReadData[7]; 
  
  halLcdSetAddress( Address );    
  halLcdSendCommand(Read_Block_Address_Macro);      
  
  LCD_COMM_OUT &= ~LCD_CS_PIN;              // start transfer CS=0
  UCB2TXBUF = 0x77;                         // Transmit first character 0x75
  
  while (!(UCB2IFG & UCTXIFG)); 
  while (UCB2STAT & UCBUSY);
  
  //Read 5 dummies values and 2 valid address data  
  P9SEL &= ~BIT1;                           //Change SPI2C Dir
  P9SEL |= BIT2;      
  
  for (i = 0; i < 7; i ++ )
  {
    P4OUT |= BIT2;
    P4OUT &= ~BIT2;
    UCA0IFG &= ~UCRXIFG;
    UCA0TXBUF = 1;                          // load dummy byte 1 for clk
    while (!(UCA0IFG & UCRXIFG)); 
    ReadData[i] = UCA0RXBUF; 
  } 
  LCD_COMM_OUT |= LCD_CS_PIN;               // Stop Transfer CS = 1
  
  P9SEL |= BIT1;                            //Change SPI2C Dir
  P9SEL &= ~BIT2;
  
  Value = (ReadData[5] << 8) + ReadData[6];  
  return Value;
}

/**********************************************************************//**
 * @brief  Draw a Pixel of grayscale at coordinate (x,y) to LCD 
 * 
 * @param  x         x-coordinate for grayscale value 
 * 
 * @param  y         y-coordinate for grayscale value
 * 
 * @param  GrayScale The intended grayscale value of the pixel - one of 
 *                   four possible settings.
 * 
 * @return none
 *************************************************************************/
void halLcdPixel( int x, int y, unsigned char GrayScale)
{
  int  Address, Value;
  unsigned char offset;
  //Each line increments by 0x20
  if ( (x>=0 ) && (x<LCD_COL) && (y>=0) && (y<LCD_ROW))
  {
    Address = (y << 5) + (x >> 3) ;         //Narrow down to 8 possible pixels    
    offset = x & 0x07;                      //3 LSBs = pos. within the 8 columns
    
    Value = LCD_MEM[(y << 4)+ y + (x>>3)];  //y * 17 --> row. x>>3 --> column
    switch(GrayScale)
    {
      case PIXEL_OFF:  
        Value &= ~ ( 3 << (offset * 2 ));   //Both corresponding bits are off
        break;
      case PIXEL_LIGHT:
        Value &= ~ ( 1 << ((offset * 2) + 1));
        Value |= 1<< ( offset * 2 );        //Lower bit is on
  
        break;
      case PIXEL_DARK:
        Value &= ~ ( 1 << (offset * 2) );   //Lower bit is on
        Value |= ( 1 << ( (offset * 2) + 1));
  
        break;
      case PIXEL_ON: 
        Value |=  ( 3 << (offset * 2 ));    //Both on
        break;
    }
    halLcdDrawBlock( Address, Value );
  }
}

/**********************************************************************//**
 * @brief  Clears entire LCD CGRAM as well as LCD_MEM.
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void halLcdClearScreen()
{
  int i;
  
  halLcdSetAddress(0);   
  
  // Clear the entire LCD CGRAM 
  for ( i = 0; i < LCD_Size; i++)    
    halLcdDrawCurrentBlock(0x00);
  
  halLcdSetAddress(0);                      // Reset LCD address 
}

/**********************************************************************//**
 * @brief  Loads an image of size = rows * columns, starting at the 
 *         coordinate (x,y).
 * 
 * @param  Image[] The image to be loaded
 * 
 * @param  Rows    The number of rows in the image. Size = Rows * Columns.
 * 
 * @param  Columns The number of columns in the image. Size = Rows * Columns.
 * 
 * @param  x       x-coordinate of the image's starting location 
 * 
 * @param  y       y-coordinate of the image's starting location 
 * 
 * @return none
 *************************************************************************/
void halLcdImage(const unsigned int * Image, int Columns, int Rows, int x, int y)
{  
  int i,j, CurrentLocation, index=0 ;
   
  CurrentLocation = (y << 5) + (x >> 3);
  halLcdSetAddress( CurrentLocation );
  for (i=0; i < Rows; i++)
  {
    for (j=0; j < Columns; j++)
      halLcdDrawCurrentBlock(Image[index++]);   
    CurrentLocation += 0x20;
    halLcdSetAddress(CurrentLocation );  
  }  
}

/**********************************************************************//**
 * @brief  Clears an image of size rows x columns starting at (x, y). 
 * 
 * @param  Columns The size, in columns, of the image to be cleared.
 * 
 * @param  Rows    The size, in rows, of the image to be cleared.
 * 
 * @param  x       x-coordinate of the image to be cleared
 * 
 * @param  y       y-coordinate of the image to be cleared
 * 
 * @return none
 *************************************************************************/
void halLcdClearImage(int Columns, int Rows, int x, int y)
{
   int i,j, Current_Location;
   Current_Location = (y << 5) + (x >> 3);
   halLcdSetAddress( Current_Location );
   for (i=0; i < Rows; i++)
   {
     for (j=0; j < Columns; j++)
        halLcdDrawCurrentBlock(0);   
     Current_Location += 0x20;
     halLcdSetAddress(Current_Location );  
   }  
}

/**********************************************************************//**
 * @brief  Writes Value to LCD CGRAM. Pointers internal to the LCD 
 *         are also updated. 
 * 
 * @param  Value The value to be written to the current LCD pointer
 * 
 * @return none
 *************************************************************************/
void halLcdDrawTextBlock(unsigned int Value)
{   
  int temp;
  
  Draw_Block_Value_Macro[4] = Value >> 8;    
  Draw_Block_Value_Macro[5] = Value & 0xFF;  
  LCD_MEM[ LcdTableAddress ] = Value;        
  
  halLcdSendCommand(Draw_Block_Value_Macro);
  
  LcdAddress++;
  temp = LcdAddress >> 5;                   // Divided by 0x20
  temp = temp + (temp << 4); 
  //Multiplied by (1+16) and added by the offset
  LcdTableAddress = temp + (LcdAddress & 0x1F); 
  
  // If LcdAddress gets off the right edge, move to next line 
  if ((LcdAddress & 0x1F) > 0x10)
    halLcdSetAddress( (LcdAddress & 0xFFE0) + 0x20 );
  
  if (LcdAddress >= LCD_Size)
    halLcdSetAddress( 0 );  
}

/**********************************************************************//**
 * @brief  Displays the string to the LCD starting at current location.
 * 
 * Writes all the data to LCD_MEM first, then updates all corresponding 
 * LCD CGRAM locations at once, in a continuous fashion.
 *  
 * @param  String[]  The string to be displayed on LCD.
 * 
 * @param  TextStyle Value that specifies whether the string is to be 
 *                   inverted or overwritten. 
 *                   - Invert    = 0x01 
 *                   - Overwrite = 0x04 
 * 
 * @return none
 *************************************************************************/
void halLcdPrint( char String[], unsigned char TextStyle) 
{
  int i, j, Counter=0, BlockValue;
  int Address, LCD_MEM_Add, ActualAddress;
  int temp;
  char LookUpChar;
  
  ActualAddress = LcdAddress;
  Counter =  LcdAddress & 0x1F;  
  i=0;
  
  while (String[i]!=0)                      // Stop on null character
  { 
    LookUpChar = fonts_lookup[String[i]]; 
    
    for (j=0;j < FONT_HEIGHT ;j++)      
    {
      Address = ActualAddress + j*0x20;
      temp = Address >> 5;
      temp += (temp <<4);

      LCD_MEM_Add = temp + (Address & 0x1F); 
      
      BlockValue = LCD_MEM[ LCD_MEM_Add ];
      
      if (TextStyle & INVERT_TEXT)         
        if (TextStyle & OVERWRITE_TEXT)
          BlockValue = 0xFFFF - fonts[LookUpChar*13+j]; 
        else
          BlockValue |= 0xFFFF - fonts[LookUpChar*13+j]; 
        
      else     
        if (TextStyle & OVERWRITE_TEXT)
          BlockValue = fonts[LookUpChar*(FONT_HEIGHT+1) +j]; 
        else
          BlockValue |= fonts[LookUpChar*(FONT_HEIGHT+1) +j]; 
        
      halLcdDrawBlock( Address, BlockValue);          
    }
    
    Counter++;
    if (Counter == 17)
    {
      Counter = 0;        
      ActualAddress += 0x20*FONT_HEIGHT  - 16;
      if (ActualAddress > LCD_Last_Pixel-0x20*FONT_HEIGHT )
        ActualAddress = 0;
    }
    else 
      ActualAddress++;
    i++;
  }
  halLcdSetAddress(ActualAddress);
  
}

/**********************************************************************//**
 * @brief  Displays the string to the LCD starting at (x,y) location. 
 * 
 * Writes all the data to LCD_MEM first, then updates all corresponding 
 * LCD CGRAM locations at once, in a continuous fashion.
 * 
 * @param  String[]  String to be displayed on LCD
 * 
 * @param  x         x-coordinate of the write location on the LCD 
 * 
 * @param  y         y-coordinate of the write location on the LCD
 * 
 * @param  TextStyle Value that specifies whether the string is to be 
 *                   inverted or overwritten. 
 *                   - Invert    = 0x01 
 *                   - Overwrite = 0x04 
 *************************************************************************/
void halLcdPrintXY( char String[], int x, int y, unsigned char TextStyle)  
{
  //Each line increments by 0x20
  halLcdSetAddress( (y << 5) + (x >> 3)) ;  //Narrow down to 8 possible pixels    
  halLcdPrint(String,  TextStyle);
}

/**********************************************************************//**
 * @brief  Displays a string on the LCD on the specified line.  
 * 
 * @param  String[]  The string to be displayed on LCD.
 * 
 * @param  Line      The line on the LCD on which to print the string.
 * 
 * @param  TextStyle Value that specifies whether the string is to be 
 *                   inverted or overwritten. 
 *                   - Invert    = 0x01 
 *                   - Overwrite = 0x04 
 * 
 * @return none
 *************************************************************************/
void halLcdPrintLine(char String[], unsigned char Line, unsigned char TextStyle)  
{
  int temp; 
  temp = Line * FONT_HEIGHT ;
  halLcdSetAddress( temp << 5 ) ;           // 0x20 = 2^5  
  halLcdPrint(String, TextStyle);
}

/**********************************************************************//**
 * @brief  Prints a string beginning on a given line and column.  
 * 
 * @param  String[]  The string to be displayed on LCD.
 * 
 * @param  Line      The line on which to print the string of text
 * 
 * @param  Col       The column on which to print the string of text
 * 
 * @param  TextStyle Value that specifies whether the string is to be 
 *                   inverted or overwritten. 
 *                   - Invert    = 0x01 
 *                   - Overwrite = 0x04 
 * 
 * @return none
 *************************************************************************/
void halLcdPrintLineCol(char String[], unsigned char Line, unsigned char Col,
                        unsigned char TextStyle)  
{
  int temp; 
  
  temp = Line * FONT_HEIGHT;
  temp <<= 5;
  temp += Col;
  
  halLcdSetAddress( temp ) ;                // 0x20 = 2^5                     
  halLcdPrint(String, TextStyle);
}


/**********************************************************************//**
 * @brief  Draws a horizontral line from (x1,y) to (x2,y) of GrayScale level
 * 
 * @param  x1        x-coordinate of the first point 
 * 
 * @param  x2        x-coordinate of the second point
 * 
 * @param  y         y-coordinate of both points
 * 
 * @param  GrayScale Grayscale level of the horizontal line
 * 
 * @return none
 *************************************************************************/
void halLcdHLine( int x1, int x2, int y, unsigned char GrayScale)
{
  int x_dir, x;
  if ( x1 < x2 )
    x_dir = 1;
  else 
    x_dir = -1;
  x = x1;    
  while (x != x2)
  {
    halLcdPixel( x,y, GrayScale); 
    x += x_dir;
  }
}

/**********************************************************************//**
 * @brief  Draws a vertical line from (x,y1) to (x,y2) of GrayScale level
 * 
 * @param  x         x-coordinate of both points
 * 
 * @param  y1        y-coordinate of the first point 
 * 
 * @param  y2        y-coordinate of the second point
 * 
 * @param  GrayScale GrayScale level of the vertical line
 * 
 * @return none
 *************************************************************************/
void halLcdVLine( int x, int y1, int y2, unsigned char GrayScale)
{
  int y_dir, y;
  if ( y1 < y2 )
    y_dir = 1;
  else 
    y_dir = -1;
  y = y1;    
  while (y != y2)
  {
    halLcdPixel( x,y, GrayScale); 
    y += y_dir;
  }
}

/**********************************************************************//**
 * @brief  Draws a line from (x1,y1) to (x2,y2) of GrayScale level.
 *         
 * Uses Bresenham's line algorithm.
 * 
 * @param  x1         x-coordinate of the first point        
 *  
 * @param  y1         y-coordinate of the first point
 * 
 * @param  x2         x-coordinate of the second point
 * 
 * @param  y2         y-coordinate of the second point
 * 
 * @param  GrayScale  Grayscale level of the line 
 * 
 * @return none
 *************************************************************************/
void halLcdLine( int x1, int y1, int x2, int y2, unsigned char GrayScale) 
{
  int x, y, deltay, deltax, d;  
  int x_dir, y_dir;

  if ( x1 == x2 )
    halLcdVLine( x1, y1, y2, GrayScale );
  else
  {
    if ( y1 == y2 )
      halLcdHLine( x1, x2, y1, GrayScale );
    else                                    // a diagonal line
    {
      if (x1 > x2)
        x_dir = -1;
      else x_dir = 1;
      if (y1 > y2)
        y_dir = -1;
      else y_dir = 1;
      
      x = x1;
      y = y1;
      deltay = ABS(y2 - y1);
      deltax = ABS(x2 - x1);

      if (deltax >= deltay)
      {
        d = (deltay << 1) - deltax;
        while (x != x2)
        {
          halLcdPixel(x, y,  GrayScale);
          if ( d < 0 )
            d += (deltay << 1);
          else
          {
            d += ((deltay - deltax) << 1);
            y += y_dir;
          }
          x += x_dir;
        }                
      }
      else
      {
        d = (deltax << 1) - deltay;
        while (y != y2)
        {
          halLcdPixel(x, y, GrayScale);
          if ( d < 0 )
            d += (deltax << 1);
          else
          {
            d += ((deltax - deltay) << 1);
            x += x_dir;
          }
          y += y_dir;
        }        
      }
    }  
  }
}


/**********************************************************************//**
 * @brief  Draw a circle of Radius with center at (x,y) of GrayScale level.
 *         
 * Uses Bresenham's circle algorithm
 * 
 * @param  x         x-coordinate of the circle's center point
 * 
 * @param  y         y-coordinate of the circle's center point
 * 
 * @param  Radius    Radius of the circle
 * 
 * @param  GrayScale Grayscale level of the circle 
 *************************************************************************/
void halLcdCircle(int x, int y, int Radius, int GrayScale)
{
  int xx, yy, ddF_x, ddF_y, f;
  
  ddF_x = 0;
  ddF_y = -(2 * Radius);
  f = 1 - Radius;

  xx = 0;
  yy = Radius;
  halLcdPixel(x + xx, y + yy, GrayScale);
  halLcdPixel(x + xx, y - yy, GrayScale);
  halLcdPixel(x - xx, y + yy, GrayScale);
  halLcdPixel(x - xx, y - yy, GrayScale);
  halLcdPixel(x + yy, y + xx, GrayScale);
  halLcdPixel(x + yy, y - xx, GrayScale);
  halLcdPixel(x - yy, y + xx, GrayScale);
  halLcdPixel(x - yy, y - xx, GrayScale);
  while (xx < yy)
  {
    if (f >= 0)
    {
      yy--;
      ddF_y += 2;
      f += ddF_y;
    }
    xx++;
    ddF_x += 2;
    f += ddF_x + 1;
    halLcdPixel(x + xx, y + yy, GrayScale);
    halLcdPixel(x + xx, y - yy, GrayScale);
    halLcdPixel(x - xx, y + yy, GrayScale);
    halLcdPixel(x - xx, y - yy, GrayScale);
    halLcdPixel(x + yy, y + xx, GrayScale);
    halLcdPixel(x + yy, y - xx, GrayScale);
    halLcdPixel(x - yy, y + xx, GrayScale);
    halLcdPixel(x - yy, y - xx, GrayScale);
  }
}

/**********************************************************************//**
 * @brief  Scrolls a single row of pixels one column to the left. 
 * 
 * The column that is scrolled out of the left side of the LCD will be 
 * displayed the right side of the LCD. 
 * 
 * @param  y    The row of pixels to scroll. y = 0 is at the top-left
 *              corner of the LCD. 
 * 
 * @return none
 *************************************************************************/
void halLcdScrollRow(int y)
{
  int i, Address, LcdTableAddressTemp;
  unsigned int temp;
  
  Address = y << 5;
  
  halLcdSetAddress( Address );
  
  //Multiplied by (1+16) and added by the offset
  LcdTableAddressTemp = y + (y << 4);       
  temp = ((LCD_MEM[LcdTableAddressTemp] & 0x0003) <<14);
  
  for (i = 0; i < 0x10; i++)      
    halLcdDrawCurrentBlock( ( (LCD_MEM[LcdTableAddressTemp+i] & 0xFFFC ) >> 2 ) \
    + ((LCD_MEM[LcdTableAddressTemp+i+1] & 0x0003) << 14 ));
    
  halLcdDrawCurrentBlock( (( LCD_MEM[LcdTableAddressTemp + 0x10] & 0xFFFC ) >> 2) + temp);    
}

/**********************************************************************//**
 * @brief  Scrolls multiple rows of pixels, yStart to yEnd, 
 *         one column to the left. 
 * 
 * The column that is scrolled out of the left side of the LCD will be 
 * displayed the right side of the LCD. y = 0 is at the top-left of the 
 * LCD screen.
 * 
 * @param  yStart The beginning row to be scrolled
 * 
 * @param  yEnd   The last row to be scrolled 
 * 
 * @return none
 *************************************************************************/
void halLcdHScroll(int yStart, int yEnd)
{
  int i ;  
  
  for (i = yStart; i < yEnd+1; i++)
    halLcdScrollRow(i);
}

/**********************************************************************//**
 * @brief  Scrolls a line of text one column to the left. 
 * 
 * @param  Line The line of text to be scrolled.  
 * 
 * @return none
 *************************************************************************/
void halLcdScrollLine(int Line)
{
  int i, Row ;
  
  Row = Line * FONT_HEIGHT;
  
  for (i = Row; i < Row + FONT_HEIGHT ; i++)
    halLcdScrollRow(i);
}




/**************************************************************************************/

//--Calibration constants and user configuration values stored in INFOB Flash
//  at production; --
__no_init unsigned char boardMode @ 0x1900;
__no_init unsigned char lcdBackLightLevelSetting @ 0x1908;
__no_init unsigned char lcdContrastSetting @ 0x1910;


#define TIME_OUT          10
#define TIME_OUT2         3
#define MENU_MAX          6
#define SETTING_MENU_MAX  6
#define MENU_ITEM_WIDTH   14

enum{  LPM4_MODE, LPM3_MODE, ACTIVE_MODE, APPLICATION_MODE } ;
enum{  APP_CLOCK, APP_BAL_BALL, APP_USB, APP_AUDIO,  PMM_MCLK, MENU_SETTING};
enum{  SET_TIME, SET_CONTRAST, SET_BACKLIGHT, SET_TEMPERATURE_CONVERSION, 
	   CONFIGURE_ACCELEROMETER, EXIT_SETTING};

unsigned char boardMode;
unsigned char lcdBackLightLevelSetting;
unsigned char lcdContrastSetting;


unsigned char boardModeLOCAL;
unsigned char lcdBackLightLevelSettingLOCAL;
unsigned char lcdContrastSettingLOCAL;
unsigned char temperatureConversionLOCAL;
long temperatureCalibrationCLOCAL;
unsigned char wakeUpOnAccLOCAL;

static char menuText[]={
  "123456789012345\0"
  "Hello world 1\0"
  "Hello world 2\0"          
  "Hello world 3\0"
  "Text No.1    \0"
  "Text No.2    \0"
  "1234567890123\0"  
};

unsigned char menuPos, settingMenuPos;
unsigned char timeOutCounter  = 0; 
unsigned char CpuMode, accWake=0, menuPos, settingMenuPos;

volatile unsigned char buttonsPressed;
volatile unsigned char buttonDebounce; 

char TemperatureStr[]= "\0\0\0\0\0\0";
char VccStr[] = "0.0V"; 
unsigned char RTCAccHalfSec = 0, RTCExit64Hz= 0 , RTCExitSec= 0, RTCAccSec = 0;

//#include "clock.c"

void saveSettings(void)
{  
  flashEraseSegment((unsigned long)&boardMode);
  FLASH_UNLOCK;
  boardMode = boardModeLOCAL ;
  lcdBackLightLevelSetting = lcdBackLightLevelSettingLOCAL ;
  FLASH_LOCK;  
}

/**********************************************************************//**
 * @brief  Loads calibration and user-config data from INFOB flash. 
 * 
 * @param  none
 * 
 * @return none
 *************************************************************************/
void loadSettings(void)
{  
  boardModeLOCAL = boardMode;
  lcdBackLightLevelSettingLOCAL = lcdBackLightLevelSetting;
  lcdContrastSettingLOCAL = lcdContrastSetting;
 if ( lcdContrastSettingLOCAL < 60 || lcdContrastSettingLOCAL > 128) 
   lcdContrastSettingLOCAL = 100;

}


void setupRTC(void);

/**********************************************************************//**
 * @brief  Checks for the board revision and returns a value < 0 if wrong
 *         revision is specified in main.c 
 * 
 * @param  none 
 *  
 * @return Whether or not the board revision matches the software
 * - 0 - The board revision does not match the software
 * - 1 - The board revision matches the software
 *************************************************************************/
unsigned char assert_board_version( void )
{
  P8DIR &= ~BIT7;                           // Set P8.7 input
  P8OUT |= BIT7;                            // Set pullup resistor
  P8REN |= BIT7;                            // Enable pull up resistors 
  
  #ifdef REV_02
    if(!(P8IN & BIT7))                      // Board rev = 0_02? 
      return 0;
  #else 
    if((P8IN & BIT7))                       // Board rev = 0_03? 
      return 0;
  #endif    
  
  P8DIR |= BIT7;                            // Set P8.7 output
  P8OUT &= ~BIT7;                           // Set P8.7 = 0 
  P8REN &= ~BIT7;                           // Disable pull up resistors   

  return 1; 
}


/**********************************************************************//**
 * @brief  Draws and manages the selection of the menu options.
 * 
 * @param  menuText The text that constitues the application menu. 
 * 
 * @param  menuNum  The enumerated value that represents the current 
 *                  menu selection. 
 * 
 * @return The updated, or latest, menu selection. 
 *************************************************************************/
unsigned char activeMenuMode(char *menuText, unsigned char menuNum)
{
  unsigned char menuPosition, quit = 0;
  int i;
    

  //RTCExitSec = 1;                           // To update digital clock
  //halButtonsInterruptEnable( BUTTON_ALL );  

  halLcdClearScreen();    
  menuPosition = 0;
  //Print menu title
  halLcdPrintLine(menuText, 0, 0 );
  
  //Print menu items
  for (i=0;i<menuNum;i++)
    halLcdPrintLine(&menuText[i*MENU_ITEM_WIDTH+16], i+1,  OVERWRITE_TEXT );
  //First line is inverted text, automatic selection
  //halLcdPrintLine(&menuText[0*MENU_ITEM_WIDTH+16],  1,     \
                  INVERT_TEXT | OVERWRITE_TEXT );  
  //timeOutCounter = 0;
  //buttonsPressed = 0;
  
  //halAdcSetQuitFromISR( 0 );
  
  while (CpuMode == ACTIVE_MODE && quit == 0)
  {     
    TA0CTL &= ~TAIFG; 
    

  }  

  
  return menuPosition;
}

/**********************************************************************//**
 * @brief  This is the example code's main function. 
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
