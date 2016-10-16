

#include "msp430x54xA.h"
#include "lcd.c"
// Functions Declarations
// 1) Gate functions - control step motors
void openEntranceGate();
void closeEntranceGate();
void rotateRightM1(int steps);
void rotateLeftM1(int steps);

void openExitGate();
void closeExitGate();
void rotateRightM2(int steps);
void rotateLeftM2(int steps);

// 2) Initilizations
void InitilaizeUART();

// 3) Delay
void delay(long value);

// 4) General
setPinInput(int port, int pin);
setPinOutput(int port, int pin);
setPinHigh(int port, int pin);
setPinLow(int port, int pin);
// Photoresist Sensors

int temp = 0;
// ==========
// == MAIN ==
// ==========
void main(void)
{
    
    /* Check for the version of the board */ 
 if(!assert_board_version())
    while(1);  
   
  //Initialize clock and peripherals 
  halBoardInit();  
  halBoardStartXT1();	
  halBoardSetSystemClock(SYSCLK_16MHZ);
  loadSettings();
  
  //Initialize LCD and backlight   
  halLcdInit();       
  halLcdBackLightInit();    
  halLcdSetBackLight(lcdBackLightLevelSettingLOCAL);
  halLcdSetContrast(lcdContrastSettingLOCAL);
  halLcdClearScreen(); 

  CpuMode = ACTIVE_MODE;

 
  
/////////////////////////////////////////////////////////////////////  
///////////////////////////////////////////////////////////////////// 




  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
  __no_operation();                         // For debugger
}
// Initializations
void InitializeUART(){
	P5SEL |= BIT6+BIT7;                       // P5.6,p5.7 UART option select
	UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
	UCA1CTL1 |= UCSSEL_2;                     // SMCLK = 16MHz
	UCA1CTL0 &= ~UC7BIT;                      // WordSize = 8 Bit
	UCA1CTL0 &= ~UCSPB;                       // One stop bit
	UCA1CTL0 &= ~UCPEN;                       // Parity disabled 
	UCA1BR0 = 145;                            // 16MHz 115200 (see User's Guide)
	UCA1BR1 = 0;                              // 16MHz 115200
	UCA1MCTL |= UCBRS_5 + UCBRF_0;            // Modulation UCBRSx=5, UCBRFx=0
	UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
	
}
void InitializeWD(){
	WDTCTL = WDT_MDLY_32;                     // WDT 32ms, SMCLK, interval timer 
    SFRIE1 |= WDTIE;                          // Enable WDT interrupt
}
void InitializeADC12(){
  WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer
  P6SEL = 0x0F;                             // Enable A/D channel inputs
  ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_8; // Turn on ADC12, extend sampling time
                                            // to avoid overflow of results
  ADC12CTL1 = ADC12SHP+ADC12CONSEQ_3;       // Use sampling timer, repeated sequence
  ADC12MCTL0 = ADC12INCH_0;                 // ref+=AVcc, channel = A0
  ADC12MCTL1 = ADC12INCH_1;                 // ref+=AVcc, channel = A1
  ADC12MCTL2 = ADC12INCH_2;                 // ref+=AVcc, channel = A2
  ADC12MCTL3 = ADC12INCH_3+ADC12EOS;        // ref+=AVcc, channel = A3, end seq.
  ADC12IE = 0x08;                           // Enable ADC12IFG.3
  ADC12CTL0 |= ADC12ENC;                    // Enable conversions
  ADC12CTL0 |= ADC12SC;                     // Start convn - software trigger
}
// Delay
void delay(long value){
	// At clk 16MHZ:
	// value = 2000000 ~ 500ms
	// value = 
	long ind = 0;
	for(ind = 0 ; ind < value; ind++ );	
}
//Gates Function (rotating step motors)
void openEntranceGate(){
	rotateLeftM1(124); // ~90 degrees     
}
void closeEntranceGate(){
	rotateRightM1(124);
}
void rotateLeftM1(int steps){
	// Description
	/*
	this function will rotate the step motor N steps to the right
	the pins of the motor are connected:
	INT1 -> P10.7
	INT2 -> P10.5
	INT3 -> P10.3
	INT4 -> P10.1
	
	Steps to degrees:
	124 steps ~ 90 degrees
	
	*/
	for ( int ind = 0; ind < steps ; ind++){
	  setPinHigh(10,7);
	  delay(20000); 
	  setPinLow(10,7);   
		  
	  setPinHigh(10,5);
	  delay(20000);
	  setPinLow(10,5);
	  
	  setPinHigh(10,3);
	  delay(20000);
	  setPinLow(10,3);
	  
	  setPinHigh(10,1);
	  delay(20000);
	  setPinLow(10,1);
	}
}
void rotateRightM1(int steps){
	// Description
	/*
	this function will rotate the step motor N steps to the left
	the pins of the motor are connected:
	INT1 -> P10.7
	INT2 -> P10.5
	INT3 -> P10.3
	INT4 -> P10.1
	
	Steps to degrees:
	124 steps ~ 90 degrees
	
	*/
	for ( int ind = 0; ind < steps ; ind++){
	  setPinHigh(10,1);
	  delay(20000); 
	  setPinLow(10,1);   
		  
	  setPinHigh(10,3);
	  delay(20000);
	  setPinLow(10,3);
	  
	  setPinHigh(10,5);
	  delay(20000);
	  setPinLow(10,5);
	  
	  setPinHigh(10,7);
	  delay(20000);
	  setPinLow(10,7);
	}
}

void openExitGate(){
	rotateRightM2(124); // ~90 degrees     
}
void closeExitGate(){
	rotateLeftM2(124);
}
void rotateRightM2(int steps){
	// Description
	/*
	this function will rotate the step motor N steps to the right
	the pins of the motor are connected:
	INT1 -> P10.6
	INT2 -> P10.4
	INT3 -> P10.2
	INT4 -> P10.0
	
	Steps to degrees:
	124 steps ~ 90 degrees
	
	*/
	for ( int ind = 0; ind < steps ; ind++){
	  setPinHigh(10,0);
	  delay(20000); 
	  setPinLow(10,0);   
		  
	  setPinHigh(10,2);
	  delay(20000);
	  setPinLow(10,2);
	  
	  setPinHigh(10,4);
	  delay(20000);
	  setPinLow(10,4);
	  
	  setPinHigh(10,6);
	  delay(20000);
	  setPinLow(10,6);
	}
}
void rotateLefttM2(int steps){
	int bit = 0;
	switch(pin){
		case 0:
			bit = BIT0 ;
			break;
		case 1:
			bit = BIT1 ;
			break;
		case 2:
			bit = BIT2 ;
			break;
		case 3:
			bit = BIT3 ;
			break;
		case 4:
			bit = BIT4 ;
			break;
		case 5:
			bit = BIT5 ;
			break;
		case 6:
			bit = BIT6 ;
			break;
		case 7:
			bit = BIT7 ;
			break;
	}
	switch(port){
		case 1:
			P1OUT &= ~bit ;
			break;
		case 2:
			P2OUT &= ~bit ;
			break;
		case 3:
			P3OUT &= ~bit ;
			break;
		case 4:
			P4OUT &= ~bit ;
			break;
		case 5:
			P5OUT &= ~bit ;
			break;
		case 6:
			P6OUT &= ~bit ;
			break;
		case 7:
			P7OUT &= ~bit ;
			break;
		case 8:
			P8OUT &= ~bit ;
			break;
		case 9:
			P9OUT &= ~bit ;
			break;
		case 10:
			P10OUT &= ~bit ;
			break;
		case 11:
			P11OUT &= ~bit ;
			break;

	}
}
//
void updateParkingStatus(){
	
	for (int ind = 0 ; ind < 4 ; ind++ ){
		
	}
}
// ========
// Interups
// ========

// UART interrupt Service Routine
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{

  switch(__even_in_range(UCA1IV,4))
  {
  case 0:// Vector 0 - no interrupt                                   
    break;                             
  case 2:// Vector 2 - RXIFG = Recieve buffer full 
    break;
  case 4:// Vector 4 - TXIFG
    break;                             
  default: 
    break;
  }
}
// ADC12 Interrupt Service Routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12ISR (void)
#else
#error Compiler not supported!
#endif
{
  static unsigned int index = 0;

  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6: break;                           // Vector  6:  ADC12IFG0
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12:                                  // Vector 12:  ADC12IFG3
    A0results = ADC12MEM0;           		// Move A0 results, IFG is cleared
    A1results = ADC12MEM1;           		// Move A1 results, IFG is cleared
    A2results = ADC12MEM2;           		// Move A2 results, IFG is cleared
    A3results = ADC12MEM3;           		// Move A3 results, IFG is cleared
											// Increment results index, modulo; Set Breakpoint1 here

  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break; 
  }  
}

// Watchdog Timer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) WDT_ISR (void)
#else
#error Compiler not supported!
#endif
{
  updateParkingStatus();                            
}	