

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
void InitializeMotorsPins();
void InitializeADC12();
void InitializeWD();
void InitializeLEDsPins();
void InitializeUltrasonic();

// 3) Delay
void delay(long value);

// 4) General
void setPinInput(int port, int pin);
void setPinOutput(int port, int pin);
void setPinHigh(int port, int pin);
void setPinLow(int port, int pin);
// Photoresist Sensors


// Variables
volatile int A1result, A2result, A3result, B1result;
// A1result <- P6.0
// A2result <- P6.1
// A3result <- P6.2
// B1result <- P6.3
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

  halLcdPrintLine("Welcome to BGU SmartLot!", 0, 1);						// Welcome message to when booting up
 
  
/////////////////////////////////////////////////////////////////////  
///////////////////////////////////////////////////////////////////// 
	InitializeMotorsPins();
	InitializeADC12();



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
  P6DIR = 0x0F;								// Set P6.0 P6.1 P6.2 P6.3 as input pins
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
void InitializeMotorsPins(){
	// MOTOR 1: 
	// INT1 -> P10.7
	// INT2 -> P10.5
	// INT3 -> P10.3
	// INT4 -> P10.1
	// MOTOR 2: 
	// INT1 -> P10.6
	// INT2 -> P10.4
	// INT3 -> P10.2
	// INT4 -> P10.0
	
	P10SEL = 0x00 ; // All P10 pins are set to general I/O
	P10DIR = 0xFF ; // All P10 pins are configured as outputs
}
void InitializeLEDsPins(){
	
	// A1 RED
	P8SEL &= ~BIT6;
	setPinOutput(8,6);
	setPinLow(8,6);	
	// A2 RED
	P7SEL &= ~BIT3;
	setPinOutput(7,3);
	setPinLow(7,3);
	// A3 RED
	P5SEL &= ~BIT4;
	setPinOutput(5,4);
	setPinLow(5,4);
	// B1 RED
	P4SEL &= ~BIT6;
	setPinOutput(4,6);
	setPinLow(4,6);
	// A1 GREEN
	P8SEL &= ~BIT5;
	setPinOutput(8,5);
	setPinLow(8,5);
	// A2 GREEN
	P5SEL &= ~BIT5;
	setPinOutput(5,5);
	setPinLow(5,5);
	// A3 GREEN
	P4SEL &= ~BIT7;
	setPinOutput(4,7);
	setPinLow(4,7);
	// B1 GREEN
	P4SEL &= ~BIT5;
	setPinOutput(4,5);
	setPinLow(4,5);

}
void InitializeUltrasonic(){
	
  P1DIR |= 0x01;                            // Set P1.0 to output direction
  P1REN |= 0x10;                            // Enable P1.4 internal resistance
  P1OUT |= 0x10;                            // Set P1.4 as pull-Up resistance
  P1IE |= 0x10;                             // P1.4 interrupt enabled
  P1IES |= 0x10;                            // P1.4 Hi/Lo edge
  P1IFG &= ~0x10;                           // P1.4 IFG cleared
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
void rotateLeftM2(int steps){
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
	  setPinHigh(10,6);
	  delay(20000); 
	  setPinLow(10,6);   
		  
	  setPinHigh(10,4);
	  delay(20000);
	  setPinLow(10,4);
	  
	  setPinHigh(10,2);
	  delay(20000);
	  setPinLow(10,2);
	  
	  setPinHigh(10,0);
	  delay(20000);
	  setPinLow(10,0);
	}
}
// General
void setPinInput(int port, int pin){
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
			P1DIR &= ~bit ;
			break;
		case 2:
			P2DIR &= ~bit ;
			break;
		case 3:
			P3DIR &= ~bit ;
			break;
		case 4:
			P4DIR &= ~bit ;
			break;
		case 5:
			P5DIR &= ~bit ;
			break;
		case 6:
			P6DIR &= ~bit ;
			break;
		case 7:
			P7DIR &= ~bit ;
			break;
		case 8:
			P8DIR &= ~bit ;
			break;
		case 9:
			P9DIR &= ~bit ;
			break;
		case 10:
			P10DIR &= ~bit ;
			break;
		case 11:
			P11DIR &= ~bit ;
			break;

	}
}
void setPinOutput(int port, int pin){
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
			P1DIR |= bit ;
			break;
		case 2:
			P2DIR |= bit ;
			break;
		case 3:
			P3DIR |= bit ;
			break;
		case 4:
			P4DIR |= bit ;
			break;
		case 5:
			P5DIR |= bit ;
			break;
		case 6:
			P6DIR |= bit ;
			break;
		case 7:
			P7DIR |= bit ;
			break;
		case 8:
			P8DIR |= bit ;
			break;
		case 9:
			P9DIR |= bit ;
			break;
		case 10:
			P10DIR |= bit ;
			break;
		case 11:
			P11DIR |= bit ;
			break;

	}
}
void setPinHigh(int port, int pin){
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
			P1OUT |= bit ;
			break;
		case 2:
			P2OUT |= bit ;
			break;
		case 3:
			P3OUT |= bit ;
			break;
		case 4:
			P4OUT |= bit ;
			break;
		case 5:
			P5OUT |= bit ;
			break;
		case 6:
			P6OUT |= bit ;
			break;
		case 7:
			P7OUT |= bit ;
			break;
		case 8:
			P8OUT |= bit ;
			break;
		case 9:
			P9OUT |= bit ;
			break;
		case 10:
			P10OUT |= bit ;
			break;
		case 11:
			P11OUT |= bit ;
			break;

	}
}
void setPinLow(int port, int pin){
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
    A1result = ADC12MEM0;           		// Move A0 results, IFG is cleared
    A2result = ADC12MEM1;           		// Move A1 results, IFG is cleared
    A3result = ADC12MEM2;           		// Move A2 results, IFG is cleared
    B1result = ADC12MEM3;           		// Move A3 results, IFG is cleared
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
  //updateParkingStatus();                            
}	
// Port 1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
  P1OUT ^= 0x01;                            // P1.0 = toggle
  P1IFG &= ~0x010;                          // P1.4 IFG cleared
}