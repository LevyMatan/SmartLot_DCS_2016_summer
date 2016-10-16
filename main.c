

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

// Delay
void delay(long value);

// General
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

  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  P5SEL |= BIT6+BIT7;                       // P5.6,p5.7 UART option select
  UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA1CTL1 |= UCSSEL_2;                     // SMCLK
  UCA1CTL0 &= UC7BIT;                       // WordSize = 8 Bit
  UCA1CTL0 &= ~UCSPB;                       // One stop bit
  UCA1CTL0 &= UCPEN;                        // Parity disabled 
  UCA1BR0 = 145;                            // 16MHz 115200 (see User's Guide)
  UCA1BR1 = 0;                              // 16MHz 115200
  UCA1MCTL |= UCBRS_5 + UCBRF_0;            // Modulation UCBRSx=5, UCBRFx=0
  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt

  // Step Motor Test
  /*
  P5DIR |= BIT0 ;
  P6DIR |= BIT7 ;
  P7DIR |= BIT5 ;
  P7DIR |= BIT7 ;
  */
  
  


  // Control Led test 

  // photoResist test
  /*
  ADC12MCTL0 = ADC12INCH_7; 		    // P6.7, Vref+
  ADC12CTL0 = ADC12ON+ADC12SHT0_8+ADC12MSC; // ADC12 on, sampling time, multiple sample conversion
  ADC12CTL1 = ADC12SHP+ADC12CONSEQ_2;       // Use sampling timer, set mode
  ADC12IE = 0xFF;                           // Enable interrupts from a/d ports
  ADC12CTL0 |= ADC12ENC;                    // Enable conversions
  ADC12CTL2 |= ADC12RES_0;
  ADC12CTL2 &= 0xFFF7;
  ADC12CTL0 |= ADC12SC;                     // Start conversion
  */
  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
  __no_operation();                         // For debugger
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
// ========
// Interups
// ========
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

#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
	/*
  avg=0;
  if( adcdelay<100)          //sample and display 100 
    adcdelay++;
  if(adcdelay==100)
  {
    ADC12IE = 0x00; 
    adcdelay=0;
  }  
*/  
  switch(__even_in_range(ADC12IV,34))
  {
  case 6:
    temp = ADC12MEM0;                       // Move results, IFG is cleared
    break;
    
  }
}	