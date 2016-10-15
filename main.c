

#include "msp430x54xA.h"
#include "lcd.c"
void openGate();
void closeGate();
void delay(long value);
int temp = 0;
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
  halLcdClearScreen();

  // Step Motor Test
  /*
  P5DIR |= BIT0 ;
  P6DIR |= BIT7 ;
  P7DIR |= BIT5 ;
  P7DIR |= BIT7 ;
  */
  
  

    P5DIR |= BIT0 ;
  // Control Led test 
  while(1){

  P5OUT |= BIT0 ;
 
  delay();

  P5OUT &= ~BIT0 ;
 
  delay();
  }
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
void delay(long value){
	// At clk 16MHZ:
	// value = 2000000 ~ 500ms
	// value = 
	long ind = 0;
	for(ind = 0 ; ind < value; ind++ );	
}

void openGate(){
	for (int ind = 0; ind < 124 ; ind++){
    
  P5OUT |= BIT0 ;
  delay(20000);
  P5OUT &= ~BIT0 ;
  
  P6OUT |= BIT7 ;
  delay(20000);
  P6OUT &= ~BIT7 ;
  
  P7OUT |= BIT5 ;
  delay(20000);
  P7OUT &= ~BIT5 ;
  
  P7OUT |= BIT7 ;
  delay(200004); 
  P7OUT &= ~BIT7 ;
    
  }
}
void closeGate(){
	for ( int ind = 0; ind < 124 ; ind++){
      
	  P7OUT |= BIT7 ;
	  delay(); 
	  P7OUT &= ~BIT7 ;   
		  
	  P7OUT |= BIT5 ;
	  delay();
	  P7OUT &= ~BIT5 ;
	  
	  P6OUT |= BIT7 ;
	  delay();
	  P6OUT &= ~BIT7 ; 
	  
	  P5OUT |= BIT0 ;
	  delay();
	  P5OUT &= ~BIT0 ;
  }
}


// Echo back RXed character, confirm TX buffer is ready first
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