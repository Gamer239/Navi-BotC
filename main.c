#include "msp430x22x4.h"
#include "stdint.h"
//#include <stdio.h>

#define CLK 1200000
#define MAX_TICKS 100000        // Blink length (loop passes)
#define TIMER_A_CTL P2OUT
#define TIMER_B_CTL P4OUT

volatile uint16_t pulse_count;      //Global Pulse Count
volatile float freq;             //Global Frequency
volatile uint32_t currEdgeStmp;
volatile uint32_t prevEdgeStmp;
volatile uint32_t i;
volatile uint16_t cycles;
volatile uint8_t edge;

#pragma vector=TIMERA1_VECTOR
__interrupt void IsrCntPulseTACC1 (void) 
//--------------------------------------------------------------------------
// Func:  At TACCR1 IRQ, increm pulse count & toggle built-in Red LED 
// Args:  None
// Retn:  None
//--------------------------------------------------------------------------
{ 
  switch (__even_in_range(TAIV, 10))  // I.D. source of TA IRQ
  {                 
    case TAIV_TACCR1:                 // handle chnl 1 IRQ
      pulse_count += 1;
      P1OUT ^= 0x01;                  // Toggle P1.0 = toggle LED
      
      //freq += 1;*/
      //rising edge
      if (edge == 0)
      {
        prevEdgeStmp = TACCR1;
      }
      //falling edge
      else
      {
        currEdgeStmp = TACCR1;
        if ( currEdgeStmp < prevEdgeStmp )
        {
              cycles = ( 0xFFFF - prevEdgeStmp ) + currEdgeStmp; //find the cycles to max and then add our current count
        }
        //not rollover
        else
        {
              cycles = currEdgeStmp - prevEdgeStmp; //get the number of cycles that have elapsed
        }
        freq = (float)cycles / (float)CLK;
        freq = freq * 1000000;
        freq = freq / 148.;
        //printf("%f\n", freq);
      }
        
      edge = !edge;
      TACTL &= 0xFFFE; 				  //reset irq value
      break;
    case TAIV_TACCR2:                 // ignore chnl 2 IRQ
    case TAIV_TAIFG:                  // ignore TAR rollover IRQ
    default:                          // ignore everything else
      break;
  }
}

void InitPorts (void)
//------------------------------------------------------------------------
// Func:  Initialize the ports for I/O on TA1 Capture
// Args:  None
// Retn:  None
//------------------------------------------------------------------------
{
  P1DIR |= 0x01;                      // Config P1.0 as Output (LED)
  P2DIR &= ~0x08;                     // P2.3 = Input
  P2SEL |= 0x08;                      // P2.3 = TA1 = TA compare OUT1
  P2DIR |= 0x01;
  P2OUT |= 0x01;                      // Toggle P2.0 = toggle LED
}

void SetupBasicFunc (void)
{
  TACTL   = TASSEL_2 | ID_0 | MC_2;          // SMCLK | Div by 1 | Contin Mode
  TACCTL1 = CM0 | CM1 | CCIS0 | CAP | SCS | CCIE;  // Ris Edge | Falling Edge | inp = CCI1B | 
                                             // Capture | Sync Cap | Enab IRQ

  pulse_count = 0;                           // Init input pulse counter
  freq = 0;                                  //Init frequency
  currEdgeStmp = 0;
  prevEdgeStmp = 0;
  i=0;
  edge = 0;
  _BIS_SR(GIE);                          // IRQs enab
}

void StartPinger( uint8_t ping_num )
{
  if (ping_num == 0)
  {
    P2OUT |= 0x01;                          // Set Pin High P2.0
  }
  for (i = 1000; i != 0; i--) {} ;   // Empty S-Ware delay loop
  if (ping_num == 0)
  {
    P2OUT &= ~0x01;                          // Set Pin Low P2.0
  }
  for (i = MAX_TICKS; i != 0; i--) {} ;   // Empty S-Ware delay loop
}

void main(void)
//------------------------------------------------------------------------
// Func:  Init I/O ports & IRQs, enter LoPwr Mode
// Args:  None
// Retn:  None
//------------------------------------------------------------------------
{ WDTCTL = WDTPW | WDTHOLD;                  //Stop Watchdog Timer
   
  InitPorts();                               //  Configure I/O Pins
  SetupBasicFunc();

  while(1)
  {
    StartPinger(0);
  }
  _BIS_SR(LPM1_bits + GIE);                  // Enter LPM w/ IRQs enab
}

