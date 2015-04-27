#include "msp430x22x4.h"
#include "stdint.h"

#define CLK 1200000
#define MAX_TICKS 10000        // Blink length (loop passes)
#define MAX_RANGE 100

volatile uint16_t pulse_count[3];      //Global Pulse Count
volatile float dist[3];             //Global Frequency
volatile uint32_t fallingEdge[3];
volatile uint32_t risingEdge[3];
volatile uint32_t i;
volatile uint16_t cycles[3];
volatile uint8_t edge[3];
volatile float pinger[3];

// STATE MACHINE VARIBLES //
volatile uint8_t CurrentState;
volatile uint8_t TurnCounter;

void TimerReadPinger( uint8_t ping_num )
{
  uint32_t cur_ccr_val = TACCR0;
  //fetch the correct value based on ping_num
  if (ping_num == 0)
  {
    cur_ccr_val = TACCR0;
  }
  else if (ping_num == 1)
  {
    cur_ccr_val = TACCR1;
  }
  else if ( ping_num == 2)
  {
    cur_ccr_val = TBCCR0;
  }
  
  //increase the count of total echos we've seen
  pulse_count[ping_num] += 1;
   
  //rising edge
  if (edge[ping_num] == 0)
  {
    risingEdge[ping_num] = cur_ccr_val;
  }
  //falling edge
  else
  {
    fallingEdge[ping_num] = cur_ccr_val;
    if ( fallingEdge[ping_num] < risingEdge[ping_num] )
    {
          cycles[1] = ( 0xFFFF - risingEdge[ping_num] ) + fallingEdge[ping_num]; //find the cycles to max and then add our current count
    }
    //not rollover
    else
    {
          cycles[ping_num] = fallingEdge[ping_num] - risingEdge[ping_num]; //get the number of cycles that have elapsed
    }
  }
    
  edge[ping_num] = !edge[ping_num];
  
  if (ping_num != 2)
  {
    TACTL &= 0xFFFE; 				  //reset irq value
  }
  else
  {
    TBCTL &= 0xFFFE; 				  //reset irq valu
  }
}

#pragma vector=TIMERA0_VECTOR
__interrupt void Isrtimera0 (void)
{
  TimerReadPinger( 0 );
}

#pragma vector=TIMERB0_VECTOR
__interrupt void Isrtimerb0 (void)
{
  P1OUT |= 0x01;
  TimerReadPinger( 2 );
}

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
        TimerReadPinger( 1 );
      break;
    case TAIV_TACCR2:                 // ignore chnl 2 IRQ
    case TAIV_TAIFG:                  // ignore TAR rollover IRQ
    default:                          // ignore everything else
      break;
  }
}

uint8_t MotorController (uint8_t MotorSelect, uint8_t MotorSpeed)
//------------------------------------------------------------------------
// Func:  Easy Motor Controller
// Args:  uint8_t MotorSelect (0 = Motor 1, 1 = Motor 2)
//        uint8_t MotorSpeed (1 = Full Reverse, 64 = Stop, 127 = Full Forward)
// Retn:  0 Successful Exit
//        1 Motor Select Failure (Something other than 0 or 1 sent in)
//------------------------------------------------------------------------
{
    if((MotorSelect == 0) || (MotorSelect == 1))
    {
      if(MotorSelect == 0)
      {
        while ( !(IFG2 & UCA0TXIFG)) {};    // Confirm that Tx Buff is empty
	UCA0TXBUF = MotorSpeed;             // Set motor speed to inputted speed
			  
	return 0;
      }
      else
      {
        while ( !(IFG2 & UCA0TXIFG)) {};    // Confirm that Tx Buff is empty
	UCA0TXBUF = MotorSpeed + 128;       // Inputted motor speed
	return 0;
      }
    }
    else
    {
      return 1;
    }
}

void HallwayLogic(uint8_t StateMachine)
//------------------------------------------------------------------------
// Func:  Run the state machine for the robot
// Args:  uint8_t StateMachine 
//                - 0 (NOP STATE): NOP Mode
//                - 1 (STRAIGHT MODE): This is the default mode of operation
//                - 2 (TURN MODE): Enter turning mode
//                - 3 (STOP MODE): Stop the robot
// Retn:  None
//------------------------------------------------------------------------
{
  uint8_t MotorOneReturn = 0;
  uint8_t MotorTwoReturn = 0;
  CurrentState = 0;

  if(StateMachine == 1)
  {
    while(!MotorController(0, 100)) {};
    while(!MotorController(1, 100)) {};
  }
  else if(StateMachine == 2)
  {
    TurnCounter++;
    
    while(!MotorController(0, 100)) {};
    while(!MotorController(1, 80)) {};
  }
  else if(StateMachine == 3)
  {
    while(!MotorController(0, 0)) {};
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
  P2DIR &= ~0x0C;                     // P2.3 % 2.2 = Input
  P2SEL |= 0x0C;                      // P2.3 & 2.2 = TA1 & TA0 = TA compare OUT1
  P2DIR |= 0x13;
  P2OUT |= 0x13;                      // Toggle P2.0 = toggle LED
  P4DIR &= ~0x08;                     // P4.3 = Input
  P4SEL |= 0x08;                      // P4.3 = TB0 = TB compare OUT1
}

void SetupBasicFunc (void)
{
  TACTL   = TASSEL_2 | ID_0 | MC_2;          // SMCLK | Div by 1 | Contin Mode
  TACCTL0 = CM0 | CM1 | CCIS0 | CAP | SCS | CCIE;  // Ris Edge | Falling Edge | inp = CCI1B | 
                                             // Capture | Sync Cap | Enab IRQ
  TACCTL1 = CM0 | CM1 | CCIS0 | CAP | SCS | CCIE;  // Ris Edge | Falling Edge | inp = CCI1B | 
                                             // Capture | Sync Cap | Enab IRQ
  TBCTL   = TASSEL_2 | ID_0 | MC_2;          // SMCLK | Div by 1 | Contin Mode
  TBCCTL0 = CM0 | CM1 | CCIS0 | CAP | SCS | CCIE;  // Ris Edge | Falling Edge | inp = CCI1B | 
                                             // Capture | Sync Cap | Enab IRQ

  pulse_count[0] = 0;                           // Init input pulse counter
  dist[0] = 0;                                  //Init distuency
  pinger[0] = 0;
  pinger[1] = 0;
  pinger[2] = 0;
  fallingEdge[0] = 0;
  risingEdge[0] = 0;
  i=0;
  edge[0] = 0;
  _BIS_SR(GIE);                          // IRQs enab
}

void CalculateDist( uint8_t ping_num )
{
  dist[ping_num] = (float)cycles[ping_num] / (float)CLK;
  dist[ping_num] = dist[ping_num] * 1000000;
  dist[ping_num] = dist[ping_num] / 148.;
  
  float dif = pinger[ping_num] - dist[ping_num];
  if (dif < 0)
  {
    dif = dif * -1;
  }
  
  if (dif > 10 && dif < MAX_RANGE && pinger[ping_num] != 0 )
  {
    pinger[ping_num] = pinger[ping_num];
  }
  else
  {
    pinger[ping_num] = dist[ping_num];
  }
}

void StartPinger( uint8_t ping_num )
{
  if (ping_num == 0)
  {
    P2OUT |= 0x01;                          // Set Pin High P2.0
  }
  if (ping_num == 1)
  {
    P2OUT |= 0x02;
  }
  if (ping_num == 2)
  {
    P2OUT |= 0x10;
  }
  for (i = 100; i != 0; i--) {} ;   // Empty S-Ware delay loop
  if (ping_num == 0)
  {
    P2OUT &= ~0x01;                          // Set Pin Low P2.0
  }
  if (ping_num == 1)
  {
    P2OUT &= ~0x02;
  }
  if (ping_num == 2)
  {
    P2OUT &= ~0x10;
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
  CurrentState = 0;
  P1OUT &= ~0x01;
  while(1)
  {
    StartPinger(0);
    StartPinger(1);
    StartPinger(2);
    CalculateDist(0);
    CalculateDist(1);
    CalculateDist(2);
  }

  //HallwayLogic(1);

  _BIS_SR(LPM1_bits + GIE);                  // Enter LPM w/ IRQs enab
}
