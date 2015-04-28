#include "msp430x22x4.h"
#include "stdint.h"

#define CLK 1200000
#define MAX_TICKS 2000        // Blink length (loop passes)
#define MAX_RANGE 300

volatile uint16_t pulse_count[3];      //Global Pulse Count
volatile float dist[3];             //Global Frequency
volatile uint32_t fallingEdge[3];
volatile uint32_t risingEdge[3];
volatile uint32_t i;
volatile uint16_t cycles[3];
volatile uint8_t edge[3];
volatile float pinger[3];
volatile float history[9];
volatile uint8_t waiting;
volatile uint32_t left_motor;
volatile uint32_t right_motor;

volatile uint8_t stopCondition;
volatile uint8_t dodgeCondition;

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
          cycles[ping_num] = ( 0xFFFFFFFF - risingEdge[ping_num] ) + fallingEdge[ping_num]; //find the cycles to max and then add our current count
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
  waiting = 0;
}

#pragma vector=TIMERB0_VECTOR
__interrupt void Isrtimerb0 (void)
{
  //P1OUT |= 0x01;
  //TimerReadPinger( 0 );
  waiting = 0;
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
        waiting = 0;
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
        right_motor = MotorSpeed;
			  
	return 0;
      }
      else
      {
        while ( !(IFG2 & UCA0TXIFG)) {};    // Confirm that Tx Buff is empty
	UCA0TXBUF = MotorSpeed + 128;       // Inputted motor speed
        left_motor = MotorSpeed;
	return 0;
      }
    }
    else
    {
      return 1;
    }
}

float VoteForPinger( uint8_t ping_num )
{
  float diff1 = history[ping_num*3] - history[ping_num*3+1];
  float diff2 = history[ping_num*3+1] - history[ping_num*3+2];
  float diff3 = history[ping_num*3] - history[ping_num*3+2];
  
  if (diff1 < 0)
  {
    diff1 = diff1 * -1;
  }
  
  if (diff2 < 0)
  {
    diff2 = diff2 * -1;
  }
  
  if (diff3 < 0)
  {
    diff3 = diff3 * -1;
  }
  
  if ( diff1 < diff2 && diff1 < diff3 )
  {
    return history[ping_num*3];
  }
  else if ( diff2 < diff1 && diff2 < diff3 )
  {
    return history[ping_num*3+1];
  }
  else if ( diff3 < diff1 && diff3 < diff2 )
  {
    return history[ping_num*3+2];
  }
  else
  {
    return pinger[ping_num];
  }
}

void CalculateDist( uint8_t ping_num )
{
  //dist[ping_num] = (float)cycles[ping_num] / (float)CLK;
  //dist[ping_num] = dist[ping_num] * 1000000;
  //dist[ping_num] = dist[ping_num] / 12;
  //dist[ping_num] = dist[ping_num] / 148.;
  //dist[ping_num] = cycles[ping_num] / 177.6;
  
  float dif = pinger[ping_num] - dist[ping_num];
  if (dif < 0)
  {
    dif = dif * -1;
  }
  
 // if (dist[ping_num] > 100 || dist[ping_num] < 1)
  //{
    //return;
  //}
  
  //update the history
  history[ping_num*3] = history[ping_num*3+1];
  history[ping_num*3+1] = history[ping_num*3+2];
  history[ping_num*3+2] = cycles[ping_num];
  
  pinger[ping_num] = VoteForPinger(ping_num);
  
  /*if (dif > 10 && dif < MAX_RANGE && pinger[ping_num] != 0 )
  {
    pinger[ping_num] = pinger[ping_num];
  }
  else
  {
    pinger[ping_num] = dist[ping_num];
  }*/
}

void StartPinger( uint8_t ping_num )
{
  //left
  if (ping_num == 1)
  {
    P2OUT |= 0x01;                          // Set Pin High P2.0
  }
  //front
  if (ping_num == 0)
  {
    P2OUT |= 0x10;
  }
  //right
  if (ping_num == 2)
  {
    P2OUT |= 0x02;
  }
  for (i = 1000; i != 0; i--) {} ;   // Empty S-Ware delay loop
  if (ping_num == 1)
  {
    P2OUT &= ~0x01;                          // Set Pin Low P2.0
  }
  if (ping_num == 0)
  {
    P2OUT &= ~0x10;
  }
  if (ping_num == 2)
  {
    P2OUT &= ~0x02;
  }
  for (i = MAX_TICKS; i != 0; i--) {} ;   // Empty S-Ware delay loop
  
  //have an emtpy loop to let right/left pings disapate
  if (ping_num != 3)
  {
    CalculateDist(ping_num);
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
//                - 4 
// Retn:  None
//------------------------------------------------------------------------
{
  uint8_t MotorOneReturn = 0;
  uint8_t MotorTwoReturn = 0;
  CurrentState = 0;

  if(StateMachine == 1)
  {
    MotorController(0, 48);
    MotorController(1, 52);
    P1OUT |= 0x01;                      // Start of TX => toggle LEDs
    P1OUT &= ~0x02;                      // Start of TX => toggle LEDs
  }
  else if(StateMachine == 2)
  {
    TurnCounter++;
    uint8_t pinger_sel = 0;
    
    MotorController(0, 20);
    MotorController(1, 58);
    P1OUT |= 0x02;                      // Start of TX => toggle LEDs
    P1OUT &= ~0x01;                      // Start of TX => toggle LEDs
    for (i = MAX_TICKS*6.5; i != 0; i--) {} ;   // Empty S-Ware delay loop
    
    //MotorController(0, 32);
    //MotorController(1, 32);
    
    for (i = MAX_TICKS*4; i != 0; i--) {
    } ;   // Empty S-Ware delay loop;
  }
  else if(StateMachine == 3)
  {
    MotorController(0, 64);
    MotorController(1, 64);
    P1OUT |= 0x03;                      // Start of TX => toggle LEDs
    
    for (i = MAX_TICKS*4; i != 0; i--) {} ;   // Empty S-Ware delay loop
    
    MotorController(0, 80);
    MotorController(1, 80);
        
    do 
    {
      StartPinger(0);
      for (i = MAX_TICKS; i != 0; i--) {} ;   // Empty S-Ware delay loop
    }
    while(pinger[0] < 6500);
    
    stopCondition++;
    
    
    
  }
  //dodge ALL the obstacles
  else if (StateMachine == 4)
  {
    //head right
      MotorController(0, 47);
      MotorController(1, 5);
    
    do 
    {
      StartPinger(0);
      for (i = MAX_TICKS; i != 0; i--) {} ;   // Empty S-Ware delay loop
    }
    while(pinger[0] < 5200);
    
    /*MotorController(0, 40);
    MotorController(1, 40);
    
    do 
    {
      StartPinger(1);
      for (i = MAX_TICKS; i != 0; i--) {} ;   // Empty S-Ware delay loop
    }
    while(pinger[1] < 585);*/
    
    
    /*MotorController(0, 30);
    MotorController(1, 40);
    
    do 
    {
      StartPinger(1);
      for (i = MAX_TICKS*8; i != 0; i--) {} ;   // Empty S-Ware delay loop
    }
    while(pinger[1] < 1770);*/
    
    
    
    //for (i = MAX_TICKS*7; i != 0; i--) {} ;   // Empty S-Ware delay loop
    
    //go straight
    //MotorController(0, 30);
    //MotorController(1, 30);
    //for (i = MAX_TICKS*7; i != 0; i--) {} ;   // Empty S-Ware delay loop
    
    //head left
    //MotorController(0, 5);
    //MotorController(1, 40);
    //for (i = MAX_TICKS*7; i != 0; i--) {} ;   // Empty S-Ware delay loop
    dodgeCondition++;
  }
  else
  {
    P1OUT &= ~0x03;                      // Start of TX => toggle LEDs
  }
}



void InitPorts (void)
//------------------------------------------------------------------------
// Func:  Initialize the ports for I/O on TA1 Capture
// Args:  None
// Retn:  None
//------------------------------------------------------------------------
{
  P1DIR |= 0x03;                      // Config P1.0 as Output (LED)
  P2DIR &= ~0x0C;                     // P2.3 % 2.2 = Input
  P2SEL |= 0x0C;                      // P2.3 & 2.2 = TA1 & TA0 = TA compare OUT1
  P2DIR |= 0x13;
  P2OUT |= 0x13;                      // Toggle P2.0 = toggle LED
  P4DIR &= ~0x08;                     // P4.3 = Input
  P4SEL |= 0x08;                      // P4.3 = TB0 = TB compare OUT1
  P3SEL = 0x30;                       // P3.4,5 = USCI_A0 TXD/RXD
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
  
  // Config. UART Clock & Baud Rate 
  BCSCTL1 = CALBC1_1MHZ;                // DCO = 1 MHz
  DCOCTL  = CALDCO_1MHZ;                // DCO = 1 MHz
  UCA0CTL1 |= UCSSEL_2;                 // UART use SMCLK

  UCA0MCTL = UCBRS0;                    // Map 1MHz -> 9600 (Tbl 15-4)
  UCA0BR0  = 104;                       // Map 1MHz -> 9600 (Tbl 15-4)
  UCA0BR1  = 0;                         // Map 1MHz -> 9600 (Tbl 15-4)

  UCA0CTL1 &= ~UCSWRST;                 // Enable USCI state mach

  while ( !(IFG2 & UCA0TXIFG)) {};      // Confirm that Tx Buff is empty
  UCA0TXBUF = 0x00;                     // Init robot to stopped state

  pulse_count[0] = 0;                           // Init input pulse counter
  dist[0] = 0;                                  //Init distuency
  pinger[0] = 0;
  pinger[1] = 0;
  pinger[2] = 0;
  cycles[0] = 0;
  cycles[1] = 0;
  cycles[2] = 0;
  fallingEdge[0] = 0;
  risingEdge[0] = 0;
  for(i=0; i < 9; i++)
  {
    history[i] = 0;
  }
  i=0;
  edge[0] = 0;
  
  stopCondition = 0;
  
  _BIS_SR(GIE);                          // IRQs enab
}

void CorrectionLogic(void)
//------------------------------------------------------------------------
// Func:  Init I/O ports & IRQs, enter LoPwr Mode
// Args:  None
// Retn:  None
// Design Note: pinger[1] = Left Pinger
//------------------------------------------------------------------------
{
   //sweet spot
  if (pinger[1] > 2200 && pinger[1] < 2700)
  {
	MotorController(0, 40);
	MotorController(1, 40);
  }
  //to close to left
  else if( pinger[1] < 2200 )
  {
	if(pinger[1] > 1500)
	{
	  MotorController(0, 45);  //right motor
	  MotorController(1, 35);
	  P1OUT &= ~0x03;
	}// Start of TX => toggle LEDs
	else if(pinger[1] > 1000)
	{
	  MotorController(0, 50);  //right motor
	  MotorController(1, 30);
	}
	else if(pinger[1] > 500)
	{
	  MotorController(0, 55);  //right motor
	  MotorController(1, 25);
	}
	else
	{
	  MotorController(0,60);
	  MotorController(1,20);
	}
  }
  //to far right
  else if ( pinger[1] > 2700 )
  {
	if(pinger[1] > 5000)
	{
	  MotorController(0, 20);
	  MotorController(1, 60);  
	}
	else if(pinger[1] > 4200)
	{
	  MotorController(0, 25);  //right motor
	  MotorController(1, 55);
	}
	else if(pinger[1] > 3200)
	{
	  MotorController(0, 30);  //right motor
	  MotorController(1, 50);
	}
	else
	{
	  MotorController(0,35);
	  MotorController(1,45);
	}                     // Start of TX => toggle LEDs
  }
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
  uint8_t pinger_sel = 1;
  uint8_t j = 0;
  waiting = 0;
  
  for (i = 5000; i != 0; i--) {} ;
  
  //while(pinger[0] == 0 && pinger[1] == 0 && pinger[2] == 0)
  for (i = 50; i != 0; i--) {} ;
  {
    //P2OUT ^= 0x03;
    StartPinger(j);
    j++;
    j=j%2;
  }
  
  //while(1){};
  
  while(1)
  {
    StartPinger(pinger_sel);
    
    if (pinger[0] < 1770 && pinger[0] != 0)
    {
      //force stop if we're to close
      HallwayLogic(3);
      j=0;
      //while(j<5000){j++;};
    }
    /*
    else if (pinger[1] > 6500)
    {
      HallwayLogic(2);
    }
    */
    //dodge, dodge, dodge
    else if (pinger[0] < 5200 && pinger[0] != 0)
    {
      HallwayLogic(4);
    }
    //the hallway sensor is reading high
    
    else
    {
     CorrectionLogic();
    }
    pinger_sel++;
    pinger_sel = pinger_sel % 2;
    //pinger_sel = 1;
  }
  
  _BIS_SR(LPM1_bits + GIE);                  // Enter LPM w/ IRQs enab
}
