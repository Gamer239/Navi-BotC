#include "msp430x22x4.h"
#include "stdint.h"

#define CLK 1200000
#define MAX_TICKS 2000        // Blink length (loop passes)
#define MAX_RANGE 300
#define FRONT_PINGER pinger[0]
#define LEFT_PINGER pinger[1]

volatile uint16_t pulseCount[3];      //Global Pulse Count
volatile uint32_t fallingEdge[3];
volatile uint32_t risingEdge[3];
volatile uint32_t i;
volatile uint16_t cycles[3];
volatile uint8_t edge[3];
volatile float pinger[3];
volatile float history[9];
volatile uint32_t leftMotor;
volatile uint32_t rightMotor;

// STATE MACHINE VARIBLES //
volatile uint8_t currentState;
volatile uint8_t turnCounter;

void TimerReadPinger( uint8_t pingNum )
//------------------------------------------------------------------------
// Func:  Process the given pinger's edge to determine the number of
//        cycles that have elapsed
// Args:  pingNum = the number of the pinger to read/save the results to
// Retn:  None
//------------------------------------------------------------------------
{
  uint32_t curCcrVal = TACCR0;
  //fetch the correct value based on pingNum
  if (pingNum == 0)
  {
    curCcrVal = TACCR0;
  }
  else if (pingNum == 1)
  {
    curCcrVal = TACCR1;
  }
  else if ( pingNum == 2)
  {
    curCcrVal = TBCCR0;
  }
  
  //increase the count of total echos we've seen
  pulseCount[pingNum] += 1;
   
  //rising edge
  if (edge[pingNum] == 0)
  {
    risingEdge[pingNum] = curCcrVal;
  }
  //falling edge
  else
  {
    fallingEdge[pingNum] = curCcrVal;
    if ( fallingEdge[pingNum] < risingEdge[pingNum] )
    {
      //find the cycles to max and then add our current count
      cycles[pingNum] = ( 0xFFFFFFFF - risingEdge[pingNum] ) + 
        fallingEdge[pingNum]; 
    }
    //not rollover
    else
    {
      //get the number of cycles that have elapsed
      cycles[pingNum] = fallingEdge[pingNum] - risingEdge[pingNum]; 
    }
  }
    
  edge[pingNum] = !edge[pingNum];                 //flip the current edge
  
  if (pingNum != 2)
  {
    TACTL &= 0xFFFE; 				  //reset irq value
  }
  else
  {
    TBCTL &= 0xFFFE; 				  //reset irq value
  }
  
}

#pragma vector=TIMERA0_VECTOR
__interrupt void IsrTimerA0 (void)
//------------------------------------------------------------------------
// Func:  At TACCR0 IRQ, get pulse esge and compute pulse length
// Args:  None
// Retn:  None
//------------------------------------------------------------------------
{
  TimerReadPinger( 0 );
}

#pragma vector=TIMERA1_VECTOR
__interrupt void IsrCntPulseTACC1 (void) 
//--------------------------------------------------------------------------
// Func:  At TACCR1 IRQ, get pulse edge and compute pulse length 
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

uint8_t MotorController (uint8_t motorSelect, uint8_t motorSpeed)
//------------------------------------------------------------------------
// Func:  Easy Motor Controller
// Args:  motorSelect = (0 = Motor 1, 1 = Motor 2)
//        motorSpeed = (1 = Full Forward, 64 = Stop, 127 = Full Reverse)
// Retn:  the success or failure of setting  the motor's speed
//------------------------------------------------------------------------
{
  if((motorSelect == 0) || (motorSelect == 1))
  {
    if(motorSelect == 0)
    {
      while ( !(IFG2 & UCA0TXIFG)) {};    // Confirm that Tx Buff is empty
      UCA0TXBUF = motorSpeed;             // Set motor speed to inputted speed
      rightMotor = motorSpeed;
      return 0;
    }
    else
    {
      while ( !(IFG2 & UCA0TXIFG)) {};    // Confirm that Tx Buff is empty
      UCA0TXBUF = motorSpeed + 128;       // Inputted motor speed
      leftMotor = motorSpeed;
      return 0;
    }
  }
  else
  {
    return 1;
  }
}

float VoteForPinger( uint8_t pingNum )
//------------------------------------------------------------------------
// Func:  Figure out the requested pingers next value by finding the
//        the closest two values in the history to help ignore spikes
//        in the readings
// Args:  pingNum = the number of the pinger to read/save the results to
// Retn:  the selected value based on the difference of the history values
//------------------------------------------------------------------------
{
  //compute the difference if each of the pingers from the others
  float diff1 = history[pingNum*3] - history[pingNum*3 + 1];
  float diff2 = history[pingNum*3 + 1] - history[pingNum*3 + 2];
  float diff3 = history[pingNum*3] - history[pingNum*3 + 2];
  
  //get the absolute value of each of the differences
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
  
  //check to see if the difference of the oldest two history
  //values is the smallest
  if ( diff1 < diff2 && diff1 < diff3 )
  {
    return history[pingNum*3];
  }
  else if ( diff2 < diff1 && diff2 < diff3 )
  {
    return history[pingNum*3 + 1];
  }
  else if ( diff3 < diff1 && diff3 < diff2 )
  {
    return history[pingNum*3 + 2];
  }
  else
  {
    return pinger[pingNum];
  }
}

void CalculateDist( uint8_t pingNum )
//------------------------------------------------------------------------
// Func:  Compute the next pinger value using the history 
// Args:  pingNum = the number of the pinger to read/save the results to
// Retn:  None
//------------------------------------------------------------------------
{    
  //update the history of the past seen pinger readings
  history[pingNum*3]     = history[pingNum*3 + 1];
  history[pingNum*3 + 1] = history[pingNum*3 + 2];
  history[pingNum*3 + 2] = cycles[pingNum];
  
  pinger[pingNum] = VoteForPinger(pingNum);
}

void StartPinger( uint8_t pingNum )
//------------------------------------------------------------------------
// Func:  Sends the gpio signal to start the pinger and then give it time
//        to return a result that is then computed
// Args:  pingNum = the number of the pinger to read/save the results to
// Retn:  None
//------------------------------------------------------------------------
{
  //set the GPIO high
  //front
  if (pingNum == 0)
  {
    P2OUT |= 0x10;                            // Set Pin High P2.1
  }
  //left
  if (pingNum == 1)
  {
    P2OUT |= 0x01;                            // Set Pin High P2.0
  }

  for (i = 1000; i != 0; i--) {} ;            // Wait for at least 10us

  //set the GPIO low
  if (pingNum == 0)
  {
    P2OUT &= ~0x10;                           // Set Pin Low P2.1
  }
  if (pingNum == 1)
  {
    P2OUT &= ~0x01;                           // Set Pin Low P2.0
  }

  for (i = MAX_TICKS; i != 0; i--) {} ;       // Wait for a response
  
  //Compute the result
  if (pingNum != 3)
  {
    CalculateDist(pingNum);
  }
}

void HallwayLogic(uint8_t stateMachine)
//------------------------------------------------------------------------
// Func:  Run the state machine for the robot
// Args:  stateMachine = runs the logic for the requested state
// Retn:  None
//------------------------------------------------------------------------
{
  currentState = 0;
  //1 (STRAIGHT MODE): This is the default mode of operation
  if(stateMachine == 1)
  {
    MotorController(0, 48);
    MotorController(1, 52);
    P1OUT |= 0x01;                            // Start of TX => toggle LEDs
    P1OUT &= ~0x02;                           // Start of TX => toggle LEDs
  }
  //2 (TURN MODE): Enter turning mode
  else if(stateMachine == 2)
  { 
    MotorController(0, 20);
    MotorController(1, 58);
    P1OUT |= 0x02;                            // Start of TX => toggle LEDs
    P1OUT &= ~0x01;                           // Start of TX => toggle LEDs
    for (i = MAX_TICKS*6.5; i != 0; i--) {} ; // Empty S-Ware delay loop
    for (i = MAX_TICKS*4; i != 0; i--) {} ;   // Empty S-Ware delay loop;
  }
  //3 (STOP MODE): Collision Correction
  else if(stateMachine == 3)
  {
    MotorController(0, 64);                   //Stop Motors
    MotorController(1, 64);
    P1OUT |= 0x03;                            // Start of TX => toggle LEDs
    
    for (i = MAX_TICKS*4; i != 0; i--) {} ;   // Empty S-Ware delay loop
    
    MotorController(0, 90);                   //Reverse to allow for correction
    MotorController(1, 90);
        
    do 
    {
      StartPinger(0);
      for (i = MAX_TICKS; i != 0; i--) {} ;   // Empty S-Ware delay loop
    }
    while(FRONT_PINGER < 3800);
  }
  //4 (DODGE MODE): Dodge the robot
  else if (stateMachine == 4)
  {
    //head right
    MotorController(0, 70);
    MotorController(1, 5);
    
    do 
    {
      StartPinger(0);
      for (i = MAX_TICKS; i != 0; i--) {} ;   // Empty S-Ware delay loop
      
      StartPinger(1);
      for (i = MAX_TICKS; i != 0; i--) {} ;   // Empty S-Ware delay loop
    }
    while(FRONT_PINGER < 4000 && LEFT_PINGER < 2000 );
  }
  else
  {
    P1OUT &= ~0x03;                           // Start of TX => toggle LEDs
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
  P2DIR &= ~0x0C;                     // P2.3 & 2 = Input
  P2SEL |= 0x0C;                      // P2.3 & 2 = TA1 & TA0 = TA compare OUT1
  P2DIR |= 0x13;                      // P2.0 & 2.1 & 2.4 = Output
  P2OUT |= 0x13;                      // P2.0 & 2.1 & 2.4 = Output
  P3SEL = 0x30;                       // P3.4,5 = USCI_A0 TXD/RXD
}


void SetupBasicFunc (void)
//------------------------------------------------------------------------
// Func:  Initialize the ports for I/O on TA1 and TA0 and UART
// Args:  None
// Retn:  None
//------------------------------------------------------------------------
{
  TACTL   = TASSEL_2 | ID_0 | MC_2;          // SMCLK | Div by 1 | Contin Mode
  TACCTL0 = CM0 | CM1 | CCIS0 | CAP | SCS | CCIE;  // Ris Edge | Falling Edge |
                                             // inp = CCI0B | Capture |
                                             // Sync Cap | Enab IRQ
  TACCTL1 = CM0 | CM1 | CCIS0 | CAP | SCS | CCIE;  // Ris Edge | Falling Edge | 
                                             // inp = CCI1B | Capture |
                                             // Sync Cap | Enab IRQ
  
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

  //initialize all of our used variables to 0
  pulseCount[0] = 0;                   // Init input pulse counter
  FRONT_PINGER = 0;
  LEFT_PINGER = 0;
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
  
  _BIS_SR(GIE);                          // IRQs enab
}

void CorrectionLogic(void)
//------------------------------------------------------------------------
// Func:  Determine and apply the correction bias needed to stay close
//        to the wall. This will allow us to detect turns easier.
// Args:  None
// Retn:  None
//------------------------------------------------------------------------
{
  //sweet spot
  if (LEFT_PINGER > 2200 && LEFT_PINGER < 2700)
  {
    MotorController(0, 40);
    MotorController(1, 40);
  }
  //to close to left
  else if( LEFT_PINGER < 2200 )
  {
    if(LEFT_PINGER > 1500)
    {
      MotorController(0, 45);  //right motor
      MotorController(1, 35);
      P1OUT &= ~0x03;
    }
    else if(LEFT_PINGER > 1000)
    {
      MotorController(0, 50);  //right motor
      MotorController(1, 30);
    }
    else if(LEFT_PINGER > 500)
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
  else if ( LEFT_PINGER > 2700 )
  {
    if(LEFT_PINGER > 4500)
    {
      MotorController(0, 20);
      MotorController(1, 60);  
    }
    else if(LEFT_PINGER > 4000)
    {
      MotorController(0, 25);  //right motor
      MotorController(1, 55);
    }
    else if(LEFT_PINGER > 3200)
    {
      MotorController(0, 30);  //right motor
      MotorController(1, 50);
    }
    else
    {
      MotorController(0,35);
      MotorController(1,45);
    }
  }
}


void main(void)
//------------------------------------------------------------------------
// Func:  Init I/O ports & IRQs, enter LoPwr Mode
// Args:  None
// Retn:  None
//------------------------------------------------------------------------
{ WDTCTL = WDTPW | WDTHOLD;                  //Stop Watchdog Timer
   
  InitPorts();                               //Configure I/O Pins
  SetupBasicFunc();                          //Set default values and configure
                                             //timers and uart
  currentState = 0;
  P1OUT &= ~0x01;
  uint8_t pingerSel = 1;
  uint8_t j = 0;
  
  //allow our devices to warmup
  for (i = 5000; i != 0; i--) {} ;
  
  //get initial readings from our pingers before moving
  for (i = 50; i != 0; i--)
  {
    StartPinger(j);
    j++;
    j=j%2;
  }
  
  while(1)
  {
    StartPinger(pingerSel);
    
     //force stop if we're to close
    if (FRONT_PINGER < 1770 && FRONT_PINGER != 0)
    {
      HallwayLogic(3);
    }
    //dodge an object
    else if (FRONT_PINGER < 4000 && FRONT_PINGER != 0)
    {
      HallwayLogic(4);
    }
    //the hallway sensor is reading high
    else
    {
     CorrectionLogic();
    }

    //update which pinger to scan next
    pingerSel++;
    pingerSel = pingerSel % 2;
  }
}
