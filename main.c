
//Taken from: http://processors.wiki.ti.com/index.php/MSP430_LaunchPad_PushButton
//comments have been modified

//David Sheppard

//Final Project


#include <msp430f5529.h>


//definitions:
#define BUTTON1 BIT2    // button1 = 1.2
#define BUTTON2 BIT3    // button2 = 1.3
#define BUTTON3 BIT4    // button3 = 1.4
#define PIR BIT5        // PIR = 1.5
#define BUZZER BIT0     // buzzer = 3.0
#define CORRECT BIT1    // status LED = 3.1

//global vars:
const unsigned int combination_key[5] = {1,2,3,1,1};    // default combination
unsigned int combination[5] = {0};                      // combination that has been entered
unsigned int combination_digits_entered = 0;            // number of digits that have been entered thus far
unsigned int triggered = 0;                             // 1 if PIR has gone off and not reset yet
unsigned int counter = 0;                               // used for counting number of times timer CCR) has overflowed
unsigned int timerdone = 1;                             // true if timer has counted for 10 seconds



//prototypes
void alert(int);

int main(void)  //begin main function
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

// SET UP UART***********************************************************************************
    P4SEL |= BIT5 + BIT4;                     //enable UART for these pins
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;
    UCA1BR0 = 9;                              // 1MHz 115200 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 115200
    UCA1MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

//SET UP TIMER A1*********************************************************************************
      //using Timer A1
      TA1CCTL1 = CCIE;                        // CCR interrupt enabled for TA1
      TA1CCR0 = 62500;                        // Set the period in the Timer A CCR0 to 1 second (note: will be using ID_3)

//SET UP INPUT AND OUTPUT PINS********************************************************************
// Port 1
    P1REN |= BUTTON1 + BUTTON2 + BUTTON3 + PIR;             // resistor enable for inputs
    P1SEL &= ~(BUTTON1 + BUTTON2 + BUTTON3 + PIR);          // set as GPIO
    P1DIR &= ~(BUTTON1 + BUTTON2 + BUTTON3 + PIR);          // set others as input
// Port 3
    P3SEL &= ~(BUZZER + CORRECT);                           // set as GPIO
    P3DIR |= (BUZZER + CORRECT);                            // Set buzzer and status LED as outputs
//Initialize
    P3OUT &= ~(BUZZER + CORRECT);                           //shut off buzzer
    P1IE |= BUTTON1 + BUTTON2 + BUTTON3 + PIR;              // enable P1.3 interrupt
//Interrupt handling
    P1IFG &= ~BUTTON1 + BUTTON2 + BUTTON3 + PIR;            // clear the P1.3 interrupt flag

    __enable_interrupt();                                   // enable interrupts


}

void alert(int correct){

    //reset entered combination
    int i;
    for(i = 0; i < 5; i++){
        combination[i] = 0;
    }

    if(correct == 1){       //no intruder
        correct = 0;        // reset correct bit
        P3OUT |= CORRECT;   // set status LED high
    }
    else{                   //intruder
        P3OUT &= ~CORRECT;  // shut off status LED
        char message[] = {73, 110, 116, 114, 117, 100, 101, 114, 10, 13};   // ascii values of 'Intruder\n'
        // send message
        int i = 0;
         unsigned int size = 10;                // 10 ascii values to send
         for(i = 0; i < size; i++){             //send chars one at a time
             while (!(UCA1IFG & UCTXIFG));      //wait for TX buffer to be ready
             UCA1TXBUF = message[i];            //send out char
         }
         P3OUT |= BUZZER;                       //turn on buzzer
         i = 0;
         for(i = 0; i < 10; i++){               //delay for 10 seconds (keep buzzer on this long)
             __delay_cycles(1000000);           // 1 second delay
         }

    }

    P3OUT &= ~BUZZER;                               //shut off buzzer after sending message or canceling sending message
    P1IFG &= ~(BUTTON1 + BUTTON2 + BUTTON3 + PIR);  //clear interrupt flags
    triggered = 0;                                  //device is back in standby mode
}


// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)   //take care of interrupt coming from port 1
{

    __delay_cycles(2000);                                   // small delay for debouncing

    switch(P1IV){
    case 0x000C:                                            // if PIR is triggered*****************************************************
        P1IE &= ~PIR;                                       // disable port 1 interrupts (button won't cause interrupt; for debouncing)
        P1IFG &= ~PIR;                                      // clear interrput flag
        triggered = 1;                                      // PIR has been triggered
        __delay_cycles(300000);                             // delay for debouncing
        P1IE |= PIR;                                        // reenable port 1 interrupts, so buttons can be read again
        break;
    case 0x000A:                                            // BUTTON3 has triggered****************************************************
        if(triggered ==1){                                  // only read button if PIR has triggered
            P1IE &= ~BUTTON3;                               // disable port 1 interrupts (button won't cause interrupt; for debouncing)
            P1IFG &= ~BUTTON3;                              // clear interrput flag
            combination[combination_digits_entered] = 3;    // record number for combination
            __delay_cycles(300000);                         // delay for debouncing
            P1IE |= BUTTON3;                                // reenable port 1 interrupts, so button can be read again
        }
        break;
    case 0x0008:                                            // BUTTON2 has triggered****************************************************
        if(triggered == 1){                                 // only read button if PIR has triggered
            P1IE &= ~BUTTON2;                               // disable port 1 interrupts (button won't cause interrupt; for debouncing)
            P1IFG &= ~BUTTON2;                              // clear interrput flag
            combination[combination_digits_entered] = 2;    // record number for combination
            __delay_cycles(300000);                         // delay for debouncing
            P1IE |= BUTTON2;                                // reenable port 1 interrupts, so button can be read again
        }
        break;
    case 0x0006:                                            // BUTTON1 has triggered****************************************************
        if(triggered == 1){                                 // only read button if PIR has triggered
            P1IE &= ~BUTTON1;                               // disable port 1 interrupts (button won't cause interrupt; for debouncing)
            P1IFG &= ~BUTTON1;                              // clear interrput flag
            combination[combination_digits_entered] = 1;    // record number for combination
            __delay_cycles(300000);                         // delay for debouncing
            P1IE |= BUTTON1;                                // reenable port 1 interrupts, so button can be read again
        }
       break;
    default: break;
    }

    __delay_cycles(300000);                                 // more delays for debouncing

    unsigned int digits_correct = 0;                        // initialie nubmer of correct digits to 0

    // if PIR has triggered AND PIR is not the current interrupt AND an actual combination digit has been recorded, increment number of digits entered
    if(triggered == 1 && P1IV != 0x000C && combination[combination_digits_entered] != 0)
        ++combination_digits_entered;

    if(combination_digits_entered == 5){                    // if all 5 digits have been entered
        triggered = 0;                                      // triggered set back to 0 (getting ready to go back to standby
        int correct = 0;                                    // initialize number of digits correct
        unsigned int i;
        for(i = 0; i < 5; ++i){                             //check all 5 entered digits against the key
            if(combination_key[i] == combination[i])        // if this digit is correct, increment digits_correct
                ++digits_correct;

            if(digits_correct == 5)                         // if we get this far, all values matched
                correct = 1;                                // correct is true
        }
        combination_digits_entered = 0;                     // reset digits entered to 0
        alert(correct);                                     // call on message to decide what to do now that user has entered combination
    }

}

// Timer Interrupt
// used for counting to 10
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
    {
      switch(__even_in_range(TA1IV,14)) //testing timer interrupt vector
      {
        case  0: break;                          // No interrupt
        case  2:                 //if CCR1 is reached, set output low
                 break;
        case  4: break;                          // CCR2 not used
        case  6: break;                          // reserved
        case  8: break;                          // reserved
        case 10: break;                          // reserved
        case 12: break;                          // reserved
        case 14:                    // if CCR0 overflows
            if(counter < 9){        // want CCR0 to overflow 9 times in order to reach 10 seconds
                counter++;
            }
        else{                       // if 10 seconds have passed (1 overflow per second)
                counter = 0;        // reset number of overflows to 0
                TA0CTL = MC_0;      // stop timer
                TA0R = 0;           // reset timer reg contents
                timerdone = 1;      // timer is done
            }
                 break;
        default: break;
      }
      TA1IV &= ~TA1IV_TA1IFG;       // Clear the Timer interrupt Flag
    }


//For testing UART connectivity, uncomment the following lines:
/*
// Echo back RXed character, confirm TX buffer is ready first
// UART interrupt
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)

{
  switch(__even_in_range(UCA1IV,4))         //looking for a specific interrupt case: when RX has value
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG (UART is receiving data in RX)

        while (!(UCA1IFG & UCTXIFG));       //wait for TX buffer to be ready
        UCA1TXBUF = UCA1RXBUF;               //send out temp reading

    break;
  case 4:break;                             // Vector 4 - TXIFG

  default: break;
  }
}*/



