
//Taken from: http://processors.wiki.ti.com/index.php/MSP430_LaunchPad_PushButton
//comments have been modified

//David Sheppard

//Final Project


#include <msp430f5529.h>


//definitions:
#define LED BIT0        //defining LED1 as BIT0
#define BUTTON1 BIT2
#define BUTTON2 BIT3
#define BUTTON3 BIT4
#define BUTTON4 BIT5
#define TRIGGER BIT0
#define BUZZER BIT0
#define CORRECT BIT1

//global vars:
unsigned int done_combination = 0;
const unsigned int combination_key[5] = {1,2,3,1,1};
unsigned int combination[5] = {0};
unsigned int combination_digits_entered = 0;
unsigned int interrupt_edge[4] = {0};
unsigned int num_of_interrupts = 0;
unsigned int triggered = 0;
unsigned int counter = 0;
unsigned int timerdone = 1;



//prototypes
void wifi(int);

int main(void)  //begin main function
{
    WDTCTL = WDTPW + WDTHOLD;                                   // Stop watchdog timer

    P4SEL |= BIT5 + BIT4;                     //enable UART for these pins
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;
    UCA1BR0 = 9;                              // 1MHz 115200 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 115200
    UCA1MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

    //timer initialization************************************************************************
      //using Timer A1
      TA1CCTL1 = CCIE;                          // CCR interrupt enabled for TA1
      TA1CCR0 = 62500;                            //Set the period in the Timer A CCR0 to
      //TA1CCR1 = 131;                            //The initial period in microseconds that the power is ON.
                                                //It's initialized to half the time, which translates to a 50% duty cycle.


    P1REN |= BUTTON1 + BUTTON2 + BUTTON3 + BUTTON4;

    // IO:
    P1DIR |= (LED);                                             // Set P1.0 (LED) to be an output
    P1SEL &= ~(BUTTON1 + BUTTON2 + BUTTON3 + BUTTON4);          //set as GPIO
    P1DIR &= ~(BUTTON1 + BUTTON2 + BUTTON3 + BUTTON4);          //set as input

    P3DIR |= (BUZZER + CORRECT);                                             // Set P1.0 (LED) to be an output
    P3SEL &= ~(BUZZER + CORRECT);          //set as GPIO

    P2SEL &= ~(TRIGGER);          //set as GPIO
    P2DIR &= ~(TRIGGER);          //set as input

    P3OUT &= ~(BUZZER + CORRECT);               //shut off buzzer
    P1OUT &= ~LED;                                              // shut off LED0
    P1IE |= BUTTON1 + BUTTON2 + BUTTON3 + BUTTON4;              // enable P1.3 interrupt
    P2IE |= TRIGGER;              // enable P2.0 interrupt


    P1IFG &= ~BUTTON1 + BUTTON2 + BUTTON3 + BUTTON4;            // clear the P1.3 interrupt flag
    P2IFG &= ~TRIGGER;



    //__delay_cycles(100000);
    __enable_interrupt();                                       // enable interrupts


}

void wifi(int correct){

    //reset entered combination
    int i;
    for(i = 0; i < 5; i++){
        combination[i] = 0;
    }

    //send notification
    if(correct == 1){
        //no intruder
        P1OUT &= ~LED;
        correct = 0;
        P3OUT |= CORRECT;
    }
    else{
        //intruder
        P3OUT &= ~CORRECT;
        //char message[] = {35, 84, 101, 115, 116, 95, 84, 111, 112, 105, 99, 32, 73, 110, 116, 114, 117, 100, 101, 114, 10, 13};
        char message[] = {73, 110, 116, 114, 117, 100, 101, 114, 10, 13};
        int i = 0;
         unsigned int size = 10;
         for(i = 0; i < size; i++){
             while (!(UCA1IFG & UCTXIFG));       //wait for TX buffer to be ready
             UCA1TXBUF = message[i];               //send out message
         }
         P3OUT |= BUZZER;
         i = 0;
         for(i = 0; i < 10; i++){
             __delay_cycles(1000000);
         }
         //TA1CTL = TASSEL_2 + MC_1 + ID_3 + TAIE;   //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.
         //timerdone = 0;
         //while(timerdone == 0); //wait for 10s timer to finish
    }

    P3OUT &= ~BUZZER;   //shut off buzzer after sending message of canceling sending message
    P1IFG &= ~(BUTTON1 + BUTTON2 + BUTTON3 + BUTTON4);
    triggered = 0;
}

/*
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)   //take care of interrupt coming from port 1
{
    triggered = 1;
    P3OUT |= BUZZER;
}*/

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)   //take care of interrupt coming from port 1
{


    //6: 1.2, 8:1.3, A:1.4, C:1.5
    __delay_cycles(2000);
    /*++num_of_interrupts;
    if(num_of_interrupts > 1){*/

    switch(P1IV){
    case 0x000C:
        P1IE &= ~BUTTON4;                     //disable port 1 interrupts (button won't cause interrupt)
        P1IFG &= ~BUTTON4;
        //if((P1IN & BUTTON4) == 0x00)
            //combination[combination_digits_entered] = 4;
        triggered = 1;
        __delay_cycles(300000);
 //       P1IES ^= BUTTON4;                // toggle the interrupt edge
        P1IE |= BUTTON4;         //reenable port 1 interrupts, so button can be read again
        break;
    case 0x000A:
        if(triggered ==1){
        P1IE &= ~BUTTON3;                     //disable port 1 interrupts (button won't cause interrupt)
        P1IFG &= ~BUTTON3;
        //if((P1IN & BUTTON3) == 0x00)
            combination[combination_digits_entered] = 3;
        __delay_cycles(300000);
        P1IE |= BUTTON3;         //reenable port 1 interrupts, so button can be read again
//        P1IES ^= BUTTON3;                // toggle the interrupt edge
        break;}
    case 0x0008:
        if(triggered == 1){
        P1IE &= ~BUTTON2;                     //disable port 1 interrupts (button won't cause interrupt)
        P1IFG &= ~BUTTON2;
        //if((P1IN & BUTTON2) == 0x00)
            combination[combination_digits_entered] = 2;
        __delay_cycles(300000);
        P1IE |= BUTTON2;         //reenable port 1 interrupts, so button can be read again
//        P1IES ^= BUTTON2;                // toggle the interrupt edge
        break;}
    case 0x0006:
        if(triggered == 1){
       P1IE &= ~BUTTON1;                     //disable port 1 interrupts (button won't cause interrupt)
       P1IFG &= ~BUTTON1;
       //if((P1IN & BUTTON1) == 0x00)
           combination[combination_digits_entered] = 1;
//       P1IES ^= BUTTON1;                // toggle the interrupt edge
       __delay_cycles(300000);
       P1IE |= BUTTON1;         //reenable port 1 interrupts, so button can be read again
       break;}
    default: break;
    }

    __delay_cycles(300000);

    unsigned int digits_correct = 0;

    if(triggered == 1 && P1IV != 0x000C && combination[combination_digits_entered] != 0)
        ++combination_digits_entered;

    if(combination_digits_entered == 5){
        triggered = 0;
        int correct = 0;
        unsigned int i;
        for(i = 0; i < 5; ++i){
            if(combination_key[i] == combination[i])
                ++digits_correct;

            if(digits_correct == 5) //if we get this far, all values matched
                correct = 1;

        }
        combination_digits_entered = 0;
        wifi(correct);
    }

}

    //Timer Interrupt
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
        case 14:                  // if CCR0 overflows
            if(counter < 9){
                counter++;
            }
            else{
                counter = 0;
                TA0CTL = MC_0;           //stop timer
                TA0R = 0;               //reset timer reg contents
                timerdone = 1;
            }
                 break;
        default: break;
      }
      TA1IV &= ~TA1IV_TA1IFG; // Clear the Timer interrupt Flag
    }


// Echo back RXed character, confirm TX buffer is ready first
//UART interrupt
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
}



