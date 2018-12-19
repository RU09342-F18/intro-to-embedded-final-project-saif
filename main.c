/*
* SAIF (Security at its Finest) Final Project for Intro to Embedded Systems
*   Written by: David Sheppard
*   Some code based on samples taken from the TI Resource Explorer (See disclaimer at end of code)
*
* Created: 10 December 2018
* Last Updated: 18 December 2018 (commented and cleaned up code)
*
* Purpose: The SAIF system acts as a motion sensor that can be implemented as a security system.
* When the motion sensor is triggered, the users has 10 seconds to enter the correct combination on the buttons
* before a buzzer goes off and an intruder alert is sent via UART.
*
* Basic Components:
*   PIR: detects motion
*   Buzzer: sounds alarm
*   3 Buttons: act as keypad for entering combination
*   Status LED: comes on if combination is correct
*   UART Connection: prints intruder alert if combination is incorrect or timer counts to 10 seconds
*   Timer: gives the user 10 seconds to enter a combination and times buzzer for 10 seconds
*
* Ports:
*
*   Port | Use
*   ---------------------
*    1.2 | button 1
*    1.3 | button 2
*    1.4 | button 3
*    1.5 | output of PIR
*    3.0 | buzzer control
*    3.1 | status LED
*
* Notes:
*   115200 baud rate
*   dependent upon msp430f5529.h header file
*   should be compied in Code Composer Studio
*   written in CCS 8.1.0
*
*/

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
void alert(int);    //for acting upon results of combination

int main(void)                                //begin main function
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
//Initialize values
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

    if(correct == 1){                           //no intruder
        correct = 0;                            // reset correct bit
        P3OUT |= CORRECT;                       // set status LED high
    }
    else{                                       //intruder
        P3OUT &= ~CORRECT;                      // shut off status LED
        //initialize message:
        char message[] = {73, 110, 116, 114, 117, 100, 101, 114, 10, 13};   // ascii values of 'Intruder\n'
        // send message:
        int i = 0;
        const unsigned int size = 10;           // 10 ascii values to send
        for(i = 0; i < size; i++){              //send chars one at a time
            while (!(UCA1IFG & UCTXIFG));       //wait for TX buffer to be ready
            UCA1TXBUF = message[i];             //send out char
        }
        P3OUT |= BUZZER;                        //turn on buzzer
        i = 0;
        for(i = 0; i < 10; i++){                //delay for 10 seconds (keep buzzer on this long)
            __delay_cycles(1000000);            // 1 second delay
        }

    }

    P3OUT &= ~BUZZER;                               //shut off buzzer after sending message or canceling sending message
    P1IFG &= ~(BUTTON1 + BUTTON2 + BUTTON3 + PIR);  //clear interrupt flags
    triggered = 0;                                  //device is back in standby mode
}


// Port 1 interrupt service routine (for buttons and PIR inputs)
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
    }                                                       // end switch(P1IV)

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
        }                                                   // end testing combination
        combination_digits_entered = 0;                     // reset digits entered to 0
        alert(correct);                                     // call on message to decide what to do now that user has entered combination
    }                                                       //end if(combination_digits_entered == 5)

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



// To test UART connectivity, uncomment the following block:

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





/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F552x Demo - USCI_A0, 115200 UART Echo ISR, DCO SMCLK
//
//   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
//   USCI_A0 RX interrupt triggers TX Echo.
//   Baud rate divider with 1048576hz = 1048576/115200 = ~9.1 (009h|01h)
//   ACLK = REFO = ~32768Hz, MCLK = SMCLK = default DCO = 32 x ACLK = 1048576Hz
//   See User Guide for baud rate divider table
//
//                 MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//            |     P3.3/UCA0TXD|------------>
//            |                 | 115200 - 8N1
//            |     P3.4/UCA0RXD|<------------
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************
