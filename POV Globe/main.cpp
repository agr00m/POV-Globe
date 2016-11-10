/*********************************************************
 *  POV Globe                                            *
 *********************************************************
 *   Author: Aaron Groom                                 *
 *  Created: 10/21/2016 8:40 AM                          *
 *	Modified: 09/28/2014 10:00 AM                         *
 *	 Version: 0.1                                         *
 *********************************************************
 * Project Design
 *
 *  Processor:  ATMEGA328P
 *
 *	Inputs:
 *			PD2 - INT0 - Reed switch
 *
 *  Outputs:
 *			PB0 - Column 0       PD0 - Row 0
 *			PB1 - Column 1       PD1 - Row 1
 *			PB2 - Column 2       PD3 - Row 2
 *			PB3 - Column 3       PD4 - Row 3
 *			PB4 - Column 4       PD5 - Row 4
 *			PB5 - Column 5       PD6 - Row 5
 *       PC1 - Column 6       PD7 - Row 6
 *       PC2 - Column 7       PC0 - Row 7
 *       PC3 - Column 8
 *            ___________________
 *            |                 |
 *      RST --| 1  PC6   PC5 28 |-- N/C
 *    Row 7 --| 2  PD0   PC4 27 |-- Col 9
 *    Row 6 --| 3  PD1   PC3 26 |-- Col 8
 *  Reed SW --| 4  PD2   PC2 25 |-- Col 7
 *    Row 5 --| 5  PD3   PC1 24 |-- Col 6
 *    Row 4 --| 6  PD4   PC0 23 |-- Row 0
 *       V+ --| 7  VCC   GND 22 |-- GND
 *      GND --| 8  GND  AREF 21 |-- N/C
 *     XTAL --| 9  PB6   VCC 20 |-- V+
 *     XTAL --| 10 PB7   PB5 19 |-- Col 5 (SCK)
 *    Row 3 --| 11 PD5   PB4 18 |-- Col 4 (MISO)
 *    Row 2 --| 12 PD6   PB3 17 |-- Col 3 (MOSI)
 *    Row 1 --| 13 PD7   PB2 16 |-- Col 2
 *    Col 0 --| 14 PB0   PB1 15 |-- Col 1
 *            |_________________|
 *
 *******************************************************/
#define F_CPU 16000000UL	// 16 MHz Clock

#include <avr/io.h>
#include <avr/interrupt.h>
#include "pumpkin2-hex.h"

// BITWISE MACROS
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define SETBITMASK(x,y) (x |= (y))
#define CLEARBITMASK(x,y) (x &= (~y))

/* ----- FUNCTION DEFINITIONS ----- */
void init(void);
void rowsOff(void);
void columnsOff(void);
void setColumn(uint8_t, uint8_t);
void setRow(uint8_t, uint8_t);

void TC0_Enable(uint8_t);
uint8_t TC0_ReadReset(void);
void TC0_Write(uint8_t);

void TC1_Enable(uint8_t);
uint16_t TC1_Read(void);
void TC1_Write(uint16_t);

/* ----- GLOBAL VARIABLES ----- */
uint16_t t0OverflowCounter = 0;     // Timer0 overflow counter
uint8_t currentImageColumn = 0;     // Image column to display

/************************************************************************/
/* External Interrupt 0                                                 */
/************************************************************************/
/* INT0 called when reed switch is activated. The # of clock cycles     */
/* from the previous interrupt are counted, then divided by the number  */
/* of total image columns then set to Timer1's CTC TOP value. Timer1,   */
/* Timer1 overflow counter and current column counter are reset to 0.   */
/************************************************************************/
ISR(INT0_vect) {
   if (t0OverflowCounter > 30) {
      OCR1A = ((((256 * t0OverflowCounter) + TC0_ReadReset())) / imageColumns) - 1;  // Set Timer1 Output Compare Match value
      TC1_Write(0);
      currentImageColumn = 0;    // Reset current column
      t0OverflowCounter = 0;     // Reset Timer0 overflow counter
   }
}

/************************************************************************/
/* Timer0 Overflow Interrupt                                            */
/************************************************************************/
/* Increment timer 0 overflow counter.                                  */
/************************************************************************/
ISR(TIMER0_OVF_vect){
   t0OverflowCounter++;
}

/************************************************************************/
/* Timer1 Output Compare Match Interrupt                                */
/************************************************************************/
/* Increments the current display column                                */
/************************************************************************/
ISR(TIMER1_COMPA_vect) {
   currentImageColumn++;
}

int main(void) {
   uint8_t rowValue = 0xFF;
   init();
   while (1) {
      for (uint8_t col = 0; col < 10; col++) {           // Loop through each block of 8 LEDs (columns)
         rowValue = image[currentImageColumn][col];      // Read value for block of 8 LEDs
         setColumn(col, 0);                              // Enable column
         for (uint8_t row = 0; row < 8; row++) {         // Loop through each LED (rows)
            if ((rowValue & (1<<row)) >> row) {
               setRow(row, 1);
            }
            setRow(row, 0);                              // Turn LED off
         }
         setColumn(col, 1);                              // Disable column
      }
   }
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/
/* Initialize I/O pins, timers and interrupts and starts timer 0 & 1.   */
/************************************************************************/
void init(void){
   
   // Setup GPIO data direction
   DDRB  |= 0b00111111;	   // DDRB - PB0-PB5 Output
   DDRC  |= 0b00011111;	   // DDRC - PC0-PC5 OUtput
   DDRD   = 0b11111011;	   // DDRD - PD0-PD1 & PD3-PD7 Output, PD2 Input
   SETBIT(PORTD, PD2);	   // Enable pull-up on PD2(INT0)

   rowsOff();              // Set rows LOW
   columnsOff();           // Set columns HIGH
   
   SETBIT(TIMSK0, TOIE0);  // Timer0 Overflow Interrupt Enable
   SETBIT(TCCR1B, WGM12);  // Timer1 CTC Mode
   OCR1A = 0xFF;
   SETBIT(TIMSK1, OCIE1A); // Timer1 Output Compare A Match Interrupt Enable
   SETBIT(EIMSK, INT0);    // External Interrupt 0 Enable
      
   SREG   |= 0b10000000;   // Global Interrupt Enable

   TC0_Enable(3);          // Start Timer 0, clk/64 Prescaler
   TC1_Enable(3);          // Start Timer 1, clk/64 Prescaler
}

/************************************************************************/
/* rowsOff                                                              */
/************************************************************************/
/* Sets all row I/O pins to 0.                                          */
/************************************************************************/
void rowsOff(void) {
   PORTC &= 0xFE;    // 0b11111110 PC0 LOW              
   PORTD &= 0x04;    // 0b00000100 PD0-PD1, PD3-PD7 LOW 
}

/************************************************************************/
/* columnsOff                                                           */
/************************************************************************/
/* Sets all column I/O pins to 1.                                       */
/************************************************************************/
void columnsOff(void) {
   PORTB |= 0x3F;    // 0b00111111 PB0-PB5 HIGH
   PORTC |= 0x1E;    // 0b00011110 PC1-PC4 HIGH
}

void setColumn(uint8_t col, uint8_t val) {
   if (val == 1) {
      switch(col) {
         case 0: SETBIT(PORTB, PB0); break;
         case 1: SETBIT(PORTB, PB1); break;
         case 2: SETBIT(PORTB, PB2); break;
         case 3: SETBIT(PORTB, PB3); break;
         case 4: SETBIT(PORTB, PB4); break;
         case 5: SETBIT(PORTB, PB5); break;
         case 6: SETBIT(PORTC, PC1); break;
         case 7: SETBIT(PORTC, PC2); break;
         case 8: SETBIT(PORTC, PC3); break;
         case 9: SETBIT(PORTC, PC4); break;
      }
   } else {
      switch(col) {
         case 0: CLEARBIT(PORTB, PB0); break;
         case 1: CLEARBIT(PORTB, PB1); break;
         case 2: CLEARBIT(PORTB, PB2); break;
         case 3: CLEARBIT(PORTB, PB3); break;
         case 4: CLEARBIT(PORTB, PB4); break;
         case 5: CLEARBIT(PORTB, PB5); break;
         case 6: CLEARBIT(PORTC, PC1); break;
         case 7: CLEARBIT(PORTC, PC2); break;
         case 8: CLEARBIT(PORTC, PC3); break;
         case 9: CLEARBIT(PORTC, PC4); break;
      }
   }
}

void setRow(uint8_t row, uint8_t val) {
   if (val == 1) {
      switch(row) {
         case 0: SETBIT(PORTD, PD0); break;
         case 1: SETBIT(PORTD, PD3); break;
         case 2: SETBIT(PORTD, PD1); break;
         case 3: SETBIT(PORTD, PD4); break;
         case 4: SETBIT(PORTD, PD5); break;
         case 5: SETBIT(PORTD, PD6); break;
         case 6: SETBIT(PORTD, PD7); break;
         case 7: SETBIT(PORTC, PC0); break;
      }
   } else {
      switch(row) {
         case 0: CLEARBIT(PORTD, PD0); break;
         case 1: CLEARBIT(PORTD, PD3); break;
         case 2: CLEARBIT(PORTD, PD1); break;
         case 3: CLEARBIT(PORTD, PD4); break;
         case 4: CLEARBIT(PORTD, PD5); break;
         case 5: CLEARBIT(PORTD, PD6); break;
         case 6: CLEARBIT(PORTD, PD7); break;
         case 7: CLEARBIT(PORTC, PC0); break;
      }
   }
}

/* ----- TIMER/COUNTER 0 ----- */

/************************************************************************/
/* TC0_Enable                                                           */
/************************************************************************/
/* Starts and stops Timer/Counter 0 based on passed prescaler value.    */
/* 0: Off | 1: clk/1 | 2: clk/8 | 3: clk/64 | 4: clk/256 | 5: clk/1-24  */
/* 6: External falling edge | 7: External rising edge.                  */
/* Invalid prescaler disables timer.                                    */
/************************************************************************/
void TC0_Enable(uint8_t prescaler) {
   CLEARBITMASK(TCCR0B, 0x03);
   if (prescaler > 0 && prescaler < 8) {
      SETBITMASK(TCCR0B, prescaler);
   }
}

uint8_t TC0_ReadReset(void) {
   uint8_t TC0 = TCNT0;
   TC0_Write(0);
   return TC0;
}

void TC0_Write(uint8_t value) {
   TCNT0 = value;
}

/* ----- TIMER/COUNTER 1 ----- */

/************************************************************************/
/* TC1_Enable                                                           */
/************************************************************************/
/* Starts and stops Timer/Counter 1 based on passed prescaler value.    */
/* 0: Off | 1: clk/1 | 2: clk/8 | 3: clk/64 | 4: clk/256 | 5: clk/1-24  */
/* 6: External falling edge | 7: External rising edge.                  */
/* Invalid prescaler disables timer.                                    */
/************************************************************************/
void TC1_Enable(uint8_t prescaler) {
   if (prescaler == 0) {
      CLEARBITMASK(TCCR1B, 0x03);
   } else {
      SETBITMASK(TCCR1B, prescaler);
   }
}

/************************************************************************/
/* TC1_Write                                                            */
/************************************************************************/
/* Sets Timer/Counter 1 to passed value.                                */
/************************************************************************/

void TC1_Write(uint16_t value) {
   //TCNT1H = value >> 8;
   //TCNT1L = value & 0xF0;
   TCNT1 = value;
}

/************************************************************************/
/* TC1_Read                                                             */
/************************************************************************/
/* Sets Timer/Counter 1 to passed value.                                */
/************************************************************************/
uint16_t TC1_Read() {
   return TCNT1;
}