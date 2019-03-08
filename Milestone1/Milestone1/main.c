/*
	Authors: Kritharakis Emmanuel, Fotakis Tzanis
	Created on: 12 October 2017
	AVR: Atmel ATMega328P
	Created with: Atmel Studio 7

	The code below uses PB1, PB2, PB3 as outputs to turn the connected to them LEDs.
	More specifically, it blinks the LED connected to PB1 with different delays
	(50ms or 200ms) depending on which one is selected. The selected delay can be toggled
	by initiating an interrupt on INT0 (PD2) via a connected pullup push button. 
	In addition, it toggles the LED connected to PB3 every 100 matches of the compare 
	register OCR0A with the Timer/Counter0. Lastly, it sets the Watchdog Timer to reset
	the execution every 8 seconds after the last time the INT0 push button is pushed (the push
	button resets the Watchdog Timer so as the latter does not reset the execution).
*/

#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

int smallDelay=1;
int extraTime=0;

// Initialize IO Ports
void PortInit(void){
	DDRB = 1<<PORTB1 | 1<< PORTB2 | 1<<PORTB3; // Set PB1, PB2, PB3 as Outputs
	PORTB = 1<<PORTB1 | 1<< PORTB2 | 1<<PORTB3; // Initialize PB1, PB2, PB3 to high
	_delay_ms(1500);
	PORTD = 0x04; // Set PD3 as high for pull up resistor
	EIMSK = (1<<INT0); // Enable Int0
	EICRA = 0<<ISC01 | 0<<ISC00; // Trigger INT0 on Low
}

// Initialize watchdog timer
void WDT_Init(void){
	// Clear the reset flag, the WDRF bit (bit 3) of MCUSR.
	MCUSR = MCUSR & 0xF7;
	// Set the WDCE bit (bit 4) and the WDE bit (bit 3)
	// of WDTCSR. The WDCE bit must be set in order to
	// change WDE or the watchdog prescalers. Setting the
	// WDCE bit will allow updates to the prescalers and
	// WDE for 4 clock cycles then it will be reset by
	// hardware.
	WDTCSR = WDTCSR | 0x18;

	// Set the watchdog timeout prescaler value to 1024 K
	// which will yield a time-out interval of about 8.0 s.
	WDTCSR = 0x21;

	// Enable the watchdog timer interrupt.
	WDTCSR = WDTCSR | 0x40;
	MCUSR = MCUSR & 0xF7;
}

void TimerCounterInit(void){
	TCCR0A = 1<<WGM01; //Sets timer counter mode of operation (CTC - Clear timer on compare)
	OCR0A = 195;  // Number of real clock ticks
	TIMSK0  = 1<<OCIE0A; // Timer/Counter0 output compare match a interrupt enable
	TCCR0B = 1<<CS02 | 1<<CS00;  // CLKio /1024 (prescaler)
}

// Blink an LED connected at PB1 for 50ms if smallDelay==1 or 200ms if not.
void BlinkLed(void){
	PORTB = (1<<PORTB1); // Turn LED on
	if(smallDelay==1)
		_delay_ms(50);
	else
		_delay_ms(200);
	PORTB = (0<<PORTB1); // Turn LED off
	if(smallDelay==1)
		_delay_ms(50);
	else
		_delay_ms(200);
}

// Interrupt Service Routine for INT0
// When button on INT0 is pressed it toggles the smallDelay
// and resets the watchdog timer
ISR(INT0_vect){
	if(smallDelay==1)
	smallDelay=0;
	else
	smallDelay=1;
	// Debounce
	cli(); // Close Interrupts
	wdt_reset(); // Reset watchdog timers
	_delay_ms(500); // Wait 500ms
	sei(); // Enable Interrupts
}

// Interrupt Service Routine for Timer Counter
// Blinks a LED on PB3 every 100 counts;
ISR(TIMER0_COMPA_vect){
	extraTime++;
	if(extraTime > 100){
		extraTime = 0;
		PORTB ^= (1 << PORTB3);
	}
}

int main(void){
	PortInit();
	WDT_Init();
	TimerCounterInit();
	sei(); // Enable Interrupts
  	while (1){
		BlinkLed();
	}
}