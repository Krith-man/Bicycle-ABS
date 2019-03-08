/*
*	Milestone3.c
*
*	Authors: Kritharakis Emmanuel, Fotakis Tzanis
*	Created on: 10 November 2017
*	AVR: Atmel ATMega328P
*	Created with: Atmel Studio 7
*
*	The code below tries to create an Anti-Lock Braking System (ABS) for bicycles using a sliding 
*	potentiometer as input to check the brake's lever position, two Photo-Interrupter sensors, one for
*	each wheel, to get the wheels' frequencies and two servomechanisms to move each wheel's brake caliper.
*
*	It sets PB1 and PB2 as PWM outputs using OCR1A and OCR1B compare registers respectively for connecting
*	the front and rear servomechanism. Also, it uses ADC0 (PC0) as analog input signal from the sliding 
*	potentiometer, whose position is read which controls each PWM's duty cycle. More specifically, it 
*	linearly sets the PB1's (front servo) and PD2's (rear servo) PWM duty cycle to minimum when the 
*	potentiometer is at maximum position and the exact reverse when the potentiometer is at minimum 
*	position as long as there is no frequency difference between the two wheels. If there is, then the
*	slower wheel's brakes release until it reaches the speed of the faster ones, and then it brakes again.
*	The whole functionality is interrupt driven, using the ADC's "Conversion Completed" Interrupt and 
*	INT0 & INT1 pins to check the Photo-Interrupter sensors' pulse's width.
*/

// ------------------------------------------ Calibration ------------------------------------------------
#define F_CPU 16000000UL	// 16 MHz Clock Frequency
//#define DIFFERENCE_THRESHOLD 1600000
#define MAX_PERIOD 10000
// -------------------------------------------------------------------------------------------------------

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

volatile uint32_t microsFrontWheel=0;
volatile uint32_t microsRearWheel=0;
uint32_t startFrontWheel=0;
uint32_t startRearWheel=0;
uint32_t frontWheelPeriod = 0;
uint32_t rearWheelPeriod = 0;
int frontWheelUnreadPulsesNumber = 0;
int rearWheelUnreadPulsesNumber = 0;
int checkWheelsFrequenciesReturnValue=0;

// Initializes the WatchDog Timer to reset the system every 16ms if not cleared
void WDTInit(){
	// Clear the reset flag, the WDRF bit (bit 3) of MCUSR.
	MCUSR = MCUSR & 0xF7;
	// Set the WDCE (Watchdog Change Enable) bit (bit 4) and the WDE (Watchdog System 
	// Reset Enable) bit (bit 3) of WDTCSR. The WDCE bit must be set in order to
	// change WDE or the watchdog prescalers. Setting the WDCE bit will allow updates to the 
	// prescalers and WDE for 4 clock cycles then it will be reset by hardware.
	WDTCSR = WDTCSR | 0x18;

	// Set the watchdog timeout prescaler value to 2K which will yield a time-out interval of about 16ms.
	WDTCSR = 0;

	// Enable the watchdog timer interrupt.
	WDTCSR = WDTCSR | 0x40;
	MCUSR = MCUSR & 0xF7;
}

// Initializes the ADC component to convert the ACD0 input with a 128 prescaler and auto conversion
 void ADCinit(){
	ADMUX = 1 << REFS0; // AVCC with external capacitor at AREF pin, ADC0 selected
	ADMUX |= 1<<ADLAR; // ADC Left Adjust Result to use ADCH register for 8-bit operations (ignore 2 Least Significant Bits)
	ADCSRA = 1 << ADEN; // Analog to Digital Enable
	ADCSRA |= 1<<ADATE; // Auto Trigger Enable Conversion
	ADCSRA |= 1<<ADIE; // ADC Conversion Complete Interrupt activated
	ADCSRA |= 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0; // Set prescaler to clk/128
	ADCSRA |= 1<<ADSC; // Start Conversions
}

// Initializes Timer/Counter2 in CTC mode to trigger an interrupt every 160 clock ticks or 10 us
void MicrosTimerInit(){
	// Set Timer 2 to CTC mode, TOP = OCR2A, Immediate update of OCR2A, TOV Flag set on MAX, 
	// Normal port operation, OC2A disconnected
	TCCR0A = 1<<WGM01;
	TCCR0B = 1 << CS01; // Set prescaler to clk/8
	TIMSK0 = 1 << OCIE0A; // Enable CTC interrupt
	OCR0A = 20; // Set TOP value to 20
}

// Initializes the front and rear Photo-Interrupter Sensors on INT0 & INT1 respectively to trigger 
// interrupts on any of their state change
void PhotoInterruptersInit(){
	EIMSK = 1<<INT1 | 1<<INT0; // Enable INT0 and INT1
	EICRA = 0<<ISC11 | 1<<ISC10 | 0<<ISC01 | 1<<ISC00; // Trigger INT0 and INT1 on any state change
}

// Initializes PWM signal on PB1 & PB2 for front & back servo respectively
void ServoPWMinit(){
	DDRB = 1<<DDB1 | 1<<DDB2; // Set PB1 & PB2 as outputs for OC1A and OC1B respectively
	// Non-Inverting mode - Set OC1A/OC1B on compare match when up-counting. 
	// Clear OC1A/OC1B on compare match when down counting.
	TCCR1A=1<<COM1A1 | 1<<COM1B1 | 1<<WGM11;
	TCCR1B=1<<WGM13 | 1<<WGM12; // Fast PWM
	TCCR1B|=1<<CS11; // Set prescaler to clk/8
	ICR1=40000;	// PWM Frequency = 50Hz (Period = 20ms Standard).
}

// Check for difference between the frequencies of the two wheels
// Returns 0 when equal, 1 when FrontPeriod - RearPeriod > DIFFERENCE_THRESHOLD, 
// -1 when FrontPeriod - RearPeriod < DIFFERENCE_THRESHOLD
void checkWheelsFrequencies(){
	// If there are no new pulse periods measurements return the last decision
	// Considers the state when the bike is stopped and no pulses are sent from the servos
	// but still want to brake
	if(microsFrontWheel > MAX_PERIOD && microsRearWheel > MAX_PERIOD){ // Stopped
		checkWheelsFrequenciesReturnValue = 0;
		return;
	}else if(microsFrontWheel > MAX_PERIOD && rearWheelUnreadPulsesNumber>=5){ // Rear Wheel moved
		microsFrontWheel=0;
		checkWheelsFrequenciesReturnValue = 1;
		return;
	}else if (microsRearWheel > MAX_PERIOD && frontWheelUnreadPulsesNumber>=5){ // Front Wheel moved
		microsRearWheel=0;
		checkWheelsFrequenciesReturnValue = -1;
		return;
	}
	//if (frontWheelPeriod == 0 || rearWheelPeriod == 0) return;
	
	// Check if blocked (one wheel sends pulses and the other is blocked so no pulses are send)
	if ((frontWheelUnreadPulsesNumber==0 || rearWheelUnreadPulsesNumber==0) && 
		(frontWheelUnreadPulsesNumber<2 && rearWheelUnreadPulsesNumber<2)) return;
	int32_t difference = frontWheelPeriod-rearWheelPeriod;
	int32_t minPeriod = (frontWheelPeriod<rearWheelPeriod?frontWheelPeriod:rearWheelPeriod)<<4;
	if(difference>minPeriod) checkWheelsFrequenciesReturnValue = 1;
	else if (difference<-minPeriod) checkWheelsFrequenciesReturnValue = -1;
	else checkWheelsFrequenciesReturnValue = 0;
	// Reinitialize for the new measurements
	frontWheelPeriod=0;
	rearWheelPeriod=0;
	frontWheelUnreadPulsesNumber=0;
	rearWheelUnreadPulsesNumber=0;
}

// Sets the Servo PWM duty cycle to PB1 & PB2 for controlling the front & rear servo
// MinValue = 0 - MaxValue = 235 
void setServoPosition(int value){
	checkWheelsFrequencies();
	if(value>235) value=235; else if(value<0) value=0; // Check for valid value boundaries
	// If front frequency < rear frequency cut the front brake down, else apply the value
	OCR1A = 1000 + (checkWheelsFrequenciesReturnValue == 1 ? 0 : value<<4);
	// If rear frequency < front frequency cut the front brake down, else apply the value
	OCR1B = 1000 + (checkWheelsFrequenciesReturnValue == -1 ? 0 : value<<4);
}

// ADC Interrupt Service Routine
// Sets the Servo Position linearly inverted to the Slider position
ISR (ADC_vect){
	// Slider value inversion and offsetting (256 Slider values - ADCH - 128 values offset = 128 - ADCH)
	setServoPosition(128 - ADCH); // Set servos' positions equally to the sliders inverted position
	//wdt_reset(); // Reset watchdog timers
}

// Counting clock ticks for each wheel's Photo-Interrupter Sensor
ISR(TIMER0_COMPA_vect){
	microsFrontWheel++;
	microsRearWheel++;
}

// Front Photo-Interrupter Sensor Interrupt Service Routine
// Calculates the sensor's pulse width in tenths of microseconds
ISR(INT0_vect){
	// if interrupt is triggered on the rising edge store the starting time
	// else if interrupt is triggered on the falling edge sud starting time with current 
	// time to calculate the pulse's period
	if(PIND & 1<<PORTD2){ 
		startFrontWheel = microsFrontWheel;
	}else{ 
		frontWheelPeriod = microsFrontWheel-startFrontWheel;
		frontWheelUnreadPulsesNumber++;
		microsFrontWheel=0; // Restart time counting
	}
}

// Rear Photo-Interrupter Sensor Interrupt Service Routine
// Calculates the sensor's pulse width in tenths of microseconds
ISR(INT1_vect){
	// if interrupt is triggered on the rising edge store the starting time
	// else if interrupt is triggered on the falling edge sud starting time with current
	// time to calculate the pulse's period
	if(PIND & 1<<PORTD3){
		startRearWheel = microsRearWheel;
	}else{
		rearWheelPeriod = microsRearWheel-startRearWheel;
		rearWheelUnreadPulsesNumber++;
		microsRearWheel=0; // Restart time counting
	}
}

int main(void){
	ADCinit();
	MicrosTimerInit();
	PhotoInterruptersInit();
	ServoPWMinit();
	WDTInit();
	sei();
	while(1);
}