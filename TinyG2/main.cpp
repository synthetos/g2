/*
 * HW.cpp
 *
 * Created: 3/31/2013 11:19:33 AM
 *  Author: Administrator
 */ 

//#define ARDUINO_MAIN
#include "sam.h"
#include "Arduino.h"
#include "tinyg2.h"

/*
extern void SysTick_Handler( void )
{
  // Increment tick count each ms
  TimeTick_Increment() ;
}
*/

/******************** Application Code ************************/
int led = 13;

void setup( void )
{
	pinMode(led, OUTPUT);
	tg_setup();
	Serial.begin(9600);           // set up Serial library at 9600 bps
	Serial.print("Ready\n");
}

void loop( void )
{
	Serial.println("Hello world!");  // prints hello with ending line break

	digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(500);               // wait for a second
	digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
	delay(500);               // wait for a second
}

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
	init();
	delay(1);

	USBDevice.attach();
	setup();

	for (;;)
	{
		loop();
		if (serialEventRun) serialEventRun();
	}

	return 0;
}