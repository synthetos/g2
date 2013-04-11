/*
 * HW.cpp
 *
 * Created: 3/31/2013 11:19:33 AM
 *  Author: Administrator
 */ 

//#define ARDUINO_MAIN

#include "sam.h"
#include "Arduino.h"
#include <stdio.h>

//#include "tinyg2.h"

// Disable some warnings for the Arduino files
// ref: http://www.hilltop-cottage.info/blogs/adam/?p=211

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
//	tg_setup();

	SerialUSB.begin(115200);
	delay(2000);               // wait for N seconds
	SerialUSB.print("\n\nSTARTUP\n");
	
//	Serial.begin(115200);
//	delay(2000);               // wait for N seconds
//	Serial.print("\n\nSTARTUP_2\n");
}

void loop( void )
{
	const uint8_t ptr[] = ("Hello Kitty...");
	SerialUSB.write(ptr, sizeof(ptr));

//	SerialUSB.print("Hello Kitty...");
//	Serial.print("Hello Kitty2...");

//	printf("printf test...\n");

	digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(100);               // wait for a second
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

//#if defined(USBCON)
	USBDevice.attach();
//#endif

	setup();

	for (;;)
	{
		loop();
//		if (serialEventRun) serialEventRun();
	}
	return 0;
}