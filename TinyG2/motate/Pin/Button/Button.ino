/*
  Button
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 13, when pressing a pushbutton attached to pin 2. 
 
 
 The circuit:
 * LED attached from pin 13 to ground 
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.
 
 
 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe
 modified to use Motate::Pins August, 2012
 by Rob Giseburt

 This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/Button
 */

#include <MotatePins.h>
// save typing Motate::Pin
using namespace Motate; 

// create the Pin objects
InputPin<2> buttonPin;     // the number of the pushbutton pin
OutputPin<13> ledPin;      // the number of the LED pin

void setup() {
  // nothing to do here, move on
}

void loop(){
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonPin == HIGH) {     
    // turn LED on:    
    ledPin = HIGH;
  } 
  else {
    // turn LED off:
    ledPin = LOW;
  }
}
