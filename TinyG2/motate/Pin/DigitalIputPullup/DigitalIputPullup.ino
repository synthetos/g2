/*
 Input Pullup Serial
 
 This example demonstrates the use of pinMode(INPUT_PULLUP). It reads a 
 digital input on pin 2 and prints the results to the serial monitor.
 
 The circuit: 
 * Momentary switch attached from pin 2 to ground 
 * Built-in LED on pin 13
 
 Unlike pinMode(INPUT), there is no pull-down resistor necessary. An internal 
 20K-ohm resistor is pulled to 5V. This configuration causes the input to 
 read HIGH when the switch is open, and LOW when it is closed. 
 
 created 14 March 2012
 by Scott Fitzgerald
  modified to use Motate::Pins August, 2012
 by Rob Giseburt

 http://www.arduino.cc/en/Tutorial/InputPullupSerial
 
 This example code is in the public domain
 
 */

#include <MotatePins.h>
// save typing Motate::Pin
using namespace Motate; 

// create the Pin objects
InputPin<2> buttonPin(kPullUp);     // the number of the pushbutton pin
OutputPin<13> ledPin;      // the number of the LED pin

void setup(){
  //start serial connection
  Serial.begin(9600);
}

void loop(){
  //read the pushbutton value into a variable
  int sensorVal = buttonPin;
  //print out the value of the pushbutton
  Serial.println(sensorVal);
  
  // Keep in mind the pullup means the pushbutton's
  // logic is inverted. It goes HIGH when it's open,
  // and LOW when it's pressed. Turn on pin 13 when the 
  // button's pressed, and off when it's not:
  if (sensorVal == HIGH) {
    ledPin = LOW;
  } 
  else {
    ledPin = HIGH;
  }
}



