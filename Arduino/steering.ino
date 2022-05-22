/* 
 * Author: Heesung Kim
 * Senior in Department of Mechanical Engineering,
 * Sogang University, Republic of Korea
 */ 
/* Servo.h library is compatible with the 
 * avr, megaavr, sam, samd, nrf52, stm32f4, mbed, 
 * mbed_nano, mbed_portenta, mbed_rp2040 
 * architectures so you should be able to use it at adequate boards.
 */

#include <Servo.h>
// Declare the name 'front' of the class servo
Servo front; 

char state;             // char type variable received from Visual Studio
const int servoPin = 9; // servo pin attacted to pin 9
const int angle_i=80;   // initialization of servo angle (range:55~105)
int value0=angle_i;
int value=value0;

void setup(){
  // put your setup code here, to run once:
  // Start serial communication with buadrate 9600
  Serial.begin(9600); 
  Serial.println("Arduino Ready,");
  front.attach(servoPin);
  front.write(angle_i);
}

void loop(){
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    state=Serial.read(); // receive char value from Serial (in VS)
    if (state == 'L'){value=55;}
    else if (state =='l'){value=66;}
    else if (state =='N'){value=80;}
    else if (state =='r'){value=94;}
    else if (state=='R'){value=105;}
    if (value0 < value)
    {
      for (int i = value0 ; i < value; i++)
      {
        front.write(i);
        delay(15);
      }
    value0=value;
    }
    else
    {
      for (int i = value0; i > value ; i--)
      {
        front.write(i);
        delay(15);
      }
    value0 = value;
    }
  }
}
