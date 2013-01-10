#include "Servotor32.h" // call the servotor32 Library
Servotor32 hexy; // create a servotor32 object

void setup() {
  hexy.begin();
}

void loop() {
  // blink the status led
  digitalWrite(STATUS_LED, HIGH);
  hexy.delay_ms(500); // wait 500mS
  digitalWrite(STATUS_LED, LOW);
  hexy.delay_ms(500); // wait 500mS
  
  // kill all servos
  for(int i=0; i<32; i++){
    hexy.changeServo(i,-1);
  }
  
  // center servos 0,1,2,3
  hexy.changeServo(0,1500);
  hexy.changeServo(1,1500);
  hexy.changeServo(2,1500);
  hexy.changeServo(3,1500);
  
  hexy.delay_ms(100); // wait 100mS
 
  while(true){
    // get a ping from the ultrasonic sensor in CM
    //Serial.print("CM: ");
    //Serial.println(hexy.ping());
    //hexy.delay_ms(200); // wait 200mS
    if(Serial.available()) { //process input from the USB
      char inChar = (char)Serial.read();
      hexy.processChar(inChar);
    }
    if(Serial1.available()){ //process input from the board serial (i.e. bluetooth)
      char inChar = (char)Serial1.read();
      hexy.processChar(inChar);
    }
  }
  
}




