#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0
Servo myservo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  myservo.attach(PIN_SERVO);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  // put your main code here, to run repeatedly:
  float raw_dist = 2 * ir_distance() - 30;
  
  Serial.print("min:0,max:500,dist:");
  Serial.println(raw_dist);
  delay(20);

  if(raw_dist < 255) {
    myservo.writeMicroseconds(1600);
    delay(20);
  }
  else {
    myservo.writeMicroseconds(800);
    delay(20);
  }
}
