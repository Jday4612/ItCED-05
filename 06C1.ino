#define PIN_LED 7
int p, d, value;

void set_period(int period) {
  p = period;
}

void set_duty(int duty){
  d = p / 100;
  triangle(duty);
}

void triangle(int triangle){
  digitalWrite(PIN_LED, 0);
  delayMicroseconds(triangle);
  digitalWrite(PIN_LED, 1);
  delayMicroseconds(p - triangle);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
  //1ms = 1000us
  set_period(100); // period = 1000 ; period = 10000;

  if (p == 10000)
    value = 10000 / p; //0.1ms, 1ms는 1초 동안, 10ms는 2초 동안
  else
    value = 5000 / p;
}

void loop() {
  for(int i = 0 ; i <= p ; i += d){
    for(int j = 0 ; j < value ; j ++)
      set_duty(i);
  }

  for(int i = p ; 0 <= i  ; i -= d){
    for(int j = 0 ; j < value ; j ++)
      set_duty(i);
  }
}
