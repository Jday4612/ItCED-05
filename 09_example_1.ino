// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.5 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.
#define N 3 //N = 3, 10, 30

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, alpha, median; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float med[N+1];
float tmp[N];
int i = 0;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  med[N] = dist_raw;
  median = dist_raw;
  
  for(int i =0; i < N; i++){
    med[i] = med[i + 1];
    tmp[i] = med[i];
  }

  if(dist_raw == 0){
    for(int i = 0; i < N - 1; i++){
      for(int j = 0; j < N - 1; j++){
        if(tmp[j] > tmp[j + 1]){
          int temp = tmp[j];
          tmp[j] = tmp[j + 1];
          tmp[j + 1] = temp;
        }
      }
    }
    if(N%2 == 0) {
      median = (tmp[N/2] + tmp[N/2 + 1])/2;
    }
    else if(N%2 == 1) {
      median = tmp[N/2 + 1];
    }
  }

// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median:");
  Serial.print(map(median,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}
