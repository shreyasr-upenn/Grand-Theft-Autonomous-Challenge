#define PERIOD 715
#define NOISE 200
#define PIN1 4
#define PIN2 5
#define SERVO_PIN 1
#define LEFT_OUT 6
#define RIGHT_OUT 7
#define STRAIGHT_OUT 10

#define LEDC_CHANNEL 0 // use first of 6  
#define LEDC_RESOLUTION_BITS 12
#define LEDC_RESOLUTION ((1<<LEDC_RESOLUTION_BITS)-1) 
#define LEDC_FREQ_HZ 50
#define SERVOOFF 307 //? (1.5ms/20ms * 4096)

uint32_t us;
int x;     // relative angle of sensed beacon 
int pin[] = {PIN1, PIN2};

int mydigitalRead(int pin_num) {
  if (analogRead(pin_num) > 2048) {
    return 1;
  } else {
    return 0;
  }
}

void objAngle(int ch) {
  static int oldpin[2];
  static uint32_t oldtime[2];
  if (mydigitalRead(pin[ch]) != oldpin[ch]) {
    int per = us-oldtime[ch];
    Serial.print(ch);
    Serial.print(" per: ");
    Serial.println(per);
    if ((per >= 600) && (per < 800)) {//600 800
      if (ch==0) {
        x = min(x+5, 615);
        Serial.println("++");
      }
      else {
        x = max(x-5, -615);
        Serial.println("--");
      }
    }
    oldpin[ch] = mydigitalRead(pin[ch]);
    oldtime[ch] = us;
  }
}

void setup() {
  pinMode(PIN1, INPUT);
  pinMode(PIN2, INPUT);
  pinMode(LEFT_OUT, OUTPUT);
  pinMode(RIGHT_OUT, OUTPUT);
  pinMode(STRAIGHT_OUT, OUTPUT);
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(SERVO_PIN, LEDC_CHANNEL);

  Serial.begin(115200);
  Serial.println("Setup Complete");
  
  x = 0;
}

void loop() {
  //Serial.println(x);
  
  static uint32_t lastServo = micros();
  us = micros();
  objAngle(0); // update x if sensed left
  objAngle(1); // update x if sensed right

  int serve;
  
  // update the servo position
  if (us-lastServo > 1000000/10) { 
    serve = SERVOOFF+x;
    Serial.printf("servo x=%d \n", serve);
    ledcWrite(0, serve);
    lastServo = us;

    if (serve < 130) {
    Serial.println("left");
    digitalWrite(LEFT_OUT, LOW);
    digitalWrite(RIGHT_OUT, HIGH);
    digitalWrite(STRAIGHT_OUT, LOW);
  } else if (serve > 250) {
    Serial.println("right");
    digitalWrite(LEFT_OUT, HIGH);
    digitalWrite(RIGHT_OUT, LOW);
    digitalWrite(STRAIGHT_OUT, LOW);
  } else {    
    Serial.println("straight");
    digitalWrite(LEFT_OUT, LOW);
    digitalWrite(RIGHT_OUT, LOW);
    digitalWrite(STRAIGHT_OUT, HIGH);
  }
  }  
  delayMicroseconds(200);
}
