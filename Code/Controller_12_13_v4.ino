#include "html510.h"
#include "body.h"
#include <WiFi.h> 
#include <esp_now.h>
#include "Adafruit_VL53L0X.h" //ToF library
#include "Wire.h" //I2C
#include <WiFiUdp.h>
#include "vive510.h"
#include "MedianFilterLib2.h" //median software filter

/*IR BEACON TRACKER DEFINITIONS*/
//signals from ESP32C3
#define IR_LEFT_IN 4
#define IR_RIGHT_IN 5
#define IR_STRAIGHT_IN 6
uint32_t ir_us;
int ir_x;     // relative angle of sensed beacon 

/*ULTRASONIC DEFINITIONS*/
#define trigPin1 1
#define echoPin1 2
#define trigPin2 34
#define echoPin2 33
int USdistance[2]; //distance readings
MedianFilter2<long> USMedian1(3);
MedianFilter2<long> USMedian2(3);
long distance1m, distance2m;


/*TIME OF FLIGHT DEFINITIONS*/
#define SDA_PIN 13
#define SCL_PIN 14
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MedianFilter2<int> TOFMedian(3);
int frontDistm;

/*VIVE DEFINITIONS*/
int robotVive[2];
#define VIVE_SIGNALPIN1 7 // pin receiving signal from Vive circuit
Vive510 vive1(VIVE_SIGNALPIN1);
MedianFilter2<int> vivemedianx1(3);
MedianFilter2<int> vivemediany1(3);

/*MOTION DEFINITIONS*/
int moveRequest[2];
#define L_LEDC_CHANNEL 1
#define R_LEDC_CHANNEL 2

#define M_LEDC_FREQ_HZ 50
#define M_LEDC_RESOLUTION_BITS 10
#define M_LEDC_RESOLUTION ((1<<LEDC_RESOLUTION_BITS)-1)

#define L_MOTOR_PWM_PIN 12
#define L_MOTOR_DIR_PIN_1 10
#define L_MOTOR_DIR_PIN_2 11

#define R_MOTOR_PWM_PIN 15
#define R_MOTOR_DIR_PIN_1 16
#define R_MOTOR_DIR_PIN_2 17

#define MOTOR_DUTY 500


/*WIFI DEFINITIONS*/
uint8_t teamNum = 18;
IPAddress myIP(192, 168, 1, 130);     // change to your IP 

/*UDP + POLICE DEFINITONS*/
#define UDPPORT 2510 // port for game obj transmission
WiFiUDP UDPServer;
int policeVive[2];

// uncomment the router SSID and Password that is being used 
const char* ssid     = "TP-Link_05AF";
const char* password = "47543454";

int robotMode; //task requested
int lastCom; //time since last x,y location sent to staff


HTML510Server h(80);

/*WEBSITE HANDLE FUNCTIONS*/
void handleRoot(){
  pingstaff();
  h.sendhtml(body);
}

//Manual movement control
void handleForward(){
  robotMode = 0;
  pingstaff();
  Serial.println("forward");
  moveForward();
  h.sendplain("Requested");
}

void handleBackward(){
  robotMode = 0;
  pingstaff();
  Serial.println("backward");
  moveBackward();
  h.sendplain("Requested");
}

void handleRight(){
  robotMode = 0;
  pingstaff();
  Serial.println("right");
  turnRight();
  h.sendplain("Requested");
}

void handleLeft(){
  robotMode = 0;
  pingstaff();
  Serial.println("left");
  turnLeft();
  h.sendplain("Requested");
}

//Find/go to trophy: mode 1
void handleTrophy(){
  robotMode = 1;
  pingstaff();
  h.sendplain("Requested");
}

//Find/go to fake: mode 2
void handleFake() {
  robotMode = 2;
  pingstaff();
  h.sendplain("Requested"); 
}

//Find/go to police car: mode 3
void handlePolice() {
  Serial.println("Handle Police Car");
  robotMode = 3;
  pingstaff();
  String s = "Requested";
  h.sendplain(s); 
}

//Follow wall: mode 4
void handleWall() {
  robotMode = 4;
  pingstaff();
  h.sendplain("Requested"); 
}

//Move to x,y: mode 5
void handleMove() {
  robotMode = 5;
  pingstaff();
  int x = -1; int y = -1;
  
  String txt = h.getText();
  Serial.println(txt);
  char* token = strtok(&txt[0], ",");

  if (token != NULL){
    String xs = token;
    x = xs.toInt();
    Serial.println(x);
    token=strtok(NULL, ",");
  } else {
    h.sendplain("Input failure");
    return;
  }

  if (token != NULL){
    String ys = token;
    y = ys.toInt();
    Serial.println(y);
  } else {
    h.sendplain("Input failure");
    return;
  }

  moveRequest[0] = x;
  moveRequest[1] = y;
  
  String s = "Move requested to ";
  s.concat(x);
  s.concat(", ");
  s.concat(y);
  h.sendplain(s);
}

//Stop: mode 6
void handleStop() {
  robotMode = 6;
  pingstaff();
  h.sendplain("Requested"); 
}

/*WIFI FUNCTIONS*/
esp_now_peer_info_t staffcomm = {
  .peer_addr = {0x84,0xF7,0x03,0xA9,0x04,0x78}, 
  .channel = 1,             
  .encrypt = false,
};

void pingstaff() {
  esp_now_send(staffcomm.peer_addr, &teamNum, 1);     
}

void sendXY() {
  char msg[13];
  if(robotVive[0] > 9999 || robotVive[1] > 9999) {
    sprintf(msg,"%2d:0000,0000",teamNum); 
  } else {
    sprintf(msg,"%2d:%4d,%4d",teamNum, robotVive[0], robotVive[1]);     
  }
  esp_now_send(staffcomm.peer_addr, (uint8_t *) msg, 13);
}

/*UDP FUNCTIONS*/
//Recieves UDP communications
bool handleUDPServer() {
   const int UDP_PACKET_SIZE = 14; // can be up to 65535          
   uint8_t packetBuffer[UDP_PACKET_SIZE];

   int cb = UDPServer.parsePacket(); // if there is no message cb=0
   int x,y;
   int teamUDP;
   if (cb) {
      packetBuffer[13]=0; // null terminate string

      UDPServer.read(packetBuffer, UDP_PACKET_SIZE);
      teamUDP = atoi((char *)packetBuffer);
      x = atoi((char *)packetBuffer+3); // ##,####,#### 2nd indexed char
      y = atoi((char *)packetBuffer+8); // ##,####,#### 7th indexed char
      Serial.print("From Team ");
      Serial.println(teamUDP);
      Serial.println(x);
      Serial.println(y);
   }

   if (teamUDP == 0) {
      policeVive[0] = x;
      policeVive[1] = y;
      return true;
   } else {
      return false;
   }
}

/*ULTRASONIC FUNCTIONS*/
void runUltrasonic(){
  long duration1 , distance1 , duration2 , distance2;

  //send and recieve a pulse on the first sensor
  digitalWrite (trigPin2 , LOW );
  delayMicroseconds (2);
  digitalWrite (trigPin2 , HIGH );
  delayMicroseconds (10);
  digitalWrite (trigPin2 , LOW );
  duration2 = pulseIn (echoPin2 , HIGH );
  Serial.println(duration2);

  //send and recieve a pulse on the second sensor
  digitalWrite (trigPin1 , LOW ); 
  delayMicroseconds (2);  
  digitalWrite (trigPin1 , HIGH ); 
  delayMicroseconds (10); 
  digitalWrite (trigPin1 , LOW ); 
  duration1 = pulseIn (echoPin1 , HIGH );
  Serial.println(duration1);

  //Calculate the distance of the reflecting surface
  distance1 = (duration1 / 2) / 29.1; 
  distance2 = (duration2 / 2) / 29.1;
  
  Serial.print("distance 1: ");
  Serial.print(distance1);
  Serial.print("; distance 2: ");
  Serial.println(distance2);

  //filter and record the distance values
  distance1m = USMedian1.AddValue(distance1);
  distance2m = USMedian2.AddValue(distance2);
  USdistance[0] = distance1m;
  USdistance[1] = distance2m;
}

/*VIVE FUNCTIONS*/
void readVive() {
  int x, y;
  
  if (vive1.status() == VIVE_RECEIVING) {
    if (vive1.xCoord() < 9999 && vive1.yCoord() <=9999){
      x = vive1.xCoord();
      y = vive1.yCoord();
    }
    else {
      x = 0;
      y = 0;      
    }
    
    robotVive[0] = x;
    robotVive[1] = y;
    Serial.print("VIVE: ");
    Serial.print(robotVive[0]); Serial.print(", ");
    Serial.println(robotVive[1]);
  }
  else {
    x=0;
    y=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        Serial.println("VIVE: Sync only");
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected 
        Serial.println("VIVE: No signal"); 
        break;   
    }
  }
}

/*MOTION FUNCTIONS*/
//Unimplemented due to issues with the second vive circuit
void moveToXY (int x, int y) {
  Serial.println("Align robot with +x");
  if (x >= robotVive[0]) {
    Serial.println("move forward until x = robotVive[0]; repeat for y");
  } else {
    Serial.println("move backwards until x = robotVive[0]; repeat for y");
  }
}

//Motor control functions
//both motors forward
void moveForward() {
  digitalWrite(L_MOTOR_DIR_PIN_1,HIGH); 
  digitalWrite(L_MOTOR_DIR_PIN_2,LOW);
  ledcWrite(L_LEDC_CHANNEL, MOTOR_DUTY);
  digitalWrite(R_MOTOR_DIR_PIN_1,HIGH); 
  digitalWrite(R_MOTOR_DIR_PIN_2,LOW);
  ledcWrite(R_LEDC_CHANNEL, MOTOR_DUTY);
}
//both motors backward
void moveBackward() {
  ledcWrite(L_LEDC_CHANNEL, MOTOR_DUTY);
  digitalWrite(L_MOTOR_DIR_PIN_1,LOW); 
  digitalWrite(L_MOTOR_DIR_PIN_2,HIGH);
  ledcWrite(R_LEDC_CHANNEL, MOTOR_DUTY);
  digitalWrite(R_MOTOR_DIR_PIN_1,LOW); 
  digitalWrite(R_MOTOR_DIR_PIN_2,HIGH); 
}
//right forward, left backward
void turnLeft() {
  ledcWrite(R_LEDC_CHANNEL, MOTOR_DUTY);
  digitalWrite(R_MOTOR_DIR_PIN_1,HIGH); 
  digitalWrite(R_MOTOR_DIR_PIN_2,LOW);
  ledcWrite(L_LEDC_CHANNEL, MOTOR_DUTY);
  digitalWrite(L_MOTOR_DIR_PIN_1,LOW); 
  digitalWrite(L_MOTOR_DIR_PIN_2,HIGH); 
}
//left forward, right backward
void turnRight() {
  ledcWrite(L_LEDC_CHANNEL, MOTOR_DUTY);
  digitalWrite(L_MOTOR_DIR_PIN_1,HIGH); 
  digitalWrite(L_MOTOR_DIR_PIN_2,LOW);
  ledcWrite(R_LEDC_CHANNEL, MOTOR_DUTY);
  digitalWrite(R_MOTOR_DIR_PIN_1,LOW); 
  digitalWrite(R_MOTOR_DIR_PIN_2,HIGH); 
}
//Stop all movement
void stopMotion() {
  ledcWrite(L_LEDC_CHANNEL, 0);
  ledcWrite(R_LEDC_CHANNEL, 0);
}

WiFiServer server(80);
                                          
void setup() {
  /*WIFI SETUP*/
  Serial.begin(115200);          
  WiFi.mode(WIFI_MODE_STA);  
  WiFi.begin(ssid, password);
  WiFi.config(myIP,IPAddress(192,168,1,1),
             IPAddress(255,255,255,0));
  while(WiFi.status()!= WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("connected to %s on",ssid); //Serial.println(myIP);

  /*UDP SETUP*/
  UDPServer.begin(UDPPORT);     
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);    Serial.print(".");
  }
  Serial.print("Using static IP "); Serial.print(myIP); 
  Serial.print(" and UDP port "); Serial.println(UDPPORT);
  
  /*WEBSITE SETUP*/
  h.begin(80);
  h.attachHandler("/ ",handleRoot);
  h.attachHandler("/trophy",handleTrophy);
  h.attachHandler("/fake",handleFake);
  h.attachHandler("/police",handlePolice);
  h.attachHandler("/wall",handleWall);  
  h.attachHandler("/stop",handleStop);
  h.attachHandler("/move=", handleMove);
  h.attachHandler("/forward",handleForward);
  h.attachHandler("/backward",handleBackward);
  h.attachHandler("/right",handleRight);
  h.attachHandler("/left",handleLeft);
  robotMode = 0;

  /*ESPNOW SETUP*/
  esp_now_init();      
  esp_now_add_peer(&staffcomm);

  /*IR BEACON TRACKER SETUP*/
  pinMode(IR_LEFT_IN, INPUT);
  pinMode(IR_RIGHT_IN, INPUT);
  pinMode(IR_STRAIGHT_IN, INPUT);

  /*ULTRASONIC SETUP*/
  pinMode (trigPin1 , OUTPUT );
  pinMode (echoPin1 , INPUT );
  pinMode (trigPin2 , OUTPUT );
  pinMode (echoPin2 , INPUT );

  /*TIME OF FLIGHT SETUP*/
  Wire.begin(SDA_PIN, SCL_PIN);  
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1); 
  }
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));

  /*VIVE SETUP*/
  robotVive[0] = 0; robotVive[1] = 0;
  vive1.begin();

  /*MOTION SETUP*/
  moveRequest[0] = 0; moveRequest[1] = 0;
  pinMode(R_MOTOR_DIR_PIN_1, OUTPUT);
  pinMode(R_MOTOR_DIR_PIN_2, OUTPUT);
  pinMode(L_MOTOR_DIR_PIN_1, OUTPUT);
  pinMode(L_MOTOR_DIR_PIN_2, OUTPUT);
  pinMode(R_MOTOR_PWM_PIN, OUTPUT);
  pinMode(L_MOTOR_PWM_PIN, OUTPUT);
  ledcSetup(R_LEDC_CHANNEL, M_LEDC_FREQ_HZ, M_LEDC_RESOLUTION_BITS);
  ledcSetup(L_LEDC_CHANNEL, M_LEDC_FREQ_HZ, M_LEDC_RESOLUTION_BITS);
  ledcAttachPin(R_MOTOR_PWM_PIN, R_LEDC_CHANNEL);
  ledcAttachPin(L_MOTOR_PWM_PIN, L_LEDC_CHANNEL); 
 
  lastCom = 0;
  
  Serial.println("Setup complete");  
}

void loop(){
  h.serve();

  if(robotMode == 1) { //trophy finding functionality
    if (digitalRead(IR_LEFT_IN) == HIGH){
      Serial.println(" ir left") ; 
      turnLeft(); delay(300);   
      stopMotion(); delay(200);
    } else if (digitalRead(IR_RIGHT_IN) == HIGH){
      Serial.println(" ir right");
      turnRight(); delay(300);
      stopMotion(); delay(200);
    } else {
      Serial.println(" ir straight");
      moveForward(); delay(300);
      stopMotion(); delay(200);
    }
    
  } else if (robotMode == 2){ //fake finding functionality
    if (digitalRead(IR_LEFT_IN) == HIGH){
      Serial.println(" ir left") ; 
      turnLeft(); delay(300);   
      stopMotion(); delay(200);
    } else if (digitalRead(IR_RIGHT_IN) == HIGH){
      Serial.println(" ir right");
      turnRight(); delay(300);
      stopMotion(); delay(200);
    } else {
      Serial.println(" ir straight");
      moveForward(); delay(300);
      stopMotion(); delay(200);
    }
    
  } else if (robotMode == 3) { //police car finding functionality
    //unimplmented due to issues with the second vive circuit
    while (!handleUDPServer());
    Serial.print("Move to ");
    Serial.print(policeVive[0]);
    Serial.print(", ");
    Serial.println(policeVive[1]);
    moveToXY(policeVive[0], policeVive[1]);    
        
  } else if (robotMode == 4) { //wall following functionality
    //Check distance to walls on the sides
    runUltrasonic();
    if (USdistance[0] < 15 && USdistance[0] > 0) { //too close to the wall on the left
      Serial.println("close to left");
      turnRight(); delay(105);
      moveForward(); delay(100);
      turnLeft(); delay(100);
      moveForward(); delay(200);
    } else if (USdistance[1] < 15 && USdistance[1] > 0) { //too close to the wall on the right
      Serial.println("close to right");
      turnLeft(); delay(105);
      moveForward(); delay(100);
      turnRight(); delay(100);
      moveForward(); delay(200);
    } else if(USdistance[0] > 40 && (USdistance[1] > 50 || USdistance[1] == 0)){ //too far from the wall on the left
      Serial.println("far from left");
      turnLeft(); delay(120);
      moveForward(); delay(100);
      turnRight(); delay(100);
      moveForward(); delay(200);
    } else if(USdistance[1] > 40 && (USdistance[0] > 50 || USdistance[0] == 0)){ //too far from the wall on the right
      Serial.println("far from right");
      turnRight(); delay(120);
      moveForward(); delay(100);
      turnLeft(); delay(100);
      moveForward();delay(200);
    } else {
      Serial.println("forward");
      moveForward(); delay(200);
    }

    //Check existance of a wall in the front
    Serial.println("check ToF in the front");
    VL53L0X_RangingMeasurementData_t measure;
    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); 
    if (measure.RangeStatus != 4) { // phase failures have incorrect data
      int frontDist = measure.RangeMilliMeter;
      frontDistm = TOFMedian.AddValue(frontDist); //filter the signal
      Serial.print("Distance (mm): "); Serial.println(frontDistm);

      //If wall exists in the front decide which side wall is closer then turn away
      if (frontDistm < 900){
        if (USdistance[1] > 40 || USdistance[1] == 0){
          Serial.println("Turn right, then forward - Front and Left walls");
          moveBackward(); delay(300);
          turnRight(); delay(250); //change delay to get 90 deg
          stopMotion(); delay(10);
        } else {
          Serial.println("Turn left, then forward - Front and Right Walls");
          moveBackward(); delay(300);
          turnLeft(); delay(250); //change delay to get 90 deg
          stopMotion(); delay(10);
        }
      }
    } else {
      Serial.println(" out of range ");
    }
  } else if (robotMode == 5) { //Move to X,Y functionality
    //unimplmented due to issues with the second vive circuit
    Serial.println("Move to");
    moveToXY(moveRequest[0], moveRequest[1]);
        
  } else if (robotMode == 6) { //software stop
    Serial.println("Stop");
    stopMotion();
  }

  //Send the location to staff every second
  readVive();
  if ((micros() - lastCom) > 1000000) {
    sendXY();
    lastCom = micros();
  }
}
