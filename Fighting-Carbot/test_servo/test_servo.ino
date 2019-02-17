//WIFI STUFF+++++++++++++++++++++++++++++++++++++++
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "Mechatronics";
const char* password = "YayFunFun";
WiFiUDP udpServer;
//WiFiUDP GoServer;
unsigned int udpTargetPort = 1800;

const int udpPacketSize = 10;
byte packetBuffer[udpPacketSize];

// Jiaqi Zhang
IPAddress ipTarget(192,168,1,213);
// Yuanyuan Wang
//IPAddress ipTarget(192,168,1,195);
IPAddress ipLocal(192,168,1,139);
//END WIFI STUFF+++++++++++++++++++++++++++++++++++

//SERVO STUFF++++++++++++++++++++++++++++++++
int servoChannel = 1;
int servoResolution = 8;
int initPos = 28;
int attackPos = 16;
int servoPin = 13;
int servoFreq = 50;
int attack = 0;

//END SERVO STUFF++++++++++++++++++++++++++++

////MOTOR STUFF++++++++++++++++++++++++++++++++++++++
//// init PWM using ledc
int motorFreq = 500;
int motorResolution = 12; // range: 0-4095
int EN12_channel = 3;
int EN34_channel = 4;
  
// initialize motor driver pins
int left_EN = 25;
int left_HIGH = 27;
int left_LOW = 26;
int right_EN = 21;
int right_HIGH = 17;
int right_LOW = 16; 

// init commands  
int forward = 0;
int back = 0; 
int left = 0; 
int right = 0;
////END MOTOR STUFF++++++++++++++++++++++++++++++++++

//MOTOR STUFF+++++++++++++++++++++++++++++++++++++++++++
void actuate_motors(int forward, int back, int left, int right){
  // straight speed
  int straight_speed = 2000;
  // turn speed
  int turn_speed = 1300;
  // drive param
  float param = 0.87;
  if(forward==3){
    int left_dc = straight_speed;
    int right_dc = param*straight_speed;
    // change speed by readix3ng pot_inputs
    ledcWrite(EN12_channel, left_dc);
    ledcWrite(EN34_channel, right_dc);
    // left motor rotate forward
    digitalWrite(left_HIGH,HIGH);
    digitalWrite(left_LOW, LOW);
    //right motor rotate forward
    digitalWrite(right_HIGH, HIGH);
    digitalWrite(right_LOW, LOW);
  }
  else if(back==3){
    int left_dc = straight_speed;
    int right_dc = param*straight_speed;
    // change speed by readix3ng pot_inputs
    ledcWrite(EN12_channel, left_dc);
    ledcWrite(EN34_channel, right_dc);
    // left motor rotate forward
    digitalWrite(left_HIGH,LOW);
    digitalWrite(left_LOW, HIGH);
    //right motor rotate forward
    digitalWrite(right_HIGH, LOW);
    digitalWrite(right_LOW, HIGH);
  }
  else if(left==3){
    int left_dc = turn_speed;
    int right_dc = param*turn_speed;
    // change speed by readix3ng pot_inputs
    ledcWrite(EN12_channel, left_dc);
    ledcWrite(EN34_channel, right_dc);
    // left motor rotate forward
    digitalWrite(left_HIGH,LOW);
    digitalWrite(left_LOW, HIGH);
    //right motor rotate forward
    digitalWrite(right_HIGH, HIGH);
    digitalWrite(right_LOW, LOW);
  }
  else if(right==3){
    int left_dc = turn_speed;
    int right_dc = param*turn_speed;
    // change speed by readix3ng pot_inputs
    ledcWrite(EN12_channel, left_dc);
    ledcWrite(EN34_channel, right_dc);
    // left motor rotate forward
    digitalWrite(left_HIGH,HIGH);
    digitalWrite(left_LOW, LOW);
    //right motor rotate forward
    digitalWrite(right_HIGH, LOW);
    digitalWrite(right_LOW, HIGH);
  }
  else{
    // joystick neutral-position, stop
    digitalWrite(left_HIGH, LOW);
    digitalWrite(left_LOW, LOW);
    //right motor rotate forward
    digitalWrite(right_HIGH, LOW);
    digitalWrite(right_LOW, LOW);   
  }
}
//END MOTOR STUFF+++++++++++++++++++++++++++++++++++++++

//  SETUP###############################################
//  ####################################################
//  ####################################################
//  ####################################################
void setup() {
  Serial.begin(115200);
  
  //MOTOR STUFF++++++++++++++++++++++++++++++++++++++++++
  // set up motor pins
  ledcSetup(EN12_channel,motorFreq,motorResolution);
  ledcSetup(EN34_channel, motorFreq, motorResolution);
  pinMode(left_EN, OUTPUT);
  ledcAttachPin(left_EN,EN12_channel);
  pinMode(left_HIGH, OUTPUT);
  pinMode(left_LOW, OUTPUT);
  pinMode(right_EN, OUTPUT);
  ledcAttachPin(right_EN,EN34_channel);
  pinMode(right_HIGH, OUTPUT);
  pinMode(right_LOW, OUTPUT);  
  //END MOTOR STUFF++++++++++++++++++++++++++++++++++++++
  
//  //SERVO STUFF++++++++++++++++++++++++++++++++++++++++++
  ledcSetup(servoChannel, servoFreq, servoResolution);
  ledcAttachPin(servoPin, servoChannel);
//  
//  //END SERVO STUFF++++++++++++++++++++++++++++++++++++++
   //WIFI STUFF+++++++++++++++++++++++++++++++++++++++++++
  // set up wifi communication 
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.config(ipLocal, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  // set up actuator controller communication
  udpServer.begin(udpTargetPort);
  
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected as "); 
  Serial.print(WiFi.localIP());
  //END WIFI STUFF+++++++++++++++++++++++++++++++++++++++
}

void loop(){
  
    //WIFI STUFF+++++++++++++++++++++++++++++++++++++++++++++++
    // receive command
    int cb = udpServer.parsePacket();
    if(cb){
      Serial.println("Packet Received");
      udpServer.read(packetBuffer, udpPacketSize);
      //for motors
      forward = packetBuffer[1];
      back = packetBuffer[2];
      left = packetBuffer[3];
      right = packetBuffer[4];
      //for servos
      attack = packetBuffer[5];
    }  
    //END WIFI STUFF+++++++++++++++++++++++++++++++++++++++++++
    //SERVO STUFF++++++++++++++++++++++++++++++++++++++++++++++
    // Attack 
    ledcWrite(servoChannel,initPos);
    if (attack==3){
      ledcWrite(servoChannel, attackPos);
    }
    Serial.println(attack);
    
    //END SERVO STUFF++++++++++++++++++++++++++++++++++++++++++
  
    //MOTOR STUFF++++++++++++++++++++++++++++++++++++++++++++++
    // actuate 
    Serial.print("Motor: ");
    Serial.print(forward);
    Serial.print(back);
    Serial.print(left);
    Serial.println(right);
    actuate_motors(forward, back, left, right);
    //delay(50);
    //END MOTOR STUFF++++++++++++++++++++++++++++++++++++++++++
}
