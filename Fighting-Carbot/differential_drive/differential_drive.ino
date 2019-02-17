#include <WiFi.h>
#include <WiFiUdp.h>

// init PWM using ledc
int freq = 500;
int resolution = 12; // range: 0-4095
int EN12_channel = 0;
int EN34_channel = 1;

// initialize motor driver pins
int left_EN = 25;
int left_HIGH = 26;
int left_LOW = 27;
int right_EN = 21;
int right_HIGH = 17;
int right_LOW = 16; 

// init commands  
int forward = 1;
int back = 0; 
int left = 0; 
int right = 0;
  
// wifi settings
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // need to comment out later 
  // set up motor pins
  ledcSetup(EN12_channel,freq,resolution);
  ledcSetup(EN34_channel, freq, resolution);
  pinMode(left_EN, OUTPUT);
  ledcAttachPin(left_EN,EN12_channel);
  pinMode(left_HIGH, OUTPUT);
  pinMode(left_LOW, OUTPUT);
  pinMode(right_EN, OUTPUT);
  ledcAttachPin(right_EN,EN34_channel);
  pinMode(right_HIGH, OUTPUT);
  pinMode(right_LOW, OUTPUT);

////////////////////////////////////////////////
// set up wifi communication 
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.config(ipLocal, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  // set up acutator controller communication
  udpServer.begin(udpTargetPort);
  
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected as "); 
  Serial.print(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:
  // receive command
  int cb = udpServer.parsePacket();
  if(cb){
    Serial.println("Packet Received");
    udpServer.read(packetBuffer, udpPacketSize);
    forward = packetBuffer[1];
    back = packetBuffer[2];
    left = packetBuffer[3];
    right = packetBuffer[4];
  }
  // actuate 
  Serial.print(forward);
  Serial.print(back);
  Serial.print(left);
  Serial.println(right);
  actuate_motors(forward, back, left, right);
  delay(50);
}

void actuate_motors(int forward, int back, int left, int right){
  // straight speed
  int straight_speed = 2000;
  // turn speed
  int turn_speed = 1500;
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
