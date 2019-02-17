#include <WiFi.h>
#include <WiFiUdp.h>

// init PWM using ledc
int freq = 500;
int resolution = 12; // range: 0-4095
int EN12_channel = 0;
int EN34_channel = 1;

// initialize motor driver pins
int left_EN = 32;
int left_HIGH = 33;
int left_LOW = 14;
int right_EN = 22;
int right_HIGH = 21;
int right_LOW = 18; 

// wireless comm transmitted reading 
int mspeed = 0;
int mdirection = 0;
int left= 0;
int right = 0;
int left_speed = 0;
int right_speed = 0;

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
  pinMode(2, OUTPUT);
  
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
  digitalWrite(2,HIGH);
  Serial.println("WiFi connected as "); 
  Serial.print(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly
  // read speed and direction from control
  int cb = udpServer.parsePacket();
  if(cb){
    Serial.println("Packet received");
    handleUDPServer(); 
    Serial.println("Speed command: ");
    Serial.println(mspeed);
    Serial.println("Direction command: ");
    Serial.println(mdirection);
  }
  actuate_motors();
}

void handleUDPServer(){
  udpServer.read(packetBuffer, udpPacketSize);
  mspeed = packetBuffer[1] + (packetBuffer[2]<<8);
  mdirection = packetBuffer[3] + (packetBuffer[4]<<8);
}

void actuate_motors(){
  // check direction 
  if (mdirection>2800){ // left
    // set speed for left and right motor
    left_speed = 0;
    right_speed = mspeed;
    int left_dc = left_speed*96;
    int right_dc = right_speed;
    // change speed by reading pot_inputs
    ledcWrite(EN12_channel, left_dc);
    ledcWrite(EN34_channel, right_dc);
    //left motor stops
    digitalWrite(left_HIGH, LOW);
    digitalWrite(left_LOW, LOW);
    //right motor rotate forward
    digitalWrite(right_HIGH, HIGH);
    digitalWrite(right_LOW, LOW);
  } 
  else if (mdirection<800){ //right
    // set speed for left and right motor
    right_speed = 0;
    left_speed = mspeed;
    int left_dc = left_speed*0.96;
    int right_dc = right_speed;
    // change speed by reading pot_inputs
    ledcWrite(EN12_channel, left_dc);
    ledcWrite(EN34_channel, right_dc);
    //left motor rotate forward
    digitalWrite(left_HIGH, HIGH);
    digitalWrite(left_LOW, LOW);
    //right motor stops
    digitalWrite(right_HIGH,LOW);
    digitalWrite(right_LOW,LOW);
  }
  else{
//    both motors rotate forward in read speed;
    left_speed = mspeed;
    right_speed = mspeed;
    int left_dc = left_speed*0.96;
    int right_dc = right_speed;
    // change speed by reading pot_inputs
    ledcWrite(EN12_channel, left_dc);
    ledcWrite(EN34_channel, right_dc);
    // left motor rotate forward
    digitalWrite(left_HIGH, HIGH);
    digitalWrite(left_LOW, LOW);
    //right motor rotate forward
    digitalWrite(right_HIGH, HIGH);
    digitalWrite(right_LOW, LOW);
  }
  if (mspeed<400){
    left_speed = 2500;
    right_speed = 2500;
    int left_dc = left_speed*0.96;
    int right_dc = right_speed;
    // change speed by reading pot_inputs
    ledcWrite(EN12_channel, left_dc);
    ledcWrite(EN34_channel, right_dc);
    digitalWrite(left_HIGH,LOW);
    digitalWrite(left_LOW, HIGH);
    digitalWrite(right_HIGH, LOW);
    digitalWrite(right_LOW, HIGH);
  }
}
