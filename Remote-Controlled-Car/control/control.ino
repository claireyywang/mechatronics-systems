#include <WiFi.h>
#include <WiFiUdp.h>

// control pins setting
int speed_pin = 36;
int dir_pin = 34;

int mspeed = 0; 
int mdirection = 0;

// wifi setting
const char* ssid     = "Mechatronics";
const char* password = "YayFunFun";

WiFiUDP udpServer;
WiFiUDP GoServer;
unsigned int udpTargetPort = 1800;
unsigned int udpGoPort = 2390;

const int udpBufferSize = 10;
char udpBuffer[udpBufferSize];
const int udpPacketSize = 10;
byte packetBuffer[udpPacketSize];

int goReceived = 0;

// Victor Janniaud
IPAddress ipTarget(192,168,1,139);
// Yuanyuan Wang
//IPAddress ipTarget(192,168,1,195);
IPAddress ipLocal(192,168,1,213);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // to be commented
  // set up ADC pins 
  pinMode(speed_pin, INPUT);
  pinMode(dir_pin, INPUT);

////////////////////////////////////////
  // set up wifi communication sending side
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.config(ipLocal, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  //  // receive go command 
  GoServer.begin(udpGoPort);

  // set car port
  udpServer.begin(udpTargetPort);

  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("WiFi connected as "); 
  Serial.print(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:
  mspeed = analogRead(speed_pin);
  mdirection = analogRead(dir_pin);
  if (goReceived==0){
    if(GoServer.parsePacket()){
      handleGoServer();
    }
  }else{
    fncUdpSend();
  }
  delay(50);
//  Serial.println("command sent");
  
//  Serial.print("Speed reading: ");
//  Serial.println(mspeed);
//  Serial.print("Direction reading: ");
//  Serial.println(mdirection);
}

void fncUdpSend(){
  // need to expand buffer size
  udpBuffer[0] = 11;
  udpBuffer[1] = mspeed & 0xff;
  udpBuffer[2] = mspeed >> 8;
  udpBuffer[3] = mdirection & 0xff;
  udpBuffer[4] = mdirection >> 8;
  Serial.print("Speed reading: ");
  Serial.println((udpBuffer[2]<<8)+udpBuffer[1]);
  Serial.print("Direction reading: ");
  Serial.println((udpBuffer[4]<<8)+udpBuffer[3]);
  udpServer.beginPacket(ipTarget, udpTargetPort);
  udpServer.printf("%s",udpBuffer);
  udpServer.endPacket();
}

void handleGoServer(){
  GoServer.read(packetBuffer, udpPacketSize);
  char go = char(packetBuffer[0]);
  if (go == 'G'){
    goReceived = 1;
  }
  Serial.println(go);
  Serial.println(goReceived);
//  goReceived = 1;
}
