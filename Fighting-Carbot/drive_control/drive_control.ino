#include <WiFi.h>
#include <WiFiUdp.h>

int forward_pin = 32;
int back_pin = 33;
int left_pin = 34;
int right_pin = 35;

int forward = 2; 
int back = 2; 
int left = 2; 
int right = 2; 

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
  Serial.begin(115200);
  pinMode(forward_pin, INPUT);
  pinMode(back_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(right_pin, INPUT);

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
  forward = digitalRead(forward_pin);
  back = digitalRead(back_pin);
  left = digitalRead(left_pin);
  right = digitalRead(right_pin);
  Serial.print(forward);
  Serial.print(back);
  Serial.print(left);
  Serial.println(right);
  fncUdpSend();
  delay(50);
}

void fncUdpSend(){
  udpBuffer[0] = 11;
  udpBuffer[1] = forward+2;
  udpBuffer[2] = back+2;
  udpBuffer[3] = left+2;
  udpBuffer[4] = right+2;
  udpServer.beginPacket(ipTarget, udpTargetPort);
  udpServer.printf("%s",udpBuffer);
  udpServer.endPacket();
  Serial.println("Packet Sent");
}
