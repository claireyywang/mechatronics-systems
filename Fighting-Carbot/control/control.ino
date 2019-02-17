#include <WiFi.h>
#include <WiFiUdp.h>

//MOTOR STUFF+++++++++++++++++++++++++++++++++
int forward_pin = 33;
int back_pin = 32;
int left_pin = 34;
int right_pin = 35;

int forward = 2; 
int back = 2; 
int left = 2; 
int right = 2;
//END MOTOR STUFF+++++++++++++++++++++++++++++

//SERVO STUFF+++++++++++++++++++++++++++++++++
int pinA = 25;  
int pinL = 26;
int pinR = 27;

int A=2;
int L=2;
int R=2;
//END SERVO STUFF+++++++++++++++++++++++++++++

// wifi setting
const char* ssid     = "Central";
const char* password = "Y4yR0b0t5";

WiFiUDP udpServer;
WiFiUDP GoServer;
unsigned int udpTargetPort = 1800;

const int udpBufferSize = 10;
char udpBuffer[udpBufferSize];
const int udpPacketSize = 10;
byte packetBuffer[udpPacketSize];

// Victor Janniaud
IPAddress ipTarget(192,168,1,139);
// Yuanyuan Wang
//IPAddress ipTarget(192,168,1,195);
IPAddress ipLocal(192,168,1,213);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //MOTOR STUFF++++++++++++++++++++++++++++++++++
  pinMode(forward_pin, INPUT);
  pinMode(back_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(right_pin, INPUT);
  //END MOTOR STUFF++++++++++++++++++++++++++++++

  //SERVO STUFF++++++++++++++++++++++++++++++++++
  pinMode(pinA, INPUT);
  pinMode(pinL, INPUT);
  pinMode(pinR, INPUT);
  //END SERVO STUFF++++++++++++++++++++++++++++++

  ////////////////////////////////////////
  // set up wifi communication sending side
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.config(ipLocal, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

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
  //MOTOR STUFF+++++++++++++++++++++++++++++++++++++++
  forward = digitalRead(forward_pin);
  back = digitalRead(back_pin);
  left = digitalRead(left_pin);
  right = digitalRead(right_pin);
  Serial.print(forward);
  Serial.print(back);
  Serial.print(left);
  Serial.println(right);
  //END MOTOR STUFF+++++++++++++++++++++++++++++++++++

  //SERVO STUFF+++++++++++++++++++++++++++++++++++++++
  A = digitalRead(pinA);
  L = digitalRead(pinL);
  R = digitalRead(pinR);
  Serial.print(A);
  Serial.print(L);
  Serial.println(R);
  //END SERVO STUFF+++++++++++++++++++++++++++++++++++
  
  fncUdpSend();
  delay(50);
}

void fncUdpSend(){
  udpBuffer[0] = 11;

  //MOTOR STUFF+++++++++++++++++++++++++++++++++++++++
  udpBuffer[1] = forward+2;
  udpBuffer[2] = back+2;
  udpBuffer[3] = left+2;
  udpBuffer[4] = right+2;
  //END MOTOR STUFF+++++++++++++++++++++++++++++++++++
  
  //SERVO STUFF+++++++++++++++++++++++++++++++++++++++
  udpBuffer[5] = A + 2;
  udpBuffer[6] = L + 2;
  udpBuffer[7] = R + 2;
  //END SERVO STUFF+++++++++++++++++++++++++++++++++++
  
  udpServer.beginPacket(ipTarget, udpTargetPort);
  udpServer.printf("%s",udpBuffer);
  udpServer.endPacket();
  Serial.println("Packet Sent");
}
