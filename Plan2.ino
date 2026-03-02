#define Pin1 23
#define Pin2 25
#define PWM1 2

#define Pin3 27
#define Pin4 29
#define PWM2 3

//Flipper Arm Motor Pins

#define Pin5 31
#define Pin6 33
#define PWM3 4

#define Pin7 35
#define Pin8 37
#define PWM4 5

int FHomePos = 180;
int BHomePos = 180;

int Fflipper;
int Bflipper;

int Speed;
int Offset = 25;
int DefSpeed = 150;

int speedMode = 1;
int camMode = 1;
int camMode2 = 1;

int pos1 = 2855;
int pos2 = 0;

//Encoder Pins

#define EncoderF 0
#define EncoderB 1

//Appending Library

#include <SPI.h>
#include <Ethernet2.h>
#include <DynamixelMEGA_Shield.h>

//Robot Arm Stuff

const float L1 = 21; // Base to shoulder
const float L2 = 11; // Shoulder to elbow
const float L3 = 18; // Elbow to gripper

int joystickX;
int joystickY;

float targetX = 0;
float targetY = 0;
float targetZ = 0;

int baseAngle;
int armAngle;
int gripperAngle;

//Movement Speed Offset

int spdOffset = -10;

//W5500 Configuration Variables

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED   //MAC AddressA
};
IPAddress ip(192, 168, 1, 177);        //Custom IP address

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);              //Server port

void setup() {

  Dynamixel.begin(57142);  // Inicialize the servo at 57142 bps
  delay(1000);
//  Dynamixel.torqueStatus(1, 1);
//  Dynamixel.torqueStatus(2, 1);
//  Dynamixel.torqueStatus(3, 1);
//  Dynamixel.torqueStatus(4, 1);
//  Dynamixel.torqueStatus(5, 1);
//  Dynamixel.torqueStatus(6, 1);
  //Assign Motor Pins

  Serial.begin(115200);

  pinMode(Pin1, OUTPUT);
  pinMode(Pin2, OUTPUT);
  pinMode(Pin3, OUTPUT);
  pinMode(Pin4, OUTPUT);

  //Assign Flipper Motor Pins

  pinMode(Pin5, OUTPUT);
  pinMode(Pin6, OUTPUT);
  pinMode(Pin7, OUTPUT);
  pinMode(Pin8, OUTPUT);

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();

  Dynamixel.setAngleLimit(1, 1165, 2850);
  Dynamixel.speed(1, 25);
  Dynamixel.speed(2, 50);
}


void loop() {
  // listen for incoming clients
  EthernetClient client = server.available();   //Client Object variable

  if (client) {                       // If client is connected to server
    Dynamixel.moveSpeed(1, 2850, 25); 
    //Serial.println("new client");
    while (client.connected()) {      // Do loop while client is connected
      char command;
      if (client.available()) {       // Client send a packet
        command = client.read();      // Receive Command
        Serial.println(command);
        switch (command) {            // Switch to case according to command 
          case '0':
            Forward(175);
            break;
          case 'Q':
            Backward(175);
            break;
          case 'L':
            Left();
            break;
          case 'R':
            Right();
            break;
          case 'S':
            Stop();
            break;
          case 'D':
            if(pos1 <= 2855){
              Dynamixel.moveSpeed(1, pos1, 5);
              pos1 = pos1 + 1;
              delay(10);
              break;
            }
            else{
              pos1 = 2855;
            }
          case 'U':
            if(pos1 >= 1165){
              Dynamixel.moveSpeed(1, pos1, 5);
              pos1 = pos1 - 1;
              delay(5);
            }
            else{
              pos1 = 1165;
            }
            break;
          case 'Z':
            if(pos2 <= 1022){
              Dynamixel.moveSpeed(2, pos2, 50);
              pos2 = pos2 + 1;
              delay(5);
            }
            else{
              pos2 = 1023;
            }
            break;
          case 'C':
            if(pos2 >= 1){
              Dynamixel.moveSpeed(2, pos2, 50);
              pos2 = pos2 - 1;
              delay(10);
            }
            else{
              pos2 = 0;
            }            
            break;
          case 'Y':
            FFlipperUp();
            break;
          case 'X':
            BFlipperDown();
            break;
          case 'A' :
            FFlipperDown();
            break;
          case 'B' :
            BFlipperUp();
            break;
          case 'H':
            FlipperHome();
            break;
          case 's':
            FFlipperStop();
            BFlipperStop();
            break;
          case '-':
            Dynamixel.speed(1,0);
            Dynamixel.speed(2,0);
        }
      }
      client.println();
    }
    client.stop();
    //Serial.println("client disonnected");
    delay(10);
  }
}

void Right() {
  digitalWrite(Pin1, HIGH);
  digitalWrite(Pin2, LOW);
  analogWrite(PWM1, 150);
  digitalWrite(Pin3, HIGH);
  digitalWrite(Pin4, LOW);
  analogWrite(PWM2, 150);
}

void Left() {
  digitalWrite(Pin1, LOW);
  digitalWrite(Pin2, HIGH);
  analogWrite(PWM1, 150);
  digitalWrite(Pin3, LOW);
  digitalWrite(Pin4, HIGH);
  analogWrite(PWM2, 150);
  //Serial.println("Backward");
}

void Backward(int spd) {
  digitalWrite(Pin3, LOW);
  digitalWrite(Pin4, HIGH);
  analogWrite(PWM1, spd);
  digitalWrite(Pin1, HIGH);
  digitalWrite(Pin2, LOW);
  analogWrite(PWM2, spd);
  //Serial.println("Left");
}

void Forward(int spd) {

  digitalWrite(Pin3, HIGH);
  digitalWrite(Pin4, LOW);
  analogWrite(PWM1, spd);
  digitalWrite(Pin1, LOW);
  digitalWrite(Pin2, HIGH);
  analogWrite(PWM2, spd);
  //Serial.println("Right");
}

void FFlipperUp() {
  digitalWrite(Pin5, LOW);
  digitalWrite(Pin6, HIGH);
  analogWrite(PWM3, 175);
}

void FFlipperDown() {
  digitalWrite(Pin5, HIGH);
  digitalWrite(Pin6, LOW);
  analogWrite(PWM3, 175);
}

void BFlipperUp() {
  digitalWrite(Pin7, LOW);
  digitalWrite(Pin8, HIGH);
  analogWrite(PWM4, 175);
}

void BFlipperDown() {
  digitalWrite(Pin7, HIGH);
  digitalWrite(Pin8, LOW);
  analogWrite(PWM4, 175);
}

void Stop() {
  digitalWrite(Pin1, LOW);
  digitalWrite(Pin2, LOW);
  digitalWrite(Pin3, LOW);
  digitalWrite(Pin4, LOW);
}

void FFlipperStop() {
  digitalWrite(Pin5, LOW);
  digitalWrite(Pin6, LOW);
}

void BFlipperStop() {
  digitalWrite(Pin7, LOW);
  digitalWrite(Pin8, LOW);
}

void servoUp() {
  if (joystickY < 128) {
    joystickY += 1;
    //Serial.println(joystickY);
  }
}

void servoDown() {
  if (joystickY > 0) {
    joystickY -= 1;
    //Serial.println(joystickY);
  }
}

void servoRight() {
  if (joystickX < 128) {
    joystickX += 1;
    //Serial.println(joystickX);
  }
}

void servoLeft() {
  if (joystickX > 0) {
    joystickX -= 1;
    //Serial.println(joystickX);
  }
}

void FlipperHome() {
  int FendcVal = analogRead(EncoderF);
  int BendcVal = analogRead(EncoderB);
  int FAngle = map(FendcVal, 0, 1023, 0, 360);

  //Home Front Flipper

  while (FAngle != FHomePos) {
    // Move motor in the homing direction until encoder reads 0
    FFlipperUp();

    FendcVal = analogRead(EncoderF);
    FAngle = map(FendcVal, 0, 1023, 0, 360); // Update encoder value
  }
  // Stop motor after homing
  FFlipperStop();
  BFlipperStop();

  int BAngle = map(BendcVal, 0, 1023, 0, 360);
  //Home Back Flipper
  while (BAngle != FHomePos) {
    BFlipperUp();

    BendcVal = analogRead(EncoderB);
    BAngle = map(BendcVal, 0, 1023, 0, 360);  // Update encoder value
  }
  FFlipperStop();
  BFlipperStop();
}
