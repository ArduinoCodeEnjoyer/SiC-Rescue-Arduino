#include <Servo.h>
#include <math.h>
#include <Ethernet.h>
#include <SPI.h>
#include <Ramp.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
char row[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X'};
int col[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};

unsigned long previousMillisT = 0;
const long intervalT = 2; // Interval between iterations in milliseconds

int v = 0;
int j = 0;

int Speed = 150;

const unsigned long interval = 500; // 1 second

// Variable to store the last time the message was printed
unsigned long previousMillis = 0;

float interpolated[24][24];

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED   //MAC Address
};

int pos1 = 90;
int pos2 = 90;
float rlPos = 86;
float udPos = 90;
float SPos;
float EPos;
float RGPos;
float LGPos;
float TPos;
int duration = 100;

int spdf = 180;
int spdb = 0;

bool shiftGear = false;
bool EnableThermal = false;

IPAddress ip(192, 168, 1, 177);        //Custom IP address
//IPAddress espCamIP(192, 168, 89, 11); // IP address of the server
EthernetServer server(80);             //Server port

Servo FFlipper;
Servo BFlipper;

Servo LeftMotor;
Servo RightMotor;
Servo rlWrist;
Servo udWrist;
Servo tWrist;
Servo LFinger;
Servo RFinger;

Servo shoulder;
Servo elbow;

// Arm lengths
const float L1 = 202;
const float L2 = 159;

int i = 0;

String stringCommand = "";
String commandTemp = "";

float x_d = 200;
float y_d = 100;
float target_x = 250;
float target_y = 150;
float step_size = 1.0;

float XA;
float YA;
float ZA;

int factor = 20;

bool notInit = false;
bool status;

void setup() {
  Serial.begin(115200);

  Serial.println("Initializing");

  pinMode(LED_BUILTIN, OUTPUT);

  LeftMotor.attach(3);
  RightMotor.attach(4);

  FFlipper.attach(A0);
  BFlipper.attach(A1);

  FFlipHome();
  BFlipHome();

  //Wire.begin();

  delay(100);
  //  while (sensor.wakeup() == false)
  //  {
  //    Serial.print(millis());
  //    Serial.println("\tCould not connect to GY521");
  //    delay(1000);
  //  }
  //  sensor.setAccelSensitivity(2);  //  8g
  //  sensor.setGyroSensitivity(1);   //  500 degrees/s
  //
  //  sensor.setThrottle();
  //  Serial.println("start...");

  shoulder.attach(A4);
  elbow.attach(A3);
  rlWrist.attach(A8);     //LeftRightWrist
  udWrist.attach(8);    //UpDownWrist
  tWrist.attach(A7);    //TurnWrist
  LFinger.attach(6);    //right
  RFinger.attach(2);    //left
  //homeUD();
  homeServos();
  EthernetInit();
}

void loop() {
  EthernetClient client = server.available();
  if (client) {
    if (client.connected()) {
      client.print("F:" + String(pos1));
      client.print("B:" + String(pos2));
    }
    while (client.connected()) {
      char command;
      unsigned long currentMillis = millis();
      if (client.available()) {
        command = client.read();
        stringCommand += command;
      }
      else if (stringCommand.endsWith("\n\r")) {
        // Process the command
        stringCommand = removeUnwantedChars(stringCommand, "\n\r");
        //Serial.println(stringCommand);
        commandTemp = stringCommand;
        stringCommand = "";
      }
      unsigned long currentMillisT = millis();
      if (EnableThermal == true) {
        if (notInit == false) {
          status = amg.begin();
          if (!status) {
            Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
            while (1);
          }
          notInit = true;
        }

        amg.readPixels(pixels);

        bilinearInterpolation(pixels, interpolated, 8, 8, 24, 24);

        if (currentMillisT - previousMillisT >= intervalT) {
          // Save the last time you sent the data
          previousMillisT = currentMillisT;

          // Combine the strings
          String combineString = String(row[j]) + String(col[i]) + ":" + String(interpolated[i][j]);
          client.println(combineString);

          // Move to the next element
          j++;
          if (j >= 24) {
            j = 0;
            i++;
            if (i >= 24) {
              i = 0;
              EnableThermal = false;
            }
          }
        }
      }

      else if (commandTemp == "W") {
        Forward();
      }
      else if (commandTemp == "A") {
        Left();
      }
      else if (commandTemp == "S") {
        Backward();
      }
      else if (commandTemp == "D") {
        Right();
      }
      else if (commandTemp == "Space") {
        FFlipHome();
        BFlipHome();
        homeServos();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }
      else if (commandTemp == "C") {
        FFlipHome();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }
      else if (commandTemp == "V") {
        BFlipHome();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }
      else if (commandTemp == "U") {
        if (pos1 >= 10) {
          pos1 = pos1 - 1;
          FFlipper.write(pos1);
          String string = "F:" + String(pos1);
          client.print(string);
          delay(5);
        }
        else {
          pos1 = 10;
        }
      }
      else if (commandTemp == "J") {
        if (pos1 <= 160) {
          pos1 = pos1 + 1;
          FFlipper.write(pos1);
          String string = "F:" + String(pos1);
          client.print(string);
          delay(5);
        }
        else {
          pos1 = 160;
        }
      }

      else if (commandTemp == "I") {
        if (pos2 >= 10) {
          pos2 = pos2 - 1;
          BFlipper.write(pos2);
          String string = "B:" + String(pos2);
          client.print(string);
          delay(5);
        }
        else {
          pos2 = 10;
        }
      }
      else if (commandTemp == "K") {
        if (pos2 <= 160) {
          pos2 = pos2 + 1;
          BFlipper.write(pos2);
          String string = "B:" + String(pos2);
          client.print(string);
          delay(5);
        }
        else {
          pos1 = 160;
        }
      }

      else if (commandTemp == "F") {
        FFlipStretch();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }
      else if (commandTemp == "G") {
        BFlipStretch();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }

      else if (commandTemp == "Z") {
        FFlipDog();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }
      else if (commandTemp == "X") {
        BFlipDog();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }
      else if (commandTemp == "R") {
        FFlipZ();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }
      else if (commandTemp == "T") {
        BFlipZ();
        client.print("F:" + String(pos1));
        client.print("B:" + String(pos2));
        commandTemp = "";
      }

      else if (commandTemp == "Up") {
        if (SPos <= 95) {
          SPos = SPos + 1;
          shoulder.write(SPos);
          delay(15);
        }
        else {
          SPos = 95;
        }
      }
      else if (commandTemp == "Down") {
        if (SPos >= 15) {
          SPos = SPos - 1;
          shoulder.write(SPos);
          delay(15);
        }
        else {
          SPos = 15;
        }
      }

      else if (commandTemp == "NumPad7") {
        shoulderU();
      }

      else if (commandTemp == "NumPad1") {
        shoulderD();
      }

      else if (commandTemp == "NumPad8") {
        elbowU();
      }

      else if (commandTemp == "NumPad2") {
        elbowD();
      }

      else if (commandTemp == "NumPad9") {
        wristU();
      }

      else if (commandTemp == "NumPad3") {
        wristD();
      }
      else if (commandTemp == "NumPad4") {
        wristL();
      }

      else if (commandTemp == "NumPad6") {
        wristR();
      }
      else if (commandTemp == "NumPad0") {
        homeServos();
      }
      else if (commandTemp == "Left") {
        wristL();
      }
      else if (commandTemp == "Right") {
        wristR();
      }
      else if (commandTemp == "VolumeMute") {
        if (shiftGear == false) {
          spdf = 135;
          spdb = 45;
          shiftGear = true;
          //          client.print("S:Speed: 1");
          commandTemp = "";
        }
        else {
          spdf = 180;
          spdb = 0;
          shiftGear = false;
          //          client.print("S:Speed: 2");
          commandTemp = "";
        }
      }
      else if (commandTemp == "Control") {
        if (EnableThermal == false) {
          EnableThermal = true;
        }
        commandTemp = "";
      }
      else if (commandTemp == "Add") {
        GripperGrab();
      }
      else if (commandTemp == "Subtract") {
        GripperRelease();
      }
      else if (commandTemp == "Decimal") {
        GripperStick();
        commandTemp = "";
      }
      else if (commandTemp == "Divide") {
        WristRotateCW();
      }
      else if (commandTemp == "Multiply") {
        WristRotateCCW();
      }
      else if (commandTemp == "KeyUp") {
        MStop();
        commandTemp = "";
      }
    }
    client.stop();
  }
  EnableThermal = false;
  MStop();
}

void bilinearInterpolation(float *src, float dest[24][24], int srcRows, int srcCols, int destRows, int destCols) {
  float x_ratio = float(srcCols - 1) / (destCols - 1);
  float y_ratio = float(srcRows - 1) / (destRows - 1);

  for (int i = 0; i < destRows; i++) {
    for (int j = 0; j < destCols; j++) {
      int x = int(x_ratio * j);
      int y = int(y_ratio * i);
      float x_diff = (x_ratio * j) - x;
      float y_diff = (y_ratio * i) - y;

      float a = src[y * srcCols + x];
      float b = src[y * srcCols + (x + 1)];
      float c = src[(y + 1) * srcCols + x];
      float d = src[(y + 1) * srcCols + (x + 1)];

      dest[i][j] = (a * (1 - x_diff) * (1 - y_diff)) +
                   (b * x_diff * (1 - y_diff)) +
                   (c * y_diff * (1 - x_diff)) +
                   (d * x_diff * y_diff);
    }
  }
}

void Backward() {
  LeftMotor.write(spdb);
  RightMotor.write(spdf);
}

void Forward() {
  LeftMotor.write(spdf);
  RightMotor.write(spdb);
}

void Right() {
  LeftMotor.write((spdf > 135) ? spdf - factor : spdf);
  RightMotor.write((spdf > 135) ? spdf - factor : spdf);
}

void Left() {
  RightMotor.write((spdb < 0) ? spdb + factor : spdb);
  LeftMotor.write((spdb < 0) ? spdb + factor : spdb);
}

void MStop() {
  LeftMotor.write(90);
  RightMotor.write(90);
}

String removeUnwantedChars(String input, String charsToRemove) {
  String result = "";
  for (int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    if (charsToRemove.indexOf(c) == -1) { // If the character is not in the charsToRemove list
      result += c; // Append it to the result
    }
  }
  return result;
}

void homeServos() {
  shoulder.write(15);   //1            Shoulder
  elbow.write(162);  //2           Elbow
  rlWrist.write(85);   //3หันซ้ายขวา    LeftRightWrist
  udWrist.write(89);   //4หมุนมือขึ้นลง  UpDownWrist
  tWrist.write(90);   //5หมุนมือ       TurnWrist
  LFinger.write(110);  //6มือ     Left
  RFinger.write(60);    //7มือ       Right

  SPos = shoulder.read();
  EPos = elbow.read();
  udPos = udWrist.read();
  rlPos = rlWrist.read();
  TPos = tWrist.read();
  RGPos = RFinger.read();
  LGPos = LFinger.read();
}

void shoulderU() {
  if (SPos <= 95) {
    SPos = SPos + 0.1;
    shoulder.write(SPos);
    delay(5);
  }
  else {
    SPos = 95;
  }
}

void shoulderD() {
  if (SPos >= 15) {
    SPos = SPos - 0.1;
    shoulder.write(SPos);
    delay(5);
  }
  else {
    SPos = 15;
  }
}

void elbowU() {
  if (EPos >= 85) {
    EPos = EPos - 0.1;
    elbow.write(EPos);
    delay(5);
  }
  else {
    EPos = 85;
  }
}

void elbowD() {
  if (EPos <= 162) {
    EPos = EPos + 0.1;
    elbow.write(EPos);
    delay(5);
  }
  else {
    EPos = 162;
  }
}

void wristR() {
  if (rlPos >= 50) {
    rlPos = rlPos - 0.1;
    rlWrist.write(rlPos);
    delay(10);
  }
  else {
    rlPos = 50;
  }
}

void wristL() {
  if (rlPos <= 116) {
    rlPos = rlPos + 0.1;
    rlWrist.write(rlPos);
    delay(10);
  }
  else {
    rlPos = 116;
  }
}

void wristU() {
  if (rlPos >= 55) {
    udPos = udPos - 0.1;
    udWrist.write(udPos);
    delay(10);
  }
  else {
    udPos = 55;
  }
}

void wristD() {
  if (rlPos <= 130) {
    udPos = udPos + 0.1;
    udWrist.write(udPos);
    delay(10);
  }
  else {
    udPos = 130;
  }
}

void FFlipHome() {
  FFlipper.write(10);
  pos1 = FFlipper.read();
}

void BFlipHome() {
  BFlipper.write(10);
  pos2 = BFlipper.read();
}

void FFlipDog() {
  FFlipper.write(150);
  pos1 = FFlipper.read();
}

void BFlipDog() {
  BFlipper.write(150);
  pos2 = BFlipper.read();
}

void FFlipStretch() {
  FFlipper.write(85);
  pos1 = FFlipper.read();
}

void BFlipStretch() {
  BFlipper.write(85);
  pos2 = BFlipper.read();
}

void FFlipZ() {
  FFlipper.write(45);
  pos1 = FFlipper.read();
  BFlipper.write(115);
  pos2 = BFlipper.read();
}

void BFlipZ() {
  FFlipper.write(115);
  pos1 = FFlipper.read();
  BFlipper.write(45);
  pos2 = BFlipper.read();
}

void GripperGrab() {
  if (LGPos <= 180) {
    LGPos = LGPos + 1;
    LFinger.write(LGPos);
    LGPos = LFinger.read();
    if (RGPos >= 0) {
      RGPos = RGPos - 1;
      RFinger.write(RGPos);
      RGPos = RFinger.read();
    }
    else {
      RGPos = 0;
    }
    delay(15);
  }
  else {
    LGPos = 180;
  }
}

void GripperRelease() {
  if (LGPos >= 110) {
    LGPos = LGPos - 1;
    LFinger.write(LGPos);
    LGPos = LFinger.read();
    if (RGPos <= 60) {
      RGPos = RGPos + 1;
      RFinger.write(RGPos);
      RGPos = RFinger.read();
    }
    else {
      RGPos = 60;
    }
    delay(15);
  }
  else {
    LGPos = 110;
  }
}

void GripperStick() {
  for (LGPos; LGPos <= 180; LGPos++) {
    LFinger.write(LGPos);
    LGPos = LFinger.read();
    delay(15);
  }
  for (RGPos; RGPos <= 60; RGPos++) {
    RFinger.write(RGPos);
    RGPos = RFinger.read();
    delay(15);
  }
}

void WristRotateCW() {
  if (TPos <= 180) {
    tWrist.write(TPos);
    TPos = TPos + 0.1;
    delay(10);
  }
  else {
    TPos = 180;
  }
  TPos = tWrist.read();
}

void WristRotateCCW() {
  if (TPos >= 0) {
    tWrist.write(TPos);
    TPos = TPos - 0.1;
    delay(10);
  }
  else {
    TPos = 0;
  }
  TPos = tWrist.read();
}
