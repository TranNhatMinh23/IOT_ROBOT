#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h>
const byte TX = PA9;
const byte RX = PA10;
SoftwareSerial mySerial(RX, TX);

MPU6050 mpu(Wire);

#define bSpeed 130

const int ENA = PA7; // kết nối chân ENA với chân A7 STM32
const int INA1 = PA6; // kết nối chân IN1 với chân A6 STM32
const int INA2 = PA5; // kết nối chân IN2 với chân A5 STM32
const int INB1 = PA4; //kết nối chân ENA với chân A4 STM32
const int INB2 = PA3; // kết nối chân IN3 với chân A3 STM32
const int ENB = PA2; // kết nối chân IN4 với chân A2 STM32

const int HN = PA8; // kết nối chân OUT Hồng ngoại với chân A8 STM32

const int pingPin1 = PB10; // kết nối chân trig cảm biến vật cản với chân PB11 của STM
const int echoPin1 = PB11; // kết nối chân echo cảm biến vật cản với chân PB10 của STM
const int pingPin2 = PB12; // kết nối chân trig cảm biến có hàng với chân PB1 của STM32
const int echoPin2 = PB13; // kết nối chân echo cảm biến có hàng với chân PB0 của STM32

float Angle, angleQuay;
float Kp = 20;
float Ki = 0;
float Kd = 0;
long P = 0, I = 0, D = 0, PIDv = 0, pErr = 0, motorSpeedA, motorSpeedB;

bool nowHN = 0, lastHN = 0;
int c = 0, T = 0;
unsigned long timer = 0;
int a = 0;
bool demQuay, tram1 = 0, tram2 = 0, tram3 = 0, start = 0;
int coVatcan = 0, coHang = 0;

String myString;
char rdata;
String cdata;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(HN, INPUT);

  mySerial.begin(9600);

  Wire.begin();
  byte status = mpu.begin();
  while (status != 0) { } // stop everything if could not connect to MPU6050
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
}

void loop() {
  if (start == 0) {
    goStop();
    if (mySerial.available() > 0)
    {
      rdata = mySerial.read();
      myString = myString + rdata;
      if ( rdata == '\n')
      {
        String r_tram_1 = getValue(myString, ',', 0);
        String r_tram_2 = getValue(myString, ',', 1);
        String r_tram_3 = getValue(myString, ',', 2);
        String r_start = getValue(myString, ',', 3);

        tram1 = r_tram_1.toInt();
        tram2 = r_tram_2.toInt();
        tram3 = r_tram_3.toInt();
        start =  r_start.toInt();

        myString = "";
      };
    };
  }
  else {
    vatCan();
    if (coVatcan == 1) {
      goStop();
    } else {
      Auto();
    };
  };
}

void readAngle(void) {
  mpu.update();
  Angle = mpu.getAngleZ();
}

void readHN(void) {
  nowHN = digitalRead(HN);
  if (nowHN > lastHN) {
    T = T + 1;
    timer = millis();
    c = 0;
  };
  lastHN = nowHN;
}

void goLeft(int angleTurn) {
  demQuay = 1;
  if (demQuay == 1) {
    readAngle();
    angleQuay = Angle - angleTurn;
  };
  analogWrite(ENA, bSpeed + 20); //Left Motor Speed
  analogWrite(ENB, bSpeed + 30); //Right Motor Speed
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, LOW);
  while (true) {
    readAngle();
    if (Angle <= angleQuay) break;
  };
  demQuay = 0;
  a = a - angleTurn;
}

void goRight(int angleTurn) {
  demQuay = 1;
  if (demQuay == 1) {
    readAngle();
    angleQuay = Angle + angleTurn;
  };
  analogWrite(ENA, bSpeed + 30); //Left Motor Speed
  analogWrite(ENB, bSpeed + 20); //Right Motor Speed
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, HIGH);
  while (true) {
    readAngle();
    if (Angle >= angleQuay) break;
  };
  demQuay = 0;
  a = a + angleTurn;
}

void goStop(void) {
  analogWrite(ENA, 0);  //Left Motor Speed
  analogWrite(ENB, 0); //Right Motor Speed
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, LOW);
  readAngle();
}

void goPID(int angleSet) {
  readAngle();
  P = angleSet - Angle;
  I = P + pErr;
  D = P - pErr;
  PIDv = (Kp * P) + (Ki * I) + (Kd * D);
  pErr = P;

  motorSpeedA = bSpeed + PIDv;
  motorSpeedB = bSpeed - PIDv;
  motorSpeedA = constrain(motorSpeedA, 0, bSpeed + 30);
  motorSpeedB = constrain(motorSpeedB, 0, bSpeed + 30);

  analogWrite(ENA, motorSpeedA);  //Left Motor Speed
  analogWrite(ENB, motorSpeedB); //Right Motor Speed
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, LOW);
}
void goPIDLui(int angleSet) {
  readAngle();
  P = angleSet - Angle;
  I = P + pErr;
  D = P - pErr;
  PIDv = (Kp * P) + (Ki * I) + (Kd * D);
  pErr = P;

  motorSpeedA = bSpeed - PIDv;
  motorSpeedB = bSpeed + PIDv;
  motorSpeedA = constrain(motorSpeedA, 0, bSpeed + 30);
  motorSpeedB = constrain(motorSpeedB, 0, bSpeed + 30);

  analogWrite(ENA, motorSpeedA);  //Left Motor Speed
  analogWrite(ENB, motorSpeedB); //Right Motor Speed
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, HIGH);
}

void END1(void) {
  readHN();
  switch (T) {
    case 0:
      hang();
      if (coHang == 1) {
        goPID(a);
      } else {
        goStop();
      };
      break;
    case 1:
      goPID(a);
      break;
    case 2:
      goPID(a);
      break;
    case 3:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 4:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 5:
      if ((millis() - timer) <= 400) {
        goPID(a);
      }
      else if ((millis() - timer) <= 2000) {
        goStop();
      }
      else {
        goPIDLui(a);
      };
      break;
    case 6:
      goPIDLui(a);
      break;
    case 7:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 8:
      if ( c == 0) {
        goRight(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 9:
      goPID(a);
      break;
    case 10:
      hang();
      if (coHang == 1) {
        goStop();
      } else {
        goPID(a);
      };
      break;
    case 11:
      if ((millis() - timer) <= 400) {
        goPID(a);
      }
      else if ((millis() - timer) <= 2000) {
        goStop();
      }
      else {
        gozero();
        goStop();
        T = 0;
        start = 0;
      };
      break;
  }
}

void END2(void) {
  readHN();
  switch (T) {
    case 0:
      hang();
      if (coHang == 1) {
        goPID(a);
      } else {
        goStop();
      };
      break;
    case 1:
      goPID(a);
      break;
    case 2:
      goPID(a);
      break;
    case 3:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 4:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 5:
      goPID(a);
      break;
    case 6:
      if ((millis() - timer) <= 400) {
        goPID(a);
      }
      else if ((millis() - timer) <= 2000) {
        goStop();
      }
      else {
        goPIDLui(a);
      };
      break;
    case 7:
      goPIDLui(a);
      break;
    case 8:
      goPIDLui(a);
      break;
    case 9:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 10:
      if ( c == 0) {
        goRight(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 11:
      goPID(a);
      break;
    case 12:
      hang();
      if (coHang == 1) {
        goStop();
      } else {
        goPID(a);
      };
      break;
    case 13:
      if ((millis() - timer) <= 400) {
        goPID(a);
      }
      else if ((millis() - timer) <= 2000) {
        goStop();
      }
      else {
        gozero();
        goStop();
        T = 0;
        start = 0;
      };
      break;
  }
}

void END3(void) {
  readHN();
  switch (T) {
    case 0:
      hang();
      if (coHang == 1) {
        goPID(a);
      } else {
        goStop();
      };
      break;
    case 1:
      goPID(a);
      break;
    case 2:
      goPID(a);
      break;
    case 3:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 4:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 5:
      goPID(a);
      break;
    case 6:
      goPID(a);
      break;
    case 7:
      if ((millis() - timer) <= 400) {
        goPID(a);
      }
      else if ((millis() - timer) <= 2000) {
        goStop();
      }
      else {
        goPIDLui(a);
      };
      break;
    case 8:
      goPIDLui(a);
      break;
    case 9:
      goPIDLui(a);
      break;
    case 10:
      goPIDLui(a);
      break;
    case 11:
      if ( c == 0) {
        goLeft(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 12:
      if ( c == 0) {
        goRight(90);
        c = 1;
      } else {
        goPID(a);
      };
      break;
    case 13:
      goPID(a);
      break;
    case 14:
      hang();
      if (coHang == 1) {
        goStop();
      } else {
        goPID(a);
      };
      break;
    case 15:
      if ((millis() - timer) <= 400) {
        goPID(a);
      }
      else if ((millis() - timer) <= 2000) {
        goStop();
      }
      else {
        gozero();
        goStop();
        T = 0;
        start = 0;
      };
      break;
  }
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void vatCan(void) {
  long duration, distance;
  pinMode(pingPin1, OUTPUT);
  digitalWrite(pingPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin1, LOW);
  pinMode(echoPin1, INPUT);
  duration = pulseIn(echoPin1, HIGH, 4000);
  distance = microsecondsToCentimeters(duration);
  if ((distance > 0) && (distance < 40)) {
    coVatcan = 1;
  }
  else {
    coVatcan = 0;
  }
}

void hang(void) {
  long duration, distance;
  pinMode(pingPin2, OUTPUT);
  digitalWrite(pingPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin2, LOW);
  pinMode(echoPin2, INPUT);
  duration = pulseIn(echoPin2, HIGH, 4000);
  distance = microsecondsToCentimeters(duration);
  if ((distance > 0) && (distance <= 7)) {
    coHang = 1;
  }
  else {
    coHang = 0;
  }
}

void gozero(void) {
  analogWrite(ENA, bSpeed + 0); //Left Motor Speed
  analogWrite(ENB, bSpeed + 30); //Right Motor Speed
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, HIGH);
  while (true) {
    readAngle();
    if (Angle >= -40) break;
  };
  a = 0;
}

void Auto(void) {
  if ((tram1 == 1) && (tram2 == 0) && (tram3 == 0)) {
    END1();
  }
  else if ((tram1 == 0) && (tram2 == 1) && (tram3 == 0)) {
    END2();
  }
  else {
    END3();
  }
};

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
