#include<Servo.h>

int pin = 10;
int pin2 = 11;
int pin3 = 6;
int power = 8;
int pwcontrol = 9;
int serialdata1;
int serialdata2;
int serialdata3;

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  Serial.begin(9600);
  
  servo1.attach(pin);
  servo2.attach(pin2);
  servo3.attach(pin3);
  
  pinMode(power, OUTPUT);
  pinMode(pwcontrol, OUTPUT);
  
  digitalWrite(pwcontrol, HIGH);
  analogWrite(power, 130);
  
  servo1.writeMicroseconds(1458);
  servo2.writeMicroseconds(1560);
  servo3.write(90);
}

void loop() {
  while (Serial.available()==0); //wait for signal
  serialdata1=(Serial.read());
  while (Serial.available()==0);
  serialdata1 = (Serial.read())+(serialdata1*100);
  while (Serial.available()==0);
  serialdata2=(Serial.read());
  while (Serial.available()==0);
  serialdata2 = (Serial.read())+(serialdata2*100);
  servo1.writeMicroseconds(serialdata1);
  servo2.writeMicroseconds(serialdata2);
  while (Serial.available()==0);
  serialdata3=(Serial.read());
  servo3.write(serialdata3);
}
