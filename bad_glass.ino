#define motor_speed 600
#define servo_speed 5
#define s_a         0          //initial servo angle


#define enable_pin  11
#define step_pin    7
#define dir_pin     8
#define s_pin      10
#define led         13

#include "DS3231.h"
#include <Servo.h>
#include <avr/eeprom.h>
#include "GyverStepper.h"


int32_t motor_sp = 600;
uint8_t servAngle = 155;
String str = "";
String str2 = "";
unsigned long _timer = 0;

struct Backup {
  String msg;
  char msgC[10];
};

Backup backup;

GStepper< STEPPER2WIRE> stepper(400, step_pin, dir_pin, enable_pin);

Servo s;

void setup() {
  Serial.begin(9600);
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(enable_pin, OUTPUT);
  pinMode(led, OUTPUT);
//  for (int i = 0; i < 5; i++){
//    digitalWrite(led, HIGH);
//    delay(500);
//    digitalWrite(led, LOW);
//    delay(500);
//  }

  //Подключение серв
  s.attach(s_pin);
  //Настройка двигателя
  stepper.disable();
  s.write(s_a);
  Serial.println("reading...");
  eeprom_read_block((void*)&backup, 0, sizeof(backup));
  Serial.println("Message string: " + backup.msg);
  for(int i = 0; i < 6; i++) {
    str2 += backup.msgC[i];
  }
  Serial.println("Message string: " + str2);
}

void loop() {
  //Чтение из порта
  if (Serial.available() > 0) {
    delay(20);
    char c = Serial.read();
    str += c;
    digitalWrite(led, HIGH);
    delay(300);
    digitalWrite(led, LOW);
    delay(300);
  }
  else if (str != ""){
    Serial.println(str);
    int siz = sizeof(str);
    for(int i = 0; i < siz + 1; i++) {
      backup.msgC[i] = str[i];
    }
    for(int i = 0; i < siz + 1; i++){
      digitalWrite(led, HIGH);
      delay(500);
      digitalWrite(led, LOW);
      delay(500); 
    }
    backup.msg = str;
    eeprom_update_block((void*)&backup, 0, sizeof(backup));
//    if (int(str[0]) == 98) { //b
//      stepper.enable();
//      stepper.setRunMode(KEEP_SPEED);
//      stepper.setSpeed(motor_speed);
//      stepper.reverse(true);
//      str = "";
//      Serial.println("backwards");
//      while (true) {
//        stepper.tick();
//        if (Serial.available() > 0) {break;}
//      }
//    }
//    else if (int(str[0]) == 102) { //f
//      stepper.enable();
//      stepper.setRunMode(KEEP_SPEED);
//      stepper.setSpeed(motor_speed);
//      stepper.reverse(false);
//      Serial.println("forward");
//      Serial.println(stepper.getCurrent());
//      str = "";
//      _timer = millis();
//      while (true) {
//        stepper.tick();
//        if (Serial.available() > 0) {break;}
//      }
//    }
//    else if (int(str[0]) == 115) { //s
//      Serial.println("stop");
//      Serial.print("time: ");
//      Serial.println(millis() - _timer);
//      Serial.println(stepper.getCurrent());
//      stepper.reset();
//      stepper.disable();
//      str = "";
//    }
//    else {
//      Serial.println(str);
//      Serial.println(str.toInt());
//      s.write(str.toInt());
//      motor_sp = str.toInt();
//      stepper.setSpeed(motor_sp);
      str = "";
//    }
  }
}

