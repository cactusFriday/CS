/*
    backup test
*/
#define sens_pin    2
#define enable_pin  11
#define step_pin    7
#define dir_pin     8
#define pump_pin    6
#define s_pin      10

//Параметры
#define extra       860 
#define delta       1510 // отладить
#define delta_less  1200
#define motor_speed 600
#define s_a         0          //initial servo angle
#define pump_pwr    130         //0-255
#define fltr_delay  150
#define sh_glass_dur 1000
#define glass_dur   10000
#define b_dur       10000

#include "DS3231.h"
#include <Servo.h>
#include <avr/eeprom.h>
#include "GyverStepper.h"

const uint8_t servAngle = 170;
String str = "";
unsigned long _timer = 0;
unsigned long cur_mil;
bool shortage = true;

struct Backup {
  uint16_t clean_h;
  uint16_t clean_m;
  uint16_t freq;
  uint16_t doClean;
  uint8_t doMatching;
  unsigned long match_timer;
  uint8_t deltaI;
  uint8_t bckwrdsI;
//  unsigned long dayI;
};

Backup backup;

GStepper< STEPPER2WIRE> stepper(400, step_pin, dir_pin, enable_pin);
//0,9 угловой шаг -> 400 шагов полный оборот

Servo s;

DS3231 rtc(SDA, SCL);
Time t;

void setup() {
  rtc.begin();
//    rtc.setTime(18,50,0); 
  Serial.begin(9600);
  delay(10);
  t = rtc.getTime();
  delay(20);
  Serial.println("Serial started");
  Serial.print(t.hour); Serial.print(":");
  Serial.println(t.min);
  //Инициализация режима работы пинов
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(enable_pin, OUTPUT);
  pinMode(sens_pin, INPUT);
  pinMode(pump_pin, OUTPUT);
  //Подключение серв
  s.attach(s_pin);
  s.write(s_a);
  //Настройка двигателя
  stepper.disable();

  digitalWrite(pump_pin, LOW);
  backup.doMatching = 1;
  eeprom_read_block((void*)&backup, 0, sizeof(backup));
  print_struct();
  delay(1000);
  if (backup.doClean == 1) {
    Serial.println("Respawn...");
    if (respawnS()) {
      delay(1000);
      Serial.println("Respawn has ran");
    }
    else Serial.println("Thaaaaat was an error!!");
    backup.deltaI = 0;
    backup.bckwrdsI = 0;
    eeprom_update_block((void*)&backup, 0, sizeof(backup));
  }
  detachServ();
}

void loop() {
  t = rtc.getTime();
  //Чтение из порта
  if (Serial.available() > 0) {
    delay(20);
    char c = Serial.read();
    str += c;
  }
  else {
    if (str != "") {
      parseStr(str);
      str = "";
    }
  }
  //Запуск главной функции работы системы
  if (backup.doClean == 1) {
    if (!initCleaning()) {
      Serial.println("Something gone wrong :(");
      respawnS();
    }
  }
  //Чистка по времени
  if (onFreqMatch(t) && backup.doMatching == 1) {
    if (t.hour == backup.clean_h) {
      if (t.min == backup.clean_m) {
        Serial.println("It's time to clean");
        backup.match_timer = millis();
        backup.doClean = 1;
        backup.doMatching = 0;
        eeprom_update_block((void*)&backup, 0, sizeof(backup));
      }
    }
  }
  if (millis() - backup.match_timer >= 70000 && backup.doMatching == 0) {
    backup.doMatching = 1;
  }
}


//Главная функция работы системы, запускающая последовательно процессы чистки
bool initCleaning() {
  stepper.reset();
  attachServ();
  stepper.reverse(true);
  for (uint8_t i = 0; i < 2; i++)
    if (!clean(2)) return false;
  if (!backwards(0)) return false;
  stepper.reverse(true);
  for (uint8_t i = 0; i < 2; i++)
    if (!clean(1)) return false;
  if (!backwards(0)) return false;
  backup.doClean = 0;
  backup.deltaI = 0;
  backup.bckwrdsI = 0;
  stepper.disable();
  detachServ();
  eeprom_update_block((void*)&backup, 0, sizeof(backup));
  return true;
}

//функция для прохода вправо с щеткой/насосом
bool clean(uint8_t mode) {
  shortage = true;
  stepper.enable();
  Serial.println("clean");
  if (mode == 2) {        //brush
    stepper.setRunMode(KEEP_SPEED);
    stepper.setSpeed(motor_speed);
    while (onEndstop()) {
      stepper.tick();
    }
    Serial.print("pre move_delta 1: "); Serial.println(stepper.getCurrent());
    turnPump();
    if (!moveDelta(delta))                        //Выезд на стекло дельтой
      return false;
    s.write(servAngle);
    
    cur_mil = millis();
    while (!onEndstop()) {
      if (millis() - cur_mil > sh_glass_dur)      //Проверка на преждевременное срабатывание концевика
        shortage = false;
      if (onOverloadStp(glass_dur)) {             //Пока едем по стеклу с щеткой проверяем на переполнение шагов
        Serial.println("stps overload");
        return false;
      }
      stepper.tick();
    }
    if (shortage) {                               //Преждевременное срабатывание концевика
      Serial.println("stps shortage");
      return false;
    }
    
    Serial.print("pre move_delta 2: "); Serial.println(stepper.getCurrent());
    digitalWrite(pump_pin, LOW);
    if (!moveDelta(delta_less - 50))              //Выезд после стекла на металл на дельту
      return false;
    s.write(s_a);
  }
  if (mode == 1) {        //brush
    stepper.setRunMode(KEEP_SPEED);
    stepper.setSpeed(motor_speed);
    while (onEndstop()) {
      stepper.tick();
    }
    Serial.print("pre move_delta 1: "); Serial.println(stepper.getCurrent());
    if (!moveDelta(delta))                        //Выезд на стекло дельтой
      return false;
    s.write(servAngle);
    
    cur_mil = millis();
    while (!onEndstop()) {
      if (millis() - cur_mil > sh_glass_dur)      //Проверка на преждевременное срабатывание концевика
        shortage = false;
      if (onOverloadStp(glass_dur)) {             //Пока едем по стеклу с щеткой проверяем на переполнение шагов
        Serial.println("stps overload");
        return false;
      }
      stepper.tick();
    }
    if (shortage) {                               //Преждевременное срабатывание концевика
      Serial.println("stps shortage");
      return false;
    }
    
    Serial.print("pre move_delta 2: "); Serial.println(stepper.getCurrent());
    if (!moveDelta(delta_less - 50))              //Выезд после стекла на металл на дельту
      return false;
    s.write(s_a);
  }
  return true;
}

//Функция для преодоления расстояния между концевиком и щеткой
bool moveDelta(long delt) {
  backup.deltaI++;
  eeprom_update_block((void*)&backup, 0, sizeof(backup));
  stepper.setRunMode(FOLLOW_POS);
  stepper.setAcceleration(0);
  stepper.setTarget(delt, RELATIVE);
  stepper.setSpeedDeg(motor_speed);
  cur_mil = millis();
  while (true) {
    stepper.tick();
    if (stepper.getTarget() - stepper.getCurrent() == 0)
      break;
  }
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeed(motor_speed);
  return true;
}

//Функция возвращения системы в точку "домой"       **при чистке
bool backwards(uint8_t param) {
  stepper.setCurrent(0);
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeed(motor_speed);
  stepper.reverse(false);
  Serial.println("backwards");
  uint8_t i = param;
  eeprom_update_block((void*)&backup, 0, sizeof(backup));
  bool doSens = true;
  while (true) {
    if (onEndstop() && doSens) {
      i++;
      backup.bckwrdsI = i;
      eeprom_update_block((void*)&backup, 0, sizeof(backup));
      doSens = false;
      Serial.print("metal "); Serial.print(i); Serial.print(" / position: "); Serial.println(stepper.getCurrent());
      cur_mil = millis();
    }
    else if (!onEndstop() && !doSens)
      doSens = true;
    if (i == 3)
      break;
    if (onOverloadStp(b_dur)) {             //Пока едем по стеклу с щеткой проверяем на переполнение шагов
      Serial.println("stps overload");
      return false;
    }
    stepper.tick();
  }
  Serial.println("BACKWARDS: EXTRA MOVE BACK");
  Serial.print("pre bckwrds extraMove: "); Serial.println(stepper.getCurrent());
  move_follow_pos(0, 0, extra);
  stepper.reset();
  return true;
}

//функция передвижения на trt расстояние при accel и speed (FOLLOW_POS, RELATIVE)
void move_follow_pos(uint16_t accel, uint16_t speed, uint16_t trgt) {
  stepper.setRunMode(FOLLOW_POS);
  stepper.setTarget(trgt, RELATIVE);
  stepper.setAcceleration(accel);
  stepper.setSpeedDeg(motor_speed);
  while (true) {
    stepper.tick();
    if (abs(stepper.getTarget() - stepper.getCurrent()) == 0) {
      Serial.println(stepper.getCurrent());
      break;
    }
  }
}

//Функция проверки состояния концевика
bool onEndstop() {
  if (digitalRead(sens_pin) == HIGH && (millis() - _timer >= fltr_delay) )
    return false;
  else if (digitalRead(sens_pin) == LOW) {
    _timer = millis();
    return true;
  }
}

//******************************************SERVO******************************************
//Функции работы с сервами
void attachServ() {
  s.attach(s_pin);
}
void detachServ() {
  s.detach();
}

//******************************************PARSING******************************************

void parseStr(String str) {
  if (int(str[0]) == 99)                // "c" clean
  {
    Serial.println("go clean");
    backup.doClean = 1;
    backup.deltaI = 0;
    backup.bckwrdsI = 0;
  }
  else {
    parseTime(str);
  }
}

void parseTime(String str) {
  uint16_t str_size = sizeof(str);
  Serial.print("Size "); Serial.println(str_size);
  Serial.println(str);

  uint16_t freq = int(str[str_size - 2]) - 48; // The last number in array

  uint16_t clean_h = (int(str[0]) - 48) * 10 + int(str[1]) - 48;
  uint16_t clean_m = (int(str[2]) - 48) * 10 + int(str[3]) - 48;

  //writing data in structure
  backup.clean_h = clean_h;
  backup.clean_m = clean_m;
  backup.freq = freq;
  backup.doMatching = 1;
  eeprom_update_block((void*)&backup, 0, sizeof(backup));
  Serial.println(clean_h);
  Serial.println(clean_m);
}

//******************************************MATCHING******************************************

bool onFreqMatch(Time t) {
  if (t.dow % backup.freq == 0) {
    return true;
  }
  else return false;
}

void turnPump () {
  for (byte i = 0; i < pump_pwr; i++) {
    analogWrite(pump_pin, i);
    delay(10);
  }
}

//******************************************BU_MOTOR******************************************

bool respawnS() {
  digitalWrite(pump_pin, LOW);
  s.write(0);
  if (backup.bckwrdsI == 3 && 
      backup.deltaI % 4 == 0 && 
      onEndstop() )
  {
    stepper.setRunMode(KEEP_SPEED);
    stepper.setSpeedDeg(motor_speed);
    stepper.setAcceleration(0);
    cur_mil = millis();
    while(onEndstop()) {
      if (onOverloadStp(b_dur)) {
        Serial.println("Crash3");
        stepper.disable();
        return false;
      }
      stepper.tick();
    }
    if (!backwards(2)) {
      Serial.println("Crash4");
      return false;
    }
  }
  else if (backup.bckwrdsI > 0)
    rst_dflt_pos(1);
  else if (backup.deltaI == 0 && backup.bckwrdsI == 0)
    Serial.print("its ok");
  else 
    rst_dflt_pos(2);

  backup.deltaI = 0;
  backup.bckwrdsI = 0;
  backup.doClean = 0;
  return true;
}

void rst_dflt_pos (uint8_t mode) {
  Serial.println("System crash. Default initiaizing");
  switch(mode){
    case 1:
    if (onEndstop()){
      Serial.print("Ты сейчас на металле и остановился во время езды назад. твой bckwrdsI был = ");Serial.println(backup.bckwrdsI);
      if (!backwards(backup.bckwrdsI - 1)) Serial.println("Crash0");
      stepper.disable();
    }
    else {
      Serial.print("Ты сейчас на стекле и остановился во время езды назад. твой bckwrdsI был = ");Serial.println(backup.bckwrdsI);
      if (!backwards(backup.bckwrdsI)) Serial.println("Crash1");
      stepper.disable();
    }
    break;
    case 2:
    Serial.print("Ты остановился во время чистки. твой deltaI был = ");Serial.println(backup.deltaI);
    Serial.print("Передаем в bckwrds() число "); Serial.println(restore_pos_by_delta(backup.deltaI));
    if (!backwards( restore_pos_by_delta(backup.deltaI) - 1 )) Serial.println("Crash2");
    stepper.disable();
    break;
  }
  eeprom_update_block((void*)&backup, 0, sizeof(backup));
}

uint8_t restore_pos_by_delta(uint8_t param) {
  uint8_t modulo = param % 2; 
  if (modulo) {
    uint8_t odd_num = param - 1;
    odd_num = odd_num / 2;
    uint8_t ret = (odd_num % 2) ? 2 : 3;
    return ret;
  }
  else {
    uint8_t ev_num = param / 2;
    uint8_t ret = (ev_num % 2) ? 2 : 1;
    return ret;
  }
}

bool onOverloadStp(unsigned long mil_to_ride) {
  if (millis() - cur_mil > mil_to_ride)
    return true;
  else return false;
}

void print_struct() {
  Serial.print("Часы для чистки: "); Serial.println(backup.clean_h);
  Serial.print("Минуты для чистки: "); Serial.println(backup.clean_m);
  Serial.print("Раз во сколько дней чистиь: "); Serial.println(backup.freq);
  Serial.print("Флаг на чистку: "); Serial.println(backup.doClean);
  Serial.print("Флаг на проверку времени для чистки: "); Serial.println(backup.doMatching);
  Serial.print("Итератор с дельт: "); Serial.println(backup.deltaI);
  Serial.print("Итератор с езды назад: "); Serial.println(backup.bckwrdsI);
  // Serial.print(""); Serial.println(backup.);
}

