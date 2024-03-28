#include <ModbusRTUSlave.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 3);  //5=rx 3 = tx

uint16_t holdingRegisters[30];
uint16_t inputRegisters[30];
bool coils[100];
bool discreteInputs[100];

int pinDir = 8, pinSpeed = 9, pinDir2 = 10, pinSpeed2 = 11, ledRed = 6, ledGreen = 7;
bool emerCmd, startCmd, stopCmd, dirCmd, resetCmd;
bool startCmd2, stopCmd2, dirCmd2;
bool readySts, alarmSts, runSts, dirSts;
bool runSts2, dirSts2;
uint16_t cmdMode, cmdSpeed, cmdAccel, cmdDecel, targetSpeed, alarmCode;
uint16_t cmdMode2, cmdSpeed2, cmdAccel2, cmdDecel2, targetSpeed2;
uint16_t mapSpeed, counting, PIDSpeed, motorSQN;
uint16_t mapSpeed2, counting2, PIDSpeed2, motorSQN2;
unsigned long oldMillis, interval, oldMillis2, interval2;
ModbusRTUSlave modbus(mySerial);  // serial port, driver enable pin for rs-485 (optional)

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinSpeed, OUTPUT);
  pinMode(pinDir, OUTPUT);
  pinMode(pinSpeed2, OUTPUT);
  pinMode(pinDir2, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  analogWrite(pinSpeed, 255);
  analogWrite(pinSpeed2, 255);

  modbus.configureCoils(coils, 100);                       // bool array of coil values, number of coils
  modbus.configureDiscreteInputs(discreteInputs, 100);     // bool array of discrete input values, number of discrete inputs
  modbus.configureHoldingRegisters(holdingRegisters, 30);  // unsigned 16 bit integer array of holding register values, number of holding registers
  modbus.configureInputRegisters(inputRegisters, 30);      // unsigned 16 bit integer array of input register values, number of input registers
  modbus.begin(5, 38400, SERIAL_8N2);                      // slave id, baud rate, config (optional)
  Serial.begin(9600);
  motorSQN = 30;
  digitalWrite(ledGreen, HIGH);
}

void accdec() {

  unsigned long nowMillis = millis();
  if (nowMillis - oldMillis >= interval) {

    if (targetSpeed > PIDSpeed) {
      interval = cmdAccel;
      PIDSpeed++;
    }

    if (targetSpeed < PIDSpeed) {
      interval = cmdDecel;
      PIDSpeed--;
    }
    oldMillis = nowMillis;
  }
}

void accdec2() {

  unsigned long nowMillis2 = millis();
  if (nowMillis2 - oldMillis2 >= interval2) {

    if (targetSpeed2 > PIDSpeed2) {
      interval2 = cmdAccel2;
      PIDSpeed2++;
    }

    if (targetSpeed2 < PIDSpeed2) {
      interval2 = cmdDecel2;
      PIDSpeed2--;
    }
    oldMillis2 = nowMillis2;
  }
}
void loop() {
  //Serial.println(alarmCode);
  // Register For motor 1 Driver
  emerCmd = bitRead(holdingRegisters[0], 0);
  startCmd = bitRead(holdingRegisters[0], 1);
  stopCmd = bitRead(holdingRegisters[0], 2);
  dirCmd = bitRead(holdingRegisters[0], 3);
  resetCmd = bitRead(holdingRegisters[0], 4);

  cmdMode = holdingRegisters[1];   //default 0 = I/O mode, 1 = Field bus mode,
  cmdSpeed = holdingRegisters[2];  //command speed (%) 0 - 1000
  cmdAccel = holdingRegisters[3];  //command Accel (ms)
  cmdDecel = holdingRegisters[4];  //command Decel (ms)
  holdingRegisters[9] = PIDSpeed;

  if (dirSts) {
    bitSet(holdingRegisters[8], 3);
  } else {
    bitClear(holdingRegisters[8], 3);
  }
  if (runSts) {
    bitSet(holdingRegisters[8], 2);
  } else {
    bitClear(holdingRegisters[8], 2);
  }

  if (readySts) {
    bitSet(holdingRegisters[8], 0);
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledRed, LOW);

  } else {
    bitClear(holdingRegisters[8], 0);
  }

  switch (motorSQN) {
    case 0:

      if (!alarmSts) {
        motorSQN = 10;
      }

      break;
    case 10:
      readySts = HIGH;
      if (startCmd) {
        dirSts = dirCmd;
        digitalWrite(pinDir, dirSts);  //Start direction pin
        motorSQN = 20;
      }
      break;
    case 20:
      targetSpeed = cmdSpeed;
      accdec();
      mapSpeed = map(PIDSpeed, 0, 100, 255, 0);
      analogWrite(pinSpeed, mapSpeed);  //Set Speed
      runSts = HIGH;
      if (!startCmd) {
        motorSQN = 30;
      }
      break;
    case 30:
      targetSpeed = 0;
      accdec();
      mapSpeed = map(PIDSpeed, 0, 100, 255, 0);
      analogWrite(pinSpeed, mapSpeed);  //Set Speed
      if (PIDSpeed <= 0) {
        runSts = LOW;
        motorSQN = 0;
      }
      break;
    case 40:
      analogWrite(pinSpeed, 255);
      runSts = LOW;
      motorSQN = 0;
      break;
  }

  // Register For motor 2 Driver
  startCmd2 = bitRead(holdingRegisters[0], 5);
  stopCmd2 = bitRead(holdingRegisters[0], 6);
  dirCmd2 = bitRead(holdingRegisters[0], 7);

  cmdSpeed2 = holdingRegisters[5];  //command speed (%) 0 - 1000
  cmdAccel2 = holdingRegisters[6];  //command Accel (ms)
  cmdDecel2 = holdingRegisters[7];  //command Decel (ms)

  if (dirSts2) {
    bitSet(holdingRegisters[8], 5);
  } else {
    bitClear(holdingRegisters[8], 5);
  }
  if (runSts2) {
    bitSet(holdingRegisters[8], 4);
  } else {
    bitClear(holdingRegisters[8], 4);
  }

  switch (motorSQN2) {
    case 0:

      if (!alarmSts) {
        motorSQN2 = 10;
      }

      break;
    case 10:
      readySts = HIGH;
      if (startCmd2) {
        dirSts2 = dirCmd2;
        digitalWrite(pinDir2, dirSts2);  //Start direction pin
        motorSQN2 = 20;
      }
      break;
    case 20:
      targetSpeed2 = cmdSpeed2;
      accdec2();
      mapSpeed2 = map(PIDSpeed2, 0, 100, 255, 0);
      analogWrite(pinSpeed2, mapSpeed2);  //Set Speed
      runSts2 = HIGH;
      if (!startCmd2) {
        motorSQN2 = 30;
      }

      break;
    case 30:
      targetSpeed2 = 0;
      accdec2();
      mapSpeed2 = map(PIDSpeed2, 0, 100, 255, 0);
      analogWrite(pinSpeed2, mapSpeed2);  //Set Speed
      if (PIDSpeed2 <= 0) {
        runSts2 = LOW;
        motorSQN2 = 0;
      }
      break;
    case 40:
      analogWrite(pinSpeed2, 255);
      runSts2 = LOW;
      motorSQN2 = 0;
      break;
  }
  holdingRegisters[10] = PIDSpeed2;
  /*Common register for controller*/
  holdingRegisters[11] = analogRead(A0);
  holdingRegisters[12] = analogRead(A1);
  holdingRegisters[13] = analogRead(A2);
  holdingRegisters[14] = analogRead(A3);
  holdingRegisters[15] = alarmCode;
  modbus.poll();
}