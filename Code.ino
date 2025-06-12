#include <Arduino.h>
#include <TM1637Display.h>
#define CLK 12
#define DIO 13
#define TEST_DELAY 2000 // delay in miliseconds

#define MOTOR_L 22
#define MOTOR_R 23

#define PUMP_A 18
#define PUMP_B 19

volatile int washTime = 0;
bool IRActivation = false;

const uint8_t SEG_DONE[] = {
 SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, // d
 SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O
 SEG_C | SEG_E | SEG_G, // n
 SEG_A | SEG_D | SEG_E | SEG_F | SEG_G }; // E

TM1637Display display(CLK, DIO);

uint8_t blank[] = {0x00, 0x00, 0x00, 0x00};

struct Sensor {
	const uint8_t PIN;
	bool activated;
};

Sensor IR = {4, false};
Sensor manualStart = {34, false}

void IRAM_ATTR isr(obj) {
	obj.activated = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Working");
  display.setBrightness(0x0f);
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);
  pinMode(PUMP_A, OUTPUT);
  pinMode(PUMP_B, OUTPUT);
  pinMode(OPT_IN, INPUT);

  attachInterrupt(digitalPinToInterrupt(IR.PIN), isr(IR), FALLING);
  attachInterrupt(digitalPinToInterrupt(manualStart.PIN), isr(manualStart), RISING);
}

void loop() {
  if (IR.activated && IRActivation == true) {
    startWash(washTime);
  }
    else if (manualStart.activated) {
      startWash(washTime);
    }
  


  delay(10);
}

void startWash(time) {
  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_R, 180);
  analogWrite(PUMP_A, 0);
  analogWrite(PUMP_B, 230);
}

void displayNumbers() {
  display.showNumberDec(0, false); // Expect: ___0
  delay(TEST_DELAY);
}































