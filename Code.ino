#include <Arduino.h>
#include <TM1637Display.h>
#define CLK 12
#define DIO 13
#define DISPLAY_DELAY 2000 // delay in miliseconds

#define MIN_WASH_MS 30000UL
#define MAX_WASH_MS 120000UL

#define MOTOR_L 22
#define MOTOR_R 23

#define PUMP_A 18
#define PUMP_B 19

#define POT_PIN 32 

volatile long washTime = 0;
bool IRActivation = true;

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
Sensor sButton = {34, false};

void IRAM_ATTR isr_IR() {
	IR.activated = true;
}

void IRAM_ATTR isr_sButton() {
	sButton.activated = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Working");
  display.setBrightness(0x0f);

  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);
  pinMode(PUMP_A, OUTPUT);
  pinMode(PUMP_B, OUTPUT);

  pinMode(IR.PIN, INPUT);
  pinMode(sButton.PIN, INPUT);

  pinMode(POT_PIN, INPUT);
  analogSetWidth(12);  

  analogSetPinAttenuation(POT_PIN, ADC_11db);
  attachInterrupt(digitalPinToInterrupt(IR.PIN), isr_IR, FALLING);
  attachInterrupt(digitalPinToInterrupt(sButton.PIN), isr_sButton, RISING);
}

void startWash(long duration) {
  // turn on motor and pump
  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_R, 180);
  analogWrite(PUMP_A, 0);
  analogWrite(PUMP_B, 230);
  Serial.println("motor on");

  delay(duration);
  // turn off motor and pump
  analogWrite(MOTOR_R, 0);
  analogWrite(PUMP_B, 0);
  Serial.println("motor off");
  // reset button and ir sensor
  IR.activated = sButton.activated = false;

  // display "done"
  display.setSegments(SEG_DONE);
  delay(5000);
}

void loop() {
  int potVal = analogRead(POT_PIN);  
  washTime = map(potVal, 0, 4095, MIN_WASH_MS, MAX_WASH_MS);
  
  long seconds = (washTime + 500) / 1000;
  int minutes = seconds / 60;
  int secs    = seconds % 60;
  int dispVal = minutes * 100 + secs;
  display.showNumberDecEx(dispVal, 0x80 >> 1, true, 4, 0);

  if ((IR.activated && IRActivation) || sButton.activated) {
    startWash(washTime);
  }

  delay(10);
}


































