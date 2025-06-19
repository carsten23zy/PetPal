#include <Arduino.h>
#include <TM1637Display.h>

#define CLK 12
#define DIO 13

#define MIN_WASH_MS 30000UL
#define MAX_WASH_MS 120000UL

#define MOTOR_L 22
#define MOTOR_R 23

#define PUMP_A 19
#define PUMP_B 21

#define POT_PIN 32 

bool startButtonPressed = false;

bool IRenabled = false;
bool irToggled = false;
bool IRactivated = false;
bool irToggleRequested = false;

unsigned long lastToggleTime = 0;

const uint8_t SEG_DONE[] = {
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,         // d
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O
    SEG_C | SEG_E | SEG_G,                         // n
    SEG_A | SEG_D | SEG_E | SEG_F | SEG_G          // E
};

TM1637Display display(CLK, DIO);

void IRAM_ATTR onIR() {
  IRactivated = true;
}
void IRAM_ATTR onStartButton() {
  startButtonPressed = true;
}
void IRAM_ATTR onToggleIR() {
  irToggleRequested = true;
}

void setup() {
  Serial.begin(115200);
  display.setBrightness(0x0f);

  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);
  pinMode(PUMP_A, OUTPUT);
  pinMode(PUMP_B, OUTPUT);

  pinMode(33, INPUT); attachInterrupt(digitalPinToInterrupt(33),  onIR,         FALLING);
  pinMode(34, INPUT); attachInterrupt(digitalPinToInterrupt(34), onStartButton, RISING);
  pinMode(35, INPUT); attachInterrupt(digitalPinToInterrupt(35), onToggleIR,    RISING);

  analogSetWidth(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db);
  pinMode(POT_PIN, INPUT);
}

void startWash(long durationMs, int prog) {
  int motorPWM = map(prog, 1, 9, 90, 180);

  // turn on motor & pump
  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_R, motorPWM);
  analogWrite(PUMP_A, 0);
  analogWrite(PUMP_B, 230);
  Serial.println("Wash started");

  unsigned long start = millis(), now;
  while ((now = millis()) - start < (unsigned long)durationMs) {
    unsigned long remaining = durationMs - (now - start);
    // convert to whole seconds
    long remSec = (remaining + 500) / 1000;
    int m = remSec / 60;
    int s = remSec % 60;
    int dispVal = m * 100 + s; 
    // show MM:SS
    display.showNumberDecEx(dispVal, 0x80 >> 1, true, 4, 0);
    delay(200);
  }

  // turn off
  analogWrite(MOTOR_R, 0);
  analogWrite(PUMP_B, 0);
  Serial.println("Wash complete");

  // clear triggers
  IRactivated = startButtonPressed = false;

  // DONE message
  display.setSegments(SEG_DONE);
  delay(3000);
  display.clear();
}

void loop() {
  int pot = analogRead(POT_PIN);
  int prog = map(pot, 0, 4095, 1, 9);

  if (irToggleRequested) {
    irToggleRequested = false;

    unsigned long now = millis();
    if (now - lastToggleTime > 2000) {
      IRenabled = !IRenabled;
      irToggled = true;
      IRactivated = false;
      lastToggleTime = now;
    }
  }

  if (irToggled) {
    irToggled = false;
    IRactivated = false; 

    if (IRenabled) {
      // Show "IrOn"
      uint8_t irOn[4] = {
        SEG_E | SEG_F,                                  // I
        SEG_E | SEG_G,                                  // r
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,  // O
        SEG_C | SEG_E | SEG_G                           // n
      };
      display.clear();
      display.setSegments(irOn);
    } else {
      // Show "IrOF"
      uint8_t irOf[4] = {
        SEG_E | SEG_F,                                  // I
        SEG_E | SEG_G,                                  // r
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,  // O
        SEG_A | SEG_E | SEG_F | SEG_G                   // F
      };
      display.clear();
      display.setSegments(irOf);
    }

    delay(1000);
  }

  // build "P‑‑n" segments
  uint8_t segs[4];
  segs[0] = SEG_A | SEG_B | SEG_E | SEG_F | SEG_G; // 'P'
  segs[1] = SEG_G;                                // '-'
  segs[2] = SEG_G;                                // '-'
  segs[3] = display.encodeDigit(prog);            // '1'..'9'
  display.setSegments(segs);

  // trigger
  if ((IRactivated && IRenabled) || startButtonPressed) {
    long washDur = map(prog, 1, 9, MIN_WASH_MS, MAX_WASH_MS);
    startWash(washDur, prog);
  }

  delay(50);
}
