#include <Wire.h>
#include <lp55231.h>

Lp55231 ledChip;

// LED channels for RGB (modify if your chip wiring differs)
const uint8_t RED_CH   = 6;
const uint8_t GREEN_CH = 0;
const uint8_t BLUE_CH  = 1;

// Frequency control
float freq = 1.0;
float prevFreq = 1.0;

unsigned long shiftInterval1;
unsigned long nextToggle1 = 0;
bool ledState1 = false;

// Convert HSV to RGB
void HSVtoRGB(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
  int i = int(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);
  float rF, gF, bF;

  switch (i % 6) {
    case 0: rF = v; gF = t; bF = p; break;
    case 1: rF = q; gF = v; bF = p; break;
    case 2: rF = p; gF = v; bF = t; break;
    case 3: rF = p; gF = q; bF = v; break;
    case 4: rF = t; gF = p; bF = v; break;
    case 5: rF = v; gF = p; bF = q; break;
  }

  r = (uint8_t)(rF * 255);
  g = (uint8_t)(gF * 255);
  b = (uint8_t)(bF * 255);
}

// Map frequency to RGB color
void frequencyToRGB(float freq, uint8_t &r, uint8_t &g, uint8_t &b) {
  float clampedFreq = constrain(freq, 1.0, 25.0);
  float hue = (clampedFreq - 1.0) / 24.0;  // hue from 0.0 to 1.0
  HSVtoRGB(hue, 1.0, 1.0, r, g, b);
}

void setFrequency(float freqHz) {
  if (freqHz <= 0) freqHz = 1.0;
  shiftInterval1 = (unsigned long)(500.0 / freqHz);  // 1000 / (2*f)
  Serial.print("Updated frequency to: ");
  Serial.print(freqHz);
  Serial.print(" Hz (interval = ");
  Serial.print(shiftInterval1);
  Serial.println(" ms)");
}

void setup() {
  Serial.begin(19200);
  delay(5000);  // Give OpenMV time to boot
  Serial.println("-- Setup Started --");

  ledChip.Begin();
  ledChip.Enable();

  setFrequency(freq);
  nextToggle1 = millis() + shiftInterval1;

  Serial.println("-- Setup Complete --");
}

void loop() {
  // Check for UART input
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    Serial.print("Received from GENX320: ");
    Serial.println(data);

    float incomingFreq = data.toFloat();
    if (incomingFreq > 0 && incomingFreq != prevFreq) {
      freq = incomingFreq;
      prevFreq = freq;
      setFrequency(freq);
      nextToggle1 = millis() + shiftInterval1;
    }
  }

  // Toggle LED with color encoding
  unsigned long currentMillis = millis();
  if (currentMillis >= nextToggle1) {
    ledState1 = !ledState1;

    uint8_t r, g, b;
    frequencyToRGB(freq, r, g, b);

    ledChip.SetChannelPWM(RED_CH,   ledState1 ? r : 0);
    ledChip.SetChannelPWM(GREEN_CH, ledState1 ? g : 0);
    ledChip.SetChannelPWM(BLUE_CH,  ledState1 ? b : 0);

    nextToggle1 += shiftInterval1;

    Serial.print("LED State: ");
    Serial.println(ledState1 ? "ON (colored)" : "OFF");
  }
}
