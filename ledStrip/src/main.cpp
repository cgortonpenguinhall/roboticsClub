#include <Arduino.h>
#include<FastLED.h>

const int NUM_LEDS = 148;
const int DATA_PIN = 9;  // must be PWM, 9 on Pro Mini/Micro, 7 on Mega
const int potPin = A0;
const int BRIGHTNESS = 50;

CRGB leds[NUM_LEDS];

void dazzleMe() {

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Red;
  }

  FastLED.show();
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  pinMode(DATA_PIN, OUTPUT);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);  // use a low brightness to avoid huge power consumption

  dazzleMe();

}

void loop() {}