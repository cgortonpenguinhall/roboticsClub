#include <Arduino.h>
#include <FastLED.h>

const int NUM_LEDS = 148;
const int DATA_PIN = 9;  // must be PWM, 9 on Pro Mini/Micro, 7 on Mega
const int potPin = A0;
const int BRIGHTNESS = 50;

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  pinMode(DATA_PIN, OUTPUT);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);  // use a low brightness to avoid huge power consumption
}

// void loop() {
//   int redled = 147;
//   for(int dot = 0; dot < NUM_LEDS; dot++){
//     redled--;
//     leds[redled] = CRGB::Red;

//     leds[dot] = CRGB::Blue;
//     FastLED.show();
//     delay(15);

//     leds[redled] = CRGB::Black;
//     FastLED.show();
//     delay(30);

//     leds[dot] = CRGB::Black;
//     FastLED.show();
//     delay(15);
//   }
//   for(int dot = NUM_LEDS; dot >= 0; dot--){
//     redled++;
//     leds[redled] = CRGB::Red;
    
//     leds[dot] = CRGB::Blue;
//     FastLED.show();
//     delay(15);

//     leds[redled] = CRGB::Black;
//     FastLED.show();
//     delay(30);

//     leds[dot] = CRGB::Black;
//     FastLED.show();
//     delay(15);
//     }
// }

void loop() {
  int num = analogRead(A0);
  num = map(num, 0, 1023, 0, 147);
  Serial.println(num);

  leds[num] = CRGB::Blue;
  FastLED.show();
}