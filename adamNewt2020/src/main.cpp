// Set Roboclaw to mode 7 option 4
// press mode, then use mode or set buttons until 7 blinks, use lipo to save
// press set, then use mode or set buttons until 4 blinks, use lipo to save

// Includes required to use Roboclaw library
#include <Arduino.h>
#include <RoboClaw.h>

RoboClaw roboclaw(&Serial1, 10000);  // pins 18 (TX1) and 19 (RX1) are Serial1.
                                     // Connect TX1 to S1(RX) and RX1 to S2(TX)

#define address 0x80  // this is the first roboclaw connected

void goFwd(int speed) {
  Serial.println("goFwd");
  roboclaw.ForwardM1(address, speed + 3);  // speed 0-127
  roboclaw.BackwardM2(address, speed);
}

void goBack(int speed) {
  roboclaw.ForwardM2(address, speed);  // speed 0-127
  roboclaw.BackwardM1(address, speed);
}

void turnRight(int speed) {
  roboclaw.ForwardM1(address, speed);  // speed 0-127
  roboclaw.ForwardM2(address, speed);
}

void turnLeft(int speed) {
  Serial.println("turnLeft");
  roboclaw.BackwardM1(address, speed);  // speed 0-127
  roboclaw.BackwardM2(address, speed);
}

void stop() {
  Serial.println("Stop");
  roboclaw.ForwardM1(address, 0);
  roboclaw.BackwardM2(address, 0);
}

void setup() {
  // Open roboclaw serial port
  roboclaw.begin(38400);
  Serial.begin(9600);

  stop();
  delay(1000);
  Serial.println("Starting...");

  goFwd(32);
  delay(13000);
  stop();
  delay(1000);

  turnLeft(50);
  delay(2100);
  stop();
  delay(1000);

  // goBack(32);
  // delay(2000);
  // stop();
}

void loop() {}