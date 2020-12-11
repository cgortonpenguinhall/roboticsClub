// Set Roboclaw to mode 7 option 4 (this is packet serial @ 38400 baud)
// To change mode: press mode, then use mode or set buttons until 7 blinks, use lipo to save
// To change option: press set, then use mode or set buttons until 4 blinks, use lipo to save

// Includes required to use Roboclaw library
#include <Arduino.h>
#include <RoboClaw.h>

RoboClaw roboclaw(&Serial1, 10000); // pins 18 (TX1) and 19 (RX1) are Serial1.
                                    // Connect TX1 to S1(RX) and RX1 to S2(TX)

#define address 0x80 // this is the first Roboclaw connected

// Velocity PID coefficients.
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

void waitFinish()
{
  uint8_t depth1; //buffer 1 reading
  uint8_t depth2; //buffer 2 reading

  while (depth1 != 0x80 && depth2 != 0x80)
  { //wait for buffers to be empty
    roboclaw.ReadBuffers(address, depth1, depth2);
    delay(100);
  } //end of waiting for drillhead position
}

void turnLeft()
{
  Serial.println("turnLeft");
  roboclaw.SpeedDistanceM2(address, 400, 993, 1);
}

void turnRight() 
{
  Serial.println("turnRight");
  roboclaw.SpeedDistanceM1(address,400,993,1);
}

    void moveForward()
{
  roboclaw.SpeedDistanceM1M2(address, 10000, 9000, 10000, 9000, 1);
}

void driveCircle() 
{
  roboclaw.SpeedDistanceM1(address,400,2500,1);
  roboclaw.SpeedDistanceM2(address, 1000, 2500,1);
}

void setup()
{
  Serial.begin(115200);
  roboclaw.begin(38400);

  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address, Kd, Kp, Ki, qpps);

  // moveForward();
  // waitFinish();
  // delay(500);
  // turnLeft();
  // waitFinish();
  // delay(500);
  // turnRight();
  driveCircle();
}

void displayspeed(void)
{
  uint8_t status1, status2, status3, status4;
  bool valid1, valid2, valid3, valid4;

  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  Serial.print("Encoder1:");
  if (valid1)
  {
    Serial.print(enc1, HEX);
    Serial.print(" ");
    Serial.print(status1, HEX);
    Serial.print(" ");
  }
  else
  {
    Serial.print("invalid ");
  }
  Serial.print("Encoder2:");
  if (valid2)
  {
    Serial.print(enc2, HEX);
    Serial.print(" ");
    Serial.print(status2, HEX);
    Serial.print(" ");
  }
  else
  {
    Serial.print("invalid ");
  }
  Serial.print("Speed1:");
  if (valid3)
  {
    Serial.print(speed1, HEX);
    Serial.print(" ");
  }
  else
  {
    Serial.print("invalid ");
  }
  Serial.print("Speed2:");
  if (valid4)
  {
    Serial.print(speed2, HEX);
    Serial.print(" ");
  }
  else
  {
    Serial.print("invalid ");
  }
  Serial.println();
}

void loop()
{
}
