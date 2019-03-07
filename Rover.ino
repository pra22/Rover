// Main code for the rover
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>           // For Transceiver
#include <Wire.h>              // For accelerometer/gyro
#include <ServoTimer2.h>       // For servo control. The regular servo library disables PWM on pins 9 and 10.
// This one uses a different timer register that doesn't interfere with those pins.

// *** Pin definitions and constants for RF95 radio ***
#define RFM95_CS 4
#define RFM95_RST 14
#define RFM95_INT 2 
#define RF95_FREQ 434.0

// *** Ultrasonic sensor stuff ***
#define TRIG_PIN  15
#define ECHO_PIN  16
bool Clockwise[2];             // Arrays to store whether or not there are obstacles in the two positions at which measurements are taken during each sweep
bool counterClockwise[2];      // These become 1 if there is an obstacle
bool obstaclePresent[2];       // Obstacles are registered if they show up in both clockwise and counterclockwise sweeps in the same position      
#define TIMEOUT 5000           // This had to be simplified to only two scans per sweep as opposed to four due to what I think were RAM issues that were causing the arduino to freeze
// *** Accelerometer I2C address ***
const int MPU = 0x68;

// *** Encoder for distance measurement ***
#define ENCODER_PIN  3                       // interrupt pin
unsigned long ticks = 0;

// *** Initialize three instances of servos and define their pins ***
#define HOOK1 8
#define HOOK2 17
#define SERVO_SENSOR 7
#define NEUTRAL_POSITION 1450    // this is the middle position of the servo that has the ultrasonic sensor
ServoTimer2 hookServo1;
ServoTimer2 hookServo2;     
ServoTimer2 sensorServo;

// *** Variables for the motor driver ***

#define rightForward 10      // Control pins for the left motor (PWM)
#define rightReverse 9
#define leftForward 6      // Control pins for the right motor (PWM)
#define leftReverse 5
#define drivingSpeed 80     
#define reverseSpeed 40
#define turnSpeed 45
#define sharpTurnSpeed 0

RH_RF95 rf95(RFM95_CS, RFM95_INT);   // create RF95 class called rf95

bool currentState = false;
bool previousState = false;             // these are used by the handleState() function to store and change the state of the rover (driving or on standby)

void setup()
{
  // *** DC motor controller pins ***
  
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);
  analogWrite(leftForward, 0);            // stop all motors
  analogWrite(rightForward, 0);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
  
  // *** Transceiver Setup ***
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);

  // *** ISR setup ***
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countTicks, RISING);

  // *** Servos attached to respective pins ***
  pinMode(HOOK1, OUTPUT);
  pinMode(HOOK2, OUTPUT);
  pinMode(SERVO_SENSOR, OUTPUT);

  hookServo1.attach(HOOK1);
  hookServo2.attach(HOOK2);
  sensorServo.attach(SERVO_SENSOR);

  hookServo1.write(550);                   // initial stowed position of arm
  hookServo2.write(2500);               
  sensorServo.write(NEUTRAL_POSITION);                 // start with the servo in the neutral (middle) position
  
  // *** Ultrasonic sensor setup ***
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // *** Accelerometer setup ***
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);            // this is the address of the MPU 6050 module
  Wire.write(0);    
  Wire.endTransmission(true);
}

void loop() {

  unsigned long servoMillis = 0;          // this is a counter that will be used to move the sensor servo based on the millis function. It is reset to zero after every cycle.
  unsigned long lastServoMillis = 0;
  int servoAngle = 0;
  unsigned long currentMillis = 0;        // these variables are used to time the orientation check function call during driving
  unsigned long previousMillis = 0;           
  
  if (handleState()) {                     //  returns true if wakeup message is received. Also handles test signal and sends reply to ground control
    
    while (ticks < 20000) {        // Change this number to the actual number of ticks that represents 10+ feet. This is the driving mode. After exiting this loop the rover begins to dig.
                                           // All driving activity takes places within this while loop
      currentMillis = millis();
      if (currentMillis - previousMillis >= 10000) {      // Check orientation every 10 seconds and flip if needed
        previousMillis = currentMillis;
        if (getOrientation() <= -5) {
          sensorServo.write(1450);
          flip();
          servoMillis = 0;                       // reset servo position after each flip
          lastServoMillis = millis() - 500;      // this makes the value of servoMillis 500 in the next block, which corresponds to the neutral position of the sensor servo
        }
      }

      // The rover now begins to scan with the Ultrasonic sensor while the servo moves side to side.
      // Distance measurements are taken at 4 discrete instances during each sweep

      servoMillis = millis() - lastServoMillis;            // servoMillis goes from 0 to 1000 and rolls back to 0 every second.
      if (servoMillis >= 1000) {                           // the value of servoMillis is mapped to the servo angle to make it scan
        servoMillis = 0;                                   // 0 to 500 is clockwise and 500 to 1000 is counterclockwise
        lastServoMillis = millis();                        
      }
      if (servoMillis > 0 && servoMillis <= 500) {
        servoAngle = map(servoMillis, 0, 500, 1150, 1750);  // 1150 to 1750 microseconds are the pulse widths for the scanning angle. This scans left to right.
        sensorServo.write(servoAngle);
      }
      else if (servoMillis > 500 && servoMillis <= 1000) {
        servoAngle = map(servoMillis, 500, 1000, 1750, 1150);
        sensorServo.write(servoAngle);                          // scans right to left
      }
      if (servoMillis == 150) {                // while the servo is turning, the ultrasonic sensor takes distance measurements at specific angles. two measurements are taken during each sweep, one to the left and one to the right
        Clockwise[0] = obstacleCheck();        // arrays Clockwise and counterClockwise store obstacle information. 1 if obstacle exists at that position and 0 if no obstacle exists
      }                                        
      else if (servoMillis == 350) {
        Clockwise[1] = obstacleCheck();        // these values of the servoMillis variable correspond to specific angles at which we want to take distance measurements to detect obstacles
      } 
      else if (servoMillis == 650) {
        counterClockwise[0] = obstacleCheck();   // servoMillis goes from 0 to 1000 in one full sweep cycle (left to right and right to left)
      }
      else if (servoMillis == 850) {
        counterClockwise[1] = obstacleCheck();     
      }

      if (Clockwise[0] && counterClockwise[1]) 
        obstaclePresent[0] = 1;                   // Obstacles only register as true if they show up in both clockwise and counterclockwise scans
      else 
        obstaclePresent[0] = 0;

      if (Clockwise[1] && counterClockwise[0]) 
        obstaclePresent[1] = 1;
      else 
        obstaclePresent[1] = 0;

      // Use the above information about obstacles to decide how to drive

      if (!(obstaclePresent[0] | obstaclePresent[1])) {          // if there are no obstacles
        driveStraight();
      }
      else if (obstaclePresent[0] && obstaclePresent[1]){        // if there are obstacles on both sides
        sharpLeft();
      }
      else if (obstaclePresent[0]) {                             // if obstacle is on the left
        turnLeft();
      }
      else if (obstaclePresent[1]) {                             // if obstacle is on the right
        turnRight();
      }
      // once the counter reaches the equivalent of the total distance required to drive, the code exits out of this while loop.
    }
    // time to collect soil

    if (getOrientation() <= -5) {                 // getOrientation() returns acceleration in the z (vertical) direction in m/s^2
      flip();
    }
    for (int i = 0; i < 5; i++) {
      deployArm();
      delay (1000);
      reverse();
      delay(10000);
      retractArm();                      // simple series of events separated by delays
      delay(500);
      driveStraight();
      delay(2000);
      deployArm();
      reverse();
      delay(10000);
      retractArm();
    }
    analogWrite(leftForward, 0);            // stop all motors
    analogWrite(rightForward, 0);
    analogWrite(leftReverse, 0);
    analogWrite(rightReverse, 0);

    while (1) {                // infinite loop where it just sends the end-of-mission signal to ground control every second
      uint8_t done[] = "Done digging! Come get me";
      uint8_t len = sizeof(done);
      rf95.send(done, &len);
      rf95.waitPacketSent();
      delay(1000);
    }
  }
  else {
    Serial.println("Rover on standby");         // if handleState() returns 0. This is the default state before the wakeup signal is received.
  }
}

int getOrientation() {

  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  int z;
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 12, true);         // request 12 bytes from chip. six values 2 bytes each

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();         // a bunch of bit shifting to get the high and low bytes together for each value
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  z = AcZ / 1752;           // Dividing the raw value by this number gives the acceleration in m/s^2
  return z;
}

void countTicks() {        // Interrupt service routine for the encoder to measure distance travelled
  ticks++;                 // increments ticks
}

bool handleState() {  
  
  if (rf95.available())
  {
    uint8_t buf[4];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      delay(10);
      if ((char*)buf[0] == 'W') {        // wakeup signal reads "Wake up Wall E!"
        currentState = true; 
        previousState = currentState;
        uint8_t data[] = "Wakeup signal received";
        uint8_t len = sizeof(data);
        rf95.send(data, &len);
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        if (getOrientation() <= -5) {        // check orientation immediately after being activated
          delay(1000);
          flip();
        }
      }
      else if ((char*)buf[0] == 'T') {       // test signal reads "Testing connection"
        currentState = false;
        previousState = currentState;
        uint8_t data[] = "Connection test successful";
        uint8_t len = sizeof(data);
        rf95.send(data, &len);
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
      }
      else{
        currentState = false;
        previousState = currentState;
      }
    }
  }
  else{
    currentState = previousState;               // If no new message has been received, keep the previous state
  }                                             // the value returned by the function only changes if a new message is received.
  return currentState;                          // this is necessary to prevent the arduino from going back to being inactive immediately on the next scan
}

void deployArm() {

  for (int i = 550; i <= 2500; i++){
      hookServo1.write(i);
      hookServo2.write(3050 - i);
      delay(1);
  }
}

void retractArm() {

  for (int i = 550; i <= 2500; i++){
      hookServo1.write(3050 - i);
      hookServo2.write(i);
      delay(1);
  }
}

void flip() {
  deployArm();
  delay(500);
  retractArm();
  delay(500);
}

int obstacleCheck() {

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT);      // duration in microseconds for the echo to come back. returns zero if no echo is detected
    if (duration > 0) return 1;                    
    else return 0;

}

void driveStraight(){
  analogWrite(leftForward, drivingSpeed + 15);   // the left motor was turning slower than the right at the same duty cycle
  analogWrite(rightForward, drivingSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
void reverse(){
  analogWrite(leftForward, 0);
  analogWrite(rightForward, 0);
  analogWrite(leftReverse, reverseSpeed);
  analogWrite(rightReverse, reverseSpeed);
}
void turnLeft(){
  analogWrite(leftForward, turnSpeed);
  analogWrite(rightForward, drivingSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
void turnRight(){
  analogWrite(leftForward, drivingSpeed + 15);
  analogWrite(rightForward, turnSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
void sharpLeft(){
  analogWrite(leftForward, sharpTurnSpeed);
  analogWrite(rightForward, drivingSpeed);
  analogWrite(leftReverse, 0);
  analogWrite(rightReverse, 0);
}
