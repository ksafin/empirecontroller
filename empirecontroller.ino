#include <Encoder.h>
#include <Servo.h>

typedef union
{
 float val;
 uint8_t bytes[4];
} FLOATUNION;

typedef union
{
 short val;
 uint8_t bytes[2];
} SHORTUNION;

typedef union
{
 uint32_t val;
 uint8_t bytes[4];
} INTUNION;

FLOATUNION fu;
SHORTUNION su;
INTUNION iu;

// Pin definitions
#define led_r 0
#define led_g A3
#define led_b A2
#define enc_a 3
#define enc_b 2
#define red_ext 8
#define green_ext 7
#define blue_ext 9
#define pwm1 5
#define pwm2 6
#define isense A1
#define pin_servo1 A4
#define pin_servo2 A5

// Global objects
Servo servo1;
Servo servo2;
Encoder enc(enc_a, enc_b);

// Motor State
uint8_t pwm =  0;
boolean forward = true;
float ilim = 100;
uint16_t rpm;
boolean isPID = false;
float current;
boolean brake = true;
uint16_t coast_speed = 100;
uint8_t fault = 0;

// PID State
float Kp;
float Kd;
float Ki;
uint16_t target_rpm;

// SPI state
boolean hasbyte = false;
boolean hasPacket = false;
uint8_t numbytes = 999;
uint8_t curbyte = 0;
uint8_t data[1000];
boolean loopback = false;
uint8_t loopcnt = 0;
int idx = 0;
int ridx = 0;

// Encoder variables
long encPos = 0;

// Function ID's
#define FID_SETPWM    0x00  // Set Motor PWM
#define FID_SETILIM   0x01  // Set Motor Current Limit
#define FID_GETSPD    0x02  // Get Motor Speed (RPM)
#define FID_PIDEN     0x03  // Enable PID
#define FID_GETCUR    0x04  // Get Motor Current
#define FID_SETPID    0x05  // Set PID Constants
#define FID_SETSPD    0x06  // Set Motor Speed (RPM)
#define FID_RESENC    0x07  // Reset Motor Encoder
#define FID_GETENC    0x08  // Get Motor Encoder Ticks
#define FID_ROTPWM    0x09  // Rotate for some Revs at a given PWM
#define FID_ROTSPD    0x0A  // Rotate for some Revs at a given RPM
#define FID_COAST     0x0B  // Enable Coast
#define FID_BRAKE     0x0C  // Enable Brake
#define FID_CSPD      0x0D  // Set Coast Speed
#define FID_STOP      0x0E  // Stop Motor
#define FID_FAULT     0x0F  // Get Motor Fault
#define FID_SETANG1   0x10  // Set Servo Angle (Servo 1)
#define FID_SETSSP1   0x11  // Set Continuous Servo Speed (Servo 1)
#define FID_SETANG2   0x12  // Set Servo Angle (Servo 2)
#define FID_SETSSP2   0x13  // Set Continuous Servo Speed (Servo 2)
#define FID_TCURR     0x14  // Get Total Current
#define FID_SETPRR    0x15  // Set PPR for Motor

// LED Constants
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3
#define LED_WHITE 4

// Current Packet Data
uint8_t fid;      // Function ID
uint8_t cid;      // Subcomponent ID
uint8_t nparams;  // Num Parameters
uint8_t params[30];
uint8_t header;
uint8_t packet;

boolean goodtogo = false;

// Bit Masks
// Masks for extracting relevant data
#define componentId_mask    0b00000111
#define functionId_mask     0b11111000

void setup() {

  // Initialize on-board LED to green.
  pinMode(led_r, OUTPUT);
  pinMode(led_g, OUTPUT);
  pinMode(led_b, OUTPUT);
  setLED(LED_WHITE);

  // Initialize external LED to off
  pinMode(red_ext, OUTPUT);
  pinMode(green_ext, OUTPUT);
  pinMode(blue_ext, OUTPUT);
  digitalWrite(red_ext, HIGH);
  digitalWrite(green_ext, HIGH);
  digitalWrite(blue_ext, HIGH);

  // Initialize motor to OFF
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  digitalWrite(pwm1, LOW);
  digitalWrite(pwm2, LOW);

  // Initialize analog input
  pinMode(isense, INPUT);

  // Initialize servos
  pinMode(pin_servo1, OUTPUT);
  pinMode(pin_servo2, OUTPUT);
  servo1.attach(pin_servo1);
  servo2.attach(pin_servo2);

  // Initialize encoder
  pinMode(enc_a, INPUT);
  pinMode(enc_b, INPUT);

  // Set SPI mode to slave
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);

 // for(int i = 0; i < 255; i++) {
 //   digitalWrite(pwm1, 0);
 //   analogWrite(pwm2, i);
 //   delay(250);
 // }

  //delay(5000);
}

void loop() {
  if(getNumAhead() > 5) {
    hasPacket = false;
    //hasPacket = false;
    header = readData();
   // delay(250);
    //setLED(LED_RED);

    // Extract header data
    fid = (functionId_mask & header) >> 3;
    cid = (componentId_mask & header);

    //delayMicroseconds(1000);
    nparams = readData();

    //uint8_t checksum = header + nparams * 10;

    // Extract all parameters
    for (int i = 0; i < nparams; i++) {
      //delayMicroseconds(1000);
      params[i] = readData();
      //checksum += params[i] % 10;
    }

    //if(check == checksum) {

    // Run function
    switch(fid) {
      case FID_SETPWM: {
        pwm = params[0];
        if(pwm == 0) setLED(LED_WHITE);
        forward = (boolean) params[1];
        drivePWM();
        break; }
      case FID_SETILIM: {
        ilim = (((double)(getShort(params[0], params[1]))) / 1000.0);
        break; }
      case FID_GETSPD: {
        sendShort(rpm);
        break; }
      case FID_PIDEN: {
        isPID = params[0];
        break; }
      case FID_GETCUR: {
        sendShort((uint16_t)(current * 1000));
        break; }
      case FID_SETPID: {
        Kp = getFloat(params[0], params[1], params[2], params[3]);
        Kd = getFloat(params[4], params[5], params[6], params[7]);
        Ki = getFloat(params[8], params[9], params[10], params[11]);
        break; }
      case FID_SETSPD: {
        forward = (boolean) params[2];
        target_rpm = getShort(params[0], params[1]);
        break; }
      case FID_RESENC: {
        enc.write(0);
        break; }
      case FID_GETENC: {
        sendInt(encPos);
        break; }
      case FID_ROTPWM: {
        isPID = false;
        forward = params[1];
        pwm = params[0];
        rotatePWM(getFloat(params[2], params[3], params[4], params[5]));
        break; }
      case FID_ROTSPD: {
        isPID = true;
        forward = params[2];
        target_rpm = getShort(params[0], params[1]);
        rotateRPM(getFloat(params[3], params[4], params[5], params[6]));
        break; }
      case FID_COAST: {
        brake = false;
        break; }
      case FID_BRAKE: {
        brake = true;
        break; }
      case FID_CSPD: {
        coast_speed = getShort(params[0], params[1]);
        break; }
      case FID_STOP: {
        stopMotor();
        break; }
      case FID_FAULT: {
        Serial.write(fault);
        break; }
      case FID_SETANG1: {
        servo1.write(params[0]);
        break; }
      case FID_SETANG2: {
        servo2.write(params[0]);
        break; }
      case FID_SETSSP1: {
        servo1.write(params[0]);
        break; }
      case FID_SETSSP2: {
        servo2.write(params[0]);
        break; }
    }
   // }
    

  }

  /*if(curbyte == 2) {
    numbytes = data[idx] + 2;
  }

  if(curbyte == numbytes) {
    hasPacket = true;
    curbyte == 0;
    numbytes == 999;
  }*/

 // if (idx > 550) {
 //   goodtogo = true;
 // }
  updateExternalLED();
 // encPos = (uint32_t) enc.read();
}

uint8_t readData() {
  uint8_t val = data[ridx];
  ridx++;
  if(ridx == 1000) ridx = 0;
  return val;
}

// SPI Interrupt Control
ISR (SPI_STC_vect)
{
  data[idx] = SPDR;
  idx++;
  if(idx == 1000) {
    idx = 0;
    loopback = true;
  }
}

void waitForByte() {
  while((idx-ridx) == 0) {}
}

uint8_t getNumAhead() {
  if(idx > ridx) {
    return (idx - ridx);
    loopback = false;
  } else if ((ridx > idx) && (loopback)) {
    return ((1000 - ridx) + idx);
  } 
}

void drivePWM() {
  if(forward) {
    digitalWrite(pwm1, 0);
    analogWrite(pwm2, pwm);
  } else {
    digitalWrite(pwm2, 0);
    analogWrite(pwm1, pwm);
  }
}

void setLED(uint8_t color) {
  if(color == LED_RED) {
    digitalWrite(led_g, HIGH);
    digitalWrite(led_r, LOW);
    digitalWrite(led_b, HIGH);
  } else if (color == LED_GREEN) {
    digitalWrite(led_g, LOW);
    digitalWrite(led_r, HIGH);
    digitalWrite(led_b, HIGH);
  } else if (color == LED_BLUE) {
    digitalWrite(led_g, HIGH);
    digitalWrite(led_r, HIGH);
    digitalWrite(led_b, LOW);
  } else if (color == LED_WHITE) {
    digitalWrite(led_g, LOW);
    digitalWrite(led_r, LOW);
    digitalWrite(led_b, LOW);
  }
}

void updateExternalLED() {
  if(pwm == 0) {
    digitalWrite(red_ext, HIGH);
    digitalWrite(green_ext, HIGH);
    digitalWrite(blue_ext, LOW);
  } else if(!forward) {
    digitalWrite(red_ext, LOW);
    digitalWrite(green_ext, HIGH);
    digitalWrite(blue_ext, HIGH);
  } else if (forward) {
    digitalWrite(red_ext, HIGH);
    digitalWrite(green_ext, LOW);
    digitalWrite(blue_ext, HIGH);
  }
}

void rotatePWM(float numrots) {
  
}

void rotateRPM(float numrots) {
  
}

void stopMotor() {
  
}

void sendShort(uint16_t val) {
  su.val = val;
  Serial.write(su.bytes[0]);
  Serial.write(su.bytes[1]);
}

void sendFloat(float val) {
  fu.val = val;
  Serial.write(fu.bytes[0]);
  Serial.write(fu.bytes[1]);
  Serial.write(fu.bytes[2]);
  Serial.write(fu.bytes[3]);
}

void sendInt(int val) {
  iu.val = val;
  Serial.write(iu.bytes[0]);
  Serial.write(iu.bytes[1]);
  Serial.write(iu.bytes[2]);
  Serial.write(iu.bytes[3]);
}

float getFloat(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
  fu.bytes[0] = p1;
  fu.bytes[1] = p2;
  fu.bytes[2] = p3;
  fu.bytes[3] = p4;
  return fu.val;
}

uint16_t getShort(uint8_t p1, uint8_t p2) {
  su.bytes[0] = p1;
  su.bytes[1] = p2;
  return su.val;
}
