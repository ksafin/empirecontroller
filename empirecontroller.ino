#include <Encoder.h>
#include <Servo.h>

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

// SPI state
boolean hasbyte = false;
uint8_t data[100];
uint8_t idx = 0;
uint8_t ridx = 0;

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
#define FID_SETANG    0x10  // Set Servo Angle
#define FID_SETSSP    0x11  // Set Continuous Servo Speed
#define FID_TCURR     0x12  // Get Total Current
#define FID_SETPRR    0x13  // Set PPR for Motor

// LED Constants
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3

// Current Packet Data
uint8_t fid;      // Function ID
uint8_t scid;     // Subcomponent ID
uint8_t nparams;  // Num Parameters
uint8_t params[30];
uint8_t header;
uint8_t packet;

// Bit Masks
// Masks for extracting relevant data
#define subcomponentId_mask 0b00000001
#define parameterQty_mask   0b00001110
#define functionId_mask     0b11110000

void setup() {

  // Initialize on-board LED to green.
  pinMode(led_r, OUTPUT);
  pinMode(led_g, OUTPUT);
  pinMode(led_b, OUTPUT);
  setLED(LED_GREEN);

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
}

void loop() {
  if(hasbyte) {
    setLED(LED_BLUE);
    header = data[ridx];
    ridx++;
    if(ridx == 100) ridx = 0;

    // Extract header data
    fid = (functionId_mask & header) >> 4;
    scid = subcomponentId_mask & header;
    nparams = (parameterQty_mask & header) >> 1;

    if(fid == 0) setLED(LED_RED);

    // Extract all parameters
    for (int i = 0; i < nparams; i++) {
      params[i] = data[ridx];
      ridx++;
      if(ridx == 100) ridx = 0;
    }

    // Run function
    switch(fid) {
      case FID_SETPWM: {
        pwm = params[0];
        drivePWM();
        break;
      }
    }
    
    hasbyte = false;
  }
  
  updateExternalLED();
  encPos = enc.read();
}

// SPI Interrupt Control
ISR (SPI_STC_vect)
{
  // Save received byte, set flag
  data[idx] = SPDR;
  idx++;
  if(idx == 100) idx = 0;
  hasbyte = true;
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

