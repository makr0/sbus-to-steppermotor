#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>
#include "SBUS.h"
#include "LowPass.h"

#define M1_enable 10
#define M1_step 9 // 4
#define M1_dir 8

#define M2_enable 10
#define M2_step 5
#define M2_dir 6

#define MOTORS_RESET 14
#define M1_MS3 7
#define M2_MS3 4 // 9

#define MX_MS2 15
#define MX_MS1 16

#define CHANNEL_ENABLE 5
#define MOTOR_MAX_SPEED 15000
#define CHANNEL_MIN 342
#define CHANNEL_MAX 1706
#define CHANNEL_MITTE 1024
#define RC_TOTZONE 50
#define CHANNEL_M1 0
#define CHANNEL_M2 1

#define IDLE_TIMEOUT 200;
int idleTimer = 0;

LowPass FilterM1;
LowPass FilterM2;

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial1);
#define NUM_CHANNELS 12
#define MAPPED_CHANNELS 2
char uart_buf[120];

// channel, fail safe, and lost frames data
uint16_t raw_channels[NUM_CHANNELS];
float mapped_channels[MAPPED_CHANNELS];
bool failSafe;
bool lostFrame;
// when to update serial interface
int updateSerial = 50;

// remember enable switch state
uint16_t enable_last = 0;
int i;

// Motor speeds
float M1_speed = 0;
float M2_speed = 0;

// for Channel calculations
int stepMode = 32;

struct CTC1
{
    static void setup() {
        TCCR1A = 0; // CTC mode with TOP-OCR1A
        TCCR1B = _BV(WGM12);
        TCCR1A = (TCCR1A & ~(_BV(COM1A1) | _BV(COM1A0))) | _BV(COM1A0); // toggle channel A on compare match
        DDRB |= _BV(5); // set channel A bound pin to output mode // PB1 on 328p, use _BV(5) for PB5 on 32U4
    }

    static void set_freq(float f) {
        static const float f1 = min_freq(1), f8 = min_freq(8), f64 = min_freq(64), f256 = min_freq(256);
        uint16_t n;
        if (f >= f1)        n = 1;
        else if (f >= f8)   n = 8;
        else if (f >= f64)  n = 64;
        else if (f >= f256) n = 256;
        else                n = 1024;
        prescale(n);
        OCR1A = static_cast<uint16_t>(round(F_CPU / (2 * n * f) - 1));
    }

    static void prescale(uint16_t n) {
        uint8_t bits = 0;
        switch (n) {
            case    1:  bits = _BV(CS10);               break;
            case    8:  bits = _BV(CS11);               break;
            case   64:  bits = _BV(CS11) | _BV(CS10);   break;
            case  256:  bits = _BV(CS12);               break;
            case 1024:  bits = _BV(CS12) | _BV(CS10);   break;
            default:    bits = 0;
        }
        TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11) | _BV(CS10))) | bits;
    }
    static inline float min_freq(uint16_t n) {
        return ceil(F_CPU / (2 * n * 65536));
    }
};

struct CTC2
{
    static void setup() {
        TCCR3A = 0; // CTC mode with TOP-OCR1A
        TCCR3B = _BV(WGM12);
        TCCR3A = (TCCR3A & ~(_BV(COM3A1) | _BV(COM3A0))) | _BV(COM3A0); // toggle channel A on compare match
        DDRC |= _BV(6); // set channel A bound pin to output mode // _BV(6) for PC6 on 32U4
    }

    static void set_freq(float f) {
        static const float f1 = min_freq(1), f8 = min_freq(8), f64 = min_freq(64), f256 = min_freq(256);
        uint16_t n;
        if (f >= f1)        n = 1;
        else if (f >= f8)   n = 8;
        else if (f >= f64)  n = 64;
        else if (f >= f256) n = 256;
        else                n = 1024;
        prescale(n);
        OCR3A = static_cast<uint16_t>(round(F_CPU / (2 * n * f) - 1));
    }

    static void prescale(uint16_t n) {
        uint8_t bits = 0;
        switch (n) {
            case    1:  bits = _BV(CS10);               break;
            case    8:  bits = _BV(CS11);               break;
            case   64:  bits = _BV(CS11) | _BV(CS10);   break;
            case  256:  bits = _BV(CS12);               break;
            case 1024:  bits = _BV(CS12) | _BV(CS10);   break;
            default:    bits = 0;
        }
        TCCR3B = (TCCR3B & ~(_BV(CS32) | _BV(CS31) | _BV(CS30))) | bits;
    }
    static inline float min_freq(uint16_t n) {
        return ceil(F_CPU / (2 * n * 65536));
    }
};

void MOTOR_STEP(int mode)
{
  switch (mode)
  {
  case 1:
    digitalWrite(M1_MS3, LOW);
    digitalWrite(M2_MS3, LOW);
    digitalWrite(MX_MS2, LOW);
    digitalWrite(MX_MS1, LOW);
    break;
  case 2:
    digitalWrite(M1_MS3, LOW);
    digitalWrite(M2_MS3, LOW);
    digitalWrite(MX_MS2, LOW);
    digitalWrite(MX_MS1, HIGH);
    break;
  case 4:
    digitalWrite(M1_MS3, LOW);
    digitalWrite(M2_MS3, LOW);
    digitalWrite(MX_MS2, HIGH);
    digitalWrite(MX_MS1, LOW);
    break;
  case 8:
    digitalWrite(M1_MS3, LOW);
    digitalWrite(M2_MS3, LOW);
    digitalWrite(MX_MS2, HIGH);
    digitalWrite(MX_MS1, HIGH);
    break;
  case 32:
    digitalWrite(M1_MS3, HIGH);
    digitalWrite(M2_MS3, HIGH);
    digitalWrite(MX_MS2, HIGH);
    digitalWrite(MX_MS1, HIGH);
    break;
  }
}

void enableMotors(int mode) {
    digitalWrite(M1_enable, !mode); // enable/disable motor
    digitalWrite(M2_enable, !mode); // enable/disable motor
}

void setMotorSpeed(int motor, int speed){
  if(motor==1) {
    speed = FilterM1.step(speed);
    digitalWrite(M1_dir,speed>0);
    CTC1::set_freq(abs(speed));
  }
  if(motor==2) {
    speed = FilterM2.step(speed);
    digitalWrite(M2_dir,speed>0);
    CTC2::set_freq(abs(speed));
  }
}

void setup()
{
  CTC1::setup();
  CTC2::setup();
  delay(100);
  pinMode(M1_enable, OUTPUT);
  pinMode(M1_step, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M2_enable, OUTPUT);
  pinMode(M2_step, OUTPUT);
  pinMode(M2_dir, OUTPUT);
  pinMode(MOTORS_RESET, OUTPUT);
  digitalWrite(MOTORS_RESET, HIGH);
  pinMode(M1_MS3, OUTPUT);
  pinMode(M2_MS3, OUTPUT);
  pinMode(MX_MS2, OUTPUT);
  pinMode(MX_MS1, OUTPUT);

  MOTOR_STEP(stepMode);

  enableMotors(false);
  // begin the SBUS communication
  Serial.begin(9600);
  //while(!Serial);
  Serial.print("SBUS to stepper driver for 2 Motors\n");
  x8r.begin();
}

void debug_channels()
{
  for (i = 0; i < NUM_CHANNELS; i++)
  {
    sprintf(uart_buf, "%4d ", raw_channels[i]);
    Serial.print(uart_buf);
  }
  sprintf(uart_buf, "F: %4d L:%4d M1: %4ld M2: %4ld", failSafe, lostFrame, (int32_t)M1_speed, (int32_t)M2_speed);
  Serial.print(uart_buf);
  Serial.print("\r");
}
void loop()
{

  // look for a good SBUS packet from the receiver
  if (x8r.read(&raw_channels[0], &failSafe, &lostFrame))
  {
    // constrain channels, implement deadband
    for (i = 0; i < NUM_CHANNELS; i++)
    {
      raw_channels[i] = constrain(raw_channels[i], CHANNEL_MIN, CHANNEL_MAX);
      if (raw_channels[i] > (CHANNEL_MITTE + RC_TOTZONE))
      {
        raw_channels[i] = raw_channels[i] - RC_TOTZONE;
      }
      else if (raw_channels[i] < (CHANNEL_MITTE - RC_TOTZONE))
      {
        raw_channels[i] = raw_channels[i] + RC_TOTZONE;
      }
      else
      {
        raw_channels[i] = CHANNEL_MITTE;
      }
    }

    M1_speed = map(raw_channels[CHANNEL_M1], CHANNEL_MIN, CHANNEL_MAX, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    M2_speed = map(raw_channels[CHANNEL_M2], CHANNEL_MIN, CHANNEL_MAX, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

    if (updateSerial-- == 0)
    {
      updateSerial = 5;
      sprintf(uart_buf, "CHM1: %4d CHM2 %4d | M1: %4ld M2: %4ld | step %2d    \r", raw_channels[CHANNEL_M1], raw_channels[CHANNEL_M2], (int32_t)M1_speed, (int32_t)M2_speed, stepMode);
      Serial.print(uart_buf);
      //debug_channels();
    }
    if (failSafe)
    {
      setMotorSpeed(1,0);
      setMotorSpeed(2,0);
      enableMotors(false);
    }
    else
    {
      setMotorSpeed(1,M1_speed);
      setMotorSpeed(2,M2_speed);
      if(M1_speed == 0 && M1_speed==0) {
        if(idleTimer == 0) {
          enableMotors(false);
        } else {
          idleTimer--;
        }
      } else {
        if(idleTimer == 0) {
          enableMotors(true);
        }
        idleTimer = IDLE_TIMEOUT;
      }
      if (enable_last != raw_channels[CHANNEL_ENABLE])
      {
        enable_last = raw_channels[CHANNEL_ENABLE];
        if (enable_last > 1000)
        {
          enableMotors(true);
        }
        else
        {
          enableMotors(false);
        }
      }
    }
  }
}
