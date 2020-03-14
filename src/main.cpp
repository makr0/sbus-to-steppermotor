#include <Arduino.h>
#include "SBUS.h"
#include <AccelStepper.h>

#define M1_enable 10
#define M1_step 4
#define M1_dir 8

#define M2_enable 10
#define M2_step 5
#define M2_dir 6

#define MOTORS_RESET 14
#define M1_MS3 7
#define M2_MS3 9

#define MX_MS2 15
#define MX_MS1 16

#define CHANNEL_ENABLE 5
#define MOTOR_MAX_SPEED 3000
#define MOTOR_MAX_SPEED_FULLSTEP 3000
#define CHANNEL_MIN 342
#define CHANNEL_MAX 1706
#define CHANNEL_MITTE 1024
#define RC_TOTZONE 50
#define CHANNEL_M1 1
#define CHANNEL_M2 2
#define SWITCHPOINT1 120
#define SWITCHPOINT2 190
#define SWITCHPOINT3 275


AccelStepper motor1(AccelStepper::DRIVER, M1_step, M1_dir);
AccelStepper motor2(AccelStepper::DRIVER, M2_step, M2_dir);

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

void initTimerInterrupt()
{
  cli();      // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register to some 'fast' value
  //OCR1A = (8000000 >> 8) / 13;
  OCR1A = 2700;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS01 bit for no prescaler
  TCCR1B |= (1 << CS10);
  // enable timer compare interrupt on start
  TIMSK1 |= (1 << OCIE1A);
  // disable timer compare interrupt on start
  //TIMSK1 &= ~(1 << OCIE1A);

  sei(); // allow interrupts
}

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

void setup()
{
  initTimerInterrupt();
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

  motor1.setEnablePin(M1_enable);
  motor2.setEnablePin(M2_enable);
  motor1.setPinsInverted(false, false, true);
  motor2.setPinsInverted(false, false, true);
  motor1.setMaxSpeed(MOTOR_MAX_SPEED);
  motor2.setMaxSpeed(MOTOR_MAX_SPEED);
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  //motor1.setAcceleration(100);
  //motor2.setAcceleration(200);
  motor1.disableOutputs();
  motor2.disableOutputs();

  // begin the SBUS communication
  Serial.begin(9600);
  //while(!Serial);
  Serial.print("SBUS to stepper driver for 2 Motors\n");
  x8r.begin();
}

ISR(TIMER1_COMPA_vect)
{
  motor1.runSpeed();
  motor2.runSpeed();
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
    if(raw_channels[CHANNEL_M1] > CHANNEL_MITTE+SWITCHPOINT3 || raw_channels[CHANNEL_M2] > CHANNEL_MITTE+SWITCHPOINT3 ) stepMode=1;
    else if(raw_channels[CHANNEL_M1] < CHANNEL_MITTE-SWITCHPOINT3 || raw_channels[CHANNEL_M2] < CHANNEL_MITTE-SWITCHPOINT3 ) stepMode=1;
    else if(raw_channels[CHANNEL_M1] > CHANNEL_MITTE+SWITCHPOINT2 || raw_channels[CHANNEL_M2] > CHANNEL_MITTE+SWITCHPOINT2 ) stepMode=2;
    else if(raw_channels[CHANNEL_M1] < CHANNEL_MITTE-SWITCHPOINT2 || raw_channels[CHANNEL_M2] < CHANNEL_MITTE-SWITCHPOINT2 ) stepMode=2;
    else if(raw_channels[CHANNEL_M1] > CHANNEL_MITTE+SWITCHPOINT1 || raw_channels[CHANNEL_M2] > CHANNEL_MITTE+SWITCHPOINT1 ) stepMode=4;
    else if(raw_channels[CHANNEL_M1] < CHANNEL_MITTE-SWITCHPOINT1 || raw_channels[CHANNEL_M2] < CHANNEL_MITTE-SWITCHPOINT1 ) stepMode=4;
    else stepMode=32;

    M1_speed = map(raw_channels[CHANNEL_M1], CHANNEL_MIN, CHANNEL_MAX, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED) * (stepMode==32?32:stepMode==4?1:stepMode==2?1:0.7);
    M2_speed = map(raw_channels[CHANNEL_M2], CHANNEL_MIN, CHANNEL_MAX, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED) * (stepMode==32?32:stepMode==4?1:stepMode==2?1:0.7);
    M1_speed = constrain(M1_speed,-(stepMode==1?MOTOR_MAX_SPEED_FULLSTEP:MOTOR_MAX_SPEED), (stepMode==1?MOTOR_MAX_SPEED_FULLSTEP:MOTOR_MAX_SPEED));
    M2_speed = constrain(M2_speed,-(stepMode==1?MOTOR_MAX_SPEED_FULLSTEP:MOTOR_MAX_SPEED), (stepMode==1?MOTOR_MAX_SPEED_FULLSTEP:MOTOR_MAX_SPEED));
    if(abs(M1_speed+M2_speed)>150) stepMode = constrain(stepMode,4,32);
    if (updateSerial-- == 0)
    {
      updateSerial = 5;
      sprintf(uart_buf, "CHM1: %4d CHM2 %4d | M1: %4ld M2: %4ld | step %2d    \r", raw_channels[CHANNEL_M1], raw_channels[CHANNEL_M2], (int32_t)M1_speed, (int32_t)M2_speed, stepMode);
      Serial.print(uart_buf);
    }
    if (failSafe)
    {
      motor1.stop();
      motor2.stop();
      motor1.disableOutputs();
      motor2.disableOutputs();
    }
    else
    {
      MOTOR_STEP(stepMode);
      motor1.setSpeed(M1_speed);
      motor2.setSpeed(M2_speed);
      if (enable_last != raw_channels[CHANNEL_ENABLE])
      {
        enable_last = raw_channels[CHANNEL_ENABLE];
        if (enable_last > 1000)
        {
          motor1.enableOutputs();
          motor2.enableOutputs();
        }
        else
        {
          motor1.disableOutputs();
          motor2.disableOutputs();
        }
      }
    }
  }
}
