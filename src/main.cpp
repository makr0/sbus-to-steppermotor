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
#define MOTOR_MAX_SPEED 6000
#define CHANNEL_MIN 342
#define CHANNEL_MAX 1706

AccelStepper motor1(AccelStepper::DRIVER, M1_step,M1_dir);
AccelStepper motor2(AccelStepper::DRIVER, M2_step,M2_dir);


// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial1);
#define NUM_CHANNELS 12
char uart_buf[120];

// channel, fail safe, and lost frames data
uint16_t channels[NUM_CHANNELS];
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

void initTimerInterrupt() {
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register to some 'fast' value
  OCR1A = (8000000 >> 8) / 8;
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


void setup() {
  initTimerInterrupt();
  delay(100);
  pinMode(M1_enable, OUTPUT);
  pinMode(M1_step, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M2_enable, OUTPUT);
  pinMode(M2_step, OUTPUT);
  pinMode(M2_dir, OUTPUT);
  pinMode(MOTORS_RESET, OUTPUT);
  digitalWrite(MOTORS_RESET,HIGH);
  pinMode(M1_MS3, OUTPUT);
  pinMode(M2_MS3, OUTPUT);
  pinMode(MX_MS2, OUTPUT);
  pinMode(MX_MS1, OUTPUT);

  digitalWrite(M1_MS3,LOW);
  digitalWrite(M2_MS3,LOW);
  digitalWrite(MX_MS2,LOW);
  digitalWrite(MX_MS1,HIGH);

  motor1.setEnablePin(M1_enable);
  motor2.setEnablePin(M2_enable);
  motor1.setPinsInverted(false,false,true);
  motor2.setPinsInverted(false,false,true);
  motor1.setMaxSpeed(MOTOR_MAX_SPEED);
  motor2.setMaxSpeed(MOTOR_MAX_SPEED);
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor1.enableOutputs();
  motor2.enableOutputs();


  // begin the SBUS communication
  Serial.begin(9600);
  // while(!Serial);
  Serial.print("SBUS to stepper driver for 2 Motors\n");
  x8r.begin();

}

ISR(TIMER1_COMPA_vect) {
  motor1.runSpeed();
  motor2.runSpeed();
}

void loop() {

  // look for a good SBUS packet from the receiver
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    if(updateSerial-- == 0) {
      updateSerial = 20;
      for(i=0;i<NUM_CHANNELS;i++) {
        sprintf(uart_buf, "%4d ",channels[i]);
        Serial.print(uart_buf);
      }
      sprintf(uart_buf, "F: %4d L:%4d M1: %4ld M2: %4ld",failSafe,lostFrame,(int32_t)M1_speed,(int32_t)M2_speed);
      Serial.print(uart_buf);
      Serial.print("\r");
    }
    M1_speed = map(constrain(channels[0],CHANNEL_MIN,CHANNEL_MAX),CHANNEL_MIN,CHANNEL_MAX,-MOTOR_MAX_SPEED,MOTOR_MAX_SPEED);
    M2_speed = map(constrain(channels[1],CHANNEL_MIN,CHANNEL_MAX),CHANNEL_MIN,CHANNEL_MAX,-MOTOR_MAX_SPEED,MOTOR_MAX_SPEED);
    if(failSafe) {
      motor1.stop();
      motor2.stop();
    } else {
      motor1.setSpeed(M1_speed);
      motor2.setSpeed(M2_speed);
    }
    if(enable_last != channels[CHANNEL_ENABLE]) {
      enable_last = channels[CHANNEL_ENABLE];
      if(enable_last > 1000) {
        motor1.enableOutputs();motor2.enableOutputs();
      } else {
        motor1.disableOutputs();motor2.disableOutputs();
      }
    }
  }
}
