#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Motor shield motor pins.
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(3);

// Line sensor data receive pins.
const int L1LF_receive = 8;
const int R1LF_receive = 10;

const int main_loop_delay_time = 100; // main loop delay.

const int printfreq = 500 / main_loop_delay_time;

// Tunable Parameters.
const float kp = 50; // Proportional gain.
const float ki = 0; // Integral gain.
const float kd = 0; // Derivative gain.

const int delay_time = 100; // Misc delay.
const int max_speed = 255; // Maximum allowable motor speed.
const int ref_speed = 200; // Normal forward motor speed.
const int turn_speed = 150; // Turning speed.

// ---------------------
int main_loop_counter = 0;

// PID parameters.
float PIDError = 0; // PID control feedback
float P, I, D;
float pre_I = 0;
float pre_P = 0;
unsigned long current_time = 0;
unsigned long prev_time = 0;

// Line sensor data. 0 (black) or 1 (white).
int L1LF_data;
int R1LF_data;

int speedL;
int prev_speedL;
int speedR;
int prev_speedR;

class LFDetection
{
public:
    void LFDataRead(void); // Read data from line sensors.
};

void LFDetection::LFDataRead()
{
    L1LF_data = digitalRead(L1LF_receive);
    R1LF_data = digitalRead(R1LF_receive);

    if (main_loop_counter % printfreq == 0){
      Serial.println("Line Sensor 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
    }
}

// ******************************************

class MovementControl: public LFDetection
{
  public:
      void LineFollow(void);
      void PID(void);
};

void MovementControl::PID()
{
  P = -(L1LF_data - R1LF_data);
  I = pre_I + P * 0.001 * (current_time - prev_time);
  PIDError = P * kp + I * ki;
  pre_I = I;
  prev_time = current_time;

  speedL = ref_speed + PIDError;
  speedR = ref_speed - PIDError;

  if (speedL > max_speed) {
    speedL = max_speed;
  }
  if (speedR > max_speed) {
    speedR = max_speed;
  }
  if (speedL < 0) {
    speedL = 0;
  }
  if (speedR < 0) {
    speedR = 0;
  }
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);
  //LeftMotor->setSpeed(speedL);
  //RightMotor->setSpeed(speedR);
  if (speedL != prev_speedL){
    LeftMotor->setSpeed(speedL);
    prev_speedL = speedL;
  }
  if (speedR != prev_speedR){
    RightMotor->setSpeed(speedR);
    prev_speedR = speedR;
  }
}

void MovementControl::LineFollow()
{
    LFDataRead();
    PID();
    if (main_loop_counter % printfreq  == 0){
        Serial.println("Motor speed L R: " + String(speedL) + " "
         + String(speedR));
    }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Testing START!");
  AFMS.begin();

  pinMode(L1LF_receive,INPUT);
  pinMode(R1LF_receive,INPUT);
}

void loop()
{
    if (main_loop_counter % printfreq == 0){
        Serial.println("Loop: " + String(main_loop_counter) + " ------------------------");
    }
    MovementControl MC;
    LFDetection LF;

    current_time = millis();

    MC.LineFollow();

    delay(main_loop_delay_time);
    main_loop_counter++;
    if (main_loop_counter % printfreq == 0){
        Serial.println(" ");
    }

}
