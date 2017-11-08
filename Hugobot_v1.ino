#include "Adafruit_VL53L0X.h"
#include <QTRSensors.h>

#define InterruptPin  2
#define ActivatePin   3
#define S0Pin         4
#define S1Pin         7
#define S2Pin         8
#define MotorLeftA    9
#define MotorLeftB    10
#define MotorRightA   5
#define MotorRightB   6
#define RedLedPin     13

#define DOJO_LINES_LIMIT (300)
#define ENEMY_RANGE_MM (750)
#define LASER_PHASE_OK (4)
#define MAX_RANGE_MM (1000)

#define STATE_DOJO_DANGER     0
#define STATE_ENEMY_DANGER_1  1
#define STATE_ENEMY_DANGER_2  2
#define STATE_SEARCHING       3
#define STATE_WAITING         4

Adafruit_VL53L0X laser = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t laser_data;

#define QTR_NUM_SENSORS   3     // number of sensors used
#define QTR_TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define QTR_EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

// sensors 0 through 2 are connected to digital pins 4, 7 and 8, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  S0Pin, S1Pin, S2Pin
},
QTR_NUM_SENSORS, QTR_TIMEOUT, QTR_EMITTER_PIN);
unsigned int IR_SensorValues[QTR_NUM_SENSORS];

volatile uint8_t ToggleMode = 0;

typedef struct {
  uint8_t Waiting;
  uint8_t Mode;
  uint16_t DojoLines;
  uint8_t State;
  int16_t MotorLeft;
  int16_t MotorRight;
  uint16_t DistanceToEnemy;
} Robot_t;

Robot_t hugobot;

void setup() {
  pinMode(RedLedPin, OUTPUT);
  pinMode(InterruptPin, INPUT_PULLUP);
  pinMode(ActivatePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(InterruptPin), switch_mode, CHANGE);

  // Init serial comm
  Serial.begin(115200);

  // Init laser
  laser.begin();
}

void loop() {
  read_sensors();
  state_machine();
  move_motors();
  delay(10);
}

void read_sensors() {
  //Serial.println(millis());
  hugobot.Waiting = digitalRead(ActivatePin);

  // Read dojo lines
  qtrrc.read(IR_SensorValues);
  hugobot.DojoLines = IR_SensorValues[0] + IR_SensorValues[1] + IR_SensorValues[2];

  // Read distance to closest obstacle
  laser.rangingTest(&laser_data, false); // pass in 'true' to get debug data printout!

  if (laser_data.RangeStatus != LASER_PHASE_OK) {
    hugobot.DistanceToEnemy = laser_data.RangeMilliMeter;
  }
  else {
    hugobot.DistanceToEnemy = MAX_RANGE_MM;
  }
  //Serial.println(millis());
}

void state_machine() {
  switch (hugobot.State) {
    case STATE_DOJO_DANGER:
      if (hugobot.DojoLines >= DOJO_LINES_LIMIT) {
        if (hugobot.DistanceToEnemy <= ENEMY_RANGE_MM) {
          hugobot.State = STATE_ENEMY_DANGER_1;
        }
        else {
          hugobot.State = STATE_SEARCHING;
        }
      }

      if (hugobot.Waiting == LOW) {
        hugobot.State = STATE_WAITING;
      }

      break;
    case STATE_ENEMY_DANGER_1:
      if (hugobot.DojoLines < DOJO_LINES_LIMIT) {
        hugobot.State = STATE_DOJO_DANGER;
      }
      else {
        if (hugobot.DistanceToEnemy > ENEMY_RANGE_MM) {
          hugobot.State = STATE_SEARCHING;
        }
      }

      if (hugobot.Waiting == LOW) {
        hugobot.State = STATE_WAITING;
      }

      if (hugobot.State == STATE_ENEMY_DANGER_1) {
        hugobot.State = STATE_ENEMY_DANGER_2;
      }

      break;
    case STATE_ENEMY_DANGER_2:
      if (hugobot.DojoLines < DOJO_LINES_LIMIT) {
        hugobot.State = STATE_DOJO_DANGER;
      }
      else {
        if (hugobot.DistanceToEnemy > ENEMY_RANGE_MM) {
          hugobot.State = STATE_SEARCHING;
        }
      }

      if (hugobot.Waiting == LOW) {
        hugobot.State = STATE_WAITING;
      }

      if (hugobot.State == STATE_ENEMY_DANGER_2) {
        hugobot.State = STATE_ENEMY_DANGER_1;
      }

      break;
    case STATE_SEARCHING:
      if (hugobot.DojoLines < DOJO_LINES_LIMIT) {
        hugobot.State = STATE_DOJO_DANGER;
      }
      else {
        if (hugobot.DistanceToEnemy <= ENEMY_RANGE_MM) {
          hugobot.State = STATE_ENEMY_DANGER_1;
        }
      }

      if (hugobot.Waiting == LOW) {
        hugobot.State = STATE_WAITING;
      }

      break;
    case STATE_WAITING:
      if (hugobot.Waiting != LOW) {
        delay(4950);
        hugobot.State = STATE_SEARCHING;
      }
    default:
      break;
  }
}

void move_motors() {
  Serial.println(hugobot.State);
  switch (hugobot.State) {
    case STATE_DOJO_DANGER:
      hugobot.MotorLeft = -200;
      hugobot.MotorRight = 200;
      break;
    case STATE_ENEMY_DANGER_1:
      hugobot.MotorLeft = 210;
      hugobot.MotorRight = 210;
      break;
    case STATE_ENEMY_DANGER_2:
      hugobot.MotorLeft = 210;
      hugobot.MotorRight = 210;
      break;
    case STATE_SEARCHING:
      if (hugobot.Mode == 0) {
        hugobot.MotorLeft = -150;
        hugobot.MotorRight = 150;
      }
      else {
        hugobot.MotorLeft = 150;
        hugobot.MotorRight = -150;
      }
      break;
    case STATE_WAITING:
      hugobot.MotorLeft = 0;
      hugobot.MotorRight = 0;
      break;
    default:
      break;
  }

  if (hugobot.MotorLeft >= 0) {
    analogWrite(MotorLeftA, hugobot.MotorLeft);
    analogWrite(MotorLeftB, 0);
  }
  else {
    analogWrite(MotorLeftA, 0);
    analogWrite(MotorLeftB, -(hugobot.MotorLeft));
  }

  if (hugobot.MotorRight >= 0) {
    analogWrite(MotorRightA, hugobot.MotorRight);
    analogWrite(MotorRightB, 0);
  }
  else {
    analogWrite(MotorRightA, 0);
    analogWrite(MotorRightB, -(hugobot.MotorRight));
  }
}

void switch_mode() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500)
  {
    ToggleMode = !ToggleMode;
    hugobot.Mode = ToggleMode;
  }
  last_interrupt_time = interrupt_time;
}

