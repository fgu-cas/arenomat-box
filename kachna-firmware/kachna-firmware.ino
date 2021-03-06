#include <Servo.h>
#include <EEPROM.h>


#define LED_PIN 9

#define SHOCK_0_PIN 32
#define SHOCK_1_PIN 47
#define SHOCK_2_PIN 45

#define TURNTABLE_ENABLE_PIN 6
#define TURNTABLE_A_PIN 4
#define TURNTABLE_B_PIN 5
#define TURNTABLE_SENSOR_PIN 20
#define TURNTABLE_STRIPES 64

#define FEEDER_MOTOR_PIN 10
#define FEEDER_SENS_MECH 53
#define FEEDER_SENS_OPTO 39

enum {
  STANDBY,
  DROPPING,
  RUNNING,
  QUEUED
} FEEDER_STATE;

bool feederCheck() {
  return digitalRead(FEEDER_SENS_MECH) == 0;
}

long currentStripes = 0;

void update_pos() {
  currentStripes = (currentStripes + 1) % 65535;
}

#define TURNTABLE_PID_FREQUENCY 100
unsigned long pid_change = 0;
int pConstant, iConstant, dConstant, previous_error, integral = 0;
#define TURNTABLE_MEAN_LENGTH 5
unsigned long turntable_change = 0;
unsigned turntable_deltas[TURNTABLE_MEAN_LENGTH];
unsigned target_delta;
int turntable_index = 0;

void update_speed() {
  if (turntable_change != 0) {
    unsigned long now = millis();
    turntable_deltas[turntable_index] = now - turntable_change;
    turntable_index = (turntable_index + 1) % TURNTABLE_MEAN_LENGTH;
    turntable_change = now;
  } else {
    turntable_change = millis();
  }
}

void write_shock(int mA) {
  digitalWrite(SHOCK_0_PIN, (mA & 1 << 0 ? HIGH : LOW));
  digitalWrite(SHOCK_1_PIN, (mA & 1 << 1 ? HIGH : LOW));
  digitalWrite(SHOCK_2_PIN, (mA & 1 << 2 ? HIGH : LOW));
}

void setup() {
  //  attachInterrupt(digitalPinToInterrupt(TURNTABLE_SENSOR_PIN), update_speed, RISING);
  attachInterrupt(digitalPinToInterrupt(TURNTABLE_SENSOR_PIN), update_pos, RISING);
  pinMode(TURNTABLE_ENABLE_PIN, OUTPUT);
  pinMode(TURNTABLE_A_PIN, OUTPUT);
  pinMode(TURNTABLE_B_PIN, OUTPUT);
  pinMode(TURNTABLE_SENSOR_PIN, INPUT);
  pinMode(SHOCK_0_PIN, OUTPUT);
  pinMode(SHOCK_1_PIN, OUTPUT);
  pinMode(SHOCK_2_PIN, OUTPUT);
  pinMode(FEEDER_MOTOR_PIN, OUTPUT);
  pinMode(FEEDER_SENS_MECH, INPUT_PULLUP);
  pinMode(FEEDER_SENS_OPTO, INPUT);
  pinMode(LED_PIN, OUTPUT);

  pConstant = (int) EEPROM.read(0) + ((int) EEPROM.read(1) << 8);
  iConstant = (int) EEPROM.read(2) + ((int) EEPROM.read(3) << 8);
  dConstant = (int) EEPROM.read(4) + ((int) EEPROM.read(5) << 8);

  Serial.begin(9600);

  if (!feederCheck()) {
    digitalWrite(FEEDER_MOTOR_PIN, HIGH);
    FEEDER_STATE = RUNNING;
  } else {
    FEEDER_STATE = STANDBY;
  }
}

void loop() {
//  // Motor PID control
//  if (pid_change != 0 && millis() > pid_change + TURNTABLE_PID_FREQUENCY) {
//    pid_change = millis();
//
//    unsigned average_delta = 0;
//    for (int i = 0; i < TURNTABLE_MEAN_LENGTH; i++) {
//      average_delta += turntable_deltas[i];
//    }
//    average_delta /= TURNTABLE_MEAN_LENGTH;
//    int error = target_delta - average_delta;
//    integral = integral + error * TURNTABLE_PID_FREQUENCY;
//    int derivative = (error - previous_error) / TURNTABLE_PID_FREQUENCY;
//    analogWrite(TURNTABLE_ENABLE_PIN, pConstant / 10000.0 * error + iConstant / 10000.0 * integral + dConstant / 10000.0 * derivative);
//    previous_error = error;
//  }

  // Feeder logic
  switch (FEEDER_STATE) {
    case DROPPING:
      if (digitalRead(FEEDER_SENS_MECH) == 1) {
        FEEDER_STATE = RUNNING;
      }
      break;
    case QUEUED:
      if (feederCheck()) {
        FEEDER_STATE = RUNNING;
      }
      break;
    case RUNNING:
      if (feederCheck()) {
        digitalWrite(FEEDER_MOTOR_PIN, LOW);
        FEEDER_STATE = STANDBY;
      }
      break;
  }

  // Serial handling
  if (Serial.available()) {
    uint8_t message[3];
    uint8_t bytesRead = Serial.readBytes(message, 3);
    if (bytesRead != 3) {
      Serial.print("IM!"); // Incorrect message
    } else {
      uint8_t command = message[0];
      int arg = message[1] + ((int) message[2] << 8);
      switch (command) {
        case 10: // CONNECTION CHECK
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.print("OK.");
          delay(100);
          digitalWrite(LED_BUILTIN, LOW);
          break;
        case 11: // SHUT-OFF / RESET
          write_shock(0);
          analogWrite(TURNTABLE_ENABLE_PIN, 0);
          digitalWrite(TURNTABLE_A_PIN, LOW);
          digitalWrite(TURNTABLE_B_PIN, LOW);
          digitalWrite(LED_PIN, LOW);
          target_delta = 0;
          Serial.print("ZZ.");
          break;
        case 12: // DEBUG ECHO
          Serial.write(arg);
          Serial.print(".");
          break;
        case 20: // SHOCK LEVEL
          if (arg >= 0 && arg < 8) {
            write_shock(arg);
            Serial.print("YA.");
          } else {
            Serial.print("IA!");
          }
          break;
        case 30: // LED
          if (arg == 0) {
            digitalWrite(LED_PIN, LOW);
            Serial.print("YA.");
          } else if (arg == 1) {
            digitalWrite(LED_PIN, HIGH);
            Serial.print("YA.");
          } else {
            Serial.print("IA!");
          }
          break;
//        case 40: // TURNTABLE SPEED
//          target_delta = arg * 1000 / TURNTABLE_STRIPES;
//          pid_change = 1;
//          Serial.print("YA.");
//          break;
        case 41: // TURNTABLE DIRECTION
          switch (arg) {
            case 0:
              digitalWrite(TURNTABLE_A_PIN, LOW);
              digitalWrite(TURNTABLE_B_PIN, LOW);
              Serial.print("YA.");
              break;
            case 1:
              digitalWrite(TURNTABLE_A_PIN, HIGH);
              digitalWrite(TURNTABLE_B_PIN, LOW);
              Serial.print("YA.");
              break;
            case 2:
              digitalWrite(TURNTABLE_A_PIN, LOW);
              digitalWrite(TURNTABLE_B_PIN, HIGH);
              Serial.print("YA.");
              break;
            default:
              Serial.print("IA!");
          }
          break;
//        case 42: // UPDATE P CONSTANT
//          EEPROM.update(0, message[1]);
//          EEPROM.update(1, message[2]);
//          Serial.print("YA.");
//          pConstant = arg;
//          break;
//        case 43: // UPDATE I CONSTANT
//          EEPROM.update(2, message[1]);
//          EEPROM.update(3, message[2]);
//          iConstant = arg;
//          Serial.print("YA.");
//          break;
//        case 44: // UPDATE D CONSTANT
//          EEPROM.update(4, message[1]);
//          EEPROM.update(5, message[2]);
//          dConstant = arg;
//          Serial.print("YA.");
//          break;
        case 45: // SET PWM DIRECTLY
          pid_change = 0;
          analogWrite(TURNTABLE_ENABLE_PIN, arg);
          Serial.print("YA.");
          break;
        case 50: // FEEDER
          switch (FEEDER_STATE) {
            case STANDBY:
              digitalWrite(FEEDER_MOTOR_PIN, HIGH);
              FEEDER_STATE = DROPPING;
              break;
            case DROPPING:
            case RUNNING:
              FEEDER_STATE = QUEUED;
              break;
            case QUEUED:
              // do nothing, don't queue more food
              break;
          }
          Serial.print("YA.");
          break;
        case 60: // POSITION
          Serial.write(currentStripes & 0xff);
          Serial.write((currentStripes >> 8) & 0xff);
          Serial.print(".");
          break;
        default: // UNKNOWN COMMAND
          Serial.print("UC!");
      }
    }
  }
}
