#include <Servo.h>
#include <EEPROM.h>


#define LED_PIN 9

#define SHOCK_0_PIN 32
#define SHOCK_1_PIN 47
#define SHOCK_2_PIN 45

#define TURNTABLE_ENABLE_PIN 6
#define TURNTABLE_A_PIN 5
#define TURNTABLE_B_PIN 4
#define TURNTABLE_SENSOR_PIN 21
#define TURNTABLE_STRIPES 64


enum SHOCK_STATE {
  OUTSIDE,
  SHOCKING,
  PAUSE
} shock;
unsigned long shock_change;
unsigned shock_length = 0;
unsigned pause_length = 0;
boolean shock_level_changed = false;
int shock_level = 0;

#define TURNTABLE_PID_FREQUENCY 100
unsigned long pid_change = 0;
int pConstant, iConstant, dConstant, previous_error, integral = 0;
#define TURNTABLE_MEAN_LENGTH 5
unsigned long turntable_change = 0;
unsigned turntable_deltas[TURNTABLE_MEAN_LENGTH];
unsigned target_delta;
int turntable_index = 0;

void update_speed(){
  if (turntable_change != 0){
    unsigned long now = millis();
    turntable_deltas[turntable_index] = now - turntable_change;
    turntable_index = (turntable_index+1)%TURNTABLE_MEAN_LENGTH;
    turntable_change = now;
  } else {
    turntable_change = millis();
  }
}

void write_shock(int mA){
  digitalWrite(SHOCK_0_PIN, (mA & 1 << 0 ? HIGH : LOW));
  digitalWrite(SHOCK_1_PIN, (mA & 1 << 1 ? HIGH : LOW));
  digitalWrite(SHOCK_2_PIN, (mA & 1 << 2 ? HIGH : LOW));
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(TURNTABLE_SENSOR_PIN), update_speed, RISING);
  pinMode(TURNTABLE_ENABLE_PIN, OUTPUT);
  pinMode(TURNTABLE_A_PIN, OUTPUT);
  pinMode(TURNTABLE_B_PIN, OUTPUT);
  pinMode(TURNTABLE_SENSOR_PIN, INPUT);
  pinMode(SHOCK_0_PIN, OUTPUT);
  pinMode(SHOCK_1_PIN, OUTPUT);
  pinMode(SHOCK_2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(21, OUTPUT);

  pConstant = (int) EEPROM.read(0) + ((int) EEPROM.read(1) << 8);
  iConstant = (int) EEPROM.read(2) + ((int) EEPROM.read(3) << 8);
  dConstant = (int) EEPROM.read(4) + ((int) EEPROM.read(5) << 8);

  shock = SHOCKING;

  Serial.begin(9600);
}

void loop() {
  // Shock handling
  if (shock_level_changed){
    shock_level_changed = false;
    if (shock == SHOCKING){
      write_shock(shock_level);
    }
  }
  if (shock == SHOCKING){
    if (millis() >= shock_change + shock_length){
      shock_change = millis();
      write_shock(0);
      shock = PAUSE;
    }
  } else if (shock == PAUSE){
    if (millis() >= shock_change + pause_length){
      shock_change = millis();
      write_shock(shock_level);
      shock = SHOCKING;
    }
  }

  // Motor PID control
  if (pid_change != 0 && millis() > pid_change + TURNTABLE_PID_FREQUENCY){
    pid_change = millis();

    unsigned average_delta = 0;
    for (int i = 0; i < TURNTABLE_MEAN_LENGTH; i++){
      average_delta += turntable_deltas[i] / TURNTABLE_MEAN_LENGTH;
    }
    
    int error = target_delta - average_delta;
    integral = integral + error * TURNTABLE_PID_FREQUENCY;
    int derivative = (error - previous_error) / TURNTABLE_PID_FREQUENCY;
    analogWrite(TURNTABLE_ENABLE_PIN, pConstant/10000.0*error + iConstant/10000.0*integral + dConstant/10000.0*derivative); 
    previous_error = error;
  }

  // Serial handling
  if (Serial.available()){
    uint8_t message[3];
    uint8_t bytesRead = Serial.readBytes(message, 3);
    if (bytesRead != 3){
      Serial.print("IM!"); // Incorrect message
    } else {
      uint8_t command = message[0];
      int arg = message[1] + ((int) message[2] << 8);
      switch (command){
        case 10: // CONNECTION CHECK
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.print("OK.");
          delay(100);
          digitalWrite(LED_BUILTIN, LOW);
          break;
        case 11: // SHUT-OFF
          write_shock(0);
          shock = OUTSIDE;

          digitalWrite(LED_PIN, LOW);
          Serial.print("ZZ.");
          break;
        case 12: // DEBUG ECHO
          Serial.write(arg);
          Serial.print(".");
          break;
        case 20: // SHOCK LEVEL
          if (shock_length != 0 && pause_length != 0){
            if (arg == 0){
              write_shock(0);
              shock = OUTSIDE;
              Serial.print("YA.");
            } else if (arg > 0 && arg < 8){
              shock_level = arg;
              if (shock == SHOCKING){
                shock_change = millis();
                shock_level_changed = true;
                Serial.print("YA.");
              } else if (shock == OUTSIDE){
                shock_change = millis();
                write_shock(shock_level);
                shock = SHOCKING;
                Serial.print("YA.");
              }
            } else {
              Serial.print("IA!");
            }
          } else {
            Serial.print("IS!");
          }
          break;
        case 21: // SHOCK LENGTH
          shock_length = arg;
          Serial.print("YA.");
          break;
        case 22: // PAUSE LENGTH
          pause_length = arg;
          Serial.print("YA.");
          break;
        case 30: // LED
          if (arg == 0){
            digitalWrite(LED_PIN, LOW);
            Serial.print("YA.");
          } else if (arg == 1){
            digitalWrite(LED_PIN, HIGH);
            Serial.print("YA.");
          } else {
            Serial.print("IA!");
          }
          break;
        case 40: // TURNTABLE SPEED
          target_delta = arg*1000 / TURNTABLE_STRIPES;
          pid_change = 1;
          Serial.print("YA.");
        case 41: // TURNTABLE DIRECTION
          switch (arg){
            case 0:
              digitalWrite(TURNTABLE_A_PIN, LOW);
              digitalWrite(TURNTABLE_B_PIN, LOW);
              Serial.print("YA.");
            case 1:
              digitalWrite(TURNTABLE_A_PIN, HIGH);
              digitalWrite(TURNTABLE_B_PIN, LOW);
              Serial.print("YA.");
            case 2:
              digitalWrite(TURNTABLE_A_PIN, LOW);
              digitalWrite(TURNTABLE_B_PIN, HIGH);
              Serial.print("YA.");
            default:
              Serial.print("IA!");
          }
        case 42: // UPDATE P CONSTANT
          EEPROM.update(0, message[1]);
          EEPROM.update(1, message[2]);
          Serial.print("YA.");
          pConstant = arg;
          break;
        case 43: // UPDATE I CONSTANT
          EEPROM.update(2, message[1]);
          EEPROM.update(3, message[2]);
          iConstant = arg;
          Serial.print("YA.");
        case 44: // UPDATE D CONSTANT
          EEPROM.update(4, message[1]);
          EEPROM.update(5, message[2]);
          dConstant = arg;
          Serial.print("YA.");
        case 45: // SET PWM DIRECTLY
          pid_change = 0;
          analogWrite(TURNTABLE_ENABLE_PIN, arg);
          Serial.print("YA.");
        default: // UNKNOWN COMMAND
          Serial.print("UC!");
      }
    }
  }
}
