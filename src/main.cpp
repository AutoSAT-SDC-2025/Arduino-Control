#include "sbus.h"
#include "AA_MCP2515.h"
#include <VescUart.h>
#include <AccelStepper.h>
#include <Arduino.h>
#include <Servo.h>


/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1, false);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1, false);
/* SBUS data */
bfs::SbusData Sbus_Rx_Data;

#define STEP_PIN 23
#define DIR_PIN 22
#define ENDSTOP_PIN 24

#define ENDSTOP_BRAKE_MIN_PIN 22
#define ENDSTOP_BRAKE_MAX_PIN 24
#define REVERSE_PIN 26
#define PWM_PIN 3

#define EMERGENCY_STOP_PIN 29

VescUart UART;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

Servo esc;  // create servo object to control the ESC

/*
MCP2515 CAN Controller pin out
SPI CS      (CS)    pin 53
SPI Clock   (SCK)   pin 52
SPI MOSI    (SI)    pin 51
SPI MISO    (SO)    pin 50
Interrupt   (INT)   pin  2
*/
const CANBitrate::Config CAN_BITRATE = CANBitrate::Config_16MHz_500kbps;
const uint8_t CAN_PIN_CS = 53;
const int8_t CAN_PIN_INT = 2;

CANConfig config(CAN_BITRATE, CAN_PIN_CS, CAN_PIN_INT);
CANController CAN(config);

enum ID {
  CONTROLE_COMAND = 0x111,
  CONTROLE_MODE_SETTING_COMAND = 0x421,
  STATUS_SETTING_COMMAND = 0x441,
  PARKING_CONTROLE_COMMAND = 0x131,
  MOVEMENT_CONTROL_FEEDBACK_COMMAND = 0x221
};

const unsigned long DEBOUNCE_DELAY = 50;      // ms
bool  EmergencyState        = HIGH;           // debounced level (HIGH = released)
bool  lastEmergencyReading  = HIGH;           // last raw read
unsigned long lastDebounceTime = 0;           // timer

bool CAN_Enable = false; // Enable steering over CAN bus
bool SBUS_Enable = false; // Enable steering over SBUS

double CAN_Steering_Angle = 0; // CAN bus steering Angle in rad
double CAN_Speed = 0; // CAN bus speed in m/s

double SBUS_Speed_Mode = 0;
double SBUS_Steering_Angle = 0; // s.bus steering angle in rad
double SBUS_Speed = 0; // s.bus speed in m/s
bool SBUS_EmergencyState = true;

double Set_Steering_Angle = 0; // the set steering angle in rad
double Set_Speed = 0; // the set speed in m/s
bool Set_Brake = true;

double Actual_Steering_Angle = 0; // the achtual steering angle in rad
double Actual_Speed = 0; // the actual speed in m/s

double Battery_Voltage = 0;
double Curent_Draw_Drive_Motor = 0;

double STEPS_PER_RAD = 1200 / 0.576; // the amount of steps the stepper needs for 1 rad of steering agle
double END_SWITCH_OFSET = -1200; // the amount of steps from the end switch to the zero point
double MPS_TO_RPM_FACTOR = 54.6; // Conversion factor for calculating the M/s to RPM

const long interval = 20;  // interval at which to blink (milliseconds)
unsigned long previousMillis = 0;

void setup() {
  pinMode(ENDSTOP_BRAKE_MIN_PIN, INPUT_PULLUP);  // set up end switch for braking min
  pinMode(ENDSTOP_BRAKE_MAX_PIN, INPUT_PULLUP);  // set up end switch for braking max

  pinMode(REVERSE_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);  // Pull-up resistor

  /* Serial to display data */
  Serial.begin(115200);
  Serial2.begin(115200);

  while (!Serial) {}

  UART.setSerialPort(&Serial2);

  /* Begin the SBUS communication */
  sbus_rx.Begin();

  pinMode(ENDSTOP_PIN, INPUT_PULLUP);  // set up end switch for steering
  stepper.setMaxSpeed(3200000);
  stepper.setAcceleration(600000);

  // Move toward the switch slowly until it's hit
  stepper.setSpeed(-100); // Move in reverse
  while (digitalRead(ENDSTOP_PIN) == LOW) {
    stepper.runSpeed();  // Move until switch is hit
  }

  // Stop and set current position as ofset from zero
  stepper.setCurrentPosition(END_SWITCH_OFSET);

  // test contection to CAN bus controler
  while(CAN.begin(CANController::Mode::Normal) != CANController::OK) {
    Serial.println("CAN begin FAIL - delaying for 1 second");
    delay(1000);
  }
  Serial.println("CAN begin OK");

  esc.attach(4);  // ESC signal wire connected to pin 4
  esc.writeMicroseconds(1000);  // send low throttle to arm ESC
}

void loop() {
  if (sbus_rx.Read()) {
    Sbus_Rx_Data = sbus_rx.data();
    CAN_Enable = (Sbus_Rx_Data.ch[4] > 1000);
    SBUS_Enable = (Sbus_Rx_Data.ch[4] == 992);
    SBUS_EmergencyState = (Sbus_Rx_Data.ch[5] < 990);

    SBUS_Speed_Mode = (Sbus_Rx_Data.ch[6] - 10.0) / 819.0;

    SBUS_Steering_Angle = (Sbus_Rx_Data.ch[0] - 992.0) / 819.0 * 0.576;
    SBUS_Speed = (Sbus_Rx_Data.ch[1] - 992.0) / 819.0 * 1.5;
  }

  CANFrame frame_in;
  if (CAN.read(frame_in) == CANController::IOResult::OK) {
    switch (frame_in.getId()) {
      case ID::CONTROLE_COMAND:
        uint8_t CAN_Bus_Data_In[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        frame_in.getData(CAN_Bus_Data_In, sizeof(CAN_Bus_Data_In));

        CAN_Speed = (double)((int16_t)((CAN_Bus_Data_In[0] << 8) | CAN_Bus_Data_In[1])) / 1000.0;
        CAN_Steering_Angle = (double)((int16_t)((CAN_Bus_Data_In[6] << 8) | CAN_Bus_Data_In[7])) / 1000.0;

        Serial.print("Steering angle = ");
        Serial.print(CAN_Steering_Angle, 3);
        Serial.print("Rad   Speed = ");
        Serial.print(CAN_Speed, 3);
        Serial.println("m/s");

        // code block
        break;
      case ID::CONTROLE_MODE_SETTING_COMAND:
        // code block
        break;
      default:
        // code block
        break;
    }

  }

  bool raw = !digitalRead(EMERGENCY_STOP_PIN);

  if (raw != lastEmergencyReading) {
    lastDebounceTime = millis();                 // reset timer on any edge
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // reading stayed stable long enough → accept it
    if (raw != EmergencyState) {
      EmergencyState = raw;
    }
  }
  lastEmergencyReading = raw;

  if (EmergencyState || SBUS_EmergencyState){
    Set_Steering_Angle = SBUS_Steering_Angle;
    Set_Speed = 0;
    Set_Brake = true;
  }
  else if (SBUS_Enable){
    Set_Steering_Angle = SBUS_Steering_Angle;
    Set_Speed = SBUS_Speed;
    Set_Brake = false;
  }
  else if (CAN_Enable){
    Set_Steering_Angle = CAN_Steering_Angle;
    Set_Speed = CAN_Speed;
    Set_Brake = false;
  }
  else {
    Set_Steering_Angle = 0;
    Set_Speed = 0;
    Set_Brake = true;
  }

  if (digitalRead(ENDSTOP_BRAKE_MIN_PIN) == LOW && Set_Brake) {
    digitalWrite(REVERSE_PIN, HIGH);
    analogWrite(PWM_PIN, 230);
  }
  else if (digitalRead(ENDSTOP_BRAKE_MAX_PIN) == LOW && !Set_Brake) {
    digitalWrite(REVERSE_PIN, LOW);
    analogWrite(PWM_PIN, 230);
  }
  else {
    analogWrite(PWM_PIN, 0);
  }

  Actual_Steering_Angle = stepper.currentPosition() / STEPS_PER_RAD;
  stepper.moveTo(Set_Steering_Angle*STEPS_PER_RAD);
  stepper.run();

  esc.writeMicroseconds(1000 + (Set_Speed * MPS_TO_RPM_FACTOR * SBUS_Speed_Mode));

  /* USE IF SPEED CONTROLER CONECTED VIA UART/SERIAL
  if ( UART.getVescValues() ) {
    Actual_Speed = UART.data.rpm / MPS_TO_RPM_FACTOR;
    Battery_Voltage = UART.data.inpVoltage;
    Curent_Draw_Drive_Motor = UART.data.ampHours;
  }
  UART.setRPM(Set_Speed*MPS_TO_RPM_FACTOR);
  */

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    uint8_t CAN_Bus_Data_Out[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    int16_t speed_int = (int16_t)(Actual_Speed * 1000.0);
    int16_t angle_int = (int16_t)(Actual_Steering_Angle * 1000.0);

    CAN_Bus_Data_Out[0] = (uint8_t)((speed_int >> 8) & 0xFF); // High byte of speed
    CAN_Bus_Data_Out[1] = (uint8_t)(speed_int & 0xFF);        // Low byte of speed

    CAN_Bus_Data_Out[6] = (uint8_t)((angle_int >> 8) & 0xFF); // High byte of steering angle
    CAN_Bus_Data_Out[7] = (uint8_t)(angle_int & 0xFF);        // Low byte of steering angle

    CANFrame frame_out(ID::MOVEMENT_CONTROL_FEEDBACK_COMMAND, CAN_Bus_Data_Out, sizeof(CAN_Bus_Data_Out));
    CAN.write(frame_out);
    frame_out.print("CAN TX");
  }
}