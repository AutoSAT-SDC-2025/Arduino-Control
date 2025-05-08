/*
  CAN Receive Example

  This will setup the CAN controller(MCP2515) to receive CAN frames.
  Received frames will be printed to the Serial port.

  MIT License
  https://github.com/codeljo/AA_MCP2515
*/

#include "sbus.h"
#include "AA_MCP2515.h"
#include <AccelStepper.h>
#include <Arduino.h>

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1, false);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1, false);
/* SBUS data */
bfs::SbusData data;

#define STEP_PIN 23
#define DIR_PIN 22
#define ENDSTOP_PIN 24

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

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

bool CAN_Enable = false;
bool SBUS_Enable = false;

double CAN_Steering_Angle = 0;
double CAN_Speed = 0;

double SBUS_Steering_Angle = 0;
double SBUS_Speed = 0;

double Set_Steering_Angle = 0;
double Set_Speed = 0;

double Actual_Steering_Angle = 0;
double Actual_speed = 0;

double STEPPER_RANGE = 1000;
double END_SWITCH_OFSET = -1000;

const long interval = 20;  // interval at which to blink (milliseconds)
unsigned long previousMillis = 0;


void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();

  pinMode(ENDSTOP_PIN, INPUT_PULLUP);  // Pull-up resistor
  stepper.setMaxSpeed(3200000);            // adjust as needed
  stepper.setAcceleration(600000);

  // Move toward the switch slowly until it's hit
  stepper.setSpeed(-100); // Move in reverse
  while (digitalRead(ENDSTOP_PIN) == LOW) {
    stepper.runSpeed();  // Move until switch is hit
  }

  // Stop and set current position as ofset from zero
  stepper.setCurrentPosition(END_SWITCH_OFSET);

  while(CAN.begin(CANController::Mode::Normal) != CANController::OK) {
    Serial.println("CAN begin FAIL - delaying for 1 second");
    delay(1000);
  }
  Serial.println("CAN begin OK");
}

void loop() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    CAN_Enable = (data.ch[4] < 1000);
    SBUS_Enable = (data.ch[4] == 1500);


    SBUS_Steering_Angle = (data.ch[0] - 991.0) / 819.0 * STEPPER_RANGE;
    SBUS_Speed = (data.ch[0] - 991.0) / 819.0 * STEPPER_RANGE;
  }

  CANFrame frame_in;
  if (CAN.read(frame_in) == CANController::IOResult::OK) {
    switch (frame_in.getId()) {
      case ID::CONTROLE_COMAND:
        uint8_t data_in[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        frame_in.getData(data_in, sizeof(data_in));

        CAN_Speed = (double)((int16_t)((data_in[0] << 8) | data_in[1])) / 1000.0;
        CAN_Steering_Angle = (double)((int16_t)((data_in[6] << 8) | data_in[7])) / 1000.0;

        Serial.print("Rteering angle = ");
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

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    uint8_t data_out[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    int16_t speed_int = (int16_t)(Actual_speed * 1000.0);
    int16_t angle_int = (int16_t)(Actual_Steering_Angle * 1000.0);

    data_out[0] = (uint8_t)((speed_int >> 8) & 0xFF); // High byte of speed
    data_out[1] = (uint8_t)(speed_int & 0xFF);        // Low byte of speed

    data_out[6] = (uint8_t)((angle_int >> 8) & 0xFF); // High byte of steering angle
    data_out[7] = (uint8_t)(angle_int & 0xFF);        // Low byte of steering angle

    CANFrame frame_out(ID::MOVEMENT_CONTROL_FEEDBACK_COMMAND, data_out, sizeof(data_out));
    CAN.write(frame_out);
    frame_out.print("CAN TX");
  }

  

  if (SBUS_Enable){
    Set_Steering_Angle = SBUS_Steering_Angle;
    Set_Speed = SBUS_Speed;
  }
  else if (CAN_Enable){
    Set_Steering_Angle = CAN_Steering_Angle;
    Set_Speed = CAN_Speed;
  }
  else {
    Set_Steering_Angle = 0;
    Set_Speed = 0;
  }




  stepper.moveTo(Set_Steering_Angle*5000);
  stepper.run();



}