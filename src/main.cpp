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
  PARKING_CONTROLE_COMMAND = 0x131
};

bool CAN_Enable = false;
bool SBUS_Enable = false;

double CAN_Steering_Angle = 0;
double CAN_Speed = 0;

double SBUS_Steering_Angle = 0;
double SBUS_Speed = 0;

double STEPPER_RANGE = 1000;
double END_SWITCH_OFSET = -1000;



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


    StepperPosition = (data.ch[0] - 991.0) / 819.0 * STEPPER_RANGE;

  }

  CANFrame frame;
  if (CAN.read(frame) == CANController::IOResult::OK) {
    switch (frame.getId()) {
      case ID::CONTROLE_COMAND:
        uint8_t data_out[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        frame.getData(data_out, sizeof(data_out));
          
        CAN_Speed = (double)((int16_t)((data_out[0] << 8) | data_out[1])) / 1000.0;
        CAN_Steering_Angle = (double)((int16_t)((data_out[6] << 8) | data_out[7])) / 1000.0;

        Serial.print("Steering angle = ");
        Serial.print(CAN_Steering_Angle, 3);
        Serial.print("Rad   Speed = ");
        Serial.print(CAN_Speed, 3);
        Serial.println("m/s");

        stepper.moveTo(CAN_Steering_Angle*5000);
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
  

  if (SBUS_Enable){
    
  }
  else if (CAN_Enable){

  }
  else {

  }


  stepper.run();

}
