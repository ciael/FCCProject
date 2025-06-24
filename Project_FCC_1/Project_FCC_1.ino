// Relay CH1 = lampu standby (kuning)
// Relay CH2 = lampu run (hijau)
// Relay CH3 = lampu emergency (merah)
// Relay CH4 = relay FC1
// Relay CH5 = relay FC2
// Relay CH6 = relay power 200A
// Relay CH7 = DC-DC FC1
// Relay CH8 = DC-Dc FC2
#include  <Arduino_PortentaMachineControl.h> // Library for PMC 
#include "mbed.h"  // Required for Ticker
using namespace mbed;
// Address Transmit CAN 
static uint32_t const CAN_ID1 = 0x18F11381;
static uint32_t const CAN_ID2 = 0x18F11481;
const float RES_DIVIDER = 0.28057;
const float REFERENCE = 3.0;
// Pin Pembacaan Sensor Analog PMC 
const int pressureLow1 = 0;
const int currentSensor = 1;
const int voltageSensor = 2;
//Pin Digital Output PMC 
const int standbyLamp = 0;
const int runLamp = 1;
const int emerLamp = 2;
const int relayFC1 = 3;
const int relayFC2 = 4;
const int powerRelay = 5;
const int dcFC1 = 6;
const int dcFC2 = 7;






void setup() {
  Serial.begin(115200);
  MachineControl_AnalogIn.begin(SensorType::V_0_10);
  MachineControl_DigitalOutputs.begin();
  MachineControl_AnalogOut.begin();
  MachineControl_DigitalOutputs.write(standbyLamp, LOW);
  MachineControl_DigitalOutputs.write(runLamp, LOW);
  MachineControl_DigitalOutputs.write(emerLamp, LOW);
  MachineControl_DigitalOutputs.write(relayFC1, LOW);
  MachineControl_DigitalOutputs.write(relayFC2, LOW);
  MachineControl_DigitalOutputs.write(powerRelay, LOW);
  MachineControl_DigitalOutputs.write(dcFC1, LOW);
  MachineControl_DigitalOutputs.write(dcFC2, LOW);
  MachineControl_RS485Comm.begin(9600,0,500);
  MachineControl_RS485Comm.receive();


  if (!MachineControl_CANComm.begin(CanBitRate::BR_250k)) {
    Serial.println("CAN init failed.");
    while (1);
  }
  Serial.println("CAN Ready.");
}

 

}

void loop() {
  // put your main code here, to run repeatedly:

}


// Function untuk membaca sensor pressure LOW 1
float bacaPressureLow1(){
  float raw = MachineControl_AnalogIn.read(pressureLow1);
  float voltage = (raw*REFERENCE)/65535.0/RES_DIVIDER;
  voltage = constrain(voltage,0.5,4.5);
  return (voltage-0.5f)*0.25f;
}
// Function untuk membaca sensor ARUS
float bacaCurrentSensor(){
  float raw = MachineControl_AnalogIN.read(currentSensor);
  float voltage = (raw*REFERENCE)/65535.0/RES_DIVIDER;
  voltage = constrain(voltage,0.0,10.0);
  return (voltage-0.0f)*20.0f;
}
//Function untuk membaca sensor VOLTAGE
float bacaVoltageSensor(){
  float raw = MachineControl_AnalogIN.read(voltageSensor);
  float voltage = (raw*REFERENCE)/65535.0/RES_DIVIDER;
  voltage = constrain(voltage,0.0,10.0);
  return (voltage-0.0f)*30.0f;
}
//Function untuk menerima sensor pressure dari NANO
float readPressureFromRS485() {
  if (MachineControl_RS485Comm.available()) {
    String data = MachineControl_RS485Comm.readStringUntil('\n');
    data.trim();  // Remove leading/trailing whitespace

    if (data.startsWith("PRESSURE:")) {
      float value = data.substring(9).toFloat();  // Extract value after "PRESSURE:"
      if (value >= 0.0 && value <= 250.0) {  // Valid range check
        Serial.print("Received via RS485 → PRESSURE: ");
        Serial.println(value);
        return value;
      } else {
        Serial.println("Invalid pressure value received.");
      }
    } else {
      Serial.print("Unexpected RS485 message: ");
      Serial.println(data);
    }
  }
  return -1.0;  // Return -1.0 if no valid pressure received
}