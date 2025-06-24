/*
  Portenta Machine Control Analog Input Example
  Custom: Read from Verivolt IsoBlock on Channel 0
  Sensor Output: 0–10V corresponds to 0–300VDC input
*/

#include <Arduino_PortentaMachineControl.h>

// IsoBlock outputs 0–10V for 0–300VDC
const float VDC_MAX = 300.0;                    // Maximum sensed voltage
const float VOUT_MAX = 10.0;                    // Maximum output voltage from sensor
const float SCALE_FACTOR = VDC_MAX / VOUT_MAX;  // 300/10 = 30
float actual_vdc;
// Portenta Machine Control's analog input uses resistor divider internally
const float RES_DIVIDER = 0.28057;
const float REFERENCE = 3.0;  // 3.0V reference

void setup() {
  Serial.begin(9600);

  // Initialize analog input in 0–10V mode
  MachineControl_AnalogIn.begin(SensorType::V_0_10);
}

void loop() {

  float sensor_voltage_sum = 0;

  for (int i = 0; i < 30; i++) {
    float raw_adc = MachineControl_AnalogIn.read(0);
    float sensor_voltage = (raw_adc * REFERENCE) / 65535.0 / RES_DIVIDER;
    sensor_voltage_sum += sensor_voltage;
    delay(20);
  }

  float sensor_voltage = sensor_voltage_sum / 30;

  actual_vdc = (1 / (0.957) * (sensor_voltage * SCALE_FACTOR)) - 0.9958;

  if(sensor_voltage < 1.792){
    actual_vdc = (1/(0.986)*actual_vdc) - 0.742;
  }

  Serial.print("CH0 Sensor Voltage: ");
  Serial.print(sensor_voltage, 3);
  Serial.print(" V -> ");
  Serial.print("Measured Voltage: ");
  Serial.print(actual_vdc, 2);
  Serial.println(" VDC");

  delay(500);
}
