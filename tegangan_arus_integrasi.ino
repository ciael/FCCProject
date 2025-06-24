#include <Arduino_PortentaMachineControl.h>

//set up sensor arus
// Current transducer scaling factor: 0-10V corresponds to 0-200A
const float VOLTAGE_TO_CURRENT = 20.0;  // A per volt
float current;

//set up sensor tegangan
// Verivolt IsoBlock outputs 0–10V for 0–300VDC
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
  sensor_tegangan();
  sensor_arus();
}

void sensor_arus() {
  float raw_adc_c = MachineControl_AnalogIn.read(1);
  float sensor_voltage_c = (raw_adc_c * REFERENCE) / 65535.0 / RES_DIVIDER;
  current = sensor_voltage_c * VOLTAGE_TO_CURRENT;
}

void sensor_tegangan() {
  float sensor_voltage_sum = 0;

  for (int i = 0; i < 30; i++) {
    float raw_adc = MachineControl_AnalogIn.read(2);  //PIN 2 untuk sensor tegangan
    float sensor_voltage = (raw_adc * REFERENCE) / 65535.0 / RES_DIVIDER;
    sensor_voltage_sum += sensor_voltage;
    delay(20);
  }

  float sensor_voltage = sensor_voltage_sum / 30;

  actual_vdc = (1 / (0.957) * (sensor_voltage * SCALE_FACTOR)) - 0.9958;

  if (sensor_voltage < 1.792) {
    actual_vdc = (1 / (0.986) * actual_vdc) - 0.742;
  }
}