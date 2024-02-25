// This program controls the LEDs and BME 280 via manual control

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO08x.h>

// Define LED pin properties
const int blue_LED_pin = 4;
const int red_LED_pin = 5;
const int freq = 5000; // PWM frequency
const int ledChannel = 0; // PWM channel
const int resolution = 8; // PWM bit resolution

// Define Sea level reference pressure (hPa)
#define SEALEVELPRESSURE_HPA (1013.25)

// Create BME object and check for identification
Adafruit_BME280 bme;

#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(blue_LED_pin, OUTPUT);
  pinMode(red_LED_pin, OUTPUT);
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(red_LED_pin, ledChannel);

  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  while (1);
  }
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
  Serial.println("Enter A Number between 0 and 8191: ");
}

void loop() {
  // Read command from serial monitor
  if (Serial.available() >0){
    int command = Serial.parseInt();
    while (Serial.available()>0){
      char t;
      t = Serial.read();
    }
    Serial.print("Command Entered: ");
    Serial.println(command);
    if ((command < 0) || (command> 8191)){
      Serial.println("ENTER A NUMBER FROM 0 TO 8191");
    }
    else{
      if(bitRead(command,0)){
        Serial.print("Temperature = ");
        Serial.print(bme.readTemperature());
        Serial.println(" *C");
      }

      if(bitRead(command,1)){
        Serial.print("Humidity = ");
        Serial.print(bme.readHumidity());
        Serial.println(" %");
      }

      if(bitRead(command,2)){
        Serial.print("Pressure = ");
        Serial.print(bme.readPressure());
        Serial.println(" hPa");
      }

      if(bitRead(command,3)){
        Serial.print("Altitude = ");
        Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(" m");
      }

      int blue_LED_bit = bitRead(command,4);
      int red_LED_value = command >> 5;
      digitalWrite(blue_LED_pin, blue_LED_bit);
      ledcWrite(ledChannel, red_LED_value);
      Serial.print("Blue LED Value: ");
      Serial.println(blue_LED_bit);
      Serial.print("Red LED Value: ");
      Serial.println(red_LED_value);
      Serial.println("");
      Serial.flush();

        if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
   
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);
  }
    }
  Serial.println("Enter A Number between 0 and 8191: ");
  }
}