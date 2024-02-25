// Import Libraries
#include <Arduino.h>
#include <WiFi.h> 
#include <WebServer.h>
#include <Arduino_JSON.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>


// Define constants
const int blue_LED_pin = 4;
const int red_LED_pin=5;
const int freq = 5000; // LED PWM frequency
const int ledChannel = 0; // LED Channel
const int resolution = 8; // LED PWM resolution

// Define Sea level reference pressure (hPa)
#define SEALEVELPRESSURE_HPA (1013.25)
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

// Pin Request Status
bool LED_request = LOW;

// Create object for DHT
Adafruit_BME280 bme; // DHT object

/* Put your SSID & Password */
const char* ssid = "Verizon_C4ZKVV";  // Enter SSID here
const char* password = "dna-blear7-bow";  //Enter Password here

WebServer server(80); // Web Server open on port 80

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
// Pin Request Status
bool LED_request = LOW;
bool dht_request = LOW;

// Create object for BME280
Adafruit_BME280 bme;

/* Put your SSID & Password */
const char* ssid = "Verizon_C4ZKVV";  // Enter SSID here
const char* password = "dna-blear7-bow";  //Enter Password here

WebServer server(80); // Web Server open on port 80

void handle_OnConnect() {
  LED_request = LOW;
  if (bno08x.getSensorEvent(&sensorValue)) {
  quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
  
  static long last = 0;
  long now = micros();
  long time = now - last;
  }
  JSONVar doc;
  doc["yaw"]= ypr.yaw;
  doc["pitch"] = ypr.pitch;
  doc["roll"] = ypr.roll;
  doc["id"]="Circuit 1";
  doc["Temperature"] = bme.readTemperature();
  doc["Humidity"] = bme.readHumidity();
  doc["Pressure"] = bme.readPressure()/1000;
  doc["Altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
  delay(1000);
  server.send(200, "application/json", JSON.stringify(doc));
}

void set_LEDs(){
  String body = server.arg("plain");
  JSONVar myObject = JSON.parse(body);
  digitalWrite(blue_LED_pin, (int) myObject["blue_led"]);
  ledcWrite(ledChannel,(int) myObject["red_led"]); // Set LED brightness
  server.send(200,"application/json",JSON.stringify(myObject));
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(blue_LED_pin, OUTPUT);
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
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(red_LED_pin, ledChannel);

  Serial.println("Connecting to ");
  Serial.println(ssid);

  //connect to your local wifi network
  WiFi.begin(ssid, password);
  delay(100);

  //check wifi is connected to wifi network
  while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("ESP32 Address on Local Network: ");  
  Serial.println(WiFi.localIP());
 
  server.on("/", handle_OnConnect);
  server.on("/led",HTTP_POST,set_LEDs);
  server.onNotFound(handle_NotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
