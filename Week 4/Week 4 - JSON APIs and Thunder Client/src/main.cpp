// Import Libraries
#include <Arduino.h>
#include <WiFi.h> 
#include <WebServer.h>
#include <Arduino_JSON.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// Define constants
const int blue_LED_pin=4;
const int red_LED_pin=5;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

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

void handle_OnConnect() {
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Reading events");
  Serial.println("Getting BME280 Measurements");

  LED_request = LOW;
  digitalWrite(blue_LED_pin, LOW);
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
  doc["Temperature"] = bme.readTemperature();
  doc["Humidity"] = bme.readHumidity();
  doc["Pressure"] = bme.readPressure()/1000;
  doc["Altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);

  delay(2000);
  server.send(200, "application/json", JSON.stringify(doc));
}

void handle_led_change()
  {
  Serial.println("Changing blue LED status");
  String LED_status;
  LED_request = !LED_request;
  digitalWrite(blue_LED_pin, LED_request);
  if (LED_request){
    LED_status = "ON";
  }
  else{
    LED_status = "OFF";
  }
  
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
  doc["Temperature"] = bme.readTemperature();
  doc["Humidity"] = bme.readHumidity();
  doc["Pressure"] = bme.readPressure()/1000;
  doc["Altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
  doc["Red_LED"] = "ON";
  doc["Blue_LED"] = LED_status;
  delay(2000);
  server.send(200, "application/json", JSON.stringify(doc));
}

void set_LED_value_post(){
  Serial.println("Setting Values of LEDs");
  Serial.println(server.hasArg("plain"));
  String body = server.arg("plain");
  Serial.println(body);
  JSONVar myObject = JSON.parse(body);

  if (myObject.hasOwnProperty("blueled")){
    digitalWrite(blue_LED_pin,(int) myObject["blueled"]);
    }
    
  if (myObject.hasOwnProperty("redled")){
    ledcWrite(ledChannel,(int) myObject["redled"]);
    }
    
  server.send(200,"application/json",JSON.stringify(myObject));
  }

void set_LED_value_get(){
  Serial.println("Setting Value of LEDs via query parameters");
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
  doc["Temperature"] = bme.readTemperature();
  doc["Humidity"] = bme.readHumidity();
  doc["Pressure"] = bme.readPressure()/1000;
  doc["Altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
  if (server.hasArg("blueled")){
    String b=server.arg("blueled");
    doc["Blue_LED"]= b.toInt();
    digitalWrite(blue_LED_pin, b.toInt());  
  }

  if (server.hasArg("redled")){
    String r=server.arg("redled");
    doc["Red_LED"]= r.toInt();
    ledcWrite(ledChannel,r.toInt());
  }
  delay(2000);
  server.send(200, "application/json", JSON.stringify(doc));
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(blue_LED_pin, OUTPUT);
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(red_LED_pin, ledChannel);
  bool status;
    status = bme.begin(0x76);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    }

  Serial.print("Connecting to ");
  Serial.println(ssid);

  //Connect to your local wifi network
  WiFi.begin(ssid, password);
  delay(100);

  //check wifi is connected to wifi network
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("ESP32 Network Address: ");
  Serial.println(WiFi.localIP());
  
  server.on("/", handle_OnConnect);
  server.on("/ledchange", handle_led_change);
  server.on("/led_set_get", HTTP_GET, set_LED_value_get);
  server.on("/led_set_post",HTTP_POST,set_LED_value_post);
  server.onNotFound(handle_NotFound);
  
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
