// Import Libraries
#include <WiFi.h> // Wifi Library
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// Define constants
const int RED_LED = 5;
const int BLUE_LED = 4;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
#define BNO08X_RESET -1

// Define Sea level reference pressure (hPa)
#define SEALEVELPRESSURE_HPA (1013.25)

// Pin Request Status
// bool LED_request = LOW;
// bool dht_request = LOW;

// Create object for BME280
Adafruit_BME280 bme;

/* Put your Soft AP SSID & Password*/
const char* AP_ssid ="PetersESP32";
const char* AP_password = "12345678";

/* Put your Network SSID & Password */
const char* network_ssid = "SimonESP32"; 
const char* network_password = "12345678";

/* Put IP Address details for AP mode */
IPAddress local_ip(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 200);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80); // Web Server open on port 80 

String SendHTML(String t_string, String h_string,
                String p_string, String a_string, String yaw, String pitch, String roll) {
    String ptr = "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">"
    "<title>LED Control</title>"
    "<style>"
    "html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; }"
    "input.largerCheckbox { width: 20px; height: 20px; }"
    "body { margin-top: 50px; }"
    "div { text-align: center; }"
    "h1 { color: #444444; margin: 50px auto 30px; }"
    "h3 { color: #444444; margin-bottom: 50px; }"
    "p { font-size: 14px; color: #888; margin-bottom: 10px; }"
    "table, th, td {border: 1px solid black; border-collapse: collapse; }"
    "</style>"
    "</head>"
    "<body style=\"background-color:aquamarine\";>"
    "<div>"
    "<h1>LED Commander</h1>"
    "<form action=\"\" method=\"post\">"
    "<label for=\"bluetoggle\">Blue LED Toggle </label>"
    "<input type=\"checkbox\" class=\"largerCheckbox\" id=\"blue\" name=\"blueled\"><br><br>"
    "<label for=\"blue\">Red LED Value</label>"
    "<input type=\"text\" id=\"redtoggle\" name=\"redled\" value=\"\" maxlength=\"3\" size=\"2\"><br><br>"
    "<input type=\"submit\" value=\"Submit\">"
    "</form>"
    // ptr += "<p>" + t_string + "</p>"
    // "<p>" + h_string + "</p>"
    // "<p>" + p_string + "</p>"
    // "<p>" + a_string + "</p>"
    "<table>"
    "<tr>"
    "<th>Characteristic</th>"
    "<th>BME280 Readings</th>"
    "</tr>"
    "<tr>"
    "<td>Temperature</td>"
    "<td>" + t_string + "</td>"
    "</tr>"
    "<tr>"
    "<td>Humidity</td>"
    "<td>" + h_string + "</td>"
    "</tr>"
    "<tr>"
    "<td>Pressure</td>"
    "<td>" + p_string + "</td>"
    "</tr>"
    "<tr>"
    "<td>Altitude</td>"
    "<td>" + a_string + "</td>"
    "</tr>"
    "</table>"
    "<table>"
    "<tr>"
    "<th>Characteristic</th>"
    "<th>BNO08X Readings</th>"
    "</tr>"
    "<tr>"
    "<td>Yaw</td>"
    "<td>" + yaw + "</td>"
    "</tr>"
    "<tr>"
    "<td>Pitch</td>"
    "<td>" + pitch + "</td>"
    "</tr>"
    "<tr>"
    "<td>Roll</td>"
    "<td>" + roll + "</td>"
    "</tr>"
    "</table>"
    "</div>"
    "</body>"
    "</html>";
    return ptr;
  }
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

void handle_OnConnect() {
    String blueLED = server.arg("blueled"); 
    digitalWrite(BLUE_LED, blueLED.equals("on") ? HIGH : LOW); 
    
    String redLED = server.arg("redled"); 
    int rledval = redLED.toInt();
    if (rledval >= 0 && rledval < 256) {
        ledcWrite(ledChannel, redLED.toInt());
    }
    float temp = bme.readTemperature();
    float humid = bme.readHumidity();
    float press = bme.readPressure();
    float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    String tempStr = "Current Temperature: " + String(temp, 2) + " C";
    String humidStr = "Current Humidity: " + String(humid, 2) + "%";
    String pressStr = "Current Pressure: " + String(press/1000, 2) + " kPa";
    String altStr = "Current Altitude: " + String(alt, 2) + " m";
    
    //Serial.println("LED Status: LOW");
    Serial.println(tempStr);
    Serial.println(humidStr);
    Serial.println(pressStr);
    Serial.println(altStr);

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
    String yawStr = String(ypr.yaw, 4);
    String pitchStr = String(ypr.pitch, 4);
    String rollStr = String(ypr.roll, 4);
    server.send(200, "text/html", SendHTML(tempStr, 
                humidStr, pressStr, altStr, yawStr, pitchStr, rollStr));
}

void handle_NotFound() {
    server.send(404, "text/plain", "Not found");
}

void setup() {
    Serial.begin(115200);
    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(RED_LED,LOW);

    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(RED_LED, ledChannel);
    ledcWrite(ledChannel, 0);
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit BNO08x test!");

    // Try to initialize!
    if (!bno08x.begin_I2C()) {
      Serial.println("Failed to find BNO08x chip");
      while (1) { delay(10); }
    }
    Serial.println("BNO08x Found!");


    setReports(reportType, reportIntervalUs);
    bool status;
    status = bme.begin(0x76);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    }
    while(Serial.available() == 0){
    Serial.println("Which mode would you like to work in? AP, STA, or both? Use 1, 2, or 3 respectively");
    delay(7000);
    int userAns = Serial.parseInt();
    if(userAns == 1)
    {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(AP_ssid, AP_password);
      delay(2000);
      WiFi.softAPConfig(local_ip, gateway, subnet);
      delay(100);
    }
    if (userAns == 2)
    {
      WiFi.mode(WIFI_STA);
      WiFi.begin(network_ssid, network_password);
    }
    if (userAns == 3)
    {
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(AP_ssid, AP_password);
      delay(2000);
      WiFi.softAPConfig(local_ip, gateway, subnet);
      delay(100);
      WiFi.begin(network_ssid, network_password);
    }
  }
    // while (WiFi.status() != WL_CONNECTED){
    //   delay(500);
    //   Serial.println("Connecting to Wifi...");
    // }
    delay(100);

    server.on("/", handle_OnConnect);
    server.onNotFound(handle_NotFound);
    server.begin();
  
    Serial.print("HTTP server started at AP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("HTTP server started at Network Address: ");
    Serial.println(WiFi.localIP());
}   

void loop() {
    server.handleClient();
    
}