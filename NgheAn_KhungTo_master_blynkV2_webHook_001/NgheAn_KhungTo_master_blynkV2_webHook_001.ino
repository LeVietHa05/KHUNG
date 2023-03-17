#define BLYNK_TEMPLATE_ID "TMPLB3rN5jOm"
#define BLYNK_DEVICE_NAME "khung to"
#define BLYNK_AUTH_TOKEN "v5O1CZf3AabQHj27-EFG6_LyimyP1jfc"         //khung to master
#define REMOTE_BLYNK_AUTH_TOKEN "6OrE8IYLgN-MJ1DFoybxtluQ7x0mVaWz"  //khung to slave
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define RXPin D7      //chan rx softuart gps
#define TXPin D8      //chan tx softuart gps
#define RXSIM D5      //chan rx softuart sim (chan rx cua sim00l se noi vao D6, tX- D5)
#define TXSIM D6      //chan tx softuart sim
#define GPSBaud 9600  //toc do giao tiep voi gps
#define SIMBaud 9600  //toc do giao tiep voi SIM

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "AmericanStudy T1";
char pass[] = "66668888";
String server_name = "http://sgp1.blynk.cloud/external/api/";
String slave_token = REMOTE_BLYNK_AUTH_TOKEN;  // token for the receiving device
String master_token = BLYNK_AUTH_TOKEN;        // token for the sending device

BlynkTimer timer;
PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
TinyGPSPlus gps;
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);

float latitude;   //Storing the Latitude
float longitude;  //Storing the Longitude
float nhiptim, oxy, nhietdo;
int is_fall = 0;

void togleLed() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void mlx_max() {
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
  nhietdo = mlx.readObjectTempC();
  Blynk.virtualWrite(V1, nhietdo);
  Blynk.virtualWrite(V2, nhiptim);
  Blynk.virtualWrite(V3, oxy);
  Serial.print(nhietdo);
  Serial.print("\t");
  Serial.print(nhiptim);
  Serial.print("\t");
  Serial.print(oxy);
  Serial.print("\n");
}

void checkNghieng() {
  get_int_api_bridge(master_token, 9, &is_fall);
  if (is_fall == 1) {
    Blynk.logEvent("benh_nhan_bi_nga");
    mysim.print("AT+CMGF=1\r");
    delay(500);
    mysim.println("AT+CMGS=\"+84984964932\"\r");  //can doi so dien thoai khac
    delay(500);
    mysim.println("!CANH BAO! Benh nhan bi nga!!!");
    delay(500);
    mysim.print((char)26);
    delay(10000);
    mysim.println();
    Serial.println("benh nhan bi nga");
  } else {
    Serial.println("benh nhan khong nga");
  }
  send_int_api_bridge(master_token, 9, 0);
}


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("\n MD2 - Khung tap di cho nguoi gia!!!");

  Blynk.begin(auth, ssid, pass);
  timer.setInterval(3000L, togleLed);
  timer.setInterval(2000L, mlx_max);
  timer.setInterval(1222L, checkNghieng);

  pinMode(LED_BUILTIN, OUTPUT);

  mysim.begin(SIMBaud);
  mygps.begin(GPSBaud);

  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;)
      ;
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_11MA);
  mlx.begin(0x5A);

  Wire.setClock(100000);
}

void loop() {
  Blynk.run();
  pox.update();
  timer.run();
  //  gps
  while (mygps.available() > 0) {
    if (gps.encode(mygps.read()))
      if (gps.location.isValid()) {
        latitude = (gps.location.lat());
        longitude = (gps.location.lng());
        Blynk.virtualWrite(V0, longitude, latitude);
      }
  }
}

void get_int_api_bridge(String token, int virtual_pin, int* value_to_get) {
  WiFiClient my_wifi_client;
  HTTPClient http;
  String server_path = server_name + "get?token=" + token + "&pin=v" + String(virtual_pin);

  http.begin(my_wifi_client, server_path.c_str());

  Serial.print("Getting int");
  //  Serial.print(value_to_send);
  Serial.print(" of pin V");
  Serial.println(virtual_pin);

  long request_time = millis();
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
    Serial.println(payload);
    *value_to_get = payload.toInt();
  } else {
    Serial.print("Error code: ");
    Serial.print(httpResponseCode);
    Serial.print(" <-----------------------------------");
  }

  Serial.print("Response time = ");
  Serial.print(millis() - request_time);
  Serial.println(" milliseconds");
  Serial.println();

  // Free resources
  http.end();
}

void send_float_api_bridge(String token, int virtual_pin, float value_to_send) {
  WiFiClient my_wifi_client;
  HTTPClient http;
  String server_path = server_name + "update?token=" + token + "&pin=v" + String(virtual_pin) + "&value=" + float(value_to_send);

  http.begin(my_wifi_client, server_path.c_str());

  Serial.print("Sending (float)");
  Serial.print(value_to_send);
  Serial.print(" to pin V");
  Serial.println(virtual_pin);

  long request_time = millis();
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.print(httpResponseCode);
    Serial.print(" <-----------------------------------");
  }

  Serial.print("Response time = ");
  Serial.print(millis() - request_time);
  Serial.println(" milliseconds");
  Serial.println();

  // Free resources
  http.end();
}

void send_int_api_bridge(String token, int virtual_pin, int value_to_send) {
  WiFiClient my_wifi_client;
  HTTPClient http;
  String server_path = server_name + "update?token=" + token + "&pin=v" + String(virtual_pin) + "&value=" + int(value_to_send);

  http.begin(my_wifi_client, server_path.c_str());

  Serial.print("Sending int");
  Serial.print(value_to_send);
  Serial.print(" to pin V");
  Serial.println(virtual_pin);

  long request_time = millis();
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.print(httpResponseCode);
    Serial.print(" <-----------------------------------");
  }

  Serial.print("Response time = ");
  Serial.print(millis() - request_time);
  Serial.println(" milliseconds");
  Serial.println();

  // Free resources
  http.end();
}
