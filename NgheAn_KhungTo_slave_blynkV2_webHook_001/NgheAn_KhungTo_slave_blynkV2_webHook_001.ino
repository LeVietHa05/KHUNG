#define BLYNK_TEMPLATE_ID "TMPL6NSy9OedF"
#define BLYNK_TEMPLATE_NAME "khung to"
#define BLYNK_AUTH_TOKEN "9MTpFhRSnRiFKlAhcD9WvaLWXCEk7pIU"        // khung to slave
#define REMOTE_BLYNK_AUTH_TOKEN "zz9ZA5917JIhH8QDhYVI77IzhM5CdvI6" // khung to master
#define BLYNK_PRINT Serial
//===============================================
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
//===============================================

#define TRIG D1
#define ECHO D2
#define NGHIENG D5
#define COI D6 // dieu khien relay ca coi va den bao (HIGH -trigger
#define HN D7
#define RELAY D8 // dieu khien relay
//===============================================

char auth[] = BLYNK_AUTH_TOKEN;
String server_name = "http://sgp1.blynk.cloud/external/api/";
String master_token = REMOTE_BLYNK_AUTH_TOKEN; // token for the receiving device
String slave_token = BLYNK_AUTH_TOKEN;         // token for the sending device
//===============================================

BlynkTimer timer;
//===============================================

unsigned long duration, last_check;
int distance;
int nghieng1 = 1, nghieng2 = 1;
int xung = 0, a = 1, b = 1;
float chuViBanhXe = 22.3, khoangcach = 0;
//===============================================

void togleLed()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
//===============================================

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // You can add custom code here if needed
}
//===============================================

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("\n MD1 - Khung tap di cho nguoi gia!!!");
  //------------------------------------------

  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.autoConnect("ESP8266", "password");
  //------------------------------------------

  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());
  timer.setInterval(3000L, togleLed);
  //------------------------------------------

  pinMode(NGHIENG, INPUT);
  pinMode(COI, OUTPUT);
  pinMode(HN, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}
//===============================================

void loop()
{
  // put your main code here, to run repeatedly:
  Blynk.run();
  //------------------------------------------

  canhBaoNghieng();
  //------------------------------------------

  doKhoangCach();
  //------------------------------------------

  khoangCachSieuAm();
}
//===============================================

void doKhoangCach()
{
  if (digitalRead(HN) == 0)
  {
    a = 0;
  }
  else
  {
    a = 1;
  }
  if (a < b)
  {
    xung++;
    khoangcach = xung * chuViBanhXe;
    // bridge1.virtualWrite(V101, khoangcach);
    send_float_api_bridge(master_token, 10, khoangcach);
  };
  b = a;
}
//===============================================

void canhBaoNghieng()
{
  if (digitalRead(NGHIENG) == 1)
  {
    delay(500);
    if (digitalRead(NGHIENG) == 1)
    {
      Blynk.logEvent("benh_nhan_bi_nga"); // them event tren server
      // bridge1.virtualWrite(V100, 1);
      send_int_api_bridge(master_token, 9, 1);
      Serial.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
      digitalWrite(COI, LOW);
      digitalWrite(RELAY, HIGH);
    }
  }
  else
  {
    digitalWrite(COI, HIGH);
    digitalWrite(RELAY, LOW);
    delay(500);
  }
}
//===============================================

void khoangCachSieuAm()
{
  digitalWrite(TRIG, 0);
  delayMicroseconds(2);
  digitalWrite(TRIG, 1);
  delayMicroseconds(5);
  digitalWrite(TRIG, 0);
  duration = pulseIn(ECHO, HIGH);
  distance = int(duration / 2 / 29.412);

  Serial.print(distance);
  Serial.print(" cm");
  Serial.print("\n");
  if (distance < 60 && distance > 0)
  {
    delay(100);
    if (distance < 50 && distance > 0)
    {
      Serial.println("keu do khoang cach");
      digitalWrite(COI, LOW);
      digitalWrite(RELAY, HIGH);
      delay(1000);
      digitalWrite(COI, HIGH);
      digitalWrite(RELAY, LOW);
    }
    else
    {
      digitalWrite(COI, HIGH);
      digitalWrite(RELAY, LOW);
    }
  }
}
//===============================================

void send_int_api_bridge(String token, int virtual_pin, int value_to_send)
{
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

  if (httpResponseCode > 0)
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
  }
  else
  {
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
//===============================================

void send_float_api_bridge(String token, int virtual_pin, float value_to_send)
{
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

  if (httpResponseCode > 0)
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
  }
  else
  {
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
