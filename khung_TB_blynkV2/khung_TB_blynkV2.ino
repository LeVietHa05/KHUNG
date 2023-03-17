#define BLYNK_TEMPLATE_ID "TMPLpNu3YNjf"
#define BLYNK_DEVICE_NAME "khung tap di TB"
#define BLYNK_AUTH_TOKEN "LP2j7aRNDv24Jrw5eB7Q9Uol0Vak5-WF"


#define DEBUG

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_MLX90614.h>
#include "MAX30100_PulseOximeter.h"
#include <Wire.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "AmericanStudy T1";
char pass[] = "66668888";

BlynkTimer timer;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
PulseOximeter pox;

float nhiptim, oxy, nhietdo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("\nKhung tap di cho nguoi gia!!!");

  pinMode(LED_BUILTIN, OUTPUT);

  Blynk.begin(auth, ssid, pass);

  Serial.println("Khoi tao cam bien nhip tim va cam bien oxy..");
  if (!pox.begin()) {
    Serial.println("FAILED, check the sensor");
  } else {
    Serial.println("SUCCESS");
  }

  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
  
  Serial.println("Khoi tao cam bien than nhiet.... SUCCESS");
  mlx.begin(0x5A);
  
  timer.setInterval(1000L, max_mlx);
  timer.setInterval(1100L, toggleLed);

  Wire.setClock(100000);
}

void toggleLed() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void max_mlx () {
  nhietdo = mlx.readObjectTempC();
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
  Blynk.virtualWrite(V1, nhietdo);
  Blynk.virtualWrite(V2, nhiptim);
  Blynk.virtualWrite(V3, oxy);
#ifdef DEBUG
  Serial.println(nhietdo);
  Serial.println(nhiptim);
  Serial.println(oxy);
  Serial.println();
#endif
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run();
  pox.update();
}
