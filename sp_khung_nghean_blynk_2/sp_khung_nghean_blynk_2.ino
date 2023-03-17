#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


#define RXPin D7                      //chan rx softuart gps
#define TXPin D8                      //chan tx softuart gps
#define RXSIM D5                    //chan rx softuart sim (chan rx cua sim00l se noi vao D6, tX- D5)
#define TXSIM D6                 //chan tx softuart sim
#define GPSBaud 9600                  //toc do giao tiep voi gps
#define SIMBaud 9600  //toc do giao tiep voi SIM


PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
TinyGPSPlus gps;
WidgetMap myMap(V0);  // V0 for virtual pin of Map Widget
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);
BlynkTimer timer;
WidgetBridge bridge1(V126);

float latitude;     //Storing the Latitude
float longitude;    //Storing the Longitude
float nhiptim, oxy, nhietdo;

char auth[] = "LXx1KbUIKgM_FoWMDexIrtx4zsGLjZjS"; //
char remoteAuth[] = "k3t7MTSMI2T_B4btl5oAf70v5lOUD997";
char ssid[] = "Khktcxh";
char pass[] = "88888888";

BLYNK_CONNECTED() {
  bridge1.setAuthToken(remoteAuth); // Token of the master device
}

BLYNK_WRITE(V100) { //bi nga hay khong
  int is_fall = param.asInt(); // co bi nga hay khong
  if (is_fall == 1) {
    mysim.print("AT+CMGF=1\r");
    delay(500);
    mysim.println("AT+CMGS=\"+84984964932\"\r"); //can doi so dien thoai khac
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
}

BLYNK_WRITE(V101) { //CO2
  float khoangcach = param.asFloat();
  Blynk.virtualWrite(V4, khoangcach); 
}

void mlx_max() {
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
  nhietdo = mlx.readObjectTempC();
  Blynk.virtualWrite(V1, nhietdo);
  Blynk.virtualWrite(V2, nhiptim);
  Blynk.virtualWrite(V3, oxy);
  Serial.print(nhietdo);        Serial.print("\t");
  Serial.print(nhiptim);    Serial.print("\t");
  Serial.print(oxy);    Serial.print("\n");
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("\nKhung tap di cho nguoi gia!!!");

  pinMode(LED_BUILTIN, OUTPUT);

  Blynk.begin(auth, ssid,  pass);

  timer.setInterval(2000L, mlx_max);
  timer.setInterval(3000L, togleLed);
  mysim.begin(SIMBaud);
  mygps.begin(GPSBaud); //khoi dong cho gps

  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
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
  while (mygps.available() > 0)
  {
    if (gps.encode(mygps.read()))
      if (gps.location.isValid() )
      {
        latitude = (gps.location.lat());
        longitude = (gps.location.lng());
        myMap.location(1, latitude, longitude, "GPS_Location");
      }
  }
}

void togleLed () {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
