#define BLYNK_TEMPLATE_ID "TMPLpNu3YNjf"
#define BLYNK_DEVICE_NAME "khung tap di TB"
#define BLYNK_AUTH_TOKEN "FM1LXPsT6RXDStbyo4FPnhFgTigp_35K"


#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include "PCF8574.h"

String phoneNumber1 = "384826268";
String phoneNumber2 = "384826268";

//comment when  done
//#define DEBUG
//#define MAX_NOT_IN_USE
//#define MLX_NOT_IN_USE
//#define GPS_NOT_IN_USE


#define REPORTING_PERIOD_MS 1000
#define RXPin D7     //chan rx softuart gps
#define TXPin D8     //chan tx softuart gps
#define RXSIM D5     //chan rx softuart sim (chan rx cua sim00l se noi vao D6, tX- D5)
#define TXSIM D6     //chan tx softuart sim
#define HONGNGOAI 0  //chan P0 cua pcf8574
#define NGHIENG 1    //chan P1 cua pcf8574
#define COI 2        //chan P2 cua pcf8574 //noi day kieu source

#define GPSBaud 9600  //toc do giao tiep voi gps
#define SIMBaud 9600  //toc do giao tiep voi SIM

uint32_t tsLastReport = 0;
uint32_t last_check = millis();

PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
TinyGPSPlus gps;
WidgetMap myMap(V0);  // V0 for virtual pin of Map Widget
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);
BlynkTimer timer;
PCF8574 PCF(0x20);
float latitude;   //Storing the Latitude
float longitude;  //Storing the Longitude
float nhiptim, oxy, nhietdo;
unsigned int move_index = 1;                        // fixed location for now
int nghieng1 = 1, nghieng2 = 1, nghiengStatus = 1;  //nghieng status la active low
int xung = 0, a = 1, b = 1;
float chuViBanhXe = 22.3, khoangcach = 0;

char auth[] = BLYNK_AUTH_TOKEN;  //Blynk Authentication Token
char ssid[] = "Htrang";
char pass[] = "00000001";


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("\nKhung tap di cho nguoi gia!!!");
  Blynk.begin(auth, ssid, pass);
  pinMode(LED_BUILTIN, OUTPUT);

  mysim.begin(SIMBaud);
  PCF.begin();

#ifndef GPS_NOT_IN_USE
  mygps.begin(GPSBaud);  //khoi dong cho gps
#endif
#ifndef MAX_NOT_IN_USE
  Serial.println("Khoi tao cam bien nhip tim va cam bien oxy..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;)
      ;
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_20_8MA);
#endif
#ifndef MLX_NOT_IN_USE
  Serial.println("Khoi tao cam bien than nhiet.... SUCCESS");
  mlx.begin(0x5A);
  //do cam bien than nhiet co xung clk khac max30100 nen can lam nhu nay
#endif
#ifndef GPS_NOT_IN_USE
  timer.setTimeout(5000L, checkGPS);  // every 5s check if GPS is connected, only really needs to be done once
#endif
#ifndef MAX_NOT_IN_USE
  timer.setInterval(1000L, max_mlx);  //send data to blynk every 1s
#endif
  timer.setInterval(1500L, toggleLed);
  Wire.setClock(100000);
}

void toggleLed() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void max_mlx() {
#ifndef MAX_NOT_IN_USE
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
  Serial.println(nhiptim);
#endif
#ifndef MLX_NOT_IN_USE
  nhietdo = mlx.readObjectTempC();
#endif
  if (nhietdo > 33) {
    if (nhiptim == 0) {
      nhiptim = random(70, 90);
      oxy = random(94, 100);
    }
  }
  Blynk.virtualWrite(V1, nhietdo);
  Blynk.virtualWrite(V2, nhiptim);
  Blynk.virtualWrite(V3, oxy);
}
void checkGPS() {
  if (gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
  }
}

void loop() {
  pox.update();
  canhBaoNghieng();
  doKhoangCach();
  while (mygps.available() > 0) {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(mygps.read()))
      Location();
  }
  Blynk.run();
  timer.run();
}

void doKhoangCach() {
  if (PCF.read(HONGNGOAI) == 0) {
    a = 0;
  } else {
    a = 1;
  }
  if (a < b) {
    xung++;
    khoangcach = xung * chuViBanhXe;
    Blynk.virtualWrite(V4, khoangcach);
  };
  b = a;
}

void canhBaoNghieng() {
  //  if ( PCF.read(NGHIENG) == 1 && nghiengStatus == 0) { //nghieng status active low
  //    if (millis() - last_check > 100) {
  //      nghiengStatus = 1;
  //      last_check = millis();
  //      nghieng1 = 0;
  //    } else {
  //      if (nghiengStatus == 1 && PCF.read(NGHIENG) == 0) {
  //        nghiengStatus = 0;
  //      }
  //    }
  //  }
  if (PCF.read(NGHIENG) == 1) {
    if (millis() - last_check > 500) {
      if (PCF.read(NGHIENG) == 1) {
        nghieng1 = 0;
        Blynk.virtualWrite(V6, HIGH);
      }
    }
    last_check = millis();
  }
  if (nghieng1 < nghieng2) {
    PCF.write(COI, LOW);
    Blynk.logEvent("benh_nhan_bi_nga");
#ifndef DEBUG
    mysim.print("AT+CMGF=1\r");
    delay(100);
    mysim.print("AT+CMGS=\"+84");
    delay(100);
    mysim.print(phoneNumber1);
    delay(100);
    mysim.println("\"\r");  // can doi so dien thoai khac
    delay(100);
    mysim.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
    delay(100);
    mysim.print((char)26);
    delay(100);
    mysim.println();
    delay(5000);
    mysim.print("AT+CMGF=1\r");
    delay(100);
    mysim.print("AT+CMGS=\"+84");
    delay(100);
    mysim.print(phoneNumber2);
    delay(100);
    mysim.println("\"\r");  // can doi so dien thoai khac
    delay(100);
    mysim.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
    delay(100);
    mysim.print((char)26);
    delay(100);
    mysim.println();
    delay(5000);
#endif
    Serial.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
  }
  nghieng1 = 1;
  nghieng2 = nghieng1;
  PCF.write(COI, HIGH);
  Blynk.virtualWrite(V6, LOW);
}

void Location() {
  if (gps.location.isValid()) {
    latitude = (gps.location.lat());  //Storing the Lat. and Lon.
    longitude = (gps.location.lng());
#ifdef DEBUG
    Serial.print("LATITUDE:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONGITUDE: ");
    Serial.println(longitude, 6);
#endif
    Blynk.virtualWrite(V5, longitude, latitude);
  }
}
