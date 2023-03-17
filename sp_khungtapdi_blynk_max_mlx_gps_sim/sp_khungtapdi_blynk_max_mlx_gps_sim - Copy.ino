#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include "PCF8574.h"


//comment when  done
//#define DEBUG
//#define MAX_NOT_IN_USE
//#define MLX_NOT_IN_USE
//#define GPS_NOT_IN_USE

//max30100 va mlx, pcf8574 se dung chan d1 d2: scl va sda
//dung them mach mo rong pcf8574, su dung i2c => chuyen sim sang chan d7 va d8, nghieng va hong ngoai + coi se sang pcf

#define REPORTING_PERIOD_MS     1000
#define RXPin D7                      //chan rx softuart gps
#define TXPin D8                      //chan tx softuart gps
#define RXSIM D5                    //chan rx softuart sim (chan rx cua sim00l se noi vao D6, tX- D5)
#define TXSIM D6                 //chan tx softuart sim
#define HONGNGOAI 0 //chan P0 cua pcf8574
#define NGHIENG 1 //chan P1 cua pcf8574
#define COI 2 //chan P2 cua pcf8574 //noi day kieu source

#define GPSBaud 9600                  //toc do giao tiep voi gps
#define SIMBaud 9600  //toc do giao tiep voi SIM

uint32_t tsLastReport = 0;
uint32_t last_check = 0;

PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
TinyGPSPlus gps;
WidgetMap myMap(V0);  // V0 for virtual pin of Map Widget
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);
BlynkTimer timer;
PCF8574 PCF(0x20);

float latitude;     //Storing the Latitude
float longitude;    //Storing the Longitude
float nhiptim, oxy, nhietdo;
unsigned int move_index = 1;       // fixed location for now
int nghieng1 = 1, nghieng2 = 1;
int xung = 0, a = 1, b = 1;
float chuViBanhXe = 22.3 , khoangcach = 0;

char auth[] = "DIEN_CODE_VAO_DAY";            //Blynk Authentication Token
char ssid[] = "NHAP_TEN_WIFI_VAO_DAY";            // WiFi SSID
char pass[] = "NHAP_PASS_WIFI_VAO_DAY";            // WiFi Password


void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("\nKhung tap di cho nguoi gia!!!");
  Blynk.begin(auth, ssid,  pass);
  pinMode(LED_BUILTIN, OUTPUT);

  mysim.begin(SIMBaud);
  PCF.begin();

#ifndef GPS_NOT_IN_USE
  mygps.begin(GPSBaud); //khoi dong cho gps
#endif
#ifndef MAX_NOT_IN_USE
  Serial.println("Khoi tao cam bien nhip tim va cam bien oxy..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
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
  timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once
#endif
#ifndef MAX_NOT_IN_USE
  timer.setInterval(1000L, max_mlx); //send data to blynk every 1s
#endif
  timer.setInterval(1100L, toggleLed);
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
  Blynk.virtualWrite(V1, nhietdo);
  Blynk.virtualWrite(V2, nhiptim);
  Blynk.virtualWrite(V3, oxy);
  Blynk.run();
  delay(1);
}
void checkGPS()
{
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
}

void loop()
{
  pox.update();
  canhBaoNghieng();
  doKhoangCach();
  while (mygps.available() > 0)
  {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(mygps.read()))
      Location();
  }
  Blynk.run();
  timer.run();
}

void doKhoangCach () {
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
  if (PCF.read(NGHIENG) == 1) {
    if  (millis() - last_check > 100) {
      if (PCF.read(NGHIENG) == 1) {
        nghieng1 = 0;
      }
    }
    last_check = millis();
  }
  if (nghieng1 < nghieng2) {
    PCF.write(COI, LOW);
    Blynk.notify("Benh nhan bi nga, can ho tro ngay!!!");
#ifndef DEBUG
    mysim.print("AT+CMGF=1\r");
    delay(500);
    mysim.println("AT+CMGS=\"+84916964955\""); // can doi so dien thoai khac
    delay(500);
    mysim.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
    delay(500);
    mysim.print((char)26);
    delay(10000);
#endif
    Serial.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");

    delay(1000);
  }
  nghieng1 = 1;
  nghieng2 = nghieng1;
  PCF.write(COI, HIGH);
}

void Location()
{
  if (gps.location.isValid() )
  {
    latitude = (gps.location.lat());     //Storing the Lat. and Lon.
    longitude = (gps.location.lng());
#ifdef DEBUG
    Serial.print("LATITUDE:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONGITUDE: ");
    Serial.println(longitude, 6);
#endif
    myMap.location(move_index, latitude, longitude, "GPS_Location");
  }
}
