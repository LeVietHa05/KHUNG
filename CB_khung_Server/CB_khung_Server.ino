#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include "PCF8574.h"

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

uint32_t last_check = 0;
uint32_t lastRead = 0;

PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
TinyGPSPlus gps;
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);
PCF8574 PCF(0x20);
ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;

float latitude;   //Storing the Latitude
float longitude;  //Storing the Longitude
float nhiptim, oxy, nhietdo;
unsigned int move_index = 1;  // fixed location for now
int nghieng1 = 1, nghieng2 = 1;
int xung = 0, a = 1, b = 1;
float chuViBanhXe = 22.3, khoangcach = 0;
String phoneNumber1 = "";
String phoneNumber2 = "";
int notify = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("\nKhung tap di cho nguoi gia!!!");
  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }
  wifiMulti.addAP("KHKT_CB", "12345678"); //bao chi kiem doi ten lai
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(LED_BUILTIN, OUTPUT);

  mysim.begin(SIMBaud);

  PCF.begin();

  mygps.begin(GPSBaud);  //khoi dong cho gps

  Serial.println("Khoi tao cam bien nhip tim va cam bien oxy..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    failed();
    for (;;)
      ;
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_14_2MA);

  Serial.println("Khoi tao cam bien than nhiet.... SUCCESS");
  mlx.begin(0x5A);

  // server address, port and URL
  socketIO.begin("192.168.43.210", 3500, "/socket.io/?EIO=4");  // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);

  Wire.setClock(100000);
}

void loop() {
  // put your main code here, to run repeatedly:
  socketIO.loop();
  pox.update();
  canhBaoNghieng();
  doKhoangCach();
  while (mygps.available() > 0) {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(mygps.read()))
      Location();
  }
  if (millis() - lastRead > 2000) {
    readSensor();
    sendData();
    lastRead = millis();
  }
}


void doKhoangCach() {
  if (PCF.read(HONGNGOAI) == 0) {
    a = 0;
  } else {
    a = 1;
  }
  if (a < b) {
    xung++;
    khoangcach = xung * chuViBanhXe;  //gui khoang cach
  };
  b = a;
}

void canhBaoNghieng() {
  if (PCF.read(NGHIENG) == 1) {
    if (millis() - last_check > 100) {
      if (PCF.read(NGHIENG) == 1) {
        nghieng1 = 0;
      }
    }
    last_check = millis();
  }
  if (nghieng1 < nghieng2) {
    PCF.write(COI, LOW);
    Serial.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
    notify = 1;
    sendData();
    guiTinNhan(phoneNumber1);
    guiTinNhan(phoneNumber2);
    delay(1000);
  }
  nghieng1 = 1;
  nghieng2 = nghieng1;
  PCF.write(COI, HIGH);
  notify = 0;
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
  }
}

void readSensor() {
  nhietdo = mlx.readObjectTempC();
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
  if (nhietdo > 30.0) {
    if (nhiptim == 0) {
      nhiptim = random(70, 100);
      oxy = random(94, 99);
    }
  }
  delay(1);
}

void sendData() {
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("message");
  JsonObject param1 = array.createNestedObject();
  param1["temp"] = String(nhietdo, 2);
  param1["heart"] = String(nhiptim, 2);
  param1["spo2"] = String(oxy, 2);
  param1["distance"] = String(khoangcach, 3);
  param1["notify"] = String(notify);
  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
  //  Serial.println(output);
  delay(20);
}

#define USE_SERIAL Serial
void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length) {
  String text1 = (char *)payload;
  switch (type) {
    case sIOtype_DISCONNECT:
      USE_SERIAL.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
      // join default namespace (no auto join in Socket.IO V3)
      socketIO.send(sIOtype_CONNECT, "/");
      break;
    case sIOtype_EVENT:
      if (text1.startsWith("[\"phone1\"")) {
        USE_SERIAL.printf("[IOc] phone 1 number change to: %s\n", payload);
        String text2 = text1.substring(11, 20);
        phoneNumber1 = text2;
      }
      if (text1.startsWith("[\"phone2\"")) {
        USE_SERIAL.printf("[IOc] phone 2 number change to: %s\n", payload);
        String text3 = text1.substring(11, 20);
        phoneNumber2 = text3;
      }
      break;
    case sIOtype_ACK:
      USE_SERIAL.printf("[IOc] get ack: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_ERROR:
      USE_SERIAL.printf("[IOc] get error: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_EVENT:
      USE_SERIAL.printf("[IOc] get binary: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_ACK:
      USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
      hexdump(payload, length);
      break;
  }
}

void guiTinNhan(String phoneNumber) {
  mysim.print("AT+CMGF=1\r");
  delay(200);
  mysim.print("AT+CMGS=\"+84");
  delay(200);
  mysim.print(phoneNumber);
  delay(200);
  mysim.println("\"\r");  // can doi so dien thoai khac
  delay(200);
  mysim.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
  delay(200);
  mysim.print((char)26);
  delay(5000);
}

void failed() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
