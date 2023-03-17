#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define RXPin D7  //chan rx softuart gps
#define TXPin D8  //chan tx softuart gps
#define RXSIM D5  //chan rx softuart sim (chan rx cua sim00l se noi vao D6, tX- D5)
#define TXSIM D6  //chan tx softuart sim

#define GPSBaud 9600  //toc do giao tiep voi gps
#define SIMBaud 9600  //toc do giao tiep voi SIM

TinyGPSPlus gps;
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);
ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;

float latitude;   //Storing the Latitude
float longitude;  //Storing the Longitude
float nhiptim, oxy, nhietdo, khoangcach, temp, humid, dust, mq2, mq135;
int notify = 0;
String phoneNumber1 = "834217367";
String phoneNumber2 = "902055664";

long lastSend = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();

  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }
  wifiMulti.addAP("Cút lộn xào me", "12345678");  //bao chi kiem doi ten lai
  wifiMulti.addAP("AmericanStudy T1, 66668888");
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
  mygps.begin(GPSBaud);

  socketIO.begin("csskvn.com", 80, "/socket.io/?EIO=4");  // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);
}

void loop() {
  socketIO.loop();

  while (mygps.available() > 0) {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(mygps.read()))
      Location();
  }
  if (millis() - lastSend > 2000) {
    sendData();
    lastSend = millis();
  }
  while (Serial.available() > 0) {
    String buffer = "";
    buffer = Serial.readString();

    if (buffer.indexOf("Khoang cach:") != 1) {
      khoangcach = buffer.substring(13, buffer.indexOf("cm")).toFloat();
    }
    if (buffer.indexOf("Nghieng") != 1) {
      notify = 1;
      if (phoneNumber1.length() == 9)
        guiTinNhan(phoneNumber1);
      if (phoneNumber2.length() == 9)
        guiTinNhan(phoneNumber2);
    }
    if (buffer.indexOf("Humid") != 1) {
      humid = buffer.substring(6, buffer.indexOf("%")).toFloat();
      temp = buffer.substring(buffer.indexOf("Temp:") + 6, buffer.indexOf("C")).toFloat();
    }
    if (buffer.indexOf("Nhiptim:") != 1) {
      nhiptim = buffer.substring(9, buffer.indexOf("bpm")).toFloat();
      oxy = buffer.substring(buffer.indexOf("Oxy") + 5, buffer.indexOf("%")).toFloat();
      nhietdo = buffer.substring((buffer.indexOf("Nhietdo: ") + 9), buffer.indexOf("C")).toFloat();
    }
    if (buffer.indexOf("Dust") != 1) {
      dust = buffer.substring(14, buffer.indexOf("ug")).toFloat();
    }
    if (buffer.indexOf("MQ2") != 1) {
      mq2 = buffer.substring(9, buffer.indexOf("ppm")).toFloat();
    }
    if (buffer.indexOf("MQ135") != 1) {
      mq135 = buffer.substring(11, buffer.indexOf("ppm")).toFloat();
    }
  }
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
  param1["dust"] = String(dust, 2);
  param1["mq2"] = String(mq2, 2);
  param1["mq135"] = String(mq135, 2);
  param1["airTemp"] = String(temp, 2);
  param1["airHumid"] = String(humid, 2);
  notify = 0;
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
  mysim.println();
}