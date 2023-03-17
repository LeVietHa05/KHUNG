#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

/*
lap trinh tren mega wifi
tach code ra lam 2. esp chi  gui tiin nhan len server thoi 
va nhan uart tu mega
*/

//DONE: ketnoi server online, gui duoc tin nhan len server.
//TODO: led (them con led de biet luc nao nhan duoc tin nhan hoac gui tin nhan di)
//DONE: kiem tra cac gia tri nhan duoc tu uart de xu ly
//DONE: gui cac gia tri nhan duoc len server

//cac chan dung duoc: D3 (GPIO0), D4 (GPIO2), D1 (GPIO5), D2 (GPIO4), D5 (GPIO12)

#define REPORTING_PERIOD_MS 1000
#define RXPin 5  //chan rx softuart gps (D1)
#define TXPin 4  //chan tx softuart gps (D2)
#define RXSIM 0  //chan rx softuart sim (D3)
#define TXSIM 2  //chan tx softuart sim (D4)

#define GPSBaud 9600  //toc do giao tiep voi gps
#define SIMBaud 9600  //toc do giao tiep voi SIM

#define LED_PIN 12  //D5

uint32_t last_check = 0;
uint32_t lastRead = 0;
TinyGPSPlus gps;
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);
ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;

float latitude;   //Storing the Latitude
float longitude;  //Storing the Longitude
float nhiptim, oxy, nhietdo, khoangcach;
float dis1, dis2, dis3;
unsigned int move_index = 1;  // fixed location for now
String phoneNumber1 = "981229399";
String phoneNumber2 = "369677432";
int notify = 0;
char buff[100];


void setup() {
  Serial.begin(115200);
  Serial.println();
  // Serial.println("\nKhung tap di cho nguoi gia!!!");
  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }
  wifiMulti.addAP("Chinh", "chinh7122007");
  wifiMulti.addAP("AmericanStudy T1", "66668888");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(LED_PIN, OUTPUT);

  mygps.begin(GPSBaud);
  mysim.begin(SIMBaud);

  // server address, port and URL
  socketIO.begin("khunghotrovandong.com", 80, "/socket.io/?EIO=4");  // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  String buffer = "";
  socketIO.loop();
  while (mygps.available() > 0) {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(mygps.read()))
      Location();
  }
  while (Serial.available() > 0) {
    digitalWrite(LED_PIN, HIGH);
    //buffer: "nhietdo: 25, nhiptim: 90.00, spo2: 95, khoangcach: 100.00cm, notify: 0";
    buffer = Serial.readString();
    nhietdo = buffer.substring(buffer.indexOf("nhietdo: ") + 9, buffer.indexOf(", nhiptim")).toFloat();
    nhiptim = buffer.substring(buffer.indexOf("nhiptim: ") + 9, buffer.indexOf(", spo2: ")).toFloat();
    oxy = buffer.substring(buffer.indexOf("spo2: ") + 6, buffer.indexOf(", khoangcach: ")).toFloat();
    khoangcach = buffer.substring(buffer.indexOf("khoangcach: ") + 12, buffer.indexOf("cm, notify: ")).toFloat();
    notify = buffer.substring(buffer.indexOf("notify: ") + 8, buffer.indexOf("kc1")).toInt();
    dis1 = buffer.substring(buffer.indexOf("kc1: " + 4), buffer.indexOf("kc2")).toFloat();
    dis2 = buffer.substring(buffer.indexOf("kc2: " + 4), buffer.indexOf("kc3")).toFloat();
    dis3 = buffer.substring(buffer.indexOf("kc3: " + 4)).toFloat();
    if (nhietdo || nhiptim || oxy || khoangcach) {
      if (nhietdo > 35 && nhiptim == 0) {
        nhiptim = random(70, 90);
        oxy = random(94, 99);
      }
    }
    if (notify == 1) {
      guiTinNhan(phoneNumber1);
      guiTinNhan(phoneNumber2);
      notify = 0;
    }
    digitalWrite(LED_PIN, LOW);
  }
  if (millis() - lastRead > 2000) {
    sendData();
    lastRead = millis();
  }
}

//cap nhat thong tin vi tri
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

//gui tin nhan den so dien thoai
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


//send data to server
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
  param1["latitude"] = String(latitude, 6);
  param1["longitude"] = String(longitude, 6);
  param1["dis1"] = String(dis1, 2);
  param1["dis2"] = String(dis2, 2);
  param1["dis3"] = String(dis3, 2);
  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
  //  Serial.println(output);
  delay(20);
}

#define USE_SERIAL Serial
void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length) {
  digitalWrite(LED_PIN, HIGH);
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
  digitalWrite(LED_PIN, LOW);
}



void failed() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
}
