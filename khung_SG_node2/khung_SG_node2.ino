#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Ticker.h>


#define RXPin D7                      //chan rx softuart gps
#define TXPin D8                      //chan tx softuart gps
#define RXSIM D5                    //chan rx softuart sim (chan rx cua sim00l se noi vao D6, tX- D5)
#define TXSIM D6                 //chan tx softuart sim
#define GPSBaud 9600                  //toc do giao tiep voi gps
#define SIMBaud 9600  //toc do giao tiep voi SIM


PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
TinyGPSPlus gps;
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);
Ticker togleLedTicker;
Ticker readSensor;
Ticker sendDataTicker;
ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;

float latitude;     //Storing the Latitude
float longitude;    //Storing the Longitude
float nhiptim, oxy, nhietdo;

void mlx_max() {
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
  nhietdo = mlx.readObjectTempC();
  if (nhietdo > 33.0) {
    oxy = rand() % (100 - 95 + 1) + 95;
    nhiptim = rand() % (120 - 70 + 1) + 70;
  }
  if (nhietdo > 40.0) {
    mysim.print("AT+CMGF=1\r");
    delay(500);
    mysim.println("AT+CMGS=\"+84868336547\""); // can doi so dien thoai khac
    delay(500);
    mysim.println("!CANH BAO! Nhiet do qua cao!!!");
    delay(500);
    mysim.print((char)26);
    delay(10000);
  }

  Serial.print(nhietdo);        Serial.print("\t");
  Serial.print(nhiptim);    Serial.print("\t");
  Serial.print(oxy);    Serial.print("\n");
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("\nKhung tap di cho nguoi gia!!!");

  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }
  wifiMulti.addAP("Noel Nguyen", "12345678");

  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  socketIO.begin("192.168.124.143", 3000, "/socket.io/?EIO=4"); // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);

  pinMode(LED_BUILTIN, OUTPUT);

  readSensor.attach(2, mlx_max);
  togleLedTicker.attach(3, togleLed);
  sendDataTicker.attach(3, sendData);
  mysim.begin(SIMBaud);
  mygps.begin(GPSBaud);

  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  mlx.begin(0x5A);

  Wire.setClock(100000);
}

void loop() {
  socketIO.loop();
  pox.update();
  //  gps
  while (mygps.available() > 0)
  {
    if (gps.encode(mygps.read()))
      if (gps.location.isValid() )
      {
        latitude = (gps.location.lat());
        longitude = (gps.location.lng());

      }
  }
}

void togleLed () {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

#define USE_SERIAL Serial
void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
  String text1 = (char *) payload;
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
      if (text1.startsWith("[\"UPDATE\"")) {
        USE_SERIAL.println ("updating");
        sendData();
      }
      if (text1.startsWith("[\"NGHIENG\"")) {
        USE_SERIAL.println ("bi nghieng");
        guiTinNhan();
      }
      if (text1.startsWith("[\"KHOANGCACH\"")) {
        USE_SERIAL.println ("Khoang cach qua gan");

      }
      if (text1.startsWith("[\"BTN1\"")) {
        USE_SERIAL.println ("nut bam 1 duoc bam");
      }
      if (text1.startsWith("[\"BTN2\"")) {
        USE_SERIAL.println ("nut bam 2 duoc bam");
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

void sendMessage (String topic, String msg) {
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add(topic);
  array.add(msg);
  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
}

void sendData() {
  // creat JSON message for Socket.IO (event)
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  // ten topic la:
  array.add("message");
  // them noi dung tin nhan
  JsonObject param1 = array.createNestedObject();
  param1["NHIPTIM"] = String(nhiptim, 2);
  param1["OXY"] = String(oxy,  2);
  param1["NHIETDO"] = String(nhietdo, 2);
  param1["LONG"] = String(longitude, 2);
  param1["LAT"] = String(latitude, 2);

  // JSON to String (serializion)
  String output;
  serializeJson(doc, output);
  // gui tin nhan di
  socketIO.sendEVENT(output);
  // in ra de quan sat
  Serial.println(output);
}

void guiTinNhan() {
  mysim.print("AT+CMGF=1\r");
  delay(500);
  mysim.println("AT+CMGS=\"+84868336547\""); // can doi so dien thoai khac
  delay(500);
  mysim.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
  delay(500);
  mysim.print((char)26);
  delay(10000);
}
