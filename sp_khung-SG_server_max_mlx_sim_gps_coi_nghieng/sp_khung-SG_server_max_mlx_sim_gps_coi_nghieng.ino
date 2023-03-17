#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Ticker.h> //ticker (gan giong timer)
#include "PCF8574.h"

/*
   sua wifi
   sua so dien thoai
*/


#define TRIG 0                       //chan trigger - sieu am - pcf8574
#define ECHO D3                       //chan echo - sieu am - pcf8574
#define COI 2                        //chan coi - pcf8574
#define NGHIENG 3                    //chan cb nghieng - pcf8574
#define  HONGNGOAI 4                 //chan cb hong ngoai - pcf8574

#define RXPin D7                      //chan rx softuart gps
#define TXPin D8                      //chan tx softuart gps
#define GPSBaud 9600                  //toc do giao tiep voi gps
#define SIMBaud 9600
#define RXSIM D5
#define TXSIM D6

Ticker ticker1;
Ticker ticker2;
Ticker ticker3;
TinyGPSPlus gps;
SoftwareSerial mygps(RXPin, TXPin);
SoftwareSerial mysim(RXSIM, TXSIM);
ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;
PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
PCF8574 PCF(0x20);


float nhiptim, oxy, nhietdo;
float latitude;     //Storing the Latitude
float longitude;    //Storing the Longitude
int nghieng1 = 1, nghieng2 = 1;
unsigned long last_check;
unsigned long duration; // hcsr05
int distance; //hcsr05
int xung = 0, a = 1, b = 1; //hong noai
float chuViBanhXe = 22.3 , khoangcach = 0;  //hong ngoai


void setup() {
  pinMode(ECHO, INPUT);
  Serial.begin(115200);
  Serial.println();
  Serial.println("\nKhung tap di cho nguoi gia!!!");

  PCF.begin();

  //ket noi wifi
  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }
  wifiMulti.addAP("AmericanStudy T1", "66668888");
  //  wifiMulti.addAP("Redmi 9T", "12345678");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // server address, port and URL //can sua lai ip va port cho phu hop
  socketIO.begin("192.168.100.130", 3000, "/socket.io/?EIO=4"); // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);

  //khoi dong gps
  mygps.begin(GPSBaud);

  //khoi tao giao tiep suart cho sim
  mysim.begin(SIMBaud);

  //khoi tao max30100
  Serial.println("Khoi tao cam bien nhip tim va cam bien oxy..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  //khoi tao mlx
  Serial.println("Khoi tao cam bien than nhiet.... SUCCESS");
  mlx.begin(0x5A);

  //timer cho hcsr05
  ticker3.attach(1, HCSR05);
  //timer cho max va mlx
  ticker1.attach(1, max_mlx);
  //send data
  ticker2.attach(2, sendDataToServer);

  Wire.setClock(100000);

} //end of setup

void loop() {
  pox.update();
  canhBaoNghieng();
  doKhoangCach();
  while (mygps.available() > 0)
  {
    if (gps.encode(mygps.read()))
      Location();
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
    khoangcach = xung * chuViBanhXe;
  };
  b = a;
}

void Location () {
  if (gps.location.isValid() )
  {
    latitude = (gps.location.lat());     //Storing the Lat. and Lon.
    longitude = (gps.location.lng());
    Serial.print("LATITUDE:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONGITUDE: ");
    Serial.println(longitude, 6);
    Serial.println();
  }
}

void max_mlx() {
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
  nhietdo = mlx.readObjectTempC();
  Serial.print(nhiptim);
  Serial.print("\t");
  Serial.print(oxy);
  Serial.print("\t");
  Serial.print(nhietdo);
  Serial.print("\n");
}


void HCSR05 () {
  //phat xung
  PCF.write(TRIG, 0);
  delayMicroseconds(2);
  PCF.write(TRIG, 1);
  delayMicroseconds(5);
  PCF.write(TRIG, 0);
  //tinh toan thoi gian
  duration = pulseIn(ECHO, HIGH);
  //tinh toan khoang cach
  distance = int(duration / 2 / 29.412);
  if (distance < 100) {
    PCF.write(COI, LOW);
  } else PCF.write(COI, HIGH);
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
    mysim.println("AT+CMGF=1");
    mysim.println("AT+CMGS=\"+84369677432\""); //can doi so dien thoai khac
    Serial.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
    delay(1);
  }
  nghieng1 = 1;
  nghieng2 = nghieng1;
}

//web socket event
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


void sendDataToServer ()  {
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();

  array.add("message");
  JsonObject param1 = array.createNestedObject();
  param1["clientID"] = "NodeMCU SP Khung SG";
  param1["khoangcach"] = String(khoangcach, 4);
  param1["nhiptim"] = String(nhiptim, 2);
  param1["nhietdo"] = String(nhietdo, 2);
  param1["oxy"] = String(oxy, 2);
  param1["latitude"] = String(latitude);
  param1["longitude"] = String(longitude);

  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
  Serial.println(output);
}
