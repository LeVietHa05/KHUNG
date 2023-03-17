dht
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <Ticker.h>


Ticker togleLedTicker;
ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;

#define TRIG D1
#define ECHO D2
#define NGHIENG D5
#define COI D6  //dieu khien relay ca coi va den bao (HIGH -trigger
#define HN D7
#define RELAY D8 //dieu khien relay
#define USE_SERIAL Serial


unsigned long duration, lastCheck;
int distance;
int nghieng = 0;
int xung = 0, a = 1, b = 1;
float chuViBanhXe = 22.3 , khoangcach = 0;

void togleLed () {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("\n Node 1 - Khung tap di cho nguoi gia!!!");

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
  // server address, port and URL
  socketIO.begin("192.168.124.143", 3000, "/socket.io/?EIO=4"); // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);


  pinMode(NGHIENG, INPUT);
  pinMode(COI, OUTPUT);
  pinMode(HN, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  togleLedTicker.attach(3, togleLed);
}

void loop() {
  socketIO.loop();

  canhBaoNghieng();

//  doKhoangCach();

  if (millis() - lastCheck) {
    khoangCachSieuAm();
    lastCheck = millis();
  }
}

void doKhoangCach () {
  if (digitalRead(HN) == 0) {
    a = 0;
  } else {
    a = 1;
  }
  if (a < b) {
    xung++;
    khoangcach = xung * chuViBanhXe;
    //gui len server khong?
    sendMessage("DICHUYENDUOC", String(int(khoangcach)));
  };
  b = a;
}

void canhBaoNghieng () {
  if (digitalRead(NGHIENG) == 1) {
    delay(500);
    yield();
    if (digitalRead(NGHIENG) == 1) {
      nghieng = 1;
      sendMessage("NGHIENG", "help, help");
      Serial.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
      digitalWrite(COI, LOW);
      digitalWrite(RELAY, HIGH);
    }
  } else {
    digitalWrite(COI, HIGH);
    digitalWrite(RELAY, LOW);
    nghieng = 0;
    delay(1000);
    yield();
  }
}

void khoangCachSieuAm () {
  digitalWrite(TRIG, 0);
  delayMicroseconds(2);
  digitalWrite(TRIG, 1);
  delayMicroseconds(5);
  digitalWrite(TRIG, 0);
  duration = pulseIn(ECHO, HIGH);
  distance = int(duration / 2 / 29.412);

  Serial.print(distance); Serial.print(" cm"); Serial.print("\n");
  if (distance < 60 && distance > 0) {
    delay(100);
    if (distance < 50 && distance > 0) {
      Serial.println("keu do khoang cach");
      digitalWrite(COI, LOW);
      digitalWrite(RELAY, HIGH);
      sendMessage("KHOANGCACH", "khoang cach duoi 50");
      delay(1000);
      yield();
      digitalWrite(COI, HIGH);
      digitalWrite(RELAY, LOW);
    } else {
      digitalWrite(COI, HIGH);
      digitalWrite(RELAY, LOW);
    }
  }
}

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
      if (text1.startsWith("[\"BTN1\"")) {
        USE_SERIAL.println ("btn1");

      }
      if (text1.startsWith("[\"BTN2\"")) {
        USE_SERIAL.println ("btn2");

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
