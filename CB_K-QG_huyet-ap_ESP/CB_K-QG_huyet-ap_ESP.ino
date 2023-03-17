
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>

uint32_t last_check = 0;
uint32_t lastRead = 0;
uint32_t lastSendData = 0;

ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;

//gia tri toc do nho giot
int tocDoNhoGiot, tocDoNhoGiotTam;
//gia tri huyet ap tam truong, tam thu
float hApTamTruong, hApTamThu;

void setup() {
  Serial.begin(115200);
  Serial.println();

  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }
  wifiMulti.addAP("AmericanStudy T1", "66668888");
  wifiMulti.addAP("Cút lộn xào me", "12345678");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //ket noi server
  socketIO.begin("csskvn.com", 80, "/socket.io/?EIO=4");  // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);
}

void loop() {
  String buffer = "";
  socketIO.loop();
  while (Serial.available() > 0) {
    buffer = Serial.readString();
    if (buffer.indexOf("S0=") != 1) {
      tocDoNhoGiotTam = buffer.substring(buffer.indexOf("S0=") + 3).toInt();
    } else if (buffer.indexOf("S1=") != 1) {
      tocDoNhoGiot = buffer.substring(buffer.indexOf("S1=") + 3).toInt();
    } else if (buffer.indexOf("S2=") != 1) {
      hApTamThu = buffer.substring(buffer.indexOf("S2=") + 3).toInt();
    } else if (buffer.indexOf("S3=") != 1) {
      hApTamTruong = buffer.substring(buffer.indexOf("S3=") + 3).toInt();
    }
  }
  if (millis() - lastSendData > 3000) {
    sendData();
    lastSendData = millis();
  }
}


//socket handler
void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length) {
  String text1 = (char *)payload;
  switch (type) {
    case sIOtype_DISCONNECT:
      Serial1.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      Serial1.printf("[IOc] Connected to url: %s\n", payload);
      // join default namespace (no auto join in Socket.IO V3)
      socketIO.send(sIOtype_CONNECT, "/");
      break;
    case sIOtype_EVENT:
      if (text1.startsWith("[\"S0\"")) {
        Serial.print("S0=");
        Serial.print(text1.substring(5, text1.indexOf("]")));
      }
      break;
    case sIOtype_ACK:
      Serial1.printf("[IOc] get ack: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_ERROR:
      Serial1.printf("[IOc] get error: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_EVENT:
      Serial1.printf("[IOc] get binary: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_ACK:
      Serial1.printf("[IOc] get binary ack: %u\n", length);
      hexdump(payload, length);
      break;
  }
}


//send data to server
void sendData() {
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("message");
  JsonObject param1 = array.createNestedObject();
  param1["tocdonhogiot"] = String(tocDoNhoGiot);
  param1["sys"] = String(hApTamThu);
  param1["dia"] = String(hApTamTruong);
  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
  //  Serial.println(output);
  delay(20);
}