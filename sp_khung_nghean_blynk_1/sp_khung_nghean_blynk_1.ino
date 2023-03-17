
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>



#define TRIG D1
#define ECHO D2
#define NGHIENG D5
#define COI D6  //dieu khien relay ca coi va den bao (HIGH -trigger
#define HN D7
#define RELAY D8 //dieu khien relay

char auth[] = "k3t7MTSMI2T_B4btl5oAf70v5lOUD997"; //
char remoteAuth[] = "LXx1KbUIKgM_FoWMDexIrtx4zsGLjZjS";
char ssid[] = "Khktcxh";
char pass[] = "88888888";

WidgetBridge bridge1(V127);
BlynkTimer timer;

BLYNK_CONNECTED() {
  bridge1.setAuthToken(remoteAuth); // Token of the master device
}


unsigned long duration, last_check;
int distance;
int nghieng1 = 1, nghieng2 = 1;
int xung = 0, a = 1, b = 1;
float chuViBanhXe = 22.3 , khoangcach = 0;

void togleLed () {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("\n MD1 - Khung tap di cho nguoi gia!!!");

  Blynk.begin(auth, ssid,  pass);
  timer.setInterval(3000L, togleLed);

  pinMode(NGHIENG, INPUT);
  pinMode(COI, OUTPUT);
  pinMode(HN, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  Blynk.run();

  canhBaoNghieng();

  doKhoangCach();

  khoangCachSieuAm();

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
    bridge1.virtualWrite(V101, khoangcach);
  };
  b = a;
}

void canhBaoNghieng () {
  if (digitalRead(NGHIENG) == 1) {
    delay(1000);
    if (digitalRead(NGHIENG) == 1) {
      Blynk.notify("Benh nhan bi nga, can ho tro ngay!!!");
      bridge1.virtualWrite(V100, 1);
      Serial.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");
      digitalWrite(COI, LOW);
      digitalWrite(RELAY, HIGH);
    }
  } else {
    digitalWrite(COI, HIGH);
    digitalWrite(RELAY, LOW);
    delay(1000);
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
      delay(1000);
      digitalWrite(COI, HIGH);
      digitalWrite(RELAY, LOW);
    } else {
      digitalWrite(COI, HIGH);
      digitalWrite(RELAY, LOW);
    }
  }
}
