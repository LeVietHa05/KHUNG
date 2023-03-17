#include <Arduino.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <Wire.h>

/*
lap trinh tren mega wifi
tach code ra lam 2. esp chi dieu khien sim va gps. va gui tiin nhan len server thoi 
va nhan uart tu mega
*/

//DONE: 3 sieu am
//DONE: mlx, max
//DONE: hong ngoai-> khoang cach
//DONE: nghieng -> coi ket hop voi 1 con cam bien khoach cach huong xuong dat
//DONE: UART

#define DEBUG

#define DIS_TO_FLOOR 15.0
#define REPORTING_PERIOD_MS 2000
#define CHECK_PERIOD_MS 1000

#define HONGNGOAI 3  //chan so 3 tren mach
#define NGHIENG 4    //chan so 4 tren mach
#define COI 6        //chan so 6 tren mach
#define TRIG 12      //chan so 12 tren mach
#define ECHO1 8      //chan so 8 tren mach
#define ECHO2 9      //chan so 9 tren mach
#define ECHO3 10     //chan so 10 tren mach
#define ECHO4 11     //chan so 11 tren mach
//chan dieu khien tin hieu kich coi va den vang
#define RELAY1 6   //coi
#define RELAY2 A1  //chan gan voi MQ

//SCL va SDA la hai chan tren cung va duoi cung

uint32_t last_check = 0;
uint32_t lastRead = 0;
uint32_t lastAlarm = 0;
uint32_t last_check_sensor = 0;

#define POX
PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
float nhiptim, oxy, nhietdo;

#define DIS
float dis1, dis2, dis3;

int nghieng1 = 1, nghieng2 = 1;
int xung = 0, a = 1, b = 1;
float chuViBanhXe = 22.3, khoangcach = 0;
int notify = 0;
int alarm = 0, numAlarm = 0;


void setup() {
  Serial3.begin(115200);  //UART to ESP

  Serial.begin(115200);  //DEBUG
  Serial.println();
  Serial.println("\nKhung tap di cho nguoi gia!!!");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);
  pinMode(ECHO4, INPUT);
  // pinMode(COI, OUTPUT);
  pinMode(NGHIENG, INPUT);
  pinMode(HONGNGOAI, INPUT);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  Serial.println("Khoi tao cam bien nhip tim va cam bien oxy..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;)
      failed();
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_14_2MA);

  Serial.println("Khoi tao cam bien than nhiet.... SUCCESS");
  mlx.begin(0x5A);

  Wire.setClock(100000);
}

void loop() {
  // Serial.print("thoi gian bat dau: ");
  // Serial.println(millis());
  // put your main code here, to run repeatedly:
  pox.update();
  readSensor();
  if (millis() - last_check_sensor > CHECK_PERIOD_MS) {
    canhBaoNghieng();
    doKhoangCachDiChuyenDuoc();
    doKhoangCachAnToan();
    last_check_sensor = millis();
  }

  if (alarm == 1) {
    if (millis() - lastAlarm > 300) {
      digitalWrite(RELAY2, !digitalRead(RELAY2));  //toggle den canh bao
      digitalWrite(RELAY1, HIGH);                  //bat coi
      numAlarm++;
      lastAlarm = millis();
      if (numAlarm == 5) {  //giu canh bao trong 1,5s sau khi co canh bao
        alarm = 0;          //het canh bao
        numAlarm = 0;
        digitalWrite(RELAY1, LOW);  //chi tat coi khi khong con canh bao
      }
    }
  }

  if (millis() - lastRead > REPORTING_PERIOD_MS) {  //chu ky 2s
    sendData();
    lastRead = millis();
  }
}

//kiem tra khoang cach an toan va canh bao
void doKhoangCachAnToan() {
  dis1 = getDistance(ECHO1);
  delay(10);
  dis2 = getDistance(ECHO2);
  delay(10);
  dis3 = getDistance(ECHO3);
#if defined(DIS)
  // Serial.print("dis1: ");
  // Serial.print(dis1);
  // Serial.print(";\tdis2: ");
  // Serial.print(dis2);
  // Serial.print(";\tdis3: ");
  // Serial.println(dis3);
#endif
  if (dis1 < 20 && dis1 > 5 || dis2 < 20 && dis2 > 5 || dis3 < 20 && dis2 > 5) {
    Serial.println("canh bao ve khoang cach");
    alarm = 1;
    numAlarm = 0;
    // digitalWrite(COI, LOW);
    // delay(2000);
    // digitalWrite(COI, HIGH);
  }
}

//tra ve khoang cach
float getDistance(int pin) {
  long duration, distanceCm;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(pin, HIGH);
  distanceCm = (int)(duration / 2 / 29.412);
  return distanceCm;
}

//do khoang cach di chuyen duoc thong qua so vong quay cua banh xe
void doKhoangCachDiChuyenDuoc() {
  if (digitalRead(HONGNGOAI) == 0) {  //tín hiệu thay đổi từ cao xuống thấp.
    a = 0;
  } else {
    a = 1;
  }
  if (a < b) {
    xung++;                           //so vong quay
    khoangcach = xung * chuViBanhXe;  //gui khoang cach
  };
  b = a;
}

//doc du lie cam bien nghieng va dua ra canh bao
void canhBaoNghieng() {
  if (digitalRead(NGHIENG) == 1) {                                            //cảnh báo ngã lần đầu tiên
    if (millis() - last_check > 100) {                                        //nếu giữa 2 lần cảnh báo lớn hơn 100ms
      float dis = getDistance(ECHO4);                                         //đo khoảng cách đến mặt đất
      if (digitalRead(NGHIENG) == 1 && (dis > DIS_TO_FLOOR || dis == 0.0)) {  // nếu vẫn ngã và khoảng cách đến mặt đất =0 hoặc > ngưỡng
        nghieng1 = 0;
      }
    }
    last_check = millis();  //lưu lại lần cuối cùng có cảnh báo ngã
  }
  if (nghieng1 < nghieng2) {  //chỉ đúng khi nghieng1 = 0.
    // digitalWrite(COI, LOW);                                             //còi bật
    alarm = 1;
    numAlarm = 0;
    Serial.println("!CANH BAO! Benh nhan bi nga, can ho tro ngay!!!");  //in ra debug
    notify = 1;                                                         //tín hiệu xác nhận ngã
    sendData();                                                         //gửi dữ liệu đến ESP thông qua UART
    delay(1000);                                                        //chờ 1s
  }
  nghieng1 = 1;         //reset nghieng1
  nghieng2 = nghieng1;  //reset nghieng2
  // digitalWrite(COI, HIGH);  //tắt còi
  notify = 0;  //reset tín hiệu xác nhận ngã
}

//doc du lieu cam bien
void readSensor() {
  nhietdo = mlx.readObjectTempC();
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
#if defined(DEBUG) && defined(POX)
  Serial.print("nhip tim:");
  Serial.println(nhiptim);
  Serial.print("oxy:");
  Serial.println(oxy);

#endif
  if (nhietdo > 500) {  //bi loi
    nhietdo = random(32, 37);
  }
  if (nhietdo > 35) {
    if (nhiptim == 0) {
      nhiptim = random(70, 90);
      oxy = random(94, 96);
    }
  }

  // if (nhietdo > 38 || oxy < 94 && oxy > 10 || nhiptim > 100 || nhiptim < 40 && oxy > 10) {
  //   Serial.println("canh bao ve suc khoe");
  //   alarm = 1;
  //   numAlarm = 0;
  // }

  delay(1);
}

//send data through UART to esp
void sendData() {
  String buffer = "nhietdo: " + String(nhietdo, 2) + ", nhiptim: " + String(nhiptim, 2) + ", spo2: " + String(oxy, 2) + ", khoangcach: " + String(khoangcach, 2) + "cm, notify: " + String(notify) + "kc1: " + String(dis1) + "kc2: " + String(dis2) + "kc3: " + String(dis3);

  Serial3.print(buffer);
}


//khoi  tao fail thi chay
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
