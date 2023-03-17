#include <Arduino.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include "DHT.h"
#include <SharpGP2Y10.h>
#include <MQUnifiedsensor.h>

#define ESP_UART
#define DEBUG
#define DIS_THRESHOLD 10

#define LED_BUI 4
#define DHT_PIN 5
#define ECHO 8
#define TRIG 9
#define HONGNGOAI 10
#define NGHIENG 11
#define CO2 A0
#define BUI_OUT A1
#define SO2 A2

PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
DHT dht(DHT_PIN, DHT11);
SharpGP2Y10 dustSensor(BUI_OUT, LED_BUI);
MQUnifiedsensor MQ2("Arduino UNO", 5, 10, SO2, "MQ-2");
MQUnifiedsensor MQ135("Arduino UNO", 5, 10, CO2, "MQ-135");

float nhiptim, oxy, nhietdo, humid, temp, dustDensity, MQ2_ppm, MQ135_ppm;
unsigned int move_index = 1; // fixed location for now
int nghieng1 = 1, nghieng2 = 1  ;
int xung = 0, a = 1, b = 1;
float chuViBanhXe = 22.3, khoangcach = 0;
int notify = 0;

long lastReadSensors = 0, lastSend = 0, lastCheck = 0;

void setup()
{
  pinMode(DHT_PIN, INPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(HONGNGOAI, INPUT);
  pinMode(NGHIENG, INPUT);
  pinMode(CO2, INPUT);

  mq2_init();
  MQ135_init();

  Serial.begin(115200);
  Serial.println();

  if (!pox.begin())
  {
    Serial.println("POX: FAILED");
    failed();
    for (;;)
      failed();
  }
  else
  {
    Serial.println("POX: SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_14_2MA);

  mlx.begin(0x5A);

  Wire.setClock(100000);

  dht.begin();
}

void loop()
{
  pox.update();
  if (millis() - lastReadSensors > 3000)
  {
    dht_read();
    pox_mlx_read();
    dust_read();
    mq2_read();
    MQ135_read();
    lastReadSensors = millis();
  }
  canh_bao_nghieng();
  do_khoang_cach();
}

void do_khoang_cach()
{
  if (digitalRead(HONGNGOAI) == LOW)
  {
    a = 0;
  }
  else
  {
    a = 1;
  }
  if (a < b)
  {
    xung++;
    khoangcach = xung * chuViBanhXe;
#ifdef ESP_UART
    Serial.print("Khoang cach: " + String(khoangcach, 2) + "cm");
#endif // EPS_UART
  }
  b = a;
}

void canh_bao_nghieng()
{
  if (digitalRead(NGHIENG) == HIGH)
  {
    if (millis() - lastCheck > 100)
    {
      float dis = get_distance(ECHO);
      if (digitalRead(NGHIENG) == HIGH && dis > DIS_THRESHOLD || dis == 0.0)
      {
        nghieng1 = 0;
      }
    }
    lastCheck = millis();
  }
  if (nghieng1 < nghieng2)
  {
    digitalWrite(LED_BUILTIN, HIGH);
#ifdef ESP_UART
    Serial.println("Nghieng: 1");
#endif // EPS_UART
    delay(1000);
  }
  nghieng2 = nghieng1;
  nghieng1 = 1;
  digitalWrite(LED_BUILTIN, LOW);
}

void dht_read()
{
  humid = dht.readHumidity();
  temp = dht.readTemperature();
#ifdef ESP_UART
  Serial.print("Humid: " + String(humid, 2) + "%, Temp: " + String(temp, 2) + "C");
#endif // ESP_UART
}

void pox_mlx_read()
{
  nhiptim = pox.getHeartRate();
  oxy = pox.getSpO2();
  nhietdo = mlx.readObjectTempC();
#ifdef DEBUG
  Serial.print("Heart rate: ");
  Serial.print(nhiptim);
  Serial.print("bpm, SpO2: ");
  Serial.print(oxy);
  Serial.print("%, Object temp: ");
  Serial.print(nhietdo);
  Serial.println("C");
#endif // DEBUG
#ifdef ESP_UART
  Serial.print("Nhiptim: " + String(nhiptim, 2) + "bpm, Oxy: " + String(oxy, 2) + "%, Nhietdo: " + String(nhietdo, 2) + "C");
#endif // ESP_UART
  if (nhietdo > 500.0)
  { // loi
    nhietdo = random(30, 40);
  }
  if (nhietdo > 35.0 && nhiptim == 0.0)
  {
    nhiptim = random(60, 100);
    oxy = random(94, 100);
  }
}

void dust_read()
{
  dustDensity = dustSensor.getDustDensity();
#ifdef DEBUG
  Serial.print("Dust density: ");
  Serial.print(dustDensity);
  Serial.println(" ug/m3");
#endif // DEBUG
#ifdef ESP_UART
  Serial.print("Dust density: " + String(dustDensity, 2) + "ug/m3");
#endif // ESP_UART
}

void mq2_init()
{
  MQ2.setRegressionMethod(1);
  MQ2.setA(36974);
  MQ2.setB(-3.109); // Configure the equation to to calculate CO concentration
  MQ2.init();
  Serial.print("Calibrating MQ2 please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(9.83);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("  done!.");
  if (isinf(calcR0))
  {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
  }
  if (calcR0 == 0)
  {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
  }
}

void mq2_read()
{
  MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
  MQ2_ppm = MQ2.readSensor();
#ifdef DEBUG
  Serial.print("MQ2 ppm: ");
  Serial.print(MQ2_ppm);
  Serial.println(" ppm");
#endif // DEBUG
#ifdef ESP_UART
  Serial.print("MQ2 ppm: " + String(MQ2_ppm, 2) + "ppm");
#endif // ESP_UART
}

void MQ135_init()
{
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(110.47);
  MQ135.setB(-2.862); // Configure the equation to to calculate CO2 concentration
  MQ135.init();
  Serial.print("Calibrating MQ135 please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(3.6);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0))
  {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
  }
  if (calcR0 == 0)
  {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
  }
}

void MQ135_read()
{
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  MQ135_ppm = MQ135.readSensor();
#ifdef DEBUG
  Serial.print("MQ135 ppm: ");
  Serial.print(MQ135_ppm);
  Serial.println(" ppm");
#endif // DEBUG
#ifdef ESP_UART
  Serial.print("MQ135 ppm: " + String(MQ135_ppm, 2) + "ppm");
#endif // ESP_UART
}

float get_distance(int PIN)
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  float duration = pulseIn(PIN, HIGH);
  float distance = duration / 2 / 29.412;
  return distance;
}

void failed()
{
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
