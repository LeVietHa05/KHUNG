#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WifiMulti.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <PCF8574.h>
#include <Wire.h>
#include <GP2YDustSensor.h>

#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp8266.h>

// #define DEBUG

// D1 an D2 are used for the serial connection to the PCF8574
#define DHTPIN D3
#define DHTTYPE DHT11
#define RX_SIM D6
#define TX_SIM D5
#define SIMBAUD 9600
#define CO2 0         // CO2 sensor analog pin active low (pin 0 of PCF8574)
#define GAS 1         // gas sensor active low
#define DUST 2        // Dust sensor active low
#define BUZZER 3      // active low
#define RELAY_RED 4   // active low, red light connect to NO, green light connect to NC
#define RELAY_GREEN 5 // active low, red light connect to NO, green light connect to NC
#define TRIG 6        // active high
#define PUMP D8        // active high
#define ECHO D4
#define DUSTPIN A0
#define DUST_LED D7

DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial SIM800L(RX_SIM, TX_SIM);
PCF8574 pcf8574(0x20);
ESP8266WiFiMulti WiFiMulti;
BlynkTimer timer;
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1010AU0F, DUST_LED, DUSTPIN);

float temp, humi, co2, gas, dust, distance;
int pinValue;
char auth[] = "u6IC8GSYzrgVf2LR4wZuzaad0o0vghvo"; // Blynk Authentication Token
char ssid[] = "Thien";                 // WiFi SSID
char pass[] = "11111111";                         // WiFi Password
String phoneNumber = "+84369243462";              // Phone Number
String message = "HIGH CHANCE OF FIRE! RUN AWAY FOLLOW THE RED LIGHT! OR JUST CHECK YOUR HOUSE!";

BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

BLYNK_WRITE(V6)
{
  pinValue = param.asInt();
  if (pinValue == 1)
  {
    pcf8574.write(PUMP, HIGH);
    digitalWrite(PUMP, HIGH);
    Serial.println("PUMP ON");
    pcf8574.write(RELAY_RED, LOW);
    pcf8574.write(RELAY_GREEN, HIGH);
  }
  else
  {
    pcf8574.write(PUMP, LOW);
    digitalWrite(PUMP, LOW);
    Serial.println("PUMP OFF");
    pcf8574.write(RELAY_RED, HIGH);
    pcf8574.write(RELAY_GREEN, LOW);
  }
}

void changePin(int pin, int state, int delayTime)
{
  digitalWrite(pin, state);
  delay(delayTime);
}

void setPin(int pin, int state)
{
  digitalWrite(pin, state);
}

void togglePin()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// to check if the distance is less than 60cm
int checkDistance()
{
  pcf8574.write(TRIG, LOW);
  delayMicroseconds(2);
  pcf8574.write(TRIG, HIGH);
  delayMicroseconds(10);
  pcf8574.write(TRIG, LOW);
  distance = pulseIn(ECHO, HIGH);
  distance = distance / 58.2;
  if (distance < 60.0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

// FIXME: read sensor data
int readAnalogSensor(int pin, int delaytime)
{
  int sensorValue = 0;
  switch (pin)
  {
    case 0: // co2
      pcf8574.write(CO2, HIGH);
      pcf8574.write(GAS, LOW);
      pcf8574.write(DUST, LOW);
      delay(delaytime);
      // yield();
      sensorValue = analogRead(A0) - 610;
      break;
    case 1: // gas
      pcf8574.write(CO2, LOW);
      pcf8574.write(GAS, HIGH);
      pcf8574.write(DUST, LOW);
      delay(delaytime);
      // yield();
      sensorValue = analogRead(A0) - 517;
      break;
    case 2: // dust
      pcf8574.write(CO2, LOW);
      pcf8574.write(GAS, LOW);
      pcf8574.write(DUST, HIGH);
      delay(delaytime);
      // yield();
      sensorValue = dustSensor.getDustDensity() - 321;
      break;
    default:
      Serial.println("Wrong pin");
      break;
  }
  return sensorValue;
}

void sendSMS()
{
  SIM800L.print("AT+CMGF=1\r");
  delay(500);
  SIM800L.println("AT+CMGS=\"+84342621986\"\r");
  delay(500);
  SIM800L.println(message);
  delay(500);
  SIM800L.println((char)26);
  delay(10000);
  SIM800L.println();
}

void noiseRemove()
{
  float temp[5];
  float humi[5];

  for (int i = 0; i < 5; i++)
  {
    temp[i] = 0;
    humi[i] = 0;
  }
  int z = 0;
  while (z < 5)
  {
    float temporary1 = dht.readTemperature();
    float temporary2 = dht.readHumidity();
    if (isnan(temporary1) || isnan(temporary2))
    {
      Serial.println("Failed to read from DHT sensor!");
      delay(500);
    }
    else
    {
      temp[z] = temporary1;
      humi[z] = temporary2;
      z++;
      delay(500);
    }
  }

  // calcutate the average
  float tempSum = 0;
  float humiSum = 0;
  for (int i = 0; i < 5; i++)
  {
    tempSum += temp[i];
    humiSum += humi[i];
  }
  tempSum /= 5;
  humiSum /= 5;
  if (tempSum > 35)
  {
    Blynk.notify("Gas level is high! High temperature detected! PLEASE CHECK! OR RUN AWAY!");
#ifndef DEBUG
    sendSMS();
#endif // !DEBUG
    Serial.println("Gas level is high! High temperature detected! PLEASE CHECK! OR RUN AWAY!");
    pcf8574.write(BUZZER, LOW);
    pcf8574.write(RELAY_RED, LOW);
    pcf8574.write(RELAY_GREEN, HIGH);
    pcf8574.write(PUMP, HIGH);
    digitalWrite(PUMP, HIGH);
    Blynk.virtualWrite(V6, 1);
    delay(1000);
  }
  else
  {
    pcf8574.write(BUZZER, HIGH);
    pcf8574.write(RELAY_RED, HIGH);
    pcf8574.write(RELAY_GREEN, LOW);
    pcf8574.write(PUMP, LOW);
    digitalWrite(PUMP, LOW);
    Blynk.virtualWrite(V6, 0);
  }

  Blynk.virtualWrite(V1, tempSum);
  Blynk.virtualWrite(V2, humiSum);
}

void sendData()
{

  co2 = readAnalogSensor(CO2, 1000);
  gas = readAnalogSensor(GAS, 1000);
  dust = readAnalogSensor(DUST, 1000);

  //   if (/*gas > 900 &&*/ temp > 35)
  //   {
  //     Blynk.notify("Gas level is high! High temperature detected! PLEASE CHECK! OR RUN AWAY!");
  // #ifndef DEBUG
  //     sendSMS();
  // #endif // !DEBUG
  //     Serial.println("Gas level is high! High temperature detected! PLEASE CHECK! OR RUN AWAY!");
  //     pcf8574.write(BUZZER, LOW);
  //     pcf8574.write(RELAY_RED, LOW);
  //     pcf8574.write(RELAY_GREEN, HIGH);
  //     pcf8574.write(PUMP, HIGH);
  //     delay(1000);
  //   }
  //   else
  //   {
  //     pcf8574.write(BUZZER, HIGH);
  //     pcf8574.write(RELAY_RED, HIGH);
  //     pcf8574.write(RELAY_GREEN, LOW);
  //     pcf8574.write(PUMP, HIGH);
  //   }
  // send data to blynk
  Blynk.virtualWrite(V3, co2);
  Blynk.virtualWrite(V4, gas);
  Blynk.virtualWrite(V5, dust);

#ifdef DEBUG
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" *C, Humidity: ");
  Serial.print(humi);
  Serial.print(" %, CO2: ");
  Serial.print(co2);
  Serial.print(" ppm, Gas: ");
  Serial.print(gas);
  Serial.print(" ppm, Dust: ");
  Serial.print(dust);
  Serial.print(" ug/m3, Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
#endif
}

// FIXME: check setup again
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Nha thong minh");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(ECHO, INPUT);

  Blynk.begin(auth, ssid, pass);
  timer.setInterval(2000L, togglePin);
  timer.setInterval(1000L, sendData);
  timer.setInterval(50001L, noiseRemove);

  pcf8574.begin();
  SIM800L.begin(SIMBAUD);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run();
}
