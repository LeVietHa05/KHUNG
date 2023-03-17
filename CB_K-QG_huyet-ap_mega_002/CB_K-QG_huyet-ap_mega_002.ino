#include <LiquidCrystal_I2C.h>  //hien thi
#include <EEPROM.h>             //luu gia tri cho lan chay tiep theo

LiquidCrystal_I2C lcd(0x27, 16, 2);


//them phan code do huyet ap vao day

#define ESP_UART

#define nutnhan1 A1
#define nutnhan2 A2
#define nutnhan3 A3
#define LED_THU A0
#define coi 7
#define VALVE_PIN 5
#define PUMB_PIN 6

// các biến: giá trị cảm biến, tốc độ thực(giọt/phút), tốc độ đặt, tốc độ cũ
int value, tocdo, tocdocai, tocdocu;
//huyet ap tam thu va tam truong
int sbp, dbp;
int16_t temperature;
int32_t pressure;
float mmHg_kalman[100], mmHg_kalman_tam, mmHg_kalman_cu;
int index = 0;
bool reached_140_mmhg = false, downto_60_mmhg = false;

// các mốc thời gian:
// #dem_t: thời gian có giọt chảy xuống (ms),
// #t: thời gian giữa hai lần có giọt
// #check5s: thời gian kiểm tra 5s
// #check10s: thời gian kiểm tra 10s
// #show: thời gian cập nhật số trên lcd
// #show2: thời gian cập nhật số trên server
// #time_press_btn: thời gian nhấn nút
// #time_without_pulse: thời gian giữa 2 lần nhịp đập
// #last_update_nguong: lan cuoi cap nhat nguong do cam bien
uint32_t dem_t, t, check5s, check10s, show, show2, time_press_btn, time_without_pulse, last_update_nguong;

// các trạng thái của chương trình
bool start = false, flag_coi = false, canhbao = false;

//che do chuong trinh
int mode = 0;  // 0 la che do dem giot, 1 la che do huyet ap
// nguong cua cam bien. se thay doi tuy theo vi tri cua cam bien, nen can cap nhat lien tuc
int nguong_tong = 0;

void setup() {
  // khởi tạo các chân pin
  pinMode(nutnhan1, INPUT_PULLUP);
  pinMode(nutnhan2, INPUT_PULLUP);
  pinMode(nutnhan3, INPUT_PULLUP);
  pinMode(LED_THU, INPUT);
  pinMode(coi, OUTPUT);
// khởi tạo Serial3 cho truyền uart sang esp8266
#ifdef ESP_UART
  Serial3.begin(115200);
#endif
  // khởi tạo Serial để debug
  Serial.begin(9600);
  Serial.println("he thong dem nho giot");

  // đọc giá trị tốc độ từ eeprom
  tocdocai = EEPROM.read(0);
  // todocai <<= 8;
  tocdocai |= EEPROM.read(1);
  if (tocdocai > 9999 || tocdocai < 0)
    tocdocai = 0;

  // khởi tạo lcd
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  KHKT CB 2023");
  delay(1000);
  lcd.clear();
  lcd.print("      STOP");
  lcd.setCursor(0, 1);
  lcd.print("Toc do cai: ");
  lcd.print(tocdocai);
  // lcd.print("/phut    ");
  // end setup
}

void loop() {
  if (digitalRead(nutnhan1) == LOW) {
    delay(20);
    while (digitalRead(nutnhan1) == LOW) {
    }

    start = !start;
    digitalWrite(coi, HIGH);
    delay(250);
    digitalWrite(coi, LOW);
    if (start) {
      lcd.clear();
      lcd.print("Dem giot: START ");
      check5s = millis();
      check10s = millis();
      dem_t = millis();
    } else {
      lcd.clear();
      lcd.print("Dem giot: STOP  ");
      lcd.setCursor(0, 1);
      lcd.print("Toc do cai: ");
      lcd.print(tocdocai);
      // lcd.print("/phut   ");
      dem_t = millis();
    }
  }
  if (start) {
    for (int i = 0; i < 3; i++) {
      nguong_tong += analogRead(LED_THU);
      delay(1);
    }
    if (millis() - last_update_nguong > 3000) {
      if (abs((nguong_tong / 3) - value) > 20) {
        value = nguong_tong / 3;
      }
      last_update_nguong = millis();
    }
    nguong_tong = 0;
    Serial.println(value + 30);
    Serial.print(" ");
    Serial.println(analogRead(LED_THU));
    if (analogRead(LED_THU) > value +30) {
      t = millis() - dem_t;
      dem_t = millis();
      int tocdotam = int(60000 / t);

      // if (tocdotam < 200 && ((tocdotam + 15 > tocdocu) || (tocdotam - 15 < tocdocu))) {
      //   tocdocu = tocdo;
      //   tocdo = tocdotam;
      // }

      if (abs(tocdotam - tocdo) > tocdo / 3 && tocdotam < 200) {
        if (abs(tocdotam - tocdocu) < 10) {
          tocdo = tocdotam;
        }
        tocdocu = tocdotam;
      }
      if (abs(tocdotam - tocdo) < 5) {
        tocdo = tocdotam;
        tocdocu = tocdotam;
      }
      // Serial.println(tocdo);
      check5s = millis();
      check10s = millis();
      delay(10);
    }
  } else {
#ifdef ESP_UART
    if (Serial3.available() > 0) {
      delay(10);
      String data = "";
      while (Serial3.available() > 0) {
        data = Serial3.readString();
      }
      int key_check = data.indexOf("S0=");
      if (key_check != 1) {
        tocdocai = data.substring((key_check + 3), (key_check + 5)).toInt();
        EEPROM.write(0, tocdocai >> 8);
        EEPROM.write(1, tocdocai);
      }
    }
#endif
    if (digitalRead(nutnhan2) == LOW) {
      delay(20);
      while (digitalRead(nutnhan2) == LOW) {
        tocdocai++;
        lcd.setCursor(0, 1);
        lcd.print("Toc do cai: ");
        lcd.print(tocdocai);
        // lcd.print("/phut   ");
        delay(10);
      }
#ifdef ESP_UART
      Serial3.print("S0=");
      Serial3.print(tocdocai);
#endif
      EEPROM.write(0, tocdocai >> 8);
      EEPROM.write(1, tocdocai);
    }
    if (digitalRead(nutnhan3) == LOW) {
      delay(20);
      while (digitalRead(nutnhan3) == LOW) {
        if (tocdocai > 0)
          tocdocai--;
        lcd.setCursor(0, 1);
        lcd.print("Toc do cai: ");
        lcd.print(tocdocai);
        // lcd.print("/phut   ");
        delay(10);
      }
#ifdef ESP_UART
      Serial3.print("S0=");
      Serial3.print(tocdocai);
#endif
      EEPROM.write(0, tocdocai >> 8);
      EEPROM.write(1, tocdocai);
    }
  }
  if (millis() - show > 300 && start) {
    if (tocdo > 200) {
      lcd.home();
      lcd.print("     START      ");
      lcd.setCursor(0, 1);
      lcd.print("Toc do:...");
      lcd.print("/phut   ");
    } else {
      lcd.home();
      lcd.print("     START      ");
      lcd.setCursor(0, 1);
      lcd.print("Toc do:");
      lcd.print(tocdo);
      lcd.print("/phut   ");
    }
    show = millis();
  }
  if (millis() - show2 > 1000 && start) {
#ifdef ESP_UART
    Serial3.print("S1=");
    Serial3.print(tocdo);
#endif
    show2 = millis();
  }

  if (millis() - check5s > 5000) {
    tocdo = 0;
  }
  if (millis() - check10s > 10000 && start) {
    lcd.setCursor(0, 0);
    lcd.print("      STOP      ");
    lcd.setCursor(0, 1);
    lcd.print("Het nuoc        ");
    digitalWrite(coi, 1);
    delay(300);
    digitalWrite(coi, 0);
    start = false;
    Serial.print("Het nuoc");
    while (1) {
      if (digitalRead(nutnhan1) == 0) {
        delay(20);
        while (digitalRead(nutnhan1) == 0)
          ;
        break;
      }
    }
  }
#ifdef COMPLETE
  if (start && (tocdo - tocdocai) > 20 && millis() > 3000 && tocdo > 10) {
    Serial.println("canh bao nho giot nhanh");
    lcd.home();
    lcd.print("CANH BAO: nho ");
    lcd.setCursor(0, 1);
    lcd.print("giot qua nhanh  ");
    digitalWrite(coi, HIGH);
  } else if (start && (tocdocai - tocdo) > 20 && millis() > 3000 && tocdo > 10) {
    Serial.println("canh bao nho giot cham");
    digitalWrite(coi, HIGH);
    lcd.home();
    lcd.print("CANH BAO: nho ");
    lcd.setCursor(0, 1);
    lcd.print("giot qua cham  ");
  } else {
    digitalWrite(coi, LOW);
  }
#endif
}
