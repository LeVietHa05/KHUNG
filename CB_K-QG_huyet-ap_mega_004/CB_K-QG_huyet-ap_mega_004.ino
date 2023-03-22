#include <LiquidCrystal_I2C.h>  //hien thi
#include <EEPROM.h>             //luu gia tri cho lan chay tiep theo
// cho che do huyet ap
#include <XGZP6897D.h>           //cam bien ap suat
#include <SimpleKalmanFilter.h>  //bo loc nhieu

LiquidCrystal_I2C lcd(0x27, 16, 2);
SimpleKalmanFilter bo_loc(0.01, 0.1, 0.001);

// he so chia cua cam bien ap suat
#define K 64
XGZP6897D pressSensor(K);

#define HUYETAP
#define DEBUG
// #define DEMGIOT
#define ESP_UART
#define COMPLETE

#define nutnhan1 A1
#define nutnhan2 A2
#define nutnhan3 A3
#define LED_THU A6
#define coi 4
#define VALVE_PIN 8
#define VALVE_PIN_A 9
#define PUMB_PIN 6

// các biến: giá trị cảm biến, tốc độ thực(giọt/phút), tốc độ đặt, tốc độ cũ
int value, tocdo, tocdocai, tocdocu;
// huyet ap tam thu va tam truong
int sbp, dbp, upper_pressure, lower_pressure;
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
// #last_update_nguong: thời gian cập nhật ngưỡng
uint32_t dem_t, t, check5s, check10s, show, show2, time_press_btn, time_without_pulse, last_update_nguong;

// các trạng thái của chương trình
bool start = false, flag_coi = false, canhbao = false, flag_huyetap = false, flag_huyetap_running = false, flag_pumb = true, is_pressure_done = false;

// che do chuong trinh
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
  pinMode(VALVE_PIN, OUTPUT);
  pinMode(VALVE_PIN_A, OUTPUT);
  pinMode(PUMB_PIN, OUTPUT);

  digitalWrite(coi, HIGH);
  digitalWrite(PUMB_PIN, LOW);
  digitalWrite(VALVE_PIN, HIGH);
  digitalWrite(VALVE_PIN_A, LOW);

// khởi tạo Serial3 cho truyền uart sang esp8266
#ifdef ESP_UART
  Serial3.begin(115200);
#endif
  // khởi tạo Serial để debug
  Serial.begin(9600);
  Serial.println("he thong dem nho giot va do huyet ap");

  // đọc giá trị tốc độ từ eeprom
  tocdocai = EEPROM.read(0);
  // todocai <<= 8;
  tocdocai |= EEPROM.read(1);
  if (tocdocai > 9999 || tocdocai < 0)
    tocdocai = 0;
  // doc gia tri huyet ap tu eeprom
  sbp = EEPROM.read(2);
  sbp |= EEPROM.read(3);

  dbp = EEPROM.read(4);
  dbp |= EEPROM.read(5);

  if (!pressSensor.begin()) {
    Serial.println("Failed to find sensor!");
    flag_huyetap = true;
  }
  // khởi tạo lcd
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  KHKT CB 2023");
  delay(1000);
  lcd.clear();
  lcd.print("Dem giot: STOP ");
  lcd.setCursor(0, 1);
  lcd.print("Toc do cai: ");
  lcd.print(tocdocai);
}  // end setup

void loop() {
  if (digitalRead(nutnhan1) == LOW && !flag_huyetap_running) {
    delay(20);
    time_press_btn = millis();  //đếm thời gian bấm nút
    while (digitalRead(nutnhan1) == LOW) {
    }
    if ((millis() - time_press_btn) > 1000 && (millis() - time_press_btn) < 3000) {  //nếu bấm từ 1 - 3 s
      start = !start;                                                                //đảo start về stop và ngược lại
      digitalWrite(coi, LOW);                                                        //bật còi trong 0.5s
      delay(500);
      digitalWrite(coi, HIGH);
    }
    if (millis() - time_press_btn > 4000) {  //nếu bấm > 4s
      mode++;                                //mode từ 0 -> 1 và ngược lại
      if (mode == 2) {
        mode = 0;
      }
      start = false;
      digitalWrite(coi, LOW);
      delay(1000);
      digitalWrite(coi, HIGH);
    }
    if (mode == 0 && start) {  //nếu đang đếm giọt
      Serial.println("Dem giot: START ");
      lcd.clear();
      lcd.print("Dem giot: START ");
      check5s = millis();
      check10s = millis();
      dem_t = millis();
    } else if (mode == 0 && !start) {  //nếu đang đếm giọt và stop
      Serial.println("Dem giot: STOP  ");
      lcd.clear();
      lcd.print("Dem giot: STOP  ");
      lcd.setCursor(0, 1);
      lcd.print("Toc do cai: ");
      lcd.print(tocdocai);
      // lcd.print("/phut   ");
      dem_t = millis();
    } else if (mode == 1 && start && !flag_huyetap) {  //nếu đo huyết áp và start
      Serial.println("HUYET AP: START ");
      lcd.clear();
      lcd.print("HUYET AP: START ");
      flag_huyetap_running = true;  //reset các biến đo liên quan đến huyết áp
      is_pressure_done = false;
      reached_140_mmhg = false;
      downto_60_mmhg = false;
      flag_pumb = true;
      time_without_pulse = millis();
    } else if (mode == 1 && !start && !flag_huyetap) {  //nếu đo huyết áp và stop
      flag_huyetap_running = false;
      Serial.println("HUYET AP: STOP  ");
      lcd.clear();
      lcd.print("HUYET AP: STOP  ");
      lcd.setCursor(0, 1);
      lcd.print("SBP: ");  //in ra các giá trị huyết áp tâm trương và tâm thu của lần đo trước đó
      lcd.print(sbp);
      lcd.print(" DBP: ");
      lcd.print(dbp);
      time_without_pulse = millis();
    } else if (mode == 1 && flag_huyetap) {  //nếu đo huyết áp và cảm biến lỗi
      Serial.println("HUYET AP: ERROR");
      lcd.clear();
      lcd.print("HUYET AP: ERROR");
      delay(3000);
      mode = 0;  //trở về đếm giọt
      start = false;
    }
  }
  if (start && mode == 0) {  //nếu đếm giọt và start
    // cap nhat nguong bằng cách tính giá trị trung bình của 3 lần đo
    for (int i = 0; i < 3; i++) {  //đo 3 lần liên tiếp các giá trị trả về của led thu
      nguong_tong += analogRead(LED_THU);
      delay(1);
    }
    if (millis() - last_update_nguong > 3000) {   // cập nhật ngưỡng mỗi 3s/1 lần
      if (abs((nguong_tong / 3) - value) > 20) {  // cái này là để tránh ngưỡng cập nhật mấy con số nhỏ kiểu, ngưỡng từ 220 -> 230 thì thôi. chỉ cập nhật các giá trị ngưỡng theo kiểu 220 -> 300.
        value = nguong_tong / 3;                  //chia trung bình này
      }
      last_update_nguong = millis();
    }
    nguong_tong = 0;  //reset cái ngưỡng tổng để tránh nó bị tăng quá nhiều theo thời gian (vì ngưỡng của mình tính theo cách trung bình, tổng mà lên vô hạn thì chết)

#ifdef DEMGIOT  //cái này là để debug code thôi, không cần quan tâm đâu
    // dung serial plotter de debug
    // Serial.print("value:  ");
    Serial.print(value + 30);
    Serial.print(" ");
    // Serial.print("analogRead(LED_THU): ");
    Serial.println(analogRead(LED_THU));
#endif                                       // DEBUG
    if (analogRead(LED_THU) > value + 30) {  //đây, đoạn này sẽ bắt đầu tìm xem có giọt nhỏ qua.
                                             //cái analogRead(LED_THU) là cái giá trị thu được từ led thu, value là ngưỡng tính trung bình ở trên. +30 là để đảm bảo không có nhiễu.
      t = millis() - dem_t;                  //thời gian giữa 2 lần có giọt
      dem_t = millis();                      // lưu lại thời gian lần trước có giọt để tính
      int tocdotam = int(60000 / t);         //tính tốc độ bằng cách lấy 60000ms chia cho thời gian giữa 2 lần có giọt

      if (abs(tocdotam - tocdo) > tocdo / 3 && tocdotam < 200) {  //thuật toán chống nhiễu
        if (abs(tocdotam - tocdocu) < 10)
          tocdo = tocdotam;
        tocdocu = tocdotam;
      }
      if (abs(tocdotam - tocdo) < 5) {  //thuật toán cập nhật
        tocdo = tocdotam;
        tocdocu = tocdotam;
      }

      check5s = millis();  //làm mới các biến khi có giọt
      check10s = millis();
      delay(20);
    }
  } else if (mode == 0 && !start) {  //nếu đếm giọt và stop
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
    if (digitalRead(nutnhan2) == LOW) {  //nếu nhấp nút đỏ
      delay(20);
      while (digitalRead(nutnhan2) == LOW) {
        tocdocai++;  //tăng
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
    if (digitalRead(nutnhan3) == LOW) {  //nếu nhấn nút đỏ
      delay(20);
      while (digitalRead(nutnhan3) == LOW) {
        if (tocdocai > 0)
          tocdocai--;  //giảm đến 0
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
  if (millis() - show > 300 && start && mode == 0) {  //in ra tốc độ nhỏ giọt đo được
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
      lcd.print(" g/p   ");
    }
    show = millis();
  }
  if (millis() - show2 > 1000 && start && mode == 0) {
#ifdef ESP_UART
    Serial3.print("S1=");
    Serial3.print(tocdo);
#endif
    show2 = millis();
  }

  if (millis() - check5s > 5000 && start && mode == 0) {
    tocdo = 0;
  }
  if (millis() - check10s > 10000 && start && mode == 0) {  //nếu 10s không có giọt
    lcd.clear();
    lcd.print("      STOP      ");
    lcd.setCursor(0, 1);
    lcd.print("Het nuoc        ");
    digitalWrite(coi, 0);
    delay(3000);
    digitalWrite(coi, 1);
    start = false;
    Serial.print("Het nuoc");
    while (1) {  //dừng chương trình cho đến khi nhấn nút xanh
      if (digitalRead(nutnhan1) == 0) {
        delay(20);
        while (digitalRead(nutnhan1) == 0)
          ;
        break;
      }
    }
  }
  if (mode == 1 && start && !flag_huyetap && flag_huyetap_running && !is_pressure_done) {  //nếu đo huyết áp và start và chưa đo xong và có cảm biến  và đang đo huyết áp
    if (flag_pumb && !is_pressure_done)                                                    // bat dau bom khi vao
    {
      digitalWrite(VALVE_PIN, HIGH);  //dong van
      digitalWrite(PUMB_PIN, HIGH);   //bat may bom
      flag_pumb = false;              //moi lan do huyet ap chi bat 1 lan
    }
    pressSensor.readRawSensor(temperature, pressure);  //đo áp suất
    temperature = temperature / 256;
    pressure = pressure / K;                                            //chia áp suất theo hệ số có trước
    mmHg_kalman_tam = bo_loc.updateEstimate(pressure * 0.00750061683);  //đổi đơn vị từ Pa sang mmHg và đưa qua bộ lọc
#ifdef HUYETAP                                                          //debug
    Serial.print("ap suat: ");
    Serial.println(mmHg_kalman_tam);
#endif  // HUYETAP
    lcd.setCursor(0, 1);
    lcd.print("ap suat: ");
    lcd.print(String(mmHg_kalman_tam, 1));  //in ra áp suất hiện tại
    if (mmHg_kalman_tam > 160.0) {          //dat nguong 160mmhg, kết thúc quá trình bơm, bắt đầu quá trình xả
      reached_140_mmhg = true;
      digitalWrite(PUMB_PIN, LOW);  //tat bom
      delay(2000);
#ifdef DEBUG
      Serial.println("reached 140mmHg");
#endif  // DEBUG
    }
    if (reached_140_mmhg) {          //bat dau chu trinh xa khi de do huyet ap
      digitalWrite(VALVE_PIN, LOW);  //dong, cat van xa khi de giam toc do xa khi
      delay(10);
      digitalWrite(VALVE_PIN, HIGH);
      delay(10);

      if (mmHg_kalman_tam > mmHg_kalman_cu && mmHg_kalman_tam < 120) {  //tim cac lan tim dap
#ifdef DEBUG
        Serial.println("thay 1 nhip tim");
#endif                                         // DEBUG
        mmHg_kalman_cu = mmHg_kalman_tam;      //luu gia tri lan do hien tai lai de so sanh voi cac lan sau
        mmHg_kalman[index] = mmHg_kalman_tam;  //luu vao mot mảng giá trị để tí dùng
        if (upper_pressure == 0.0) {           //nếu sbp đang = 0 thì lần có tim đập đầu tiên sẽ là sbp
          upper_pressure = mmHg_kalman_tam;
          Serial.print("got upper: ");
          Serial.println(upper_pressure);
        }

        time_without_pulse = millis();      // lưu lại lần gần nhất có tim đập
        index++;                            //chỉ số của mảng giá trị
        if (index == 100) {                 //mảng đã đủ giá trị
          selectionSort(mmHg_kalman, 100);  //thuật toán sắp xếp giá trị từ bé đến lớn
          sbp = mmHg_kalman[99];
          dbp = mmHg_kalman[0];
#ifdef DEBUG
          Serial.println("ket qua (index toi 100): ");
          Serial.println(mmHg_kalman[0]);
          Serial.println(mmHg_kalman[99]);
#endif                              // DEBUG
          is_pressure_done = true;  //biến kết thúc chu trình đo huyết áp
          index = 0;
        }
      } else {
        mmHg_kalman_cu = mmHg_kalman_tam;  //luu gia tri lan do hien tai lai de so sanh voi cac lan sau
      }
      if (millis() - time_without_pulse > 3000 && upper_pressure != 0.0 && (reached_140_mmhg || downto_60_mmhg)) {  //nếu 3s không có nhịp tim thì dbp sẽ là áp suất hiện tại (nếu sbp đã có giá trị và đang trong chu trình xả khí)
        lower_pressure = mmHg_kalman_tam;                                                                           //lưu giá trị áp suất
#ifdef DEBUG
        delay(1000);
        Serial.print("LOwer_pressure: ");
        Serial.println(lower_pressure);
        Serial.print("upper pressure: ");
        Serial.println(upper_pressure);
        delay(1000);
#endif  // DEBUG
        sbp = upper_pressure;
        dbp = lower_pressure;
        is_pressure_done = true;  //biến kết thúc chu trình đo huyết áp
      }
    }
    if (reached_140_mmhg && mmHg_kalman_tam < 60.0) {  // khi áp suất giảm xuống dưới 60mmhg và đang trong quá trình xả
      downto_60_mmhg = true;
      reached_140_mmhg = false;
#ifdef DEBUG
      Serial.println("down to 60mmHg");
#endif  // DEBUG
    }
    if (mmHg_kalman_tam < 60.0 && downto_60_mmhg) {
      selectionSort(mmHg_kalman, 100);  // thuật toán sắp xếp
      sbp = mmHg_kalman[99];
      dbp = mmHg_kalman[0];
      if (dbp == 0.0) {  //tránh trường hợp dbp = 0
        for (int i = 0; i < 100; i++) {
          dbp = mmHg_kalman[i];
          if (dbp != 0.0) {
            break;
          }
        }
      }
      // if (dbp == 0.0) {
      //   dbp = random(59, 65);
      // }
      downto_60_mmhg = false;  //kết thúc chu trình xả khí
      is_pressure_done = true;
      delay(100);
#ifdef ESP_UART
      Serial3.print("S2=");
      Serial3.print(sbp);
      Serial3.print(", S3=");
      Serial3.print(dbp);
#endif
    }
    if (is_pressure_done) { //kết thức quá trình đo huyết áp
      flag_huyetap_running = false; //reset các biến về giá trị ban đầu để sẵn sàng cho lần đo tiếp theo 
      start = false;
      upper_pressure = 0.0;
      lower_pressure = 0.0;
      digitalWrite(VALVE_PIN, LOW);
      digitalWrite(PUMB_PIN, LOW);
      flag_pumb = true;
      mmHg_kalman_tam = 0.0;
      mmHg_kalman_cu = 0.0;
      for (int i = 0; i < 100; i++) {
        mmHg_kalman[i] = 0;
      }
      EEPROM.write(2, sbp >> 8);
      EEPROM.write(3, sbp);
      EEPROM.write(4, dbp >> 8);
      EEPROM.write(5, dbp);
      lcd.clear();
      lcd.print("HUYET AP: STOP ");
      lcd.setCursor(0, 1);
      lcd.print("SBP: ");
      lcd.print(sbp);
      lcd.print(" DBP: ");
      lcd.print(dbp);
      delay(1000);
    }
  }
#ifdef COMPLETE
  if (mode == 0 && start && (tocdo - tocdocai) > 20 && millis() > 3000 && tocdo > 10) {
    Serial.println("canh bao nho giot nhanh");
    // lcd.home();
    // lcd.print("CANH BAO: nho ");
    // lcd.setCursor(0, 1);
    // lcd.print("giot qua nhanh  ");
    digitalWrite(coi, LOW);
  } else if (mode == 0 && start && (tocdocai - tocdo) > 20 && millis() > 3000 && tocdo > 10) {
    Serial.println("canh bao nho giot cham");
    digitalWrite(coi, LOW);
    // lcd.home();
    // lcd.print("CANH BAO: nho ");
    // lcd.setCursor(0, 1);
    // lcd.print("giot qua cham  ");
  } else {
    digitalWrite(coi, HIGH);
  }
#endif
}

void selectionSort(float arr[], int n) {
  int i, j, min_idx;
  float temp;

  // One by one move boundary of unsorted subarray
  for (i = 0; i < n - 1; i++) {
    // Find the minimum element in unsorted array
    min_idx = i;
    for (j = i + 1; j < n; j++) {
      if (arr[j] < arr[min_idx]) {
        min_idx = j;
      }
    }
    // Swap the found minimum element with the first element
    temp = arr[min_idx];
    arr[min_idx] = arr[i];
    arr[i] = temp;
  }
#ifdef DEBUG
  Serial.println("ket qua: (sau sort):");
  Serial.println(arr[0]);
  Serial.println(arr[99]);
#endif  // DEBUG
}
