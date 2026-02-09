#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_INA219.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

// Sensor & LCD setup
Adafruit_ADS1115 ads;
Adafruit_INA219 ina219;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Ganti 0x27 jika alamat LCD berbeda

// Pin relay
const int RELAY_PIN = 32;

// Keypad setup
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {19, 18, 5, 17};
byte colPins[COLS] = {16, 4, 15};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Variabel pengukuran
float AB_cm = 0.0;
float BM_cm = 0.0;
String inputBuffer = "";
bool inputBM = false;
bool inputAB = false;
bool siapUkur = false;

// Fungsi tampil LCD
void tampilLCD(const String& baris1, const String& baris2 = "") {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(baris1);
  lcd.setCursor(0, 1);
  lcd.print(baris2);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Inisialisasi sensor
  ina219.begin();
  ads.begin(0x4A);         // ADS1115 di alamat 0x4A
  ads.setGain(GAIN_ONE);  // Gain ±4.096V

  // Inisialisasi LCD
  lcd.begin();
  lcd.backlight();

  // Inisialisasi relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  Serial.println("Schlumberger Earth Resistivity Meter Ready.");
  Serial.println("Masukkan semua jarak dalam cm. Tekan '*' untuk mulai.");

  tampilLCD("Schlumberger", "Tekan * mulai");
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    if (key == '*') {
      inputBuffer = "";
      inputBM = true;
      inputAB = false;
      siapUkur = false;
      Serial.println("Masukkan jarak dari M ke N  (cm):");
      tampilLCD("Input jarak", "M ke N (cm):");
    } 
    else if (key == '#') {
      if (inputBM) {
        BM_cm = inputBuffer.toFloat();
        inputBuffer = "";
        inputBM = false;
        inputAB = true;
        Serial.print("MN = "); Serial.print(BM_cm); Serial.println(" cm");
        tampilLCD("MN = " + String(BM_cm, 1) + " cm", "Input A ke M:");
      } 
      else if (inputAB) {
        AB_cm = inputBuffer.toFloat();
        inputBuffer = "";
        inputAB = false;
        siapUkur = true;
        Serial.print("AM = "); Serial.print(AB_cm); Serial.println(" cm");
        Serial.println("Tekan '#' untuk mulai pengukuran.");
        tampilLCD("AM = " + String(AB_cm, 1) + " cm", "Tekan # ukur");
      } 
      else if (siapUkur) {
        Serial.println("Mengukur resistivitas tanah...");
        ukurResistivitas();
        siapUkur = false;
        Serial.println("Tekan '*' untuk input ulang.");
        tampilLCD("Selesai", "Tekan * ulang");
      }
    } 
    else {
      inputBuffer += key;
      Serial.print(key);

      // Tampilkan input real-time di LCD
      if (inputBM) {
        tampilLCD("M ke N (cm):", inputBuffer);
      } else if (inputAB) {
        tampilLCD("A ke M (cm):", inputBuffer);
      }
    }
  }
}

void ukurResistivitas() {
  // Countdown 4 detik di LCD
  for (int i = 4; i > 0; i--) {
    tampilLCD("Mengukur dlm...", String(i) + " detik");
    delay(1000);
  }

  digitalWrite(RELAY_PIN, LOW);

  float current_mA = ina219.getCurrent_mA();
  float current_A = abs(current_mA / 1000.0);

  int16_t adc0 = ads.readADC_Differential_0_1();
  float voltage_V = adc0 * 0.125 / 1000.0; // 0.125 mV per LSB

  digitalWrite(RELAY_PIN, HIGH);

  Serial.print("Tegangan MN: "); Serial.print(voltage_V*1000, 6); Serial.println("mV");
  Serial.print("Arus: "); Serial.print(current_mA, 6); Serial.println(" mA");
  tampilLCD("Teg: " + String(voltage_V, 3) + "V", "Arus: " + String(current_A, 3) + "A");


  // Konversi ke meter
float AM_m = AB_cm / 100.0;       // Jarak A ke M dalam meter
float MN_m = BM_cm * 2 / 100.0;   // Jarak M ke N dikali 2, karena BM_cm adalah separuhnya (dari M ke tengah MN)

float a = AM_m;
float b = MN_m / 2.0;

float k = (3.14*((AM_m*AM_m) + (AM_m*MN_m)))/MN_m;
float rho = abs(k * (voltage_V / current_A));


  Serial.print("Resistivitas tanah: "); Serial.print(rho, 3); Serial.println(" ohm·m");
  tampilLCD("Rho: ", String(rho, 1) + " ohm.m");
  delay(2000);
}
