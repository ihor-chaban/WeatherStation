/*
    Weather Station with prediction v1.1
      - fixed out of range issue during MNK calculation
      - added map function for decimal values
      - added sensor information to the error screen
      - added debug mode
      - some code refactoring
    Author: Ihor Chaban
    Jun 2020
*/

#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>

// Software settings
#define SERVO_INVERT          1
#define SERVO_MIN             10
#define SERVO_MAX             180
#define STANDBY_TIME          10000
#define MEASURMENTS_PER_HOUR  60
#define MMHG_TO_PASCAL        133.322
#define DEBUG                 0

// Hardware settings
#define DHT_PIN               3
#define SERVO_PIN             8
#define SERVO_POWER_PIN       2
#define BACKLIGHT_BUTTON_PIN  12
#define DHTTYPE               DHT22
#define BMP180_ADDRESS        0x77
#define LCD_ADDRESS           0x27
#define LCD_WIDTH             16
#define LCD_HEIGHT            2
#define DHTPIN                DHT_PIN

#if (SERVO_INVERT == 0)
#define SERVO_LEFT  SERVO_MIN
#define SERVO_RIGHT SERVO_MAX
#else
#define SERVO_LEFT  SERVO_MAX
#define SERVO_RIGHT SERVO_MIN
#endif
#define INTERVAL (3600000 / MEASURMENTS_PER_HOUR)

Servo servo;
Adafruit_BMP085 bmp;
DHT dht(DHT_PIN, DHTTYPE);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_WIDTH, LCD_HEIGHT);
unsigned long pressure, pressure_array[MEASURMENTS_PER_HOUR], measure_timer, backlight_timer;
byte time_array[MEASURMENTS_PER_HOUR];
float temperature, humidity;
int angle, last_angle;
bool backlight_flag;

enum icons {sun_1, sun_2, cloud_1, cloud_2, termometr, droplet, celsius, barometr};
const byte icons_bytes[8][8] = {
  {0x03, 0x07, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x03},
  {0x18, 0x1C, 0x1E, 0x1E, 0x1E, 0x1E, 0x1C, 0x18},
  {0x00, 0x06, 0x09, 0x12, 0x10, 0x0F, 0x05, 0x0A},
  {0x0C, 0x12, 0x01, 0x01, 0x01, 0x1E, 0x04, 0x08},
  {0x04, 0x0A, 0x0A, 0x0A, 0x0E, 0x1F, 0x1F, 0x0E},
  {0x04, 0x04, 0x0E, 0x0E, 0x1F, 0x1F, 0x1F, 0x0E},
  {0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00},
  {0x1F, 0x05, 0x0D, 0x05, 0x1D, 0x05, 0x0F, 0x07}
};

void setup() {
#if (DEBUG)
  Serial.begin(9600);
#endif
  Wire.begin();
  pinMode(SERVO_POWER_PIN, OUTPUT);
  pinMode(BACKLIGHT_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(SERVO_POWER_PIN, HIGH);
  bmp.begin(BMP085_ULTRAHIGHRES);
  dht.begin();
  servo.attach(SERVO_PIN);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("WeatherBox");
  lcd.setCursor(0, 1);
  lcd.print("by Gorus");
  for (byte i = 0; i <= barometr; i++) {
    lcd.createChar(i, icons_bytes[i]);
  }
  lcd.setCursor(LCD_WIDTH - 2, 0);
  lcd.write(sun_1);
  lcd.write(sun_2);
  lcd.setCursor(LCD_WIDTH - 2, 1);
  lcd.write(cloud_1);
  lcd.write(cloud_2);
  delay(1000);
  servo.write(SERVO_LEFT);
  delay(1000);
  servo.write(SERVO_RIGHT);
  delay(1000);
  angle = round((SERVO_LEFT + SERVO_RIGHT) / 2.0);
  last_angle = angle;
  servo.write(angle);
  delay(1000);
  digitalWrite(SERVO_POWER_PIN, LOW);
  pressure = ReadAveragePressure();
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  measure_timer = millis();
  Wire.beginTransmission(BMP180_ADDRESS);
  if (Wire.endTransmission()) {
    ShowErrorScreen(true);
  }
  if (isnan(temperature) || isnan(humidity)) {
    ShowErrorScreen(false);
  }
  for (byte i = 0; i < MEASURMENTS_PER_HOUR; i++) {
    time_array[i] = i;
  }
  pressure_array[MEASURMENTS_PER_HOUR - 1] = pressure;
  PrintValuesToLCD();
  backlight_flag = true;
  backlight_timer = millis();
#if (DEBUG)
  Serial.println();
#endif
}

void loop() {
  if (millis() - measure_timer >= INTERVAL) {
    measure_timer = millis();
    pressure = ReadAveragePressure();
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    Wire.beginTransmission(BMP180_ADDRESS);
    if (Wire.endTransmission()) {
      ShowErrorScreen(true);
    }
    if (isnan(temperature) || isnan(humidity)) {
      ShowErrorScreen(false);
    }
    for (byte i = 0; i < MEASURMENTS_PER_HOUR - 1; i++) {
      pressure_array[i] = pressure_array[i + 1];
    }
    pressure_array[MEASURMENTS_PER_HOUR - 1] = pressure;
    PrintValuesToLCD();
    if (pressure_array[0]) {
      angle = round(mapf(CalculateMNKCoefficient() * MEASURMENTS_PER_HOUR, -250, 250, SERVO_LEFT, SERVO_RIGHT));
      angle = constrain(angle, min(SERVO_LEFT, SERVO_RIGHT), max(SERVO_LEFT, SERVO_RIGHT));
      if (angle != last_angle) {
        digitalWrite(SERVO_POWER_PIN, HIGH);
        delay(1000);
        servo.write(angle);
        delay(1000);
        digitalWrite(SERVO_POWER_PIN, LOW);
        last_angle = angle;
      }
#if (DEBUG)
      Serial.println("Set servo on angle " + String(angle));
#endif
    }
#if (DEBUG)
    Serial.println();
#endif
  }
  if (!digitalRead(BACKLIGHT_BUTTON_PIN)) {
    if (!backlight_flag) {
      backlight_flag = true;
      lcd.backlight();
    }
    backlight_timer = millis();
  }
  if (backlight_flag && millis() - backlight_timer >= STANDBY_TIME) {
    backlight_flag = false;
    lcd.noBacklight();
  }
}

float CalculateMNKCoefficient() {
  float result = 0;
  unsigned long sum_x = 0;
  unsigned long sum_y = 0;
  unsigned long sum_x2 = 0;
  unsigned long sum_xy = 0;
  long temp = 0;
  for (byte i = 0; i < MEASURMENTS_PER_HOUR; i++) {
    sum_x += time_array[i];
    sum_y += pressure_array[i];
    sum_x2 += time_array[i] * time_array[i];
    sum_xy += time_array[i] * pressure_array[i];
  }
  temp = MEASURMENTS_PER_HOUR * sum_xy;
  temp = temp - (sum_x * sum_y);
  result = (float)temp / ((MEASURMENTS_PER_HOUR * sum_x2) - (sum_x * sum_x));
#if (DEBUG)
  Serial.print("MNK coefficient is " + String(result));
#endif
  return result;
}

unsigned long ReadAveragePressure() {
  unsigned long sum = 0;
  for (byte i = 0; i < 10; i++) {
    sum += bmp.readPressure();
  }
  return (round(sum / 10.0) - 90000);
}

void PrintValuesToLCD() {
  String temp_string;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(termometr);
  if (temperature > 0) {
    lcd.print(" ");
  }
  temp_string = String(temperature);
  temp_string.remove(temp_string.length() - 1);
  lcd.print(temp_string);
  lcd.write(celsius);
  lcd.print("C");
  lcd.setCursor(LCD_WIDTH - 7, 0);
  lcd.write(droplet);
  lcd.print(" ");
  temp_string = String(humidity);
  temp_string.remove(temp_string.length() - 1);
  lcd.print(temp_string);
  lcd.print("%");
  lcd.setCursor((LCD_WIDTH / 2) - 7, 1);
  lcd.write(barometr);
  lcd.print(" ");
  temp_string = String((pressure + 90000) / MMHG_TO_PASCAL);
  lcd.print(temp_string);
  lcd.print(" mm Hg");
#if (DEBUG)
  Serial.println("Pressure array");
  for (byte i = 0; i < MEASURMENTS_PER_HOUR; i++) {
    Serial.println("\"" + String(time_array[i]) + "\",\"" + String(pressure_array[i]) + "\"");
  }
  Serial.println("Temperature " + String(temperature) + "°C, Humidity " +
                 String(humidity) + "%, Pressure " + String((pressure + 90000) / MMHG_TO_PASCAL) + " mm Hg");
#endif
}

void ShowErrorScreen(bool bmp_or_dht) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("     Error!     ");
  lcd.setCursor(0, 1);
  lcd.print((bmp_or_dht) ? "BMP" : "DHT");
  lcd.print(" sens is down");
  lcd.backlight();
  digitalWrite(SERVO_POWER_PIN, HIGH);
  delay(1000);
  servo.write(round((SERVO_LEFT + SERVO_RIGHT) / 2.0));
  delay(1000);
  digitalWrite(SERVO_POWER_PIN, LOW);
#if (DEBUG)
  Serial.println("Error! " + String((bmp_or_dht) ? "BMP" : "DHT") + " sensor is down");
#endif
  while (true) {}
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
