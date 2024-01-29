#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoBLE.h>

#define I2C_SDA 27
#define I2C_SCL 26
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

int motor1Pin1 = 16;
int motor1Pin2 = 17;
int enable1Pin = 5;

int motor2Pin1 = 15;
int motor2Pin2 = 2;
int enable2Pin = 4;
// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 200;

#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);
  Serial.println("Hello Computer");
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

  BLE.begin();

  Serial.println("Tank. Pew Pew");

  // start scanning for peripherals
  BLE.scanForUuid("E6B81F14-F9E5-40C9-A739-4DE4564264D1");
}

void acceptControl(BLEDevice peripheral) {
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  Serial.print("char count: ");
  Serial.println(peripheral.characteristicCount());

  Serial.print("char 0 uuid: ");
  Serial.println(peripheral.characteristic(0).uuid());

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }


  BLECharacteristic lxChar = peripheral.characteristic("6AB4");
  BLECharacteristic lyChar = peripheral.characteristic("A096");
  BLECharacteristic lwChar = peripheral.characteristic("11D2");
  BLECharacteristic rxChar = peripheral.characteristic("87B4");
  BLECharacteristic ryChar = peripheral.characteristic("B5EC");
  BLECharacteristic rwChar = peripheral.characteristic("3A8E");
  lxChar.subscribe();
  lyChar.subscribe();
  lwChar.subscribe();
  rxChar.subscribe();
  ryChar.subscribe();
  rwChar.subscribe();

  if (!lxChar || !lyChar || !lwChar || !rxChar || !ryChar || !rwChar) {
    Serial.println("Peripheral does not have characteristics!");
    peripheral.disconnect();
    return;
  }

  int ry = 1900;
  int rx = 1900;
  int rw = 0;
  int ly = 1900;
  int lx = 1900;
  int lw = 0;

  while (peripheral.connected()) {

    display.clearDisplay();
    display.setTextColor(WHITE);
    if (lxChar.valueUpdated()) {
      lxChar.readValue(lx);
      Serial.print("value updated: lx = ");
      Serial.println(lx);
    }
    if (lyChar.valueUpdated()) {
      lyChar.readValue(ly);
      Serial.print("value updated: ly = ");
      Serial.println(ly);
    }
    if (lwChar.valueUpdated()) {
      lwChar.readValue(lw);
      Serial.print("value updated: lw = ");
      Serial.println(lw);
    }
    if (rxChar.valueUpdated()) {
      rxChar.readValue(rx);
      Serial.print("value updated: rx = ");
      Serial.println(rx);
    }
    if (ryChar.valueUpdated()) {
      ryChar.readValue(ry);
      Serial.print("value updated: ry = ");
      Serial.println(ry);
    }
    if (rwChar.valueUpdated()) {
      rwChar.readValue(rw);
      Serial.print("value updated: rw = ");
      Serial.println(rw);
    }
    if (ry < 4096 / 3) {

      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      int duty = (4096 / 3 - ry) * 256 / (4096 / 3);
      ledcWrite(pwmChannel1, duty);
      display.setCursor(0, 20);
      display.print("+" + String(duty, DEC));
    } else if (ry > 2 * 4096 / 3) {

      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      int duty = (ry - 2 * 4096 / 3) * 256 / (4096 / 3);
      ledcWrite(pwmChannel1, duty);
      display.setCursor(0, 20);
      display.print("-" + String(duty, DEC));
    } else {
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
      ledcWrite(pwmChannel1, 0);
      display.setCursor(0, 20);
      display.print("0");
    }
    if (ly < 4096 / 3) {

      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      int duty = (4096 / 3 - ly) * 256 / (4096 / 3);
      ledcWrite(pwmChannel2, duty);
      display.setCursor(64, 20);
      display.print("+" + String(duty, DEC));
    } else if (ly > 2 * 4096 / 3) {

      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      int duty = (ly - 2 * 4096 / 3) * 256 / (4096 / 3);
      ledcWrite(pwmChannel2, duty);
      display.setCursor(64, 20);
      display.print("-" + String(duty, DEC));
    } else {
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW);
      ledcWrite(pwmChannel2, 0);
      display.setCursor(64, 20);
      display.print("0");
    }
    display.display();
  }
}

void loop() {
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    BLE.stopScan();
    delay(100);

    acceptControl(peripheral);
    BLE.scanForUuid("E6B81F14-F9E5-40C9-A739-4DE4564264D1");
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Connect");
  display.setCursor(0, 24);
  display.print("Controller");
  display.display();
  delay(100);
}
