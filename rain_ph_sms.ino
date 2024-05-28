#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024  // set RX buffer to 1Kb
#define SerialAT Serial1
#define SerialMon Serial
#define DUMP_AT_COMMANDS
#define SMS_TARGET "+84*********"  // target phone number
#define GSM_PIN ""
const char apn[] = "YOUR-APN";  //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

#define rain_pulse 22  // rain fall pin
#define pH_pin 39      // pH pin
#define Offset 0.6745
#define LED 12  // Led pin
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth 40  //times of collection
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

#include "ThingSpeak.h"
#include <WiFi.h>
#include <TinyGsmClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint16_t rain_count = 0;
static float pHValue = 0;
static float voltage = 0;
unsigned long lastRainTime = 0;
const unsigned long debounceDelay = 100;
int pHArray[ArrayLenth];
int pHArrayIndex = 0;

bool rainWarning = false;
bool pHLow = false;
bool pHHigh = false;
bool gpsLocation = false;

// WIFI
const char *ssid = "Vo tuyen 217";     // your network SSID (name)
const char *password = "votuyen217@";  // your network password
WiFiClient client;

// ThingSpeak
unsigned long myChannelNumber = 2519307;         // thingspeak Channel ID
const char *myWriteAPIKey = "LEZA8G40BLR4R00W";  //Write API Key
TinyGsm modem(SerialAT);

void IRAM_ATTR get_rain() {
  unsigned long currentTime = millis();
  if (currentTime - lastRainTime >= debounceDelay) {
    portENTER_CRITICAL_ISR(&mux);
    rain_count++;
    portEXIT_CRITICAL_ISR(&mux);
    lastRainTime = currentTime;
  }
}

void enableGPS(void) {
  // Set Modem GPS Power Control Pin to HIGH ,turn on GPS power
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+CGPIO=0,48,1,1");
  if (modem.waitResponse(10000L) != 1) {
    DBG("Set GPS Power HIGH Failed");
  }
  modem.enableGPS();
}

void disableGPS(void) {
  // Set Modem GPS Power Control Pin to LOW ,turn off GPS power
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+CGPIO=0,48,1,0");
  if (modem.waitResponse(10000L) != 1) {
    DBG("Set GPS Power LOW Failed");
  }
  modem.disableGPS();
}

void modemPowerOn() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(PWR_PIN, LOW);
}

void modemPowerOff() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(1500);  //Datasheet Ton mintues = 1.2S
  digitalWrite(PWR_PIN, LOW);
}

void modemRestart() {
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(rain_pulse, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rain_pulse), get_rain, FALLING);
  Serial.begin(115200);
  delay(10);
  modemPowerOn();
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(10);

  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);
  Serial.println("\nWait...");
  delay(1000);

  Serial.println("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }

  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED) {
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      delay(4000);
    }
  }
  ThingSpeak.begin(client);
  delay(2000);
}

void loop() {
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();

  if (millis() - samplingTime > samplingInterval) {
    pHArray[pHArrayIndex++] = analogRead(pH_pin);
    if (pHArrayIndex == ArrayLenth) pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 3.3 / 4096;
    pHValue = 3.7142 * voltage + Offset;
    samplingTime = millis();
  }

  if (millis() - printTime > printInterval) {
    Serial.print("  pH value: ");
    Serial.println(pHValue, 2);
    ThingSpeak.setField(2, pHValue);
    digitalWrite(LED, digitalRead(LED) ^ 1);
    Serial.print("  Rain fall: ");
    Serial.println(rain_count * 0.2);
    int rain_fall = rain_count * 0.2;
    ThingSpeak.setField(1, rain_fall);
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    delay(2000);

    if (Serial.available() > 0) {
      if (Serial.read() == '0') {
        portENTER_CRITICAL_ISR(&mux);
        rain_count = 0;
        portEXIT_CRITICAL_ISR(&mux);
      }
    }
    if (!modem.testAT()) {
      return;
    }
    // Send SMS if conditions are met and SMS hasn't been sent yet
    if (!rainWarning && rain_count >= 50) {
      sendSMS("Canh bao mua: nguy co xay ra sat lo dat");
      rainWarning = true;  // Mark sent sms
    }
    if (!pHLow && (pHValue <= 6.5)) {
      sendSMS(" Canh bao: xuat hien mua axit !");
      pHLow = true;
    }
    if (!pHHigh && (pHValue >= 8.5)) {
      sendSMS("Canh bao: mua co pH cao");
      pHHigh = true;
    }
    if ((!gpsLocation) && (rainWarning || pHLow || pHHigh)) {
      // get data GPS
      enableGPS();
      float lat, lon;
      while (1) {
        if (modem.getGPS(&lat, &lon)) {
          String gpsData = "The location: https://www.google.com/maps?q=" + String(lat, 6) + "," + String(lon, 6);
          sendSMS(gpsData);
          gpsLocation = true;
          break;
        }
        delay(2000);
      }
      disableGPS();
    }
    printTime = millis();
  }

  while (SerialAT.available()) {
    SerialMon.write(SerialAT.read());
  }
}

void sendSMS(String message) {
  Serial.println("Sending SMS...");
  if (!modem.init()) {
    Serial.println("Failed to initialize modem, skipping SMS sending");
    return;
  }
  modem.sendSMS(SMS_TARGET, message);
  delay(500);
}

double avergearray(int *arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to averaging!");
    return 0;
  }
  if (number < 5) {
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0];
      max = arr[1];
    } else {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;
          max = arr[i];
        } else {
          amount += arr[i];
        }
      }
    }
    avg = (double)amount / (number - 2);
  }
  return avg;
}
