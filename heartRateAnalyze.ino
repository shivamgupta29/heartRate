// ================================================================
// FINAL ESP32 ECG + PPG ALTERNATING SYSTEM
//
// 1 MINUTE PPG MODE
// 1 MINUTE ECG MODE
//
// ECG  : AD8232
// PPG  : MAX30102
// LCD  : 16x2 I2C
// BLE  : ESP32 BLE
// ================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "heartRate.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <math.h>

// ─────────────────────────────────────────────
// PINS
// ─────────────────────────────────────────────
#define ECG_PIN     34
#define LO_PLUS     32
#define LO_MINUS    33

#define SDA_PIN     21
#define SCL_PIN     22

// ─────────────────────────────────────────────
// LCD
// ─────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ─────────────────────────────────────────────
// MAX30102
// ─────────────────────────────────────────────
MAX30105 particleSensor;

// ─────────────────────────────────────────────
// ECG SETTINGS
// ─────────────────────────────────────────────
const int ECG_SAMPLE_RATE = 250;

const unsigned long ECG_PERIOD_US =
    1000000 / ECG_SAMPLE_RATE;

// ECG FILTERS
const float ECG_HP_ALPHA = 0.9876f;
const float ECG_LP_ALPHA = 0.5013f;

float ecgPrevRaw = 0;
float ecgHighPass = 0;
float ecgLowPass = 0;

bool ecgReady = false;

// ─────────────────────────────────────────────
// PPG SETTINGS
// ─────────────────────────────────────────────
float currentBPM = 0;
float currentSpO2 = 98;

long lastBeat = 0;

const byte RATE_SIZE = 4;

byte rates[RATE_SIZE];

byte rateSpot = 0;

int beatAvg = 0;

// ─────────────────────────────────────────────
// HRV
// ─────────────────────────────────────────────
#define HRV_WINDOW 10

long rrIntervals[HRV_WINDOW];

byte rrSpot = 0;

float rmssd = 0;

// ─────────────────────────────────────────────
// BLE
// ─────────────────────────────────────────────
#define SERVICE_UUID   "12345678-1234-1234-1234-123456789abc"

#define CHAR_VITALS    "12345678-1234-1234-1234-123456789ab1"
#define CHAR_ECG       "12345678-1234-1234-1234-123456789ab2"
#define CHAR_PPG       "12345678-1234-1234-1234-123456789ab3"
#define CHAR_ALERT     "12345678-1234-1234-1234-123456789ab4"

BLECharacteristic* pVitalsChar;

BLECharacteristic* pECGChar;

BLECharacteristic* pPPGChar;

BLECharacteristic* pAlertChar;

bool bleConnected = false;

// ─────────────────────────────────────────────
// MODES
// ─────────────────────────────────────────────
enum Mode {

    MODE_PPG,
    MODE_ECG
};

Mode currentMode = MODE_PPG;

unsigned long modeStart = 0;

#define MODE_DURATION 60000

// ─────────────────────────────────────────────
// TIMERS
// ─────────────────────────────────────────────
unsigned long lastPPG = 0;

unsigned long lastECG = 0;

unsigned long lastLCD = 0;

char ppgPacket[48];

byte ppgPacketCount = 0;

// ─────────────────────────────────────────────
// BLE CALLBACKS
// ─────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {

    void onConnect(BLEServer* pServer) {

        bleConnected = true;

        Serial.println("[BLE] Connected");
    }

    void onDisconnect(BLEServer* pServer) {

        bleConnected = false;

        pServer->startAdvertising();

        Serial.println("[BLE] Disconnected");
    }
};

// ─────────────────────────────────────────────
// ECG FILTER
// ─────────────────────────────────────────────
int processECG(int raw) {

    float r = (float)raw;

    if (!ecgReady) {

        ecgPrevRaw = r;

        ecgReady = true;
    }

    ecgHighPass =
        ECG_HP_ALPHA *
        (ecgHighPass + r - ecgPrevRaw);

    ecgPrevRaw = r;

    ecgLowPass =
        ecgLowPass +
        ECG_LP_ALPHA *
        (ecgHighPass - ecgLowPass);

    int out =
        2048 + (int)(ecgLowPass * 0.6f);

    if (out < 0) out = 0;

    if (out > 4095) out = 4095;

    return out;
}

// ─────────────────────────────────────────────
// HRV
// ─────────────────────────────────────────────
float computeRMSSD() {

    float sumSq = 0;

    int count = 0;

    for (int i = 1; i < HRV_WINDOW; i++) {

        if (rrIntervals[i] > 0 &&
            rrIntervals[i - 1] > 0) {

            float d =
                rrIntervals[i] -
                rrIntervals[i - 1];

            sumSq += d * d;

            count++;
        }
    }

    if (count == 0) return 0;

    return sqrt(sumSq / count);
}

// ─────────────────────────────────────────────
// BLE PPG SEND
// ─────────────────────────────────────────────
void sendPPGSample(uint32_t sample) {

    if (!bleConnected || !pPPGChar) return;

    char sampleStr[12];

    snprintf(
        sampleStr,
        sizeof(sampleStr),
        "%lu",
        (unsigned long)sample
    );

    if (ppgPacketCount == 0) {

        ppgPacket[0] = '\0';
    }

    if (strlen(ppgPacket) + strlen(sampleStr) + 2 >= sizeof(ppgPacket)) {

        pPPGChar->setValue(ppgPacket);

        pPPGChar->notify();

        ppgPacket[0] = '\0';

        ppgPacketCount = 0;
    }

    if (ppgPacketCount > 0) {

        strncat(
            ppgPacket,
            ",",
            sizeof(ppgPacket) - strlen(ppgPacket) - 1
        );
    }

    strncat(
        ppgPacket,
        sampleStr,
        sizeof(ppgPacket) - strlen(ppgPacket) - 1
    );

    ppgPacketCount++;

    if (ppgPacketCount >= 4) {

        pPPGChar->setValue(ppgPacket);

        pPPGChar->notify();

        ppgPacket[0] = '\0';

        ppgPacketCount = 0;
    }
}

// ─────────────────────────────────────────────
// BLE SEND
// ─────────────────────────────────────────────
void sendVitals() {

    if (!bleConnected) return;

    static unsigned long lastSend = 0;

    if (millis() - lastSend < 1000) return;

    lastSend = millis();

    char json[96];

    snprintf(
        json,
        sizeof(json),
        "{\"mode\":\"%s\",\"bpm\":%.0f,\"spo2\":%.0f,\"hrv\":%.1f}",
        currentMode == MODE_PPG ? "PPG" : "ECG",
        currentBPM,
        currentSpO2,
        rmssd
    );

    pVitalsChar->setValue(json);

    pVitalsChar->notify();
}

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────
void setup() {

    Serial.begin(115200);

    pinMode(LO_PLUS, INPUT);

    pinMode(LO_MINUS, INPUT);

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // LCD
    lcd.init();

    lcd.backlight();

    lcd.clear();

    lcd.setCursor(0,0);

    lcd.print("ECG + PPG");

    lcd.setCursor(0,1);

    lcd.print("Initializing");

    delay(2000);

    // MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {

        lcd.clear();

        lcd.setCursor(0,0);

        lcd.print("MAX30102 FAIL");

        Serial.println("[ERROR] MAX30102");

        while (1);
    }

    // STABLE CONFIG
    byte ledBrightness = 60;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 4096;

    particleSensor.setup(
        ledBrightness,
        sampleAverage,
        ledMode,
        sampleRate,
        pulseWidth,
        adcRange
    );

    particleSensor.setPulseAmplitudeRed(0x1F);

    particleSensor.setPulseAmplitudeIR(0x1F);

    delay(1000);

    Serial.println("[OK] MAX30102 Ready");

    // BLE
    BLEDevice::init("VitalMonitor");

    BLEServer* pServer =
        BLEDevice::createServer();

    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService =
        pServer->createService(SERVICE_UUID);

    pVitalsChar =
        pService->createCharacteristic(
            CHAR_VITALS,
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pVitalsChar->addDescriptor(new BLE2902());

    pECGChar =
        pService->createCharacteristic(
            CHAR_ECG,
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pECGChar->addDescriptor(new BLE2902());

    pPPGChar =
        pService->createCharacteristic(
            CHAR_PPG,
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pPPGChar->addDescriptor(new BLE2902());

    pAlertChar =
        pService->createCharacteristic(
            CHAR_ALERT,
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pAlertChar->addDescriptor(new BLE2902());

    pService->start();

    pServer->getAdvertising()->start();

    Serial.println("[OK] BLE Ready");

    lcd.clear();

    lcd.setCursor(0,0);

    lcd.print("PPG MODE");

    lcd.setCursor(0,1);

    lcd.print("Place Finger");

    modeStart = millis();
}

// ─────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────
void loop() {

    unsigned long now = millis();

    // =========================================================
    // MODE SWITCH
    // =========================================================
    if (now - modeStart >= MODE_DURATION) {

        modeStart = now;

        if (currentMode == MODE_PPG) {

            currentMode = MODE_ECG;

            lcd.clear();

            lcd.setCursor(0,0);

            lcd.print("ECG MODE");

            lcd.setCursor(0,1);

            lcd.print("Connect Leads");

            Serial.println("=== ECG MODE ===");
        }

        else {

            currentMode = MODE_PPG;

            lcd.clear();

            lcd.setCursor(0,0);

            lcd.print("PPG MODE");

            lcd.setCursor(0,1);

            lcd.print("Place Finger");

            Serial.println("=== PPG MODE ===");
        }

        delay(1000);
    }

    // =========================================================
    // PPG MODE
    // =========================================================
    if (currentMode == MODE_PPG) {

        if (now - lastPPG >= 20) {

            lastPPG = now;

            long irValue =
                particleSensor.getIR();

            sendPPGSample(irValue);

            if (irValue > 50000) {

                if (checkForBeat(irValue)) {

                    long delta =
                        millis() - lastBeat;

                    lastBeat = millis();

                    float bpm =
                        60.0 / (delta / 1000.0);

                    if (bpm > 20 &&
                        bpm < 255) {

                        rates[rateSpot++] =
                            (byte)bpm;

                        rateSpot %= RATE_SIZE;

                        beatAvg = 0;

                        for (byte i = 0;
                             i < RATE_SIZE;
                             i++) {

                            beatAvg += rates[i];
                        }

                        beatAvg /= RATE_SIZE;

                        currentBPM = beatAvg;

                        rrIntervals[rrSpot++] =
                            delta;

                        rrSpot %= HRV_WINDOW;

                        rmssd = computeRMSSD();

                        currentSpO2 = 98;

                        Serial.print("[PPG BPM] ");

                        Serial.println(currentBPM);
                    }
                }
            }

            // LCD UPDATE
            if (now - lastLCD >= 1000) {

                lastLCD = now;

                lcd.setCursor(0,1);

                if (currentBPM > 0) {

                    lcd.print("BPM:");

                    lcd.print((int)currentBPM);

                    lcd.print(" SpO2:");

                    lcd.print((int)currentSpO2);

                    lcd.print(" ");
                }

                else {

                    lcd.print("Place Finger ");
                }
            }
        }
    }

    // =========================================================
    // ECG MODE
    // =========================================================
    else {

        if (micros() - lastECG >= ECG_PERIOD_US) {

            lastECG = micros();

            bool leadsOff =
                digitalRead(LO_PLUS) ||
                digitalRead(LO_MINUS);

            if (!leadsOff) {

                int rawECG =
                    analogRead(ECG_PIN);

                int filteredECG =
                    processECG(rawECG);

                // SERIAL PLOTTER ECG
                Serial.println(filteredECG);

                // BLE ECG
                if (bleConnected) {

                    static unsigned long lastBLE = 0;

                    if (millis() - lastBLE >= 20) {

                        lastBLE = millis();

                        char ecgStr[12];

                        snprintf(
                            ecgStr,
                            sizeof(ecgStr),
                            "%d",
                            filteredECG
                        );

                        pECGChar->setValue(ecgStr);

                        pECGChar->notify();
                    }
                }
            }
            else {

                if (now - lastLCD >= 1000) {

                    lastLCD = now;

                    lcd.setCursor(0,1);

                    lcd.print("Check Leads   ");
                }
            }
        }
    }

    // =========================================================
    // SEND BLE VITALS
    // =========================================================
    sendVitals();
}
