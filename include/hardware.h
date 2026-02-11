#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include "config.h"
#include "types.h"
#include "protocol.h"

// ============================================================================
// OBJETOS GLOBALES DE HARDWARE
// ============================================================================
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
extern SX1262 radio;
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;

// Datos GPS locales
extern DatosGPS myGPS;

// Brújula
extern float compassHeading;
extern bool compassOK;

// LoRa recepción
extern volatile bool loraReceivedFlag;
extern float lastRxRSSI;
extern float lastRxSNR;

// ============================================================================
// FUNCIONES MATEMÁTICAS GPS (del código original)
// ============================================================================

inline float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    lat1 = lat1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x) * RAD_TO_DEG;
    bearing = fmod((bearing + 360.0), 360.0);
    return (float)bearing;
}

inline float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return (float)(R * c);
}

// ============================================================================
// ISR LORA (declaración - implementación en main.cpp por IRAM_ATTR)
// ============================================================================
void IRAM_ATTR loraISR();

// ============================================================================
// INICIALIZACIÓN DE HARDWARE
// ============================================================================

inline void initVext() {
    pinMode(VEXT_ENABLE, OUTPUT);
    digitalWrite(VEXT_ENABLE, LOW);
    delay(100);
}

inline void initDisplay() {
    display.begin();
    display.enableUTF8Print();
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB10_tr);
    display.drawStr(10, 20, "HELTEC V4");
    display.setFont(u8g2_font_ncenB08_tr);
    display.drawStr(10, 40, "LoRa Search");
    display.drawStr(10, 55, "Iniciando...");
    display.sendBuffer();
    Serial.println("[HW] Display OK");
}

inline void initGPSHardware() {
    pinMode(GPS_ENABLE, OUTPUT);
    digitalWrite(GPS_ENABLE, LOW);
    delay(100);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("[HW] GPS OK");
}

inline bool initLoRaHardware() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int state = radio.begin(
        (float)RF_FREQUENCY / 1000000.0,
        LORA_BANDWIDTH, LORA_SF, LORA_CR + 4,
        LORA_SYNC_WORD, TX_OUTPUT_POWER, LORA_PREAMBLE, 1.8
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[HW] LoRa ERROR: %d\n", state);
        return false;
    }

    radio.setRegulatorDCDC();
    radio.setDio2AsRfSwitch(true);
    radio.setCurrentLimit(140.0);
    radio.setDio1Action(loraISR);

    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[HW] LoRa RX ERROR: %d\n", state);
        return false;
    }

    Serial.printf("[HW] LoRa OK (ID: 0x%02X)\n", getNodeId());
    return true;
}

// ============================================================================
// BRÚJULA (MAGNETÓMETRO) - Bus I2C separado (Wire1)
// ============================================================================

// Variables internas de brújula
namespace compass_internal {
    inline uint8_t MAG_ADDR = 0;
    inline bool isQMC = false;
}

inline bool initCompassHardware() {
    using namespace compass_internal;

    Wire1.begin(MAG_SDA, MAG_SCL);
    Wire1.setClock(400000);

    for (uint8_t addr : {0x0D, 0x1E}) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) {
            MAG_ADDR = addr;
            break;
        }
    }

    if (MAG_ADDR == 0x0D) {
        isQMC = true;
        // Init QMC5883L
        Wire1.beginTransmission(MAG_ADDR);
        Wire1.write(0x0B); Wire1.write(0x01);
        if (Wire1.endTransmission() != 0) return false;
        delay(10);
        uint8_t ctrl = (0b11 << 6) | (0b01 << 4) | (0b10 << 2) | 0b01;
        Wire1.beginTransmission(MAG_ADDR);
        Wire1.write(0x09); Wire1.write(ctrl);
        compassOK = (Wire1.endTransmission() == 0);
        Serial.printf("[HW] Brujula QMC5883L %s\n", compassOK ? "OK" : "ERROR");
    } else if (MAG_ADDR == 0x1E) {
        isQMC = false;
        // Init HMC5883L
        Wire1.beginTransmission(MAG_ADDR);
        Wire1.write(0x00); Wire1.write(0x70);
        if (Wire1.endTransmission() != 0) return false;
        Wire1.beginTransmission(MAG_ADDR);
        Wire1.write(0x01); Wire1.write(0xA0);
        if (Wire1.endTransmission() != 0) return false;
        Wire1.beginTransmission(MAG_ADDR);
        Wire1.write(0x02); Wire1.write(0x00);
        compassOK = (Wire1.endTransmission() == 0);
        Serial.printf("[HW] Brujula HMC5883L %s\n", compassOK ? "OK" : "ERROR");
    } else {
        Serial.println("[HW] Sin brujula detectada");
        compassOK = false;
    }
    return compassOK;
}

inline void updateCompass() {
    using namespace compass_internal;
    if (!compassOK) return;

    int16_t mx = 0, my = 0, mz = 0;
    bool ok = false;

    Wire1.beginTransmission(MAG_ADDR);
    Wire1.write(isQMC ? 0x00 : 0x03);
    if (Wire1.endTransmission(false) != 0) return;
    Wire1.requestFrom((int)MAG_ADDR, 6);
    if (Wire1.available() < 6) return;

    if (isQMC) {
        mx = Wire1.read() | (Wire1.read() << 8);
        my = Wire1.read() | (Wire1.read() << 8);
        mz = Wire1.read() | (Wire1.read() << 8);
    } else {
        mx = (Wire1.read() << 8) | Wire1.read();
        mz = (Wire1.read() << 8) | Wire1.read();
        my = (Wire1.read() << 8) | Wire1.read();
    }
    ok = true;

    if (ok) {
        float heading = atan2((float)my, (float)mx);
        if (heading < 0) heading += 2.0f * PI;
        if (heading >= 2.0f * PI) heading -= 2.0f * PI;
        compassHeading = heading * 180.0f / M_PI;
    }
}

inline void updateGPS() {
    static bool wasValid = false;
    static uint32_t lastGPSLog = 0;

    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());

        if (gps.location.isUpdated()) {
            myGPS.lat = gps.location.lat();
            myGPS.lon = gps.location.lng();
            myGPS.alt = gps.altitude.meters();
            myGPS.sats = gps.satellites.value();
            myGPS.valid = gps.location.isValid();
            myGPS.lastUpdate = millis();
        }
    }

    // Invalidar si no hay actualización por 5s
    if (myGPS.valid && (millis() - myGPS.lastUpdate > 5000)) {
        myGPS.valid = false;
    }

    // Debug: cambio de estado GPS
    if (myGPS.valid != wasValid) {
        DBG("GPS", "Fix %s (sats=%d)", myGPS.valid ? "ADQUIRIDO" : "PERDIDO", myGPS.sats);
        wasValid = myGPS.valid;
    }

    // Debug periódico
    if (millis() - lastGPSLog >= DEBUG_GPS_INTERVAL) {
        DBG("GPS", "valid=%d sats=%d lat=%.5f lon=%.5f",
            myGPS.valid, myGPS.sats, myGPS.lat, myGPS.lon);
        lastGPSLog = millis();
    }
}

// ============================================================================
// COMUNICACIÓN LORA
// ============================================================================

inline bool sendLoRaMsg(MensajeLoRa& msg) {
    // Salvar flag antes de TX por si llega algo durante standby
    loraReceivedFlag = false;

    int state = radio.standby();
    if (state != RADIOLIB_ERR_NONE) {
        DBG("TX", "ERROR standby: %d", state);
        radio.startReceive();
        return false;
    }

    state = radio.transmit((uint8_t*)&msg, sizeof(MensajeLoRa));
    bool ok = (state == RADIOLIB_ERR_NONE);

    if (ok) {
        DBG("TX", "%s -> 0x%02X seq=%d flags=0x%02X modo=%d",
            nombreTipoMsg(msg.tipo), msg.destino, msg.secuencia,
            msg.flags, msg.modo);
    } else {
        DBG("TX", "ERROR transmit: %d", state);
    }

    // Volver a RX inmediatamente - NO borrar loraReceivedFlag
    radio.startReceive();
    return ok;
}

inline bool checkLoRaRx(MensajeLoRa& msg) {
    if (!loraReceivedFlag) return false;
    loraReceivedFlag = false;

    uint8_t buf[MSG_BUFFER_SIZE];
    int state = radio.readData(buf, MSG_BUFFER_SIZE);

    if (state != RADIOLIB_ERR_NONE) {
        DBG("RX", "ERROR readData: %d", state);
        radio.startReceive();
        return false;
    }

    int len = radio.getPacketLength();
    if (len != sizeof(MensajeLoRa)) {
        DBG("RX", "Tamaño invalido: %d (esperado %d)", len, (int)sizeof(MensajeLoRa));
        radio.startReceive();
        return false;
    }

    lastRxRSSI = radio.getRSSI();
    lastRxSNR = radio.getSNR();

    memcpy(&msg, buf, sizeof(MensajeLoRa));
    radio.startReceive();

    if (!validarMsg(msg)) {
        DBG("RX", "Mensaje invalido (magic=0x%02X chk=0x%02X)", msg.magic, msg.checksum);
        return false;
    }

    DBG("RX", "%s de 0x%02X -> 0x%02X seq=%d flags=0x%02X RSSI=%.0f SNR=%.1f",
        nombreTipoMsg(msg.tipo), msg.origen, msg.destino, msg.secuencia,
        msg.flags, lastRxRSSI, lastRxSNR);

    return true;
}

// ============================================================================
// LED - PATRONES NO BLOQUEANTES
// ============================================================================

namespace led {
    inline PatronLED patronActual = LED_OFF;
    inline uint32_t lastToggle = 0;
    inline uint8_t pulseStep = 0;
    inline bool ledState = false;

    inline void setPatron(PatronLED p) {
        if (p != patronActual) {
            patronActual = p;
            pulseStep = 0;
            lastToggle = millis();
            if (p == LED_OFF) {
                digitalWrite(LED_PIN, LOW);
                ledState = false;
            } else if (p == LED_SOLID) {
                digitalWrite(LED_PIN, HIGH);
                ledState = true;
            }
        }
    }

    inline void update() {
        uint32_t now = millis();

        switch (patronActual) {
            case LED_OFF:
            case LED_SOLID:
                break;

            case LED_BLINK_SLOW:
                if (now - lastToggle >= 500) {
                    ledState = !ledState;
                    digitalWrite(LED_PIN, ledState);
                    lastToggle = now;
                }
                break;

            case LED_BLINK_FAST:
                if (now - lastToggle >= 150) {
                    ledState = !ledState;
                    digitalWrite(LED_PIN, ledState);
                    lastToggle = now;
                }
                break;

            case LED_BLINK_VERY_FAST:
                if (now - lastToggle >= 50) {
                    ledState = !ledState;
                    digitalWrite(LED_PIN, ledState);
                    lastToggle = now;
                }
                break;

            case LED_DOUBLE_PULSE:
                // 2 pulsos cortos, luego pausa
                switch (pulseStep) {
                    case 0: digitalWrite(LED_PIN, HIGH); pulseStep++; lastToggle = now; break;
                    case 1: if (now - lastToggle >= 100) { digitalWrite(LED_PIN, LOW); pulseStep++; lastToggle = now; } break;
                    case 2: if (now - lastToggle >= 100) { digitalWrite(LED_PIN, HIGH); pulseStep++; lastToggle = now; } break;
                    case 3: if (now - lastToggle >= 100) { digitalWrite(LED_PIN, LOW); pulseStep++; lastToggle = now; } break;
                    case 4: if (now - lastToggle >= 1600) { pulseStep = 0; } break;
                }
                break;

            case LED_TRIPLE_PULSE:
                // 3 pulsos cortos, luego pausa
                switch (pulseStep) {
                    case 0: digitalWrite(LED_PIN, HIGH); pulseStep++; lastToggle = now; break;
                    case 1: if (now - lastToggle >= 80) { digitalWrite(LED_PIN, LOW); pulseStep++; lastToggle = now; } break;
                    case 2: if (now - lastToggle >= 80) { digitalWrite(LED_PIN, HIGH); pulseStep++; lastToggle = now; } break;
                    case 3: if (now - lastToggle >= 80) { digitalWrite(LED_PIN, LOW); pulseStep++; lastToggle = now; } break;
                    case 4: if (now - lastToggle >= 80) { digitalWrite(LED_PIN, HIGH); pulseStep++; lastToggle = now; } break;
                    case 5: if (now - lastToggle >= 80) { digitalWrite(LED_PIN, LOW); pulseStep++; lastToggle = now; } break;
                    case 6: if (now - lastToggle >= 1500) { pulseStep = 0; } break;
                }
                break;

            case LED_LONG_PULSE:
                // Pulso largo de 500ms, pausa 1500ms
                switch (pulseStep) {
                    case 0: digitalWrite(LED_PIN, HIGH); pulseStep++; lastToggle = now; break;
                    case 1: if (now - lastToggle >= 500) { digitalWrite(LED_PIN, LOW); pulseStep++; lastToggle = now; } break;
                    case 2: if (now - lastToggle >= 1500) { pulseStep = 0; } break;
                }
                break;
        }
    }
}

// ============================================================================
// VIBRADOR - STUBS (listo para futuro)
// ============================================================================

inline void vibInit() {
#ifdef VIB_PIN
    pinMode(VIB_PIN, OUTPUT);
    digitalWrite(VIB_PIN, LOW);
#endif
}

inline void vibPulse(uint16_t ms = 200) {
#ifdef VIB_PIN
    digitalWrite(VIB_PIN, HIGH);
    delay(ms);
    digitalWrite(VIB_PIN, LOW);
#endif
    (void)ms;
}

inline void vibAlert() {
#ifdef VIB_PIN
    for (int i = 0; i < 3; i++) {
        digitalWrite(VIB_PIN, HIGH); delay(100);
        digitalWrite(VIB_PIN, LOW);  delay(100);
    }
#endif
}

inline void vibContinuous(bool on) {
#ifdef VIB_PIN
    digitalWrite(VIB_PIN, on ? HIGH : LOW);
#endif
    (void)on;
}

// ============================================================================
// DISPLAY - FUNCIONES DE DIBUJO
// ============================================================================

inline void dibujarFlechaGruesa(int cx, int cy, float angulo, int longitud) {
    float rad = angulo * PI / 180.0;
    int x_punta = cx + (int)(longitud * cos(rad));
    int y_punta = cy + (int)(longitud * sin(rad));
    int x_cola  = cx - (int)(longitud * cos(rad));
    int y_cola  = cy - (int)(longitud * sin(rad));

    float perpRad = rad + PI / 2;
    int dx = (int)round(cos(perpRad));
    int dy = (int)round(sin(perpRad));

    display.drawLine(x_cola, y_cola, x_punta, y_punta);
    display.drawLine(x_cola + dx, y_cola + dy, x_punta + dx, y_punta + dy);
    display.drawLine(x_cola - dx, y_cola - dy, x_punta - dx, y_punta - dy);

    int tam = 7;
    float angIzq = (angulo + 150) * PI / 180.0;
    float angDer = (angulo - 150) * PI / 180.0;
    int x_izq = x_punta + (int)(tam * cos(angIzq));
    int y_izq = y_punta + (int)(tam * sin(angIzq));
    int x_der = x_punta + (int)(tam * cos(angDer));
    int y_der = y_punta + (int)(tam * sin(angDer));

    display.drawTriangle(x_punta, y_punta, x_izq, y_izq, x_der, y_der);
    display.drawDisc(cx, cy, 2);
}

// Dibujar barra de progreso/RSSI
inline void dibujarBarra(int x, int y, int w, int h, float porcentaje) {
    display.drawFrame(x, y, w, h);
    int fill = (int)(w * constrain(porcentaje, 0.0f, 1.0f));
    if (fill > 0) {
        display.drawBox(x, y, fill, h);
    }
}

// ============================================================================
// BOTÓN - LECTURA NO BLOQUEANTE
// ============================================================================

namespace button {
    inline bool lastState = HIGH;
    inline uint32_t pressStart = 0;
    inline bool pressed = false;
    inline bool longPressed = false;

    inline void update() {
        bool current = digitalRead(BUTTON_PRG);

        if (lastState == HIGH && current == LOW) {
            // Flanco descendente
            pressStart = millis();
            pressed = false;
            longPressed = false;
        }

        if (current == LOW && !longPressed) {
            if (millis() - pressStart >= SOS_HOLD_TIME) {
                longPressed = true;  // Presión larga detectada
            }
        }

        if (lastState == LOW && current == HIGH) {
            // Flanco ascendente (soltar)
            if (!longPressed && (millis() - pressStart >= 50)) {
                pressed = true;  // Clic corto
            }
        }

        lastState = current;
    }

    inline bool wasPressed() {
        if (pressed) {
            pressed = false;
            DBG("BTN", "Pulsacion corta");
            return true;
        }
        return false;
    }

    inline bool wasLongPressed() {
        if (longPressed) {
            longPressed = false;
            DBG("BTN", "Pulsacion LARGA (HOLD)");
            return true;
        }
        return false;
    }
}

// ============================================================================
// INIT COMPLETO DE HARDWARE
// ============================================================================

inline bool initAllHardware() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println("  SISTEMA DE BUSQUEDA LORA - 2 EN 1");
    Serial.println("========================================\n");

    initVext();
    initDisplay();
    initGPSHardware();
    initCompassHardware();

    bool loraOK = initLoRaHardware();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(BUTTON_PRG, INPUT_PULLUP);
    vibInit();

    Serial.printf("[HW] Nodo ID: 0x%02X\n", getNodeId());
    Serial.println("[HW] Inicializacion completa\n");

    return loraOK;
}
