/*
 * ====================================================================
 * SISTEMA DE BÚSQUEDA LORA - 2 EN 1
 * ====================================================================
 * Mismo firmware para ambos nodos (Heltec WiFi LoRa 32 V4)
 * - Presión corta de PRG → Modo Buscador (Nodo A)
 * - Al recibir SEARCH_START → Modo Perdido (Nodo B)
 * - Presión larga de PRG (3s) → Modo SOS
 * ====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include "config.h"
#include "types.h"
#include "protocol.h"
#include "hardware.h"
#include "state_machine.h"

// ============================================================================
// INSTANCIAS GLOBALES DE HARDWARE
// ============================================================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RESET, LORA_BUSY);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Variables globales de hardware
DatosGPS myGPS;
float compassHeading = 0.0;
bool compassOK = false;
volatile bool loraReceivedFlag = false;
float lastRxRSSI = 0;
float lastRxSNR = 0;

// ISR LoRa (debe estar en main.cpp por IRAM_ATTR)
void IRAM_ATTR loraISR() {
    loraReceivedFlag = true;
}

// Máquina de estados
StateMachine fsm;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    // Inicializar todo el hardware
    bool loraOK = initAllHardware();

    if (!loraOK) {
        display.clearBuffer();
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(10, 25, "ERROR LoRa!");
        display.drawStr(10, 45, "Reinicia el nodo");
        display.sendBuffer();
        while (true) { delay(1000); }
    }

    // Mostrar pantalla de inicio
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB08_tr);
    display.drawStr(5, 15, "Sistema Busqueda");
    display.drawStr(15, 30, "LoRa 2 en 1");
    display.setFont(u8g2_font_5x7_tr);
    display.setCursor(20, 48);
    display.printf("Nodo ID: 0x%02X", getNodeId());
    display.drawStr(20, 60, "PRG=Buscar HOLD=SOS");
    display.sendBuffer();
    delay(2000);

    // Inicializar máquina de estados
    fsm.init();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    uint32_t now = millis();

    // Actualizar GPS continuamente
    updateGPS();

    // Actualizar brújula cada 50ms
    static uint32_t lastCompass = 0;
    if (now - lastCompass >= COMPASS_UPDATE_MS) {
        updateCompass();
        lastCompass = now;
    }

    // Actualizar máquina de estados (maneja LoRa, display, lógica)
    fsm.update();

    // Actualizar patrones LED
    led::update();
}
