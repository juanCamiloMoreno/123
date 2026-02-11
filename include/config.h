#pragma once

// ============================================================================
// CONFIGURACIÓN DE PINES - HELTEC WIFI LORA 32 V4
// ============================================================================

// OLED Display
#define OLED_SDA        17
#define OLED_SCL        18
#define OLED_RST        21

// Control de alimentación
#define VEXT_ENABLE     36      // Activo BAJO

// LED y Botón
#define LED_PIN         35      // LED blanco de la placa
#define BUTTON_PRG      0       // Botón PRG (BOOT)

// GPS
#define GPS_TX          38
#define GPS_RX          39
#define GPS_ENABLE      34      // Activo BAJO

// Magnetómetro (bus I2C separado del OLED)
#define MAG_SDA         5
#define MAG_SCL         6

// SPI (LoRa)
#define LORA_SCK        9
#define LORA_MISO       11
#define LORA_MOSI       10

// SX1262 LoRa
#define LORA_NSS        8
#define LORA_DIO1       14
#define LORA_RESET      12
#define LORA_BUSY       13

// Vibrador (futuro - descomentar cuando se conecte)
// #define VIB_PIN      XX

// ============================================================================
// CONFIGURACIÓN LORA
// ============================================================================
#define RF_FREQUENCY        915000000   // 915 MHz
#define TX_OUTPUT_POWER     22          // dBm
#define LORA_BANDWIDTH      125.0       // kHz
#define LORA_SF             7           // Spreading Factor
#define LORA_CR             1           // Coding Rate
#define LORA_PREAMBLE       8
#define LORA_SYNC_WORD      0x12

// ============================================================================
// TIEMPOS Y UMBRALES
// ============================================================================

// Timeouts (ms)
#define ACK_TIMEOUT         3000        // Espera ACK después de SEARCH_START
#define GPS_TIMEOUT         30000       // Tiempo para obtener fix GPS
#define STATUS_INTERVAL     5000        // Intervalo de envío STATUS (B)
#define BEACON_INTERVAL     2000        // Intervalo de envío BEACON (B)
#define SIGNAL_LOST_TIMEOUT 30000       // Sin STATUS = señal perdida
#define UNREACHABLE_TIMEOUT 60000       // En SIGNAL_LOST = inalcanzable
#define REUNITED_TIMEOUT    10000       // Auto-volver a IDLE/LISTENING
#define SOS_HOLD_TIME       3000        // Mantener botón para SOS
#define COMPASS_UPDATE_MS   50          // Actualizar brújula cada 50ms
#define DISPLAY_UPDATE_MS   100         // Actualizar display cada 100ms
#define RSSI_SAMPLE_MS      500         // Muestreo RSSI cada 500ms

// Reintentos
#define RETRY_MAX           5           // Máximo reintentos antes de UNREACHABLE

// Umbrales de proximidad
#define REUNITE_DISTANCE    10.0f       // Metros para declarar REUNITED (GPS)
#define REUNITE_DISTANCE_NC 5.0f        // Metros para REUNITED sin brújula
#define RSSI_CLOSE          -40         // dBm para declarar REUNITED (RSSI LoRa)

// RSSI tendencia
#define RSSI_TREND_THRESHOLD 5          // dB de diferencia para HOT/COLD

// ============================================================================
// PROTOCOLO
// ============================================================================
#define MSG_MAGIC           0xAB        // Byte mágico para validar mensajes
#define BROADCAST_ID        0xFF        // ID de destino para broadcast
#define MSG_BUFFER_SIZE     32          // Tamaño buffer mensajes

// ============================================================================
// DEBUG SERIAL
// ============================================================================
#define DEBUG_ENABLED       1           // 1=debug ON, 0=debug OFF

#if DEBUG_ENABLED
  #define DBG(tag, fmt, ...) Serial.printf("[%07lu][" tag "] " fmt "\n", millis(), ##__VA_ARGS__)
#else
  #define DBG(tag, fmt, ...) ((void)0)
#endif

// Intervalo para logs periódicos (evitar spam)
#define DEBUG_GPS_INTERVAL  5000        // Log GPS cada 5s
#define DEBUG_STATE_INTERVAL 3000       // Log estado cada 3s
