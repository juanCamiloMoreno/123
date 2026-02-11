#pragma once
#include <stdint.h>

// ============================================================================
// ROL ACTIVO DEL NODO
// ============================================================================
enum Rol : uint8_t {
    ROL_NINGUNO = 0,    // Recién encendido, esperando
    ROL_BUSCADOR,       // Nodo A - busca al otro
    ROL_PERDIDO          // Nodo B - está siendo buscado
};

// ============================================================================
// ESTADOS DEL BUSCADOR (NODO A)
// ============================================================================
enum EstadoA : uint8_t {
    A_IDLE = 0,             // Esperando acción del usuario
    A_SEARCHING,            // Búsqueda activa con GPS
    A_SEARCHING_RETRY,      // Reintentando contactar B
    A_SEARCHING_NO_GPS,     // GPS propio falló
    A_SEARCHING_RSSI_LORA,  // Navegación por RSSI LoRa
    A_NO_COMPASS,           // Sin brújula, solo distancia
    A_SIGNAL_LOST,          // Pérdida temporal de señal
    A_RECEIVING_SOS,        // Recibiendo SOS de B
    A_UNREACHABLE,          // B no responde después de reintentos
    A_REUNITED              // Reencuentro exitoso
};

// ============================================================================
// ESTADOS DEL PERDIDO (NODO B)
// ============================================================================
enum EstadoB : uint8_t {
    B_LISTENING = 0,        // Escucha duty cycle (bajo consumo)
    B_ALERTING,             // Respondiendo, enviando STATUS con GPS
    B_ALERTING_NO_GPS,      // Sin fix GPS, transición a beacon
    B_BEACON_MODE,          // Enviando beacons LoRa para RSSI
    B_SOS_MODE,             // Emergencia activada por usuario
    B_REUNITED              // Reencuentro exitoso
};

// ============================================================================
// TIPOS DE MENSAJE
// ============================================================================
enum TipoMensaje : uint8_t {
    MSG_SEARCH_START = 1,   // A→B: Iniciar búsqueda
    MSG_ACK,                // B→A: Confirmación
    MSG_STATUS,             // B→A: Posición y estado
    MSG_BEACON,             // B→A: Beacon RSSI (sin payload GPS)
    MSG_SOS_ALERT,          // B→A: Emergencia
    MSG_SOS_CANCEL,         // A→B: Cancelar SOS
    MSG_REUNITE_CONFIRM     // A↔B: Confirmación de reencuentro
};

// ============================================================================
// MODO DE BÚSQUEDA
// ============================================================================
enum ModoBusqueda : uint8_t {
    MODE_GPS = 0,           // Navegación por coordenadas GPS
    MODE_LORA_RSSI          // Navegación por RSSI de LoRa
};

// ============================================================================
// FLAGS DE ESTADO (bitfield)
// ============================================================================
#define FLAG_NONE       0x00
#define FLAG_NO_FIX     0x01    // Sin fix GPS
#define FLAG_LOW_BAT    0x02    // Batería baja
#define FLAG_SOS        0x04    // Modo emergencia
#define FLAG_NO_COMPASS 0x08    // Sin brújula

// ============================================================================
// TENDENCIA RSSI
// ============================================================================
enum TendenciaRSSI : uint8_t {
    RSSI_COLD = 0,      // Alejándose (RSSI baja)
    RSSI_WARM,           // Estable
    RSSI_HOT             // Acercándose (RSSI sube)
};

// ============================================================================
// PATRONES LED (para LED blanco único)
// ============================================================================
enum PatronLED : uint8_t {
    LED_OFF = 0,            // Apagado
    LED_SOLID,              // Encendido fijo
    LED_BLINK_SLOW,         // Parpadeo lento (1s)
    LED_BLINK_FAST,         // Parpadeo rápido (300ms)
    LED_BLINK_VERY_FAST,    // Muy rápido (100ms)
    LED_DOUBLE_PULSE,       // Doble pulso cada 2s
    LED_TRIPLE_PULSE,       // Triple pulso cada 2s
    LED_LONG_PULSE          // Pulso largo cada 2s
};

// ============================================================================
// STRUCT DE MENSAJE LORA (binario)
// ============================================================================
struct __attribute__((packed)) MensajeLoRa {
    uint8_t  magic;         // Byte mágico (0xAB)
    uint8_t  tipo;          // TipoMensaje
    uint8_t  origen;        // ID del nodo que envía
    uint8_t  destino;       // ID destino (0xFF = broadcast)
    uint8_t  secuencia;     // Número de secuencia
    uint8_t  flags;         // Flags de estado
    uint8_t  modo;          // ModoBusqueda
    int32_t  latitud;       // Latitud × 1e6
    int32_t  longitud;      // Longitud × 1e6
    uint8_t  bateria;       // Porcentaje batería
    uint8_t  checksum;      // XOR de todos los bytes anteriores
};
// sizeof = 16 bytes

// ============================================================================
// DATOS GPS
// ============================================================================
struct DatosGPS {
    bool     valid = false;
    double   lat = 0.0;
    double   lon = 0.0;
    double   alt = 0.0;
    int      sats = 0;
    uint32_t lastUpdate = 0;
};

// ============================================================================
// DATOS DEL NODO REMOTO
// ============================================================================
struct DatosRemoto {
    bool     valid = false;
    double   lat = 0.0;
    double   lon = 0.0;
    float    distance = 0.0;    // Distancia en metros
    float    bearing = 0.0;     // Rumbo hacia el otro (0-360°)
    float    rssi = 0.0;        // RSSI del último mensaje
    float    snr = 0.0;         // SNR del último mensaje
    uint32_t lastReceived = 0;  // Timestamp última recepción
};
