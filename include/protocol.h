#pragma once
#include "config.h"
#include "types.h"
#include <esp_mac.h>

// ============================================================================
// PROTOCOLO DE COMUNICACIÓN LORA
// ============================================================================

// ID del nodo (últimos 2 bytes de MAC)
inline uint8_t getNodeId() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return mac[5];  // Último byte como ID simple
}

// Calcular checksum XOR
inline uint8_t calcChecksum(const uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

// Preparar mensaje para transmisión
inline void prepararMsg(MensajeLoRa& msg, TipoMensaje tipo, uint8_t destino,
                        ModoBusqueda modo, double lat, double lon,
                        uint8_t flags, uint8_t bateria) {
    static uint8_t secuencia = 0;

    msg.magic    = MSG_MAGIC;
    msg.tipo     = tipo;
    msg.origen   = getNodeId();
    msg.destino  = destino;
    msg.secuencia = secuencia++;
    msg.flags    = flags;
    msg.modo     = modo;
    msg.latitud  = (int32_t)(lat * 1e6);
    msg.longitud = (int32_t)(lon * 1e6);
    msg.bateria  = bateria;

    // Checksum sobre todos los bytes menos el propio checksum
    msg.checksum = calcChecksum((uint8_t*)&msg, sizeof(MensajeLoRa) - 1);
}

// Validar mensaje recibido
inline bool validarMsg(const MensajeLoRa& msg) {
    if (msg.magic != MSG_MAGIC) return false;
    uint8_t cs = calcChecksum((const uint8_t*)&msg, sizeof(MensajeLoRa) - 1);
    if (cs != msg.checksum) return false;
    // Ignorar mensajes propios
    if (msg.origen == getNodeId()) return false;
    return true;
}

// Extraer coordenadas del mensaje
inline void extraerCoords(const MensajeLoRa& msg, double& lat, double& lon) {
    lat = (double)msg.latitud / 1e6;
    lon = (double)msg.longitud / 1e6;
}

// Nombre del tipo de mensaje (para debug serial)
inline const char* nombreTipoMsg(uint8_t tipo) {
    switch (tipo) {
        case MSG_SEARCH_START:    return "SEARCH_START";
        case MSG_ACK:             return "ACK";
        case MSG_STATUS:          return "STATUS";
        case MSG_BEACON:          return "BEACON";
        case MSG_SOS_ALERT:       return "SOS_ALERT";
        case MSG_SOS_CANCEL:      return "SOS_CANCEL";
        case MSG_REUNITE_CONFIRM: return "REUNITE_CONFIRM";
        default:                  return "UNKNOWN";
    }
}

// Nombre del estado A (para debug serial)
inline const char* nombreEstadoA(EstadoA e) {
    switch (e) {
        case A_IDLE:              return "IDLE";
        case A_SEARCHING:         return "SEARCHING";
        case A_SEARCHING_RETRY:   return "RETRY";
        case A_SEARCHING_NO_GPS:  return "NO_GPS";
        case A_SEARCHING_RSSI_LORA: return "RSSI_LORA";
        case A_NO_COMPASS:        return "NO_COMPASS";
        case A_SIGNAL_LOST:       return "SIGNAL_LOST";
        case A_RECEIVING_SOS:     return "RECEIVING_SOS";
        case A_UNREACHABLE:       return "UNREACHABLE";
        case A_REUNITED:          return "REUNITED";
        default:                  return "?";
    }
}

// Nombre del estado B (para debug serial)
inline const char* nombreEstadoB(EstadoB e) {
    switch (e) {
        case B_LISTENING:       return "LISTENING";
        case B_ALERTING:        return "ALERTING";
        case B_ALERTING_NO_GPS: return "ALERTING_NO_GPS";
        case B_BEACON_MODE:     return "BEACON_MODE";
        case B_SOS_MODE:        return "SOS_MODE";
        case B_REUNITED:        return "REUNITED";
        default:                return "?";
    }
}
