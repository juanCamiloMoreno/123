#pragma once
#include "config.h"
#include "types.h"
#include "protocol.h"
#include "hardware.h"

// ============================================================================
// MÁQUINA DE ESTADOS - SISTEMA DE BÚSQUEDA LORA
// ============================================================================

class StateMachine {
public:
    // Rol y estado actual
    Rol         rol = ROL_NINGUNO;
    EstadoA     estadoA = A_IDLE;
    EstadoB     estadoB = B_LISTENING;

    // Datos del nodo remoto
    DatosRemoto remoto;

    // Tracking RSSI
    float       rssiActual = -120;
    float       rssiAnterior = -120;
    TendenciaRSSI tendencia = RSSI_WARM;
    int         rssiSamples = 0;

    // Contadores y timers
    uint8_t     retryCount = 0;
    uint8_t     peerNodeId = 0;         // ID del otro nodo
    uint32_t    timerEstado = 0;        // Timer general del estado
    uint32_t    timerTx = 0;            // Timer de transmisión periódica
    uint32_t    timerDisplay = 0;       // Timer de actualización display
    uint32_t    timerGPSCheck = 0;      // Timer de verificación GPS
    uint32_t    timerRSSI = 0;          // Timer de muestreo RSSI

    // Última posición conocida (para SIGNAL_LOST)
    double      lastKnownLat = 0;
    double      lastKnownLon = 0;
    float       lastKnownDist = 0;

    // Opción seleccionada en menú UNREACHABLE
    uint8_t     menuOption = 0;

    // Flag de primer entry en estado
    bool        stateEntry = true;

    // ========================================================================
    // INIT
    // ========================================================================
    void init() {
        rol = ROL_NINGUNO;
        estadoA = A_IDLE;
        estadoB = B_LISTENING;
        stateEntry = true;
        timerEstado = millis();
        DBG("FSM", "Init -> IDLE (nodeId=0x%02X)", getNodeId());
    }

    // ========================================================================
    // UPDATE PRINCIPAL - Llamado en cada loop()
    // ========================================================================
    void update() {
        uint32_t now = millis();

        // Leer mensajes LoRa
        MensajeLoRa msgRx;
        bool hayMensaje = checkLoRaRx(msgRx);

        // Actualizar botón
        button::update();

        // En cualquier rol: escuchar SOS desde IDLE
        if (rol == ROL_NINGUNO && hayMensaje && msgRx.tipo == MSG_SOS_ALERT) {
            cambiarRolA(A_RECEIVING_SOS);
            peerNodeId = msgRx.origen;
            procesarMensajeA(msgRx);
        }

        // Dispatcher según rol
        switch (rol) {
            case ROL_NINGUNO:
                updateIdle(hayMensaje, msgRx);
                break;
            case ROL_BUSCADOR:
                updateBuscador(now, hayMensaje, msgRx);
                break;
            case ROL_PERDIDO:
                updatePerdido(now, hayMensaje, msgRx);
                break;
        }

        // Actualizar display
        if (now - timerDisplay >= DISPLAY_UPDATE_MS) {
            actualizarDisplay();
            timerDisplay = now;
        }

        // Debug periódico: resumen del estado (cada 10s)
        static uint32_t lastSummary = 0;
        if (now - lastSummary >= 10000) {
            const char* rolStr = rol == ROL_NINGUNO ? "IDLE" :
                                 rol == ROL_BUSCADOR ? "BUSCADOR" : "PERDIDO";
            const char* estStr = rol == ROL_BUSCADOR ? nombreEstadoA(estadoA) :
                                 rol == ROL_PERDIDO ? nombreEstadoB(estadoB) : "---";
            DBG("SUM", "rol=%s est=%s peer=0x%02X GPS=%d/%d cmp=%d",
                rolStr, estStr, peerNodeId, myGPS.valid, myGPS.sats, compassOK);
            lastSummary = now;
        }
    }

private:
    // ========================================================================
    // CAMBIO DE ROL
    // ========================================================================
    void cambiarRolA(EstadoA nuevoEstado) {
        rol = ROL_BUSCADOR;
        estadoA = nuevoEstado;
        stateEntry = true;
        timerEstado = millis();
        DBG("FSM", ">>> ROL=BUSCADOR Estado=%s", nombreEstadoA(nuevoEstado));
    }

    void cambiarRolB(EstadoB nuevoEstado) {
        rol = ROL_PERDIDO;
        estadoB = nuevoEstado;
        stateEntry = true;
        timerEstado = millis();
        DBG("FSM", ">>> ROL=PERDIDO Estado=%s", nombreEstadoB(nuevoEstado));
    }

    void cambiarEstadoA(EstadoA nuevoEstado) {
        DBG("FSM", "A: %s -> %s (peer=0x%02X)", nombreEstadoA(estadoA), nombreEstadoA(nuevoEstado), peerNodeId);
        estadoA = nuevoEstado;
        stateEntry = true;
        timerEstado = millis();
    }

    void cambiarEstadoB(EstadoB nuevoEstado) {
        DBG("FSM", "B: %s -> %s (peer=0x%02X)", nombreEstadoB(estadoB), nombreEstadoB(nuevoEstado), peerNodeId);
        estadoB = nuevoEstado;
        stateEntry = true;
        timerEstado = millis();
    }

    void resetToIdle() {
        DBG("FSM", "<<< RESET -> IDLE (era peer=0x%02X)", peerNodeId);
        rol = ROL_NINGUNO;
        estadoA = A_IDLE;
        estadoB = B_LISTENING;
        retryCount = 0;
        peerNodeId = 0;
        remoto.valid = false;
        stateEntry = true;
        timerEstado = millis();
        led::setPatron(LED_OFF);
    }

    // ========================================================================
    // IDLE (ROL_NINGUNO) - Ambos nodos empiezan aquí
    // ========================================================================
    void updateIdle(bool hayMsg, MensajeLoRa& msg) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_OFF);
        }

        // Botón corto → convertirse en BUSCADOR
        if (button::wasPressed()) {
            retryCount = 0;
            timerGPSCheck = millis();
            cambiarRolA(A_SEARCHING);
            return;
        }

        // Botón largo → SOS (convertirse en PERDIDO con SOS)
        if (button::wasLongPressed()) {
            cambiarRolB(B_SOS_MODE);
            return;
        }

        // Recibir SEARCH_START → convertirse en PERDIDO
        if (hayMsg && msg.tipo == MSG_SEARCH_START) {
            peerNodeId = msg.origen;
            DBG("FSM", "SEARCH_START de 0x%02X modo=%s", msg.origen,
                msg.modo == MODE_GPS ? "GPS" : "RSSI");
            if (msg.modo == MODE_GPS) {
                cambiarRolB(B_ALERTING);
            } else {
                cambiarRolB(B_BEACON_MODE);
            }
            // Esperar un poco antes de ACK para que A termine TX y vuelva a RX
            DBG("FSM", "Esperando 150ms antes de ACK...");
            delay(150);
            enviarACK();
            return;
        }
    }

    // ========================================================================
    // BUSCADOR (NODO A) - UPDATE
    // ========================================================================
    void updateBuscador(uint32_t now, bool hayMsg, MensajeLoRa& msg) {
        // Procesar mensaje recibido
        if (hayMsg) procesarMensajeA(msg);

        switch (estadoA) {
            case A_SEARCHING:       updateSearching(now); break;
            case A_SEARCHING_RETRY: updateRetry(now); break;
            case A_SEARCHING_NO_GPS: updateNoGPS(now); break;
            case A_SEARCHING_RSSI_LORA: updateRSSILoRa(now); break;
            case A_NO_COMPASS:      updateNoCompass(now); break;
            case A_SIGNAL_LOST:     updateSignalLost(now); break;
            case A_RECEIVING_SOS:   updateReceivingSOS(now); break;
            case A_UNREACHABLE:     updateUnreachable(now); break;
            case A_REUNITED:        updateReunitedA(now); break;
            default: break;
        }
    }

    // --- A_SEARCHING ---
    void updateSearching(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_BLINK_SLOW);
            timerGPSCheck = now;
            timerTx = 0;
            DBG("FSM", "SEARCHING entry: GPS=%d sats=%d compass=%d peer=0x%02X",
                myGPS.valid, myGPS.sats, compassOK, peerNodeId);
            enviarSearchStart(MODE_GPS);
        }

        // HOLD = Cancelar búsqueda
        if (button::wasLongPressed()) {
            DBG("FSM", "SEARCHING: usuario cancelo con HOLD");
            resetToIdle();
            return;
        }

        // Verificar GPS propio
        if (!myGPS.valid && (now - timerGPSCheck >= GPS_TIMEOUT)) {
            DBG("FSM", "SEARCHING: GPS timeout (%lums), cambio a NO_GPS", now - timerGPSCheck);
            cambiarEstadoA(A_SEARCHING_NO_GPS);
            return;
        }

        // Esperar ACK (si no tenemos peer confirmado)
        if (peerNodeId == 0 && (now - timerEstado >= ACK_TIMEOUT)) {
            DBG("FSM", "SEARCHING: ACK timeout (%lums), sin peer -> RETRY", now - timerEstado);
            cambiarEstadoA(A_SEARCHING_RETRY);
            return;
        }

        // Si tenemos datos del remoto, verificar proximidad
        if (remoto.valid && myGPS.valid) {
            remoto.distance = calculateDistance(myGPS.lat, myGPS.lon,
                                                remoto.lat, remoto.lon);
            remoto.bearing = calculateBearing(myGPS.lat, myGPS.lon,
                                               remoto.lat, remoto.lon);

            if (remoto.distance < REUNITE_DISTANCE) {
                DBG("FSM", "SEARCHING: dist=%.1fm < %.1fm -> REUNITED!", remoto.distance, REUNITE_DISTANCE);
                enviarReuniteConfirm();
                cambiarEstadoA(A_REUNITED);
                return;
            }

            // Verificar brújula
            if (!compassOK) {
                DBG("FSM", "SEARCHING: brujula no disponible -> NO_COMPASS");
                cambiarEstadoA(A_NO_COMPASS);
                return;
            }
        }

        // Verificar señal perdida
        if (peerNodeId != 0 && remoto.lastReceived != 0 &&
            (now - remoto.lastReceived >= SIGNAL_LOST_TIMEOUT)) {
            DBG("FSM", "SEARCHING: sin msg de peer por %lums -> SIGNAL_LOST", now - remoto.lastReceived);
            guardarUltimaPosicion();
            cambiarEstadoA(A_SIGNAL_LOST);
            return;
        }

        // Debug periódico del estado
        static uint32_t lastSearchLog = 0;
        if (now - lastSearchLog >= DEBUG_STATE_INTERVAL) {
            DBG("FSM", "SEARCHING: peer=0x%02X remoto.valid=%d dist=%.0fm lastRx=%lums ago",
                peerNodeId, remoto.valid, remoto.distance,
                remoto.lastReceived ? (now - remoto.lastReceived) : 0);
            lastSearchLog = now;
        }
    }

    // --- A_SEARCHING_RETRY ---
    void updateRetry(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            retryCount++;
            led::setPatron(LED_BLINK_FAST);
            DBG("FSM", "RETRY entry: intento %d/%d peer=0x%02X", retryCount, RETRY_MAX, peerNodeId);

            if (retryCount >= RETRY_MAX) {
                DBG("FSM", "RETRY: max reintentos alcanzado -> UNREACHABLE");
                cambiarEstadoA(A_UNREACHABLE);
                return;
            }

            enviarSearchStart(MODE_GPS);
            timerEstado = now;
        }

        // HOLD = Cancelar búsqueda
        if (button::wasLongPressed()) {
            DBG("FSM", "RETRY: usuario cancelo con HOLD");
            resetToIdle();
            return;
        }

        // Esperar ACK
        if (now - timerEstado >= ACK_TIMEOUT) {
            DBG("FSM", "RETRY: ACK timeout (%lums), reintentando...", now - timerEstado);
            stateEntry = true;
        }
    }

    // --- A_SEARCHING_NO_GPS ---
    void updateNoGPS(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_DOUBLE_PULSE);

            // Enviar SEARCH_START en modo RSSI
            enviarSearchStart(MODE_LORA_RSSI);
            timerEstado = now;
        }

        // HOLD = Cancelar búsqueda
        if (button::wasLongPressed()) {
            resetToIdle();
            return;
        }

        // Si el GPS se recupera, volver a SEARCHING
        if (myGPS.valid) {
            cambiarEstadoA(A_SEARCHING);
            return;
        }

        // Transición automática a RSSI_LORA cuando hay ACK
        if (peerNodeId != 0) {
            cambiarEstadoA(A_SEARCHING_RSSI_LORA);
            return;
        }

        // Sin ACK, reintentar
        if (now - timerEstado >= ACK_TIMEOUT) {
            cambiarEstadoA(A_SEARCHING_RETRY);
        }
    }

    // --- A_SEARCHING_RSSI_LORA ---
    void updateRSSILoRa(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_TRIPLE_PULSE);
            rssiActual = -120;
            rssiAnterior = -120;
            tendencia = RSSI_WARM;
            rssiSamples = 0;
            DBG("FSM", "RSSI_LORA entry: peer=0x%02X", peerNodeId);
        }

        // HOLD = Cancelar búsqueda
        if (button::wasLongPressed()) {
            DBG("FSM", "RSSI_LORA: usuario cancelo con HOLD");
            resetToIdle();
            return;
        }

        // Verificar proximidad por RSSI
        if (rssiActual > RSSI_CLOSE) {
            DBG("FSM", "RSSI_LORA: RSSI=%.0f > %d -> REUNITED!", rssiActual, RSSI_CLOSE);
            enviarReuniteConfirm();
            cambiarEstadoA(A_REUNITED);
            return;
        }

        // Si GPS se recupera, volver a SEARCHING con GPS
        if (myGPS.valid && remoto.valid) {
            DBG("FSM", "RSSI_LORA: GPS recuperado -> SEARCHING");
            cambiarEstadoA(A_SEARCHING);
            return;
        }

        // Señal perdida (sin beacons por mucho tiempo)
        if (remoto.lastReceived != 0 &&
            (now - remoto.lastReceived >= SIGNAL_LOST_TIMEOUT)) {
            DBG("FSM", "RSSI_LORA: sin beacons %lums -> SIGNAL_LOST", now - remoto.lastReceived);
            cambiarEstadoA(A_SIGNAL_LOST);
            return;
        }

        // Debug periódico
        static uint32_t lastRSSILog = 0;
        if (now - lastRSSILog >= DEBUG_STATE_INTERVAL) {
            const char* tend = tendencia == RSSI_HOT ? "HOT" : tendencia == RSSI_COLD ? "COLD" : "WARM";
            DBG("FSM", "RSSI_LORA: rssi=%.0f tend=%s n=%d lastRx=%lums ago",
                rssiActual, tend, rssiSamples,
                remoto.lastReceived ? (now - remoto.lastReceived) : 0);
            lastRSSILog = now;
        }
    }

    // --- A_NO_COMPASS ---
    void updateNoCompass(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_DOUBLE_PULSE);
        }

        // HOLD = Cancelar búsqueda
        if (button::wasLongPressed()) {
            resetToIdle();
            return;
        }

        // Actualizar distancia si hay datos
        if (remoto.valid && myGPS.valid) {
            float newDist = calculateDistance(myGPS.lat, myGPS.lon,
                                              remoto.lat, remoto.lon);
            // Determinar tendencia de distancia
            if (newDist < remoto.distance - 1.0f) {
                // Acercándose
            } else if (newDist > remoto.distance + 1.0f) {
                // Alejándose
            }
            remoto.distance = newDist;

            if (remoto.distance < REUNITE_DISTANCE_NC) {
                enviarReuniteConfirm();
                cambiarEstadoA(A_REUNITED);
                return;
            }
        }

        // Si brújula se recupera
        if (compassOK) {
            cambiarEstadoA(A_SEARCHING);
        }
    }

    // --- A_SIGNAL_LOST ---
    void updateSignalLost(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_DOUBLE_PULSE);
            DBG("FSM", "SIGNAL_LOST entry: peer=0x%02X lastDist=%.0fm", peerNodeId, lastKnownDist);
        }

        // HOLD = Cancelar búsqueda
        if (button::wasLongPressed()) {
            DBG("FSM", "SIGNAL_LOST: usuario cancelo con HOLD");
            resetToIdle();
            return;
        }

        // Timeout → UNREACHABLE
        if (now - timerEstado >= UNREACHABLE_TIMEOUT) {
            DBG("FSM", "SIGNAL_LOST: timeout %lums -> UNREACHABLE", now - timerEstado);
            cambiarEstadoA(A_UNREACHABLE);
            return;
        }

        // Debug periódico
        static uint32_t lastSLLog = 0;
        if (now - lastSLLog >= DEBUG_STATE_INTERVAL) {
            DBG("FSM", "SIGNAL_LOST: esperando... %lus/%lus",
                (now - timerEstado) / 1000, UNREACHABLE_TIMEOUT / 1000);
            lastSLLog = now;
        }
    }

    // --- A_RECEIVING_SOS ---
    void updateReceivingSOS(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_BLINK_VERY_FAST);
            vibAlert();
            // Enviar ACK al SOS
            enviarACK();
        }

        // Después de 3 segundos, pasar a búsqueda prioritaria
        if (now - timerEstado >= 3000) {
            cambiarEstadoA(A_SEARCHING);
        }
    }

    // --- A_UNREACHABLE ---
    void updateUnreachable(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_LONG_PULSE);
            menuOption = 0;
        }

        // Botón corto → siguiente opción
        if (button::wasPressed()) {
            menuOption = (menuOption + 1) % 2;  // 0=Reintentar, 1=Cancelar
        }

        // Botón largo → seleccionar opción
        if (button::wasLongPressed()) {
            if (menuOption == 0) {
                // Reintentar
                retryCount = 0;
                cambiarEstadoA(A_SEARCHING);
            } else {
                // Cancelar
                resetToIdle();
            }
        }
    }

    // --- A_REUNITED ---
    void updateReunitedA(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_SOLID);
            vibAlert();  // Celebración
        }

        // Auto-reset después de timeout
        if (now - timerEstado >= REUNITED_TIMEOUT) {
            resetToIdle();
            return;
        }

        // O con botón
        if (button::wasPressed()) {
            resetToIdle();
        }
    }

    // ========================================================================
    // PROCESAR MENSAJES COMO BUSCADOR (A)
    // ========================================================================
    void procesarMensajeA(MensajeLoRa& msg) {
        DBG("MSG", "A procesa %s de 0x%02X (estoy en %s, peer=0x%02X)",
            nombreTipoMsg(msg.tipo), msg.origen, nombreEstadoA(estadoA), peerNodeId);

        // Cualquier mensaje de B confirma el peer (no solo ACK)
        if (msg.tipo != MSG_SOS_ALERT) {
            if (peerNodeId == 0 || peerNodeId == msg.origen) {
                if (peerNodeId == 0) {
                    DBG("MSG", "** PEER CONFIRMADO: 0x%02X (via %s) **",
                        msg.origen, nombreTipoMsg(msg.tipo));
                }
                peerNodeId = msg.origen;
            } else {
                DBG("MSG", "IGNORADO: origen 0x%02X != peer 0x%02X", msg.origen, peerNodeId);
                return;
            }
        }

        switch (msg.tipo) {
            case MSG_ACK:
                remoto.lastReceived = millis();
                DBG("MSG", "ACK procesado: GPS_local=%d", myGPS.valid);

                if (estadoA == A_SEARCHING_RETRY) {
                    if (myGPS.valid) {
                        cambiarEstadoA(A_SEARCHING);
                    } else {
                        cambiarEstadoA(A_SEARCHING_RSSI_LORA);
                    }
                }
                break;

            case MSG_STATUS: {
                remoto.rssi = lastRxRSSI;
                remoto.snr = lastRxSNR;
                remoto.lastReceived = millis();

                if (msg.flags & FLAG_NO_FIX) {
                    remoto.valid = false;
                    DBG("MSG", "STATUS: B sin GPS (NO_FIX) -> modo RSSI");
                    if (estadoA == A_SEARCHING || estadoA == A_SEARCHING_RETRY) {
                        cambiarEstadoA(A_SEARCHING_RSSI_LORA);
                    }
                } else {
                    extraerCoords(msg, remoto.lat, remoto.lon);
                    remoto.valid = true;

                    if (myGPS.valid) {
                        remoto.distance = calculateDistance(myGPS.lat, myGPS.lon,
                                                            remoto.lat, remoto.lon);
                        remoto.bearing = calculateBearing(myGPS.lat, myGPS.lon,
                                                           remoto.lat, remoto.lon);
                        DBG("MSG", "STATUS: pos=%.5f,%.5f dist=%.0fm bear=%.0f",
                            remoto.lat, remoto.lon, remoto.distance, remoto.bearing);
                    } else {
                        DBG("MSG", "STATUS: pos=%.5f,%.5f (mi GPS invalido)", remoto.lat, remoto.lon);
                    }
                }

                // Reconectar desde estados de espera
                if (estadoA == A_SIGNAL_LOST) {
                    DBG("MSG", "STATUS reconecta desde SIGNAL_LOST");
                    cambiarEstadoA(A_SEARCHING);
                } else if (estadoA == A_SEARCHING_RETRY) {
                    if (remoto.valid && myGPS.valid) {
                        cambiarEstadoA(A_SEARCHING);
                    } else {
                        cambiarEstadoA(A_SEARCHING_RSSI_LORA);
                    }
                }
                break;
            }

            case MSG_BEACON: {
                remoto.rssi = lastRxRSSI;
                remoto.snr = lastRxSNR;
                remoto.lastReceived = millis();

                rssiAnterior = rssiActual;
                rssiActual = lastRxRSSI;
                rssiSamples++;

                const char* tend = "WARM";
                if (rssiSamples >= 2) {
                    float diff = rssiActual - rssiAnterior;
                    if (diff > RSSI_TREND_THRESHOLD) {
                        tendencia = RSSI_HOT; tend = "HOT";
                    } else if (diff < -RSSI_TREND_THRESHOLD) {
                        tendencia = RSSI_COLD; tend = "COLD";
                    } else {
                        tendencia = RSSI_WARM;
                    }
                }
                DBG("MSG", "BEACON: rssi=%.0f prev=%.0f tend=%s n=%d",
                    rssiActual, rssiAnterior, tend, rssiSamples);

                // Reconectar desde estados de espera
                if (estadoA == A_SIGNAL_LOST || estadoA == A_SEARCHING_RETRY ||
                    estadoA == A_SEARCHING) {
                    DBG("MSG", "BEACON: cambiando a RSSI_LORA desde %s", nombreEstadoA(estadoA));
                    cambiarEstadoA(A_SEARCHING_RSSI_LORA);
                }
                break;
            }

            case MSG_SOS_ALERT:
                if (estadoA != A_RECEIVING_SOS) {
                    DBG("MSG", "SOS_ALERT de 0x%02X!", msg.origen);
                    if (!(msg.flags & FLAG_NO_FIX)) {
                        extraerCoords(msg, remoto.lat, remoto.lon);
                        remoto.valid = true;
                    }
                    peerNodeId = msg.origen;
                    cambiarEstadoA(A_RECEIVING_SOS);
                }
                break;

            case MSG_REUNITE_CONFIRM:
                DBG("MSG", "REUNITE_CONFIRM recibido!");
                cambiarEstadoA(A_REUNITED);
                break;

            default:
                DBG("MSG", "Tipo no manejado por A: %d", msg.tipo);
                break;
        }
    }

    // ========================================================================
    // PERDIDO (NODO B) - UPDATE
    // ========================================================================
    void updatePerdido(uint32_t now, bool hayMsg, MensajeLoRa& msg) {
        // Procesar mensaje recibido
        if (hayMsg) procesarMensajeB(msg);

        switch (estadoB) {
            case B_ALERTING:        updateAlerting(now); break;
            case B_ALERTING_NO_GPS: updateAlertingNoGPS(now); break;
            case B_BEACON_MODE:     updateBeaconMode(now); break;
            case B_SOS_MODE:        updateSOSMode(now); break;
            case B_REUNITED:        updateReunitedB(now); break;
            default: break;
        }
    }

    // --- B_ALERTING ---
    void updateAlerting(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_BLINK_SLOW);
            vibAlert();
            timerGPSCheck = now;
            timerTx = 0;
            DBG("FSM", "B_ALERTING entry: GPS=%d sats=%d peer=0x%02X",
                myGPS.valid, myGPS.sats, peerNodeId);
        }

        // Verificar GPS
        if (!myGPS.valid && (now - timerGPSCheck >= GPS_TIMEOUT)) {
            DBG("FSM", "B_ALERTING: GPS timeout -> ALERTING_NO_GPS");
            cambiarEstadoB(B_ALERTING_NO_GPS);
            return;
        }

        // Enviar STATUS periódicamente
        if (now - timerTx >= STATUS_INTERVAL) {
            DBG("FSM", "B_ALERTING: enviando STATUS (GPS=%d sats=%d)", myGPS.valid, myGPS.sats);
            enviarStatus();
            timerTx = now;
        }
    }

    // --- B_ALERTING_NO_GPS ---
    void updateAlertingNoGPS(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_DOUBLE_PULSE);
            DBG("FSM", "B_ALERTING_NO_GPS: enviando STATUS con NO_FIX");

            MensajeLoRa msg;
            prepararMsg(msg, MSG_STATUS, peerNodeId, MODE_GPS,
                       0, 0, FLAG_NO_FIX, 100);
            sendLoRaMsg(msg);
        }

        if (myGPS.valid) {
            DBG("FSM", "B_ALERTING_NO_GPS: GPS recuperado -> ALERTING");
            cambiarEstadoB(B_ALERTING);
            return;
        }

        if (now - timerEstado >= 3000) {
            DBG("FSM", "B_ALERTING_NO_GPS: 3s sin GPS -> BEACON_MODE");
            cambiarEstadoB(B_BEACON_MODE);
        }
    }

    // --- B_BEACON_MODE ---
    void updateBeaconMode(uint32_t now) {
        static int beaconsSent = 0;
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_TRIPLE_PULSE);
            timerTx = 0;
            beaconsSent = 0;
            DBG("FSM", "B_BEACON_MODE entry: peer=0x%02X", peerNodeId);
        }

        // Enviar BEACON periódicamente
        if (now - timerTx >= BEACON_INTERVAL) {
            enviarBeacon();
            timerTx = now;
            beaconsSent++;
            if (beaconsSent % 10 == 0) {
                DBG("FSM", "B_BEACON: %d beacons enviados", beaconsSent);
            }
        }

        // Si GPS se recupera, volver a ALERTING
        if (myGPS.valid) {
            DBG("FSM", "B_BEACON: GPS recuperado -> ALERTING");
            cambiarEstadoB(B_ALERTING);
        }
    }

    // --- B_SOS_MODE ---
    void updateSOSMode(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_BLINK_VERY_FAST);
            vibContinuous(true);
            timerTx = 0;
        }

        // Enviar SOS_ALERT cada segundo
        if (now - timerTx >= 1000) {
            MensajeLoRa msg;
            uint8_t flags = FLAG_SOS;
            if (!myGPS.valid) flags |= FLAG_NO_FIX;

            prepararMsg(msg, MSG_SOS_ALERT, BROADCAST_ID, MODE_GPS,
                       myGPS.lat, myGPS.lon, flags, 100);
            sendLoRaMsg(msg);
            timerTx = now;
        }

        // Cancelar SOS con botón largo
        if (button::wasLongPressed()) {
            vibContinuous(false);
            resetToIdle();
        }
    }

    // --- B_REUNITED ---
    void updateReunitedB(uint32_t now) {
        if (stateEntry) {
            stateEntry = false;
            led::setPatron(LED_SOLID);
            vibContinuous(false);
            vibAlert();  // Celebración

            // Enviar confirmación
            enviarReuniteConfirm();
        }

        // Auto-reset
        if (now - timerEstado >= REUNITED_TIMEOUT) {
            resetToIdle();
            return;
        }

        if (button::wasPressed()) {
            resetToIdle();
        }
    }

    // ========================================================================
    // PROCESAR MENSAJES COMO PERDIDO (B)
    // ========================================================================
    void procesarMensajeB(MensajeLoRa& msg) {
        DBG("MSG", "B procesa %s de 0x%02X (estoy en %s, peer=0x%02X)",
            nombreTipoMsg(msg.tipo), msg.origen, nombreEstadoB(estadoB), peerNodeId);

        switch (msg.tipo) {
            case MSG_SEARCH_START:
                peerNodeId = msg.origen;
                DBG("MSG", "SEARCH_START repetido, re-enviando ACK (delay 150ms)");
                delay(150);
                enviarACK();
                break;

            case MSG_REUNITE_CONFIRM:
                DBG("MSG", "REUNITE_CONFIRM -> REUNITED!");
                cambiarEstadoB(B_REUNITED);
                break;

            case MSG_SOS_CANCEL:
                if (estadoB == B_SOS_MODE) {
                    DBG("MSG", "SOS_CANCEL -> REUNITED");
                    vibContinuous(false);
                    cambiarEstadoB(B_REUNITED);
                }
                break;

            default:
                DBG("MSG", "Tipo no manejado por B: %d", msg.tipo);
                break;
        }
    }

    // ========================================================================
    // FUNCIONES DE ENVÍO
    // ========================================================================
    void enviarSearchStart(ModoBusqueda modo) {
        MensajeLoRa msg;
        uint8_t flags = FLAG_NONE;
        if (!myGPS.valid) flags |= FLAG_NO_FIX;
        if (!compassOK) flags |= FLAG_NO_COMPASS;

        prepararMsg(msg, MSG_SEARCH_START,
                   peerNodeId ? peerNodeId : BROADCAST_ID,
                   modo, myGPS.lat, myGPS.lon, flags, 100);
        sendLoRaMsg(msg);
    }

    void enviarACK() {
        MensajeLoRa msg;
        prepararMsg(msg, MSG_ACK, peerNodeId, MODE_GPS,
                   myGPS.lat, myGPS.lon, FLAG_NONE, 100);
        sendLoRaMsg(msg);
    }

    void enviarStatus() {
        MensajeLoRa msg;
        uint8_t flags = FLAG_NONE;
        if (!myGPS.valid) flags |= FLAG_NO_FIX;

        prepararMsg(msg, MSG_STATUS, peerNodeId, MODE_GPS,
                   myGPS.lat, myGPS.lon, flags, 100);
        sendLoRaMsg(msg);
    }

    void enviarBeacon() {
        MensajeLoRa msg;
        prepararMsg(msg, MSG_BEACON, peerNodeId, MODE_LORA_RSSI,
                   0, 0, FLAG_NONE, 100);
        sendLoRaMsg(msg);
    }

    void enviarReuniteConfirm() {
        MensajeLoRa msg;
        prepararMsg(msg, MSG_REUNITE_CONFIRM,
                   peerNodeId ? peerNodeId : BROADCAST_ID,
                   MODE_GPS, myGPS.lat, myGPS.lon, FLAG_NONE, 100);
        sendLoRaMsg(msg);
    }

    void guardarUltimaPosicion() {
        if (remoto.valid) {
            lastKnownLat = remoto.lat;
            lastKnownLon = remoto.lon;
            lastKnownDist = remoto.distance;
        }
    }

    // ========================================================================
    // DISPLAY - PANTALLAS POR ESTADO
    // ========================================================================
    void actualizarDisplay() {
        display.clearBuffer();
        display.setFont(u8g2_font_5x7_tr);

        switch (rol) {
            case ROL_NINGUNO:
                drawIdle();
                break;
            case ROL_BUSCADOR:
                drawBuscador();
                break;
            case ROL_PERDIDO:
                drawPerdido();
                break;
        }

        display.sendBuffer();
    }

    // --- Pantalla IDLE ---
    void drawIdle() {
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(15, 12, "Sistema Listo");
        display.setFont(u8g2_font_5x7_tr);

        // Info GPS
        if (myGPS.valid) {
            display.setCursor(0, 26);
            display.printf("GPS: %.5f, %.5f", myGPS.lat, myGPS.lon);
            display.setCursor(0, 35);
            display.printf("Sats: %d  Alt: %.0fm", myGPS.sats, myGPS.alt);
        } else {
            display.drawStr(0, 26, "GPS: Buscando satelites...");
        }

        // Brújula
        display.setCursor(0, 45);
        display.printf("Brujula: %s", compassOK ? "OK" : "No");
        if (compassOK) {
            display.printf(" (%d)", (int)compassHeading);
        }

        // Instrucciones
        display.drawStr(0, 57, "PRG=Buscar  HOLD=SOS");

        // ID del nodo
        display.setCursor(90, 57);
        display.printf("ID:%02X", getNodeId());
    }

    // --- Pantallas BUSCADOR ---
    void drawBuscador() {
        switch (estadoA) {
            case A_SEARCHING:       drawSearching(); break;
            case A_SEARCHING_RETRY: drawRetry(); break;
            case A_SEARCHING_NO_GPS: drawNoGPS(); break;
            case A_SEARCHING_RSSI_LORA: drawRSSILoRa(); break;
            case A_NO_COMPASS:      drawNoCompass(); break;
            case A_SIGNAL_LOST:     drawSignalLost(); break;
            case A_RECEIVING_SOS:   drawReceivingSOS(); break;
            case A_UNREACHABLE:     drawUnreachable(); break;
            case A_REUNITED:        drawReunited(); break;
            default: break;
        }
    }

    void drawSearching() {
        // Header
        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(0, 7, "BUSCANDO...");
        display.drawStr(75, 7, "HOLD=Cancel");

        if (!remoto.valid) {
            display.drawStr(0, 25, "Esperando posicion de B");
            if (peerNodeId != 0) {
                display.setCursor(0, 35);
                display.printf("Peer: 0x%02X (conectado)", peerNodeId);
            } else {
                display.drawStr(0, 35, "Enviando SEARCH_START...");
            }
            // GPS propio
            if (myGPS.valid) {
                display.setCursor(0, 50);
                display.printf("Mi GPS: %d sats", myGPS.sats);
            }
            return;
        }

        // --- Con datos: Navegación ---
        // Lado izquierdo: Info
        if (remoto.distance < 1000) {
            display.setCursor(0, 18);
            display.printf("Dist: %.0f m", remoto.distance);
        } else {
            display.setCursor(0, 18);
            display.printf("Dist: %.1f km", remoto.distance / 1000.0);
        }

        display.setCursor(0, 28);
        display.printf("Rumbo: %.0f", remoto.bearing);

        display.setCursor(0, 38);
        display.printf("RSSI: %.0f dBm", remoto.rssi);

        display.setCursor(0, 48);
        display.printf("Sats: %d", myGPS.sats);

        // Lado derecho: Brújula con flecha al objetivo
        if (compassOK) {
            int cx = 96, cy = 32, cr = 28;
            display.drawCircle(cx, cy, cr);

            // Ángulo: bearing - compassHeading para mostrar dirección relativa
            float relAngle = remoto.bearing - compassHeading;
            // Convertir a ángulo de pantalla (0°=arriba, CW)
            float screenAngle = fmod(relAngle - 90.0 + 360.0, 360.0);
            dibujarFlechaGruesa(cx, cy, screenAngle, cr - 5);
        }

        // Indicador de cercanía
        if (remoto.distance < 20) {
            display.setFont(u8g2_font_ncenB08_tr);
            display.drawStr(0, 63, "MUY CERCA!");
        }
    }

    void drawRetry() {
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(5, 15, "Reintentando...");

        display.setFont(u8g2_font_5x7_tr);
        display.setCursor(20, 32);
        display.printf("Intento %d / %d", retryCount, RETRY_MAX);

        // Barra de progreso
        dibujarBarra(10, 40, 108, 8, (float)retryCount / RETRY_MAX);

        display.drawStr(0, 60, "Esperando...  HOLD=Cancel");
    }

    void drawNoGPS() {
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(10, 15, "Sin GPS propio");

        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(0, 30, "Cambiando a modo senal");
        display.drawStr(0, 40, "Navegacion por RSSI LoRa");

        if (peerNodeId != 0) {
            display.setCursor(0, 55);
            display.printf("Peer: 0x%02X", peerNodeId);
        }
        display.drawStr(0, 63, "HOLD=Cancelar");
    }

    void drawRSSILoRa() {
        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(0, 7, "MODO RSSI LoRa");

        // Barra RSSI (mapear -120..-30 dBm a 0..100%)
        float pct = constrain((rssiActual + 120) / 90.0f, 0.0f, 1.0f);
        dibujarBarra(0, 12, 128, 10, pct);

        display.setCursor(0, 33);
        display.printf("RSSI: %.0f dBm", rssiActual);

        // Indicador Hot/Cold
        display.setFont(u8g2_font_ncenB10_tr);
        display.setCursor(10, 52);
        switch (tendencia) {
            case RSSI_HOT:  display.print(">> CALIENTE <<"); break;
            case RSSI_COLD: display.print(">> FRIO <<"); break;
            case RSSI_WARM: display.print("-> Manten <-"); break;
        }

        display.setFont(u8g2_font_5x7_tr);
        display.setCursor(0, 63);
        display.printf("n:%d  HOLD=Cancel", rssiSamples);
    }

    void drawNoCompass() {
        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(0, 7, "SIN BRUJULA");

        if (remoto.valid && myGPS.valid) {
            display.setFont(u8g2_font_ncenB14_tr);
            display.setCursor(10, 32);
            if (remoto.distance < 1000) {
                display.printf("%.0f m", remoto.distance);
            } else {
                display.printf("%.1f km", remoto.distance / 1000.0);
            }

            display.setFont(u8g2_font_5x7_tr);
            display.drawStr(0, 48, "Camina y observa distancia");
            display.drawStr(0, 58, "Baja=OK Sube=Gira HOLD=X");
        } else {
            display.drawStr(0, 30, "Esperando datos...");
        }
    }

    void drawSignalLost() {
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(10, 15, "Senal Perdida");

        display.setFont(u8g2_font_5x7_tr);

        if (lastKnownLat != 0) {
            display.setCursor(0, 30);
            display.printf("Ultima: %.5f", lastKnownLat);
            display.setCursor(0, 39);
            display.printf("        %.5f", lastKnownLon);
            display.setCursor(0, 48);
            display.printf("Dist: %.0f m", lastKnownDist);
        }

        // Tiempo transcurrido
        uint32_t elapsed = (millis() - timerEstado) / 1000;
        display.setCursor(0, 58);
        display.printf("%ds/%ds", elapsed, UNREACHABLE_TIMEOUT / 1000);
        display.drawStr(65, 58, "HOLD=Cancel");
    }

    void drawReceivingSOS() {
        // Parpadeo en pantalla
        if ((millis() / 300) % 2 == 0) {
            display.setFont(u8g2_font_ncenB14_tr);
            display.drawStr(15, 25, "!! SOS !!");
        }

        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(5, 42, "EMERGENCIA de B");

        if (remoto.valid) {
            display.setCursor(0, 52);
            display.printf("Pos: %.5f, %.5f", remoto.lat, remoto.lon);
        }

        display.drawStr(5, 62, "Iniciando busqueda...");
    }

    void drawUnreachable() {
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(5, 12, "B no encontrado");

        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(0, 28, "Opciones:");

        // Opción 0: Reintentar
        if (menuOption == 0) {
            display.drawStr(5, 40, "> Reintentar busqueda");
            display.drawStr(5, 50, "  Cancelar");
        } else {
            display.drawStr(5, 40, "  Reintentar busqueda");
            display.drawStr(5, 50, "> Cancelar");
        }

        display.drawStr(0, 63, "PRG=Cambiar HOLD=Elegir");
    }

    void drawReunited() {
        display.setFont(u8g2_font_ncenB14_tr);
        display.drawStr(5, 28, "REUNIDOS!");

        display.setFont(u8g2_font_5x7_tr);
        if (remoto.valid && myGPS.valid) {
            display.setCursor(10, 45);
            display.printf("Dist final: %.0f m", remoto.distance);
        }

        uint32_t remaining = 0;
        if (millis() - timerEstado < REUNITED_TIMEOUT) {
            remaining = (REUNITED_TIMEOUT - (millis() - timerEstado)) / 1000;
        }
        display.setCursor(15, 60);
        display.printf("Auto-reset: %ds", remaining);
    }

    // --- Pantallas PERDIDO ---
    void drawPerdido() {
        switch (estadoB) {
            case B_ALERTING:        drawAlertingB(); break;
            case B_ALERTING_NO_GPS: drawAlertingNoGPSB(); break;
            case B_BEACON_MODE:     drawBeaconB(); break;
            case B_SOS_MODE:        drawSOSB(); break;
            case B_REUNITED:        drawReunited(); break;  // Reusar misma pantalla
            default: break;
        }
    }

    void drawAlertingB() {
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(10, 15, "Te buscan!");

        display.setFont(u8g2_font_5x7_tr);

        if (myGPS.valid) {
            display.setCursor(0, 28);
            display.printf("Mi pos: %.5f", myGPS.lat);
            display.setCursor(0, 37);
            display.printf("        %.5f", myGPS.lon);
            display.setCursor(0, 46);
            display.printf("Sats: %d", myGPS.sats);
        } else {
            display.drawStr(0, 28, "Obteniendo GPS...");
            display.setCursor(0, 37);
            display.printf("Sats: %d", myGPS.sats);
        }

        display.setCursor(0, 58);
        display.printf("Enviando STATUS a 0x%02X", peerNodeId);
    }

    void drawAlertingNoGPSB() {
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(10, 15, "Sin satelites");

        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(0, 30, "GPS no disponible");
        display.drawStr(0, 40, "Cambiando a modo beacon...");
    }

    void drawBeaconB() {
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(15, 15, "Modo Beacon");

        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(0, 30, "Enviando beacons LoRa");
        display.drawStr(0, 40, "para que A mida RSSI");

        static int beaconCount = 0;
        if (millis() - timerTx < 100) beaconCount++;
        display.setCursor(0, 55);
        display.printf("Beacons TX: ~%d", beaconCount);
    }

    void drawSOSB() {
        // Parpadeo SOS
        if ((millis() / 200) % 2 == 0) {
            display.setFont(u8g2_font_ncenB14_tr);
            display.drawStr(20, 25, "!! SOS !!");
        }

        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(15, 42, "EMERGENCIA");

        display.setFont(u8g2_font_5x7_tr);
        display.drawStr(0, 55, "Enviando SOS cada 1s");
        display.drawStr(0, 63, "HOLD PRG = Cancelar");
    }
};
