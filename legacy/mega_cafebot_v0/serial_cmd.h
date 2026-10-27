#pragma once
#include "config.h"
#include "control.h"

void imprimir_posicion() {
    Serial.print(F("POS X:"));
    Serial.print(posicion_carro_actual.x, 2);
    Serial.print(F(" Y:"));
    Serial.print(posicion_carro_actual.y, 2);
    Serial.print(F(" W:"));
    Serial.print(posicion_carro_actual.w, 4);
    Serial.print(F(" W_DEG:"));
    Serial.print(degrees(posicion_carro_actual.w), 1);
    Serial.print(F(" SERVOS:"));
    Serial.print(postura_servos_actual == SERVOS_GIRO_Z ? F("45_GIRO_Z") : F("0_RECTO"));
    Serial.print(F(" COLA:"));
    Serial.println(cola_navegacion ? cola_navegacion->count : 0);
}

void imprimir_ayuda() {
    Serial.println(F("\n=== COMANDOS SERIALES DISPONIBLES ==="));
    Serial.println(F(" G <x_mm> <y_mm> <w_rad> : Agregar punto a cola_navegacion (radianes)"));
    Serial.println(F(" D <x_mm> <y_mm> <w_deg> : Agregar punto a cola_navegacion (grados)"));
    Serial.println(F(" P                       : Obtener posicion actual (X, Y, W) y estado de servos"));
    Serial.println(F(" M <0|1>                 : Activar/desactivar telemetria continua (5 Hz)"));
    Serial.println(F(" V <vx_mm_s> <wz_rad_s>  : Control manual (si vx=0 y wz!=0, usa servos a 45 deg)"));
    Serial.println(F(" S                       : Detener el carro y alinear servos a 0 deg"));
    Serial.println(F(" C                       : Vaciar cola_navegacion"));
    Serial.println(F(" R                       : Resetear odometria (0, 0, 0) y vaciar cola"));
    Serial.println(F(" T                       : Ejecutar ruta cuadrada de prueba"));
    Serial.println(F("=====================================\n"));
}

void procesar_comando_serial() {
    if (!Serial.available()) return;

    String linea = Serial.readStringUntil('\n');
    linea.trim();
    if (linea.length() == 0) return;

    char cmd = toupper(linea.charAt(0));
    const char* args = linea.c_str() + 1;

    switch (cmd) {
        case 'G':
            {
                char* endPtr;
                float x = strtod(args, &endPtr);
                float y = strtod(endPtr, &endPtr);
                float w = strtod(endPtr, &endPtr);
                float punto[3] = { x, y, w };
                if (xQueueSend(cola_navegacion, punto, 0)) {
                    modo_actual = NAVEGACION_RUTA;
                    Serial.print(F("ENQUEUED (rad) -> X:"));
                    Serial.print(x, 1);
                    Serial.print(F(" Y:"));
                    Serial.print(y, 1);
                    Serial.print(F(" W:"));
                    Serial.println(w, 3);
                } else {
                    Serial.println(F("ERROR: cola_navegacion llena."));
                }
                break;
            }

        case 'D':
            {
                char* endPtr;
                float x = strtod(args, &endPtr);
                float y = strtod(endPtr, &endPtr);
                float w_deg = strtod(endPtr, &endPtr);
                float punto[3] = { x, y, radians(w_deg) };
                if (xQueueSend(cola_navegacion, punto, 0)) {
                    modo_actual = NAVEGACION_RUTA;
                    Serial.print(F("ENQUEUED (deg) -> X:"));
                    Serial.print(x, 1);
                    Serial.print(F(" Y:"));
                    Serial.print(y, 1);
                    Serial.print(F(" W_deg:"));
                    Serial.println(w_deg, 1);
                } else {
                    Serial.println(F("ERROR: cola_navegacion llena."));
                }
                break;
            }

        case 'P':
            imprimir_posicion();
            break;

        case 'M':
            telemetria_activa = (atoi(args) != 0);
            Serial.print(F("Telemetria continua: "));
            Serial.println(telemetria_activa ? F("ON") : F("OFF"));
            break;

        case 'V':
            {
                char* endPtr;
                float vx = strtod(args, &endPtr);
                float wz = strtod(endPtr, &endPtr);
                modo_actual = CONTROL_SERIAL_VEL;
                velocidad_carro_objetivo = { vx, wz };
                Serial.print(F("VEL -> Vx:"));
                Serial.print(vx, 1);
                Serial.print(F(" Wz:"));
                Serial.println(wz, 2);
                break;
            }

        case 'S':
            modo_actual = QUIETO;
            detener_motores_y_pids();
            rot_servos(0, 0, 0, 0);
            postura_servos_actual = SERVOS_RECTO;
            Serial.println(F("STOP: Carro detenido y servos en 0 grados."));
            break;

        case 'C':
            xQueueReset(cola_navegacion);
            hay_objetivo_activo = false;
            modo_actual = QUIETO;
            detener_motores_y_pids();
            rot_servos(0, 0, 0, 0);
            postura_servos_actual = SERVOS_RECTO;
            Serial.println(F("CLEAR: Cola vaciada."));
            break;

        case 'R':
            xQueueReset(cola_navegacion);
            hay_objetivo_activo = false;
            modo_actual = QUIETO;
            detener_motores_y_pids();
            rot_servos(0, 0, 0, 0);
            postura_servos_actual = SERVOS_RECTO;
            noInterrupts();
            m1_counter = m2_counter = m3_counter = m4_counter = 0;
            interrupts();
            posicion_carro_actual = { 0.0f, 0.0f, 0.0f };
            velocidad_carro_actual = { 0.0f, 0.0f };
            Serial.println(F("RESET: Posicion (0, 0, 0) reiniciada."));
            break;

        case 'T':
            cargar_ruta_prueba();
            break;

        case 'H':
        case '?':
            imprimir_ayuda();
            break;

        default:
            Serial.println(F("Comando desconocido. Envia 'H' para ayuda."));
            break;
    }
}