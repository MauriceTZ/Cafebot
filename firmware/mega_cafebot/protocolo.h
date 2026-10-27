#pragma once
#include "config.h"
#include "control.h"

// =====================================================================
//  Protocolo serial (texto, una orden por línea). Ver docs/protocolo_mega.md
//  Se lee carácter a carácter: nunca bloquea el lazo de control.
// =====================================================================

char linea[LARGO_LINEA];
uint8_t linea_len = 0;
bool linea_descartando = false;  // tras una línea demasiado larga, ignorar hasta '\n'
uint16_t telemetria_periodo_ms = 0;  // 0 = apagada
uint32_t telemetria_ultimo_ms = 0;

#define FW_VERSION "cafebot-mega v1.0"

void imprimir_ayuda() {
    Serial.println(F("== " FW_VERSION " ==  (mm, grados, mm/s)"));
    Serial.println(F("A d [v]        avanzar d mm recto (negativo = atras)"));
    Serial.println(F("W a [wmax]     girar a grados en el sitio (+ = izquierda)"));
    Serial.println(F("L a d [v]      cangrejo: d mm en direccion a grados"));
    Serial.println(F("G x y [th]     ir a (x,y) y opcionalmente al rumbo th (grados)"));
    Serial.println(F("V vx wz        velocidad (mm/s, rad/s); repetir antes de 500 ms"));
    Serial.println(F("K a v          velocidad cangrejo (grados, mm/s); repetir < 500 ms"));
    Serial.println(F("S              parar y vaciar cola"));
    Serial.println(F("P | E          pose | contadores de encoders"));
    Serial.println(F("R [x y th]     fijar pose (sin args = 0 0 0)"));
    Serial.println(F("Z              reiniciar encoders (solo detenido)"));
    Serial.println(F("M hz           telemetria T a hz (0 = off)"));
    Serial.println(F("I | IW | IN k | IS 0/1 | IG   IR: estado | blanco | negro k(0=TI,1=TD) | seguridad | guardar"));
    Serial.println(F("X k ang        servo auxiliar k (compuertas) a ang grados"));
    Serial.println(F("Y | Y i us | YS i k us | YG | YD   servos direccion: tabla | pulso crudo | fijar k=-1/0/1 | guardar | defecto"));
    Serial.println(F("J kp ki kff    ganancias del PI de rueda"));
    Serial.println(F("T              ruta cuadrada de prueba 500 mm"));
}

void imprimir_pose() {
    Serial.print(F("POSE "));
    imprimir_pose_corta();
    Serial.print(F(" modo=")); Serial.print(modo);
    Serial.print(F(" postura=")); Serial.print(postura);
    Serial.print(F(" cola=")); Serial.println(cola_n + (meta_activa ? 1 : 0));
}

void imprimir_telemetria() {
    // T ms x y th_mrad vx_mm_s wz_mrad_s modo postura cola ir_mask
    Serial.print(F("T "));
    Serial.print(millis()); Serial.print(' ');
    imprimir_pose_corta(); Serial.print(' ');
    Serial.print((int)vel_x); Serial.print(' ');
    Serial.print((int)(vel_w * 1000.0f)); Serial.print(' ');
    Serial.print(modo); Serial.print(' ');
    Serial.print(postura); Serial.print(' ');
    Serial.print(cola_n + (meta_activa ? 1 : 0)); Serial.print(' ');
    Serial.println(ir_mascara);
}

void imprimir_ir() {
    Serial.print(F("IR mask=")); Serial.print(ir_mascara);
    Serial.print(F(" TI=")); Serial.print(ir_analog[0]);
    Serial.print(F(" TD=")); Serial.print(ir_analog[1]);
    Serial.print(F(" blanco=")); Serial.print(calib_ir.blanco[0]); Serial.print('/'); Serial.print(calib_ir.blanco[1]);
    Serial.print(F(" negro=")); Serial.print(calib_ir.negro[0]); Serial.print('/'); Serial.print(calib_ir.negro[1]);
    Serial.print(F(" seguridad=")); Serial.println(seguridad_ir ? 1 : 0);
}

// Lee hasta n números de la línea; devuelve cuántos encontró
uint8_t leer_numeros(char* p, float* v, uint8_t n) {
    uint8_t k = 0;
    while (k < n) {
        char* fin;
        float x = strtod(p, &fin);
        if (fin == p) break;
        v[k++] = x;
        p = fin;
    }
    return k;
}

void responder_encolado(int16_t id) {
    if (id < 0) { Serial.println(F("ERR cola llena")); return; }
    if (modo != MODO_COLA) { frenar(); modo = MODO_COLA; }
    Serial.print(F("OK #")); Serial.println(id);
}

void ejecutar_linea(char* s) {
    while (*s == ' ') s++;
    if (*s == 0) return;
    char c0 = toupper(s[0]);
    char c1 = toupper(s[1]);
    bool dos_letras = (c1 >= 'A' && c1 <= 'Z');
    char* args = s + (dos_letras ? 2 : 1);
    float v[3];
    uint8_t n = leer_numeros(args, v, 3);

    if (c0 == 'Y') {  // calibración de servos de dirección
        if (dos_letras && c1 == 'S') {  // YS i k us  (k = -1, 0, 1 -> -90°, 0°, +90°)
            if (n < 3 || v[0] < 0 || v[0] > 3 || fabs(v[1]) > 1) { Serial.println(F("ERR YS i k us")); return; }
            uint8_t i = (uint8_t)v[0];
            int8_t k = (int8_t)v[1];
            cal_servos.us[i][k + 1] = (int16_t)constrain(v[2], SERVO_US_MIN, SERVO_US_MAX);
            if (modo == MODO_QUIETO) servos[i].writeMicroseconds(cal_servos.us[i][k + 1]);
        } else if (dos_letras && c1 == 'G') {
            cal_servos_guardar();
            Serial.println(F("OK Y guardado"));
            return;
        } else if (dos_letras && c1 == 'D') {  // volver a los valores por defecto
            cal_servos_defecto();
        } else if (!dos_letras && n >= 2) {  // Y i us : pulso crudo (solo detenido)
            if (modo != MODO_QUIETO || v[0] < 0 || v[0] > 3) { Serial.println(F("ERR Y i us (detenido)")); return; }
            servos[(uint8_t)v[0]].writeMicroseconds(constrain((int)v[1], SERVO_US_MIN, SERVO_US_MAX));
        }
        Serial.print(F("Y us[-max,0,+max] max=")); Serial.print((int)SERVO_ANG_CAL_DEG); Serial.print(':');
        for (uint8_t i = 0; i < 4; i++) {
            Serial.print(F(" M")); Serial.print(i + 1); Serial.print('=');
            Serial.print(cal_servos.us[i][0]); Serial.print('/');
            Serial.print(cal_servos.us[i][1]); Serial.print('/');
            Serial.print(cal_servos.us[i][2]);
        }
        Serial.println();
        return;
    }

    if (dos_letras && c0 == 'I') {
        switch (c1) {
            case 'W': ir_calibrar_blanco(); imprimir_ir(); return;
            case 'N':
                if (n < 1 || v[0] < 0 || v[0] > 1) { Serial.println(F("ERR IN 0|1")); return; }
                ir_calibrar_negro((uint8_t)v[0]); imprimir_ir(); return;
            case 'S': seguridad_ir = (n >= 1 && v[0] != 0); imprimir_ir(); return;
            case 'G': ir_guardar_eeprom(); Serial.println(F("OK IR guardado")); return;
        }
        Serial.println(F("ERR IR?"));
        return;
    }

    switch (c0) {
        case 'A':
            if (n < 1) break;
            responder_encolado(cola_agregar('A', v[0], n > 1 ? v[1] : 0, 0));
            return;
        case 'W':
            if (n < 1) break;
            responder_encolado(cola_agregar('W', radians(v[0]), n > 1 ? radians(v[1]) : 0, 0));
            return;
        case 'L':
            if (n < 2) break;
            if (fabs(v[0]) > CANGREJO_MAX_DEG) { Serial.println(F("ERR angulo cangrejo")); return; }
            responder_encolado(cola_agregar('L', v[0], v[1], n > 2 ? v[2] : 0));
            return;
        case 'G':
        case 'D':
            if (n < 2) break;
            responder_encolado(cola_agregar('G', v[0], v[1], n > 2 ? radians(v[2]) : NAN));
            return;
        case 'V':
            if (n < 2) break;
            if (modo == MODO_COLA) detener_todo(F("REEMPLAZADO"));
            modo = MODO_VEL;
            cmd_vx = v[0]; cmd_wz = v[1];
            ultimo_cmd_vel_ms = millis();
            return;  // sin respuesta: se envía a alta frecuencia
        case 'K':
            if (n < 2) break;
            if (fabs(v[0]) > CANGREJO_MAX_DEG) { Serial.println(F("ERR angulo cangrejo")); return; }
            if (modo == MODO_COLA) detener_todo(F("REEMPLAZADO"));
            modo = MODO_VEL_CANGREJO;
            cmd_ang_k = v[0]; cmd_v_k = v[1];
            ultimo_cmd_vel_ms = millis();
            return;
        case 'S':
        case 'C':
            detener_todo(F("STOP"));
            Serial.println(F("OK S"));
            return;
        case 'P':
            imprimir_pose();
            return;
        case 'E': {
            int32_t e[4];
            encoders_leer(e);
            Serial.print(F("ENC"));
            for (uint8_t i = 0; i < 4; i++) { Serial.print(' '); Serial.print(e[i]); }
            Serial.println();
            return;
        }
        case 'R':
            pose.x = n > 0 ? v[0] : 0;
            pose.y = n > 1 ? v[1] : 0;
            pose.th = n > 2 ? wrapAngle(radians(v[2])) : 0;
            imprimir_pose();
            return;
        case 'Z':
            if (modo != MODO_QUIETO) { Serial.println(F("ERR Z solo detenido")); return; }
            encoders_reset();
            encoders_resync();
            Serial.println(F("OK Z"));
            return;
        case 'M':
            telemetria_periodo_ms = (n > 0 && v[0] > 0) ? (uint16_t)(1000.0f / constrain(v[0], 1, 50)) : 0;
            Serial.print(F("OK M ")); Serial.println(telemetria_periodo_ms);
            return;
        case 'I':
            imprimir_ir();
            return;
        case 'X':
            if (n < 2 || v[0] < 0 || v[0] >= N_SERVOS_AUX) { Serial.println(F("ERR X k angulo")); return; }
            servo_aux((uint8_t)v[0], (int)v[1]);
            Serial.println(F("OK X"));
            return;
        case 'J':
            if (n == 3) { g_kp = v[0]; g_ki = v[1]; g_kff = v[2]; }
            Serial.print(F("J ")); Serial.print(g_kp, 3); Serial.print(' ');
            Serial.print(g_ki, 3); Serial.print(' '); Serial.println(g_kff, 3);
            return;
        case 'T':
            cola_agregar('G', 500, 0, radians(90));
            cola_agregar('G', 500, 500, radians(180));
            cola_agregar('G', 0, 500, radians(-90));
            responder_encolado(cola_agregar('G', 0, 0, 0));
            return;
        case 'H':
        case '?':
            imprimir_ayuda();
            return;
    }
    Serial.print(F("ERR comando: "));
    Serial.println(s);
}

void protocolo_actualizar() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            linea[linea_len] = 0;
            if (!linea_descartando) ejecutar_linea(linea);
            linea_len = 0;
            linea_descartando = false;
        } else if (linea_descartando) {
            continue;
        } else if (linea_len < LARGO_LINEA - 1) {
            linea[linea_len++] = c;
        } else {
            linea_len = 0;
            linea_descartando = true;
            Serial.println(F("ERR linea larga"));
        }
    }
    if (telemetria_periodo_ms && millis() - telemetria_ultimo_ms >= telemetria_periodo_ms) {
        telemetria_ultimo_ms = millis();
        imprimir_telemetria();
    }
}
