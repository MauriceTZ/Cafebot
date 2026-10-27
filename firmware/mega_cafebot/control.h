#pragma once
#include "config.h"
#include "encoders.h"
#include "actuadores.h"
#include "ir.h"

// =====================================================================
//  Control: PI por rueda, odometría y ejecución de movimientos
// =====================================================================

// ---------------- Estado global ----------------
struct Pose2D { float x, y, th; };          // mm, mm, rad
Pose2D pose = { 0, 0, 0 };
float vel_x = 0, vel_y = 0, vel_w = 0;      // velocidad medida en el cuerpo (mm/s, rad/s)

enum Modo : uint8_t { MODO_QUIETO = 0, MODO_COLA = 1, MODO_VEL = 2, MODO_VEL_CANGREJO = 3 };
Modo modo = MODO_QUIETO;

Postura postura = POSTURA_RECTO;  // postura con la que se calcula la odometría
float ang_cangrejo_actual = 0;    // grados

// Consignas de los modos de velocidad (V y K)
float cmd_vx = 0, cmd_wz = 0, cmd_ang_k = 0, cmd_v_k = 0;
uint32_t ultimo_cmd_vel_ms = 0;

bool seguridad_ir = false;  // si true: frena al detectar un borde nuevo mientras se mueve

// Ganancias ajustables en vivo (comando J)
float g_kp = VEL_KP, g_ki = VEL_KI, g_kff = VEL_KFF;

// ---------------- Cola de movimientos ----------------
struct Meta {
    char tipo;   // 'G' waypoint, 'A' avanzar, 'W' girar, 'L' cangrejo
    float a, b, c;
    uint16_t id;
};
Meta cola[LARGO_COLA];
uint8_t cola_ini = 0, cola_n = 0;
uint16_t siguiente_id = 1;

Meta meta;                 // meta en ejecución
bool meta_activa = false;
uint8_t meta_fase = 0;
Pose2D pose_ini;           // pose al iniciar la meta
float th_obj = 0;          // rumbo objetivo (W y fase final de G)

int16_t cola_agregar(char tipo, float a, float b, float c) {
    if (cola_n >= LARGO_COLA) return -1;
    uint8_t k = (cola_ini + cola_n) % LARGO_COLA;
    cola[k] = { tipo, a, b, c, siguiente_id };
    cola_n++;
    return siguiente_id++;
}

bool cola_sacar(Meta& m) {
    if (cola_n == 0) return false;
    m = cola[cola_ini];
    cola_ini = (cola_ini + 1) % LARGO_COLA;
    cola_n--;
    return true;
}

// ---------------- Lazo de velocidad por rueda ----------------
int32_t enc_previo[4] = { 0, 0, 0, 0 };
float v_med[4] = { 0, 0, 0, 0 };    // mm/s filtrada
float v_ref[4] = { 0, 0, 0, 0 };    // consigna con rampa
float v_obj[4] = { 0, 0, 0, 0 };    // consigna pedida
float integ[4] = { 0, 0, 0, 0 };
int16_t pwm_actual[4] = { 0, 0, 0, 0 };

void lazo_reset() {
    for (uint8_t i = 0; i < 4; i++) { v_ref[i] = 0; v_obj[i] = 0; integ[i] = 0; pwm_actual[i] = 0; }
}

void frenar(ModoParada p = PARADA_FRENO) {
    lazo_reset();
    motores_parar(p);
}

// Resincroniza el contador previo (tras resetear encoders o descartar cuentas)
void encoders_resync() {
    encoders_leer(enc_previo);
}

void lazo_ruedas() {
    for (uint8_t i = 0; i < 4; i++) {
        // Rampa de aceleración de la consigna
        float paso = ACEL_MAX_MM_S2 * dt;
        float d = v_obj[i] - v_ref[i];
        v_ref[i] += constrain(d, -paso, paso);

        if (fabs(v_ref[i]) < 0.5f && fabs(v_obj[i]) < 0.5f) {
            integ[i] = 0;
            pwm_actual[i] = 0;
            motor_pwm(i, 0, PARADA_FRENO);
            continue;
        }
        float e = v_ref[i] - v_med[i];
        float ff = g_kff * v_ref[i] + (v_ref[i] > 0 ? PWM_ARRANQUE : -PWM_ARRANQUE);
        float integ_nuevo = integ[i] + e * dt;
        float u = ff + g_kp * e + g_ki * integ_nuevo;
        // Anti-windup: solo se integra si no satura (o si el error ayuda a salir)
        if ((u < PWM_MAX && u > -PWM_MAX) || (u >= PWM_MAX && e < 0) || (u <= -PWM_MAX && e > 0))
            integ[i] = integ_nuevo;
        u = ff + g_kp * e + g_ki * integ[i];
        pwm_actual[i] = (int16_t)constrain(u, -PWM_MAX, PWM_MAX);
        motor_pwm(i, pwm_actual[i]);
    }
}

// ---------------- Cinemática ----------------
// Convenciones: x adelante, y izquierda, th antihorario. Qué rueda está en qué esquina: config.h
void cinematica_inversa(Postura p, float vx, float wz, float v_cangrejo) {
    for (uint8_t i = 0; i < 4; i++) {
        float lado = ESQUINA_Y[i];  // +1 izquierda, -1 derecha
        if (p == POSTURA_GIRO)
            v_obj[i] = -lado * wz * CINEMATICA_D_GIRO_Z * 0.5f;   // derecha avanza, izquierda retrocede
        else if (p == POSTURA_CANGREJO)
            v_obj[i] = v_cangrejo;
        else
            v_obj[i] = vx - lado * wz * CINEMATICA_W * 0.5f;      // recto con corrección de rumbo
    }
}

// Odometría con los mm recorridos por cada rueda en este ciclo
void odometria(const float d[4]) {
    float d_der = 0, d_izq = 0;
    uint8_t n_der = 0, n_izq = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (ESQUINA_Y[i] < 0) { d_der += d[i]; n_der++; }
        else { d_izq += d[i]; n_izq++; }
    }
    d_der = n_der ? d_der / n_der : 0;
    d_izq = n_izq ? d_izq / n_izq : 0;
    float dx = 0, dy = 0, dth = 0;
    if (postura == POSTURA_GIRO) {
        dth = (d_der - d_izq) / CINEMATICA_D_GIRO_Z;
    } else if (postura == POSTURA_CANGREJO) {
        float s = 0.25f * (d[0] + d[1] + d[2] + d[3]);
        float a = radians(ang_cangrejo_actual);
        dx = s * cos(a);
        dy = s * sin(a);
    } else {
        dx = 0.5f * (d_der + d_izq);
        dth = (d_der - d_izq) / CINEMATICA_W;
    }
    float th_m = pose.th + 0.5f * dth;
    float c = cos(th_m), s = sin(th_m);
    pose.x += dx * c - dy * s;
    pose.y += dx * s + dy * c;
    pose.th = wrapAngle(pose.th + dth);
    vel_x = dx / dt; vel_y = dy / dt; vel_w = dth / dt;
}

// ---------------- Mensajes ----------------
void imprimir_pose_corta() {
    Serial.print((long)pose.x); Serial.print(' ');
    Serial.print((long)pose.y); Serial.print(' ');
    Serial.print((long)(pose.th * 1000.0f));
}

void terminar_meta(const __FlashStringHelper* resultado, const __FlashStringHelper* motivo = nullptr) {
    if (!meta_activa) return;
    Serial.print(resultado); Serial.print(F(" #")); Serial.print(meta.id);
    if (motivo) { Serial.print(' '); Serial.print(motivo); }
    Serial.print(' '); imprimir_pose_corta(); Serial.println();
    meta_activa = false;
}

void detener_todo(const __FlashStringHelper* motivo) {
    if (meta_activa) {
        terminar_meta(F("ABORT"), motivo);
    } else if (modo != MODO_QUIETO) {
        Serial.print(F("EVT STOP ")); Serial.print(motivo); Serial.print(' ');
        imprimir_pose_corta(); Serial.println();
    }
    cola_ini = 0; cola_n = 0;
    modo = MODO_QUIETO;
    frenar(PARADA_FRENO);
}

// ---------------- Ejecución de metas ----------------
// Cada llamada decide la postura deseada y las velocidades del cuerpo.
struct Salida { Postura p; float ang_k; float vx; float wz; float vk; bool hecho; };

float vel_por_distancia(float restante, float vmax) {
    float v = constrain(KP_DISTANCIA * fabs(restante), VELOCIDAD_MINIMA, vmax);
    return restante >= 0 ? v : -v;
}

float vel_por_angulo(float err, float wmax) {
    float w = constrain(KP_ANGULO * err, -wmax, wmax);
    if (fabs(w) < VELOCIDAD_ANGULAR_MIN) w = (w >= 0) ? VELOCIDAD_ANGULAR_MIN : -VELOCIDAD_ANGULAR_MIN;
    return w;
}

Salida paso_meta() {
    Salida o = { POSTURA_RECTO, 0, 0, 0, 0, false };
    switch (meta.tipo) {
        case 'A': {  // avanzar a distancia relativa a (mm), vel b
            float vmax = meta.b > 0 ? meta.b : VELOCIDAD_NAVEGACION;
            float s = (pose.x - pose_ini.x) * cos(pose_ini.th) + (pose.y - pose_ini.y) * sin(pose_ini.th);
            float rest = meta.a - s;
            if (fabs(rest) < TOLERANCIA_DISTANCIA_MM) { o.hecho = true; break; }
            o.vx = vel_por_distancia(rest, vmax);
            o.wz = KP_RUMBO * wrapAngle(pose_ini.th - pose.th);
            break;
        }
        case 'W': {  // girar en el sitio un ángulo relativo a (rad), vel máx b
            float wmax = meta.b > 0 ? meta.b : VELOCIDAD_ANGULAR_MAX;
            float err = wrapAngle(th_obj - pose.th);
            if (fabs(err) < TOLERANCIA_ANGULO_RAD) { o.hecho = true; break; }
            o.p = POSTURA_GIRO;
            o.wz = vel_por_angulo(err, wmax);
            break;
        }
        case 'L': {  // cangrejo: ángulo a (grados), distancia b (mm), vel c
            float vmax = meta.c > 0 ? meta.c : VELOCIDAD_NAVEGACION;
            float dir = pose_ini.th + radians(meta.a);
            float s = (pose.x - pose_ini.x) * cos(dir) + (pose.y - pose_ini.y) * sin(dir);
            float rest = meta.b - s;
            if (fabs(rest) < TOLERANCIA_DISTANCIA_MM) { o.hecho = true; break; }
            o.p = POSTURA_CANGREJO;
            o.ang_k = meta.a;
            o.vk = vel_por_distancia(rest, vmax);
            break;
        }
        case 'G': {  // waypoint absoluto (a, b) mm con rumbo final c rad (NAN = sin rumbo final)
            float dx = meta.a - pose.x, dy = meta.b - pose.y;
            float dist = sqrt(dx * dx + dy * dy);
            float err_rumbo = wrapAngle(atan2(dy, dx) - pose.th);
            if (meta_fase < 2 && dist < TOLERANCIA_DISTANCIA_MM) meta_fase = 2;
            if (meta_fase == 0) {
                if (fabs(err_rumbo) > TOLERANCIA_ANGULO_RAD) {
                    o.p = POSTURA_GIRO;
                    o.wz = vel_por_angulo(err_rumbo, VELOCIDAD_ANGULAR_MAX);
                    break;
                }
                meta_fase = 1;
            }
            if (meta_fase == 1) {
                if (fabs(err_rumbo) > UMBRAL_DESVIO_RUMBO_RAD) { meta_fase = 0; break; }
                o.vx = vel_por_distancia(dist, VELOCIDAD_NAVEGACION);
                o.wz = KP_RUMBO * err_rumbo;
                break;
            }
            if (isnan(meta.c)) { o.hecho = true; break; }
            float err = wrapAngle(meta.c - pose.th);
            if (fabs(err) < TOLERANCIA_ANGULO_RAD) { o.hecho = true; break; }
            o.p = POSTURA_GIRO;
            o.wz = vel_por_angulo(err, VELOCIDAD_ANGULAR_MAX);
            break;
        }
        default:
            o.hecho = true;
    }
    return o;
}

void empezar_meta() {
    meta_activa = true;
    meta_fase = 0;
    pose_ini = pose;
    if (meta.tipo == 'W') th_obj = wrapAngle(pose.th + meta.a);
}

// ---------------- Tick principal (60 Hz) ----------------
void control_tick() {
    // 1. Encoders -> mm por rueda en este ciclo
    int32_t enc[4];
    encoders_leer(enc);
    float d[4];
    for (uint8_t i = 0; i < 4; i++) {
        int32_t delta = enc[i] - enc_previo[i];
        enc_previo[i] = enc[i];
        d[i] = delta * mm_por_cuenta();
    }

    // 2. Mientras los servos pivotan, las ruedas giran solas: se descartan esas cuentas
    if (!servos_actualizar()) {
        for (uint8_t i = 0; i < 4; i++) v_med[i] = 0;
        frenar(PARADA_NEUTRO);  // ruedas libres para pivotar
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
        v_med[i] = FILTRO_VEL_ALFA * (d[i] / dt) + (1.0f - FILTRO_VEL_ALFA) * v_med[i];
    odometria(d);

    // 3. Seguridad de borde
    if (ir_flancos_subida || ir_flancos_bajada) {
        Serial.print(F("EVT IR ")); Serial.print(ir_mascara); Serial.print(' ');
        imprimir_pose_corta(); Serial.println();
        bool nuevo_negro = ir_flancos_subida != 0;
        ir_flancos_subida = 0; ir_flancos_bajada = 0;
        if (seguridad_ir && nuevo_negro && modo != MODO_QUIETO) detener_todo(F("BORDE"));
    }

    // 4. Qué quiere el modo actual
    Salida s = { POSTURA_RECTO, 0, 0, 0, 0, false };
    switch (modo) {
        case MODO_QUIETO:
            frenar(PARADA_FRENO);
            return;
        case MODO_VEL:
            if (millis() - ultimo_cmd_vel_ms > WATCHDOG_VEL_MS) { detener_todo(F("WATCHDOG")); return; }
            s.p = (fabs(cmd_vx) < 1.0f && fabs(cmd_wz) > 0.01f) ? POSTURA_GIRO : POSTURA_RECTO;
            s.vx = cmd_vx; s.wz = cmd_wz;
            break;
        case MODO_VEL_CANGREJO:
            if (millis() - ultimo_cmd_vel_ms > WATCHDOG_VEL_MS) { detener_todo(F("WATCHDOG")); return; }
            s.p = POSTURA_CANGREJO; s.ang_k = cmd_ang_k; s.vk = cmd_v_k;
            break;
        case MODO_COLA:
            if (!meta_activa) {
                if (!cola_sacar(meta)) { modo = MODO_QUIETO; frenar(PARADA_FRENO); return; }
                empezar_meta();
            }
            s = paso_meta();
            if (s.hecho) {
                terminar_meta(F("DONE"));
                frenar(PARADA_FRENO);
                return;
            }
            break;
    }

    // 5. Cambio de postura: motores parados hasta que los servos lleguen
    float ang[4];
    angulos_postura(s.p, s.ang_k, ang);
    if (!servos_pedir(ang)) {
        frenar(PARADA_NEUTRO);
        postura = s.p;
        ang_cangrejo_actual = s.ang_k;
        return;
    }
    postura = s.p;
    ang_cangrejo_actual = s.ang_k;

    // 6. Cinemática inversa + PI de cada rueda
    cinematica_inversa(s.p, s.vx, s.wz, s.vk);
    lazo_ruedas();
}

void control_begin() {
    encoders_resync();
}
