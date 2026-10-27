#pragma once
#include "config.h"
#include "motor.h"
#include "cinematica.h"

// Detiene los motores eligiendo si quedan en Neutro (libre) o Freno Activo (clavado)
void detener_motores_y_pids(ModoParada modo = PARADA_NEUTRO) {
    velocidad_carro_objetivo = { 0.0f, 0.0f };
    m1_pid.set_punto_referencia(0.0f);
    m2_pid.set_punto_referencia(0.0f);
    m3_pid.set_punto_referencia(0.0f);
    m4_pid.set_punto_referencia(0.0f);
    m1_pwm = m2_pwm = m3_pwm = m4_pwm = 0;
    m1_pid.reset();
    m2_pid.reset();
    m3_pid.reset();
    m4_pid.reset();
    power(0, 0, 0, 0, modo);
}

// Inicia o supervisa la transición suave de los servos con motores en NEUTRO
bool solicitar_postura_servos(PosturaServos nueva_postura) {
    if (postura_servos_actual != nueva_postura) {
        // Liberar los motores en NEUTRO para que las ruedas rueden sin resistencia al pivotar
        detener_motores_y_pids(PARADA_NEUTRO);

        postura_servos_actual = nueva_postura;
        servo_angulo_inicio = servo_angulo_actual;
        servo_angulo_destino = (nueva_postura == SERVOS_GIRO_Z) ? ANGULO_SERVOS_Z : 0.0f;

        servos_en_transicion = true;
        t_inicio_transicion_servos = millis();
        actualizar_rampa_servos();
        return false;
    }

    if (servos_en_transicion) {
        detener_motores_y_pids(PARADA_NEUTRO);
        actualizar_rampa_servos();

        uint32_t tiempo_total = TIEMPO_GIRO_SERVOS_MS + TIEMPO_ASENTAMIENTO_SERVOS_MS;
        if (millis() - t_inicio_transicion_servos >= tiempo_total) {
            // Descartar pulsos parásitos originados por el pivotaje de las ruedas
            noInterrupts();
            m1_counter = m2_counter = m3_counter = m4_counter = 0;
            interrupts();
            servos_en_transicion = false;
            return true;
        }
        return false;
    }

    return true;
}

void set_velocidad(float v_bx, float w_bz) {
    velocidad_carro_objetivo.vx = v_bx;
    velocidad_carro_objetivo.wz = w_bz;

    float v_m13_mm_s = 0.0f;
    float v_m24_mm_s = 0.0f;
    inv_kinematics(v_bx, w_bz, v_m13_mm_s, v_m24_mm_s);

    float rev_s_13 = v_m13_mm_s / CIRCUNFERENCIA_RUEDA;
    float rev_s_24 = v_m24_mm_s / CIRCUNFERENCIA_RUEDA;

    m1_pid.set_punto_referencia(rev_s_13);
    m3_pid.set_punto_referencia(rev_s_13);
    m2_pid.set_punto_referencia(rev_s_24);
    m4_pid.set_punto_referencia(rev_s_24);
}

float actualizar_pid_motor(int32_t pulsos_ciclo, PID& pid, int32_t& pwm_acum) {
    float rev_seg = (TIMER_TRIGGER_FREQUENCY * (float)pulsos_ciclo) / PULSOS_POR_REVOLUCION;
    float salida_pid = pid.control(rev_seg, dt);
    int32_t delta_pwm = (int32_t)fmap(salida_pid, 0.0f, 1.0f, 0.0f, PWM_POR_REVOLUCION);

    pwm_acum = constrain(pwm_acum + delta_pwm, -PWM_MAX, PWM_MAX);
    return rev_seg * CIRCUNFERENCIA_RUEDA;
}

void control_carro() {
    // 1. Lectura atómica de encoders
    int32_t c1, c2, c3, c4;
    noInterrupts();
    c1 = m1_counter;
    m1_counter = 0;
    c2 = m2_counter;
    m2_counter = 0;
    c3 = m3_counter;
    m3_counter = 0;
    c4 = m4_counter;
    m4_counter = 0;
    interrupts();

    // Si los servos están interpolando su ángulo, avanzar un paso de la rampa con motores en NEUTRO
    if (servos_en_transicion) {
        actualizar_rampa_servos();
        uint32_t tiempo_total = TIEMPO_GIRO_SERVOS_MS + TIEMPO_ASENTAMIENTO_SERVOS_MS;
        if (millis() - t_inicio_transicion_servos >= tiempo_total) {
            servos_en_transicion = false;
        }
        detener_motores_y_pids(PARADA_NEUTRO);
        return;
    }

    // 2. Actualizar PIDs y calcular velocidad lineal de cada rueda
    float v1_mm = actualizar_pid_motor(c1, m1_pid, m1_pwm);
    float v2_mm = actualizar_pid_motor(c2, m2_pid, m2_pwm);
    float v3_mm = actualizar_pid_motor(c3, m3_pid, m3_pwm);
    float v4_mm = actualizar_pid_motor(c4, m4_pid, m4_pwm);

    // 3. Odometría según postura de los servos
    float v_m13_real = (v1_mm + v3_mm) * 0.5f;
    float v_m24_real = (v2_mm + v4_mm) * 0.5f;

    float v_bx_real = 0.0f, w_bz_real = 0.0f;
    forw_kinematics(v_m13_real, v_m24_real, v_bx_real, w_bz_real);

    velocidad_carro_actual.vx = v_bx_real;
    velocidad_carro_actual.wz = w_bz_real;

    if (postura_servos_actual == SERVOS_GIRO_Z) {
        posicion_carro_actual.w = wrapAngle(posicion_carro_actual.w + w_bz_real * dt);
    } else {
        float theta_mid = posicion_carro_actual.w + (w_bz_real * dt * 0.5f);
        posicion_carro_actual.x += v_bx_real * cos(theta_mid) * dt;
        posicion_carro_actual.y += v_bx_real * sin(theta_mid) * dt;
        posicion_carro_actual.w = wrapAngle(posicion_carro_actual.w + w_bz_real * dt);
    }

    // 4. Máquina de estados
    switch (modo_actual) {
        case QUIETO:
            detener_motores_y_pids(PARADA_NEUTRO);
            return;

        case CONTROL_SERIAL_VEL:
            if (fabs(velocidad_carro_objetivo.vx) < 1.0f && fabs(velocidad_carro_objetivo.wz) > 0.01f) {
                if (!solicitar_postura_servos(SERVOS_GIRO_Z)) return;
            } else {
                if (!solicitar_postura_servos(SERVOS_RECTO)) return;
            }
            set_velocidad(velocidad_carro_objetivo.vx, velocidad_carro_objetivo.wz);
            power(m1_pwm, m2_pwm, m3_pwm, m4_pwm, PARADA_NEUTRO);
            break;

        case NAVEGACION_RUTA:
            if (hay_objetivo_activo) {
                float dx = posicion_carro_objetivo.x - posicion_carro_actual.x;
                float dy = posicion_carro_objetivo.y - posicion_carro_actual.y;
                float distancia = sqrt(dx * dx + dy * dy);
                float angulo_hacia_meta = atan2(dy, dx);
                float error_rumbo = wrapAngle(angulo_hacia_meta - posicion_carro_actual.w);

                switch (fase_actual) {
                    case FASE_APUNTAR_RUMBO:
                        if (distancia <= TOLERANCIA_DISTANCIA_MM) {
                            detener_motores_y_pids(PARADA_FRENO);
                            fase_actual = FASE_ALINEAR_W_FINAL;
                            break;
                        }
                        if (fabs(error_rumbo) > TOLERANCIA_ANGULO_RAD) {
                            if (solicitar_postura_servos(SERVOS_GIRO_Z)) {
                                float cmd_wz = constrain(KP_ANGULO * error_rumbo, -VELOCIDAD_ANGULAR_MAX, VELOCIDAD_ANGULAR_MAX);
                                if (fabs(cmd_wz) < 0.25f) cmd_wz = (cmd_wz >= 0) ? 0.25f : -0.25f;
                                set_velocidad(0.0f, cmd_wz);
                                power(m1_pwm, m2_pwm, m3_pwm, m4_pwm, PARADA_NEUTRO);
                            }
                        } else {
                            // Rumbo alcanzado: frenar brevemente y pasar a avanzar recto
                            detener_motores_y_pids(PARADA_FRENO);
                            fase_actual = FASE_AVANZAR_RECTO;
                        }
                        break;

                    case FASE_AVANZAR_RECTO:
                        if (distancia > TOLERANCIA_DISTANCIA_MM) {
                            if (fabs(error_rumbo) > UMBRAL_DESVIO_RUMBO_RAD) {
                                detener_motores_y_pids(PARADA_FRENO);
                                fase_actual = FASE_APUNTAR_RUMBO;
                                break;
                            }
                            if (solicitar_postura_servos(SERVOS_RECTO)) {
                                float cmd_vx = constrain(KP_DISTANCIA * distancia, 45.0f, VELOCIDAD_NAVEGACION);
                                set_velocidad(cmd_vx, 0.0f);
                                power(m1_pwm, m2_pwm, m3_pwm, m4_pwm, PARADA_NEUTRO);
                            }
                        } else {
                            // Posición (X, Y) alcanzada: clavar posición con Freno Activo
                            detener_motores_y_pids(PARADA_FRENO);
                            fase_actual = FASE_ALINEAR_W_FINAL;
                        }
                        break;

                    case FASE_ALINEAR_W_FINAL:
                        {
                            float error_w_final = wrapAngle(posicion_carro_objetivo.w - posicion_carro_actual.w);
                            if (fabs(error_w_final) > TOLERANCIA_ANGULO_RAD) {
                                if (solicitar_postura_servos(SERVOS_GIRO_Z)) {
                                    float cmd_wz = constrain(KP_ANGULO * error_w_final, -VELOCIDAD_ANGULAR_MAX, VELOCIDAD_ANGULAR_MAX);
                                    if (fabs(cmd_wz) < 0.25f) cmd_wz = (cmd_wz >= 0) ? 0.25f : -0.25f;
                                    set_velocidad(0.0f, cmd_wz);
                                    power(m1_pwm, m2_pwm, m3_pwm, m4_pwm, PARADA_NEUTRO);
                                }
                            } else {
                                // Waypoint completado: Freno Activo para fijar el punto final
                                hay_objetivo_activo = false;
                                detener_motores_y_pids(PARADA_FRENO);
                                Serial.print(F("OK WAYPOINT -> X:"));
                                Serial.print(posicion_carro_actual.x, 1);
                                Serial.print(F(" Y:"));
                                Serial.print(posicion_carro_actual.y, 1);
                                Serial.print(F(" W_DEG:"));
                                Serial.println(degrees(posicion_carro_actual.w), 1);
                            }
                            break;
                        }
                }
            } else {
                float nuevo_punto[3];
                if (xQueueReceive(cola_navegacion, nuevo_punto, 0)) {
                    posicion_carro_objetivo.x = nuevo_punto[0];
                    posicion_carro_objetivo.y = nuevo_punto[1];
                    posicion_carro_objetivo.w = wrapAngle(nuevo_punto[2]);
                    hay_objetivo_activo = true;
                    fase_actual = FASE_APUNTAR_RUMBO;

                    Serial.print(F("-> Nuevo objetivo: X="));
                    Serial.print(posicion_carro_objetivo.x, 1);
                    Serial.print(F(" Y="));
                    Serial.print(posicion_carro_objetivo.y, 1);
                    Serial.print(F(" W_DEG="));
                    Serial.println(degrees(posicion_carro_objetivo.w), 1);
                } else {
                    if (solicitar_postura_servos(SERVOS_RECTO)) {
                        modo_actual = QUIETO;
                        detener_motores_y_pids(PARADA_NEUTRO);
                        Serial.println(F("FIN DE RUTA: Servos en 0 grados y motores en neutro."));
                    }
                }
            }
            break;
    }
}

void control_begin() {
    cola_navegacion = xQueueCreate(LARGO_COLA_NAVEGACION, sizeof(float) * 3);
}

void cargar_ruta_prueba() {
    Serial.println(F("Cargando ruta cuadrada de prueba..."));
    float cola_prueba[][3] = {
        { 500.0f, 0.0f, 0.0f },
        { 500.0f, 500.0f, 1.5708f },
        { 0.0f, 500.0f, 3.1416f },
        { 0.0f, 0.0f, 0.0f }
    };
    for (uint8_t i = 0; i < (sizeof(cola_prueba) / (sizeof(float) * 3)); i++) {
        xQueueSend(cola_navegacion, cola_prueba[i], 0);
    }
    modo_actual = NAVEGACION_RUTA;
}