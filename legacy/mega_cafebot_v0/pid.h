#pragma once

class PID {
  public:
    PID(float kp, float ki, float kd, float p_ref = 0.0f)
      : Kp(kp), Ki(ki), Kd(kd),
        punto_referencia(p_ref),
        error_integrativo(0.0f), error_derivativo(0.0f), error_total(0.0f) {}

    float control(float variable_proceso, float dt) {
        error_total = punto_referencia - variable_proceso;

        float P_salida = Kp * error_total;

        error_integrativo += error_total * dt;
        // Anti-windup para evitar saturación del término integral
        error_integrativo = constrain(error_integrativo, -10.0f, 10.0f);
        float I_salida = Ki * error_integrativo;

        float derivada = (error_total - error_derivativo) / dt;
        error_derivativo = error_total;
        float D_salida = Kd * derivada;

        return P_salida + I_salida + D_salida;
    }

    void set_punto_referencia(float p_ref) {
        punto_referencia = p_ref;
    }

    void reset() {
        error_integrativo = 0.0f;
        error_derivativo = 0.0f;
        error_total = 0.0f;
    }

  private:
    float Kp, Ki, Kd;
    float punto_referencia;
    float error_integrativo;
    float error_derivativo;
    float error_total;
};