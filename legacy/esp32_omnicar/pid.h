#pragma once


class PID {
  public:
    PID(double kp, double ki, double kd, double p_ref = 0)
      : Kp(kp), Ki(ki), Kd(kd),
        punto_referencia(p_ref),
        error_integrativo(0), error_derivativo(0), error_total(0) {}

    double IRAM_ATTR control(double variable_proceso, double dt) {
        error_total = punto_referencia - variable_proceso;

        double P_salida = Kp * error_total;

        error_integrativo += error_total * dt;
        double I_salida = Ki * error_integrativo;

        double derivada = (error_total - error_derivativo) / dt;
        error_derivativo = error_total;
        double D_salida = Kd * derivada;

        double total_salida = P_salida + I_salida + D_salida;
        return total_salida;
    }

    void set_punto_referencia(double p_ref) {
        punto_referencia = p_ref;
    }

    double get_punto_referencia(void) {
        return punto_referencia;
    }

    double get_error_total(void) {
        return error_total;
    }

  private:
    double Kp, Ki, Kd;
    double punto_referencia;
    double error_integrativo;
    double error_derivativo;
    double error_total;
};
