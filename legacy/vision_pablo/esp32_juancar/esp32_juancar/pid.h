#pragma once


class PID {
public:
  // limite_i: tope de la parte integral (en las mismas unidades que la salida),
  // para que no se "infle" cuando la rueda está trabada (anti-windup).
  PID(double kp, double ki, double kd, double limite_i = 1.0, double p_ref = 0)
    : Kp(kp), Ki(ki), Kd(kd), limite_integral(limite_i),
      punto_referencia(p_ref),
      error_integrativo(0), error_derivativo(0), error_total(0) {}

  double control(double variable_proceso, double dt) {
    error_total = punto_referencia - variable_proceso;

    double P_salida = Kp * error_total;

    error_integrativo += error_total * dt;
    if (Ki > 0) {
      double tope = limite_integral / Ki;
      error_integrativo = constrain(error_integrativo, -tope, tope);
    }
    double I_salida = Ki * error_integrativo;

    double derivada = (error_total - error_derivativo) / dt;
    error_derivativo = error_total;
    double D_salida = Kd * derivada;

    double total_salida = P_salida + I_salida + D_salida;
    return total_salida;
  }

  // Borra la memoria (integral y error anterior), por ejemplo al frenar.
  void reiniciar() {
    error_integrativo = 0;
    error_derivativo = 0;
    error_total = 0;
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
  double limite_integral;
  double punto_referencia;
  double error_integrativo;
  double error_derivativo;
  double error_total;
};
