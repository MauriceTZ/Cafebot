// Sobre el uso de Double en vez de Float...
// Dentro de una ISR (Interrupt Service Routine)
// la FPU (Floating Point Unit) está desactivada, por tanto
// no se pueden realizar operaciones de punto flotante Float,
// sin embargo, el tipo de dato Double no hace uso de esta FPU
// y realiza estas operaciones mediante software (más lento),
// por tanto es seguro usarlas mientras no tomen demasiados
// ciclos de CPU y desborden (overflow) el Timer o sean
// interrumpidas por otra ISR.

#include "timer.h"


// Importar segun el mando que se use, el de PS4 requiere
// saber la dirección MAC de la consola/PC a la que está
// emparejado el mando.

#include "mando_ps4.h"
// #include "mando_xbox.h"

#include "encoder.h"
#include "motor.h"
#include "cinematica.h"
#include "control.h"


void setup() {
    Serial.begin(115200);
    // timer_begin();
    // mando_begin();
    // encoder_begin();
    motor_begin();
    // control_begin();

    // prueba_manual();
}

void loop() {
    delay(1000 / 2);
    // Serial.println();
    // Serial.printf("Velocidad actual:\n\tVx: %lf\tVy: %lf\tWz: %lf\nPosición actual:\n\tPx: %lf\tPy: %lf\tWz: %lf\n",
    //               velocidad_carro_actual(0), velocidad_carro_actual(1), velocidad_carro_actual(2),
    //               posicion_carro_actual(0), posicion_carro_actual(1), posicion_carro_actual(2));

    // Serial.printf("mando: lx=%f, ly=%f\nvelx: %lf, vely=%lf\n", lx, ly, velx, vely);
}