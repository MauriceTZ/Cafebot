#pragma once
#include "config.h"


void motor_begin() {
    ledcAttach(MOTOR0_LPWM_PIN,
               PWM_FREQUENCY,
               PWM_BIT_RESOLUTION);
    ledcAttach(MOTOR0_RPWM_PIN,
               PWM_FREQUENCY,
               PWM_BIT_RESOLUTION);
    ledcWrite(MOTOR0_LPWM_PIN, 0);
    ledcWrite(MOTOR0_RPWM_PIN, 0);

    ledcAttach(MOTOR1_LPWM_PIN,
               PWM_FREQUENCY,
               PWM_BIT_RESOLUTION);
    ledcAttach(MOTOR1_RPWM_PIN,
               PWM_FREQUENCY,
               PWM_BIT_RESOLUTION);
    ledcWrite(MOTOR1_LPWM_PIN, 0);
    ledcWrite(MOTOR1_RPWM_PIN, 0);

    ledcAttach(MOTOR2_LPWM_PIN,
               PWM_FREQUENCY,
               PWM_BIT_RESOLUTION);
    ledcAttach(MOTOR2_RPWM_PIN,
               PWM_FREQUENCY,
               PWM_BIT_RESOLUTION);
    ledcWrite(MOTOR2_LPWM_PIN, 0);
    ledcWrite(MOTOR2_RPWM_PIN, 0);

    ledcAttach(MOTOR3_LPWM_PIN,
               PWM_FREQUENCY,
               PWM_BIT_RESOLUTION);
    ledcAttach(MOTOR3_RPWM_PIN,
               PWM_FREQUENCY,
               PWM_BIT_RESOLUTION);
    ledcWrite(MOTOR3_LPWM_PIN, 0);
    ledcWrite(MOTOR3_RPWM_PIN, 0);
    // while (1) {
    //     ledcWrite(MOTOR0_LPWM_PIN, (1 << PWM_BIT_RESOLUTION) - 10'000);
    //     ledcWrite(MOTOR0_RPWM_PIN, 0);
    //     Serial.println("motor0 L");
    //     delay(2000);

    //     ledcWrite(MOTOR0_RPWM_PIN, (1 << PWM_BIT_RESOLUTION) - 10'000);
    //     ledcWrite(MOTOR0_LPWM_PIN, 0);
    //     Serial.println("motor0 R");
    //     delay(2000);
    //     ledcWrite(MOTOR0_LPWM_PIN, 0);
    //     ledcWrite(MOTOR0_RPWM_PIN, 0);

    //     ledcWrite(MOTOR1_LPWM_PIN, (1 << PWM_BIT_RESOLUTION) - 10'000);
    //     ledcWrite(MOTOR1_RPWM_PIN, 0);
    //     Serial.println("motor1 L");
    //     delay(2000);

    //     ledcWrite(MOTOR1_RPWM_PIN, (1 << PWM_BIT_RESOLUTION) - 10'000);
    //     ledcWrite(MOTOR1_LPWM_PIN, 0);
    //     Serial.println("motor1 R");
    //     delay(2000);
    //     ledcWrite(MOTOR1_LPWM_PIN, 0);
    //     ledcWrite(MOTOR1_RPWM_PIN, 0);

    //     ledcWrite(MOTOR2_LPWM_PIN, (1 << PWM_BIT_RESOLUTION) - 10'000);
    //     ledcWrite(MOTOR2_RPWM_PIN, 0);
    //     Serial.println("motor2 L");
    //     delay(2000);

    //     ledcWrite(MOTOR2_RPWM_PIN, (1 << PWM_BIT_RESOLUTION) - 10'000);
    //     ledcWrite(MOTOR2_LPWM_PIN, 0);
    //     Serial.println("motor2 R");
    //     delay(2000);
    //     ledcWrite(MOTOR2_LPWM_PIN, 0);
    //     ledcWrite(MOTOR2_RPWM_PIN, 0);

    //     ledcWrite(MOTOR3_LPWM_PIN, (1 << PWM_BIT_RESOLUTION) - 10'000);
    //     ledcWrite(MOTOR3_RPWM_PIN, 0);
    //     Serial.println("motor3 L");
    //     delay(2000);

    //     ledcWrite(MOTOR3_RPWM_PIN, (1 << PWM_BIT_RESOLUTION) - 10'000);
    //     ledcWrite(MOTOR3_LPWM_PIN, 0);
    //     Serial.println("motor3 R");
    //     delay(2000);
    //     ledcWrite(MOTOR3_LPWM_PIN, 0);
    //     ledcWrite(MOTOR3_RPWM_PIN, 0);
    // }
}