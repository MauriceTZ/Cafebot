#pragma once
#include "config.h"
#include "control.h"
#include <PS4Controller.h>

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"


void remove_paired_devices() {
    uint8_t pairedDeviceBtAddr[20][6];
    int count = esp_bt_gap_get_bond_device_num();
    esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    for (int i = 0; i < count; i++) {
        esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
    }
}

void mando_notify() {
    if (PS4.Square() && PS4.Square() != square) {
        double nuevo_punto[3] = { posicion_carro_actual(0),
                                  posicion_carro_actual(1),
                                  posicion_carro_actual(2) };
        PS4.setRumble(255, 0);
        PS4.sendToController();
        xQueueSend(cola_navegacion, nuevo_punto, portMAX_DELAY);
        delay(100);
        PS4.setRumble(0, 0);
        PS4.sendToController();
    }
    square = PS4.Square();
    triangle = PS4.Triangle();

    Serial.printf("Datos Mando sin procesar:\n\tlx=%d, ly=%d\n\trx=%d, ry=%d\n\tl2=%d, r2=%d\n",
                  PS4.LStickX(), PS4.LStickY(),
                  PS4.RStickX(), PS4.RStickY(),
                  PS4.L2Value(), PS4.R2Value());
    static bool psbtn = false;
    if (PS4.PSButton() && PS4.PSButton() != psbtn) {
        posicion_carro_actual = { 0, 0, 0 };
        velocidad_carro_actual = { 0, 0, 0 };
        double p[3];
        while (xQueueReceive(cola_navegacion, p, 0) == pdTRUE)
            ;
        PS4.setRumble(255, 0);
        PS4.sendToController();
        delay(100);
        PS4.setRumble(0, 0);
        PS4.sendToController();
    }
    psbtn = PS4.PSButton();
    static bool options_prev = false;
    if (PS4.Options() && PS4.Options() != options_prev) {
        switch (modo_actual) {
            case QUIETO:
                modo_actual = MANDO_MANUAL;
                PS4.setLed(0, 255, 0);
                break;
            case MANDO_MANUAL:
                modo_actual = MANDO_SET_RUTA;
                PS4.setLed(0, 0, 255);
                break;
            case MANDO_SET_RUTA:
                modo_actual = QUIETO;
                PS4.setLed(255, 0, 0);
                break;
        }
        PS4.sendToController();
        set_velocidad({ 0, 0, 0 });
    }
    options_prev = PS4.Options();
    // Normalizar de [-127, 128] a [-1, 1]
    lx = fmap(PS4.LStickX(), -127, 128, -1, 1);
    if (abs(lx) < ZONA_MUERTA) lx = 0;
    ly = fmap(PS4.LStickY(), -127, 128, -1, 1);
    if (abs(ly) < ZONA_MUERTA) ly = 0;
    rx = fmap(PS4.RStickX(), -127, 128, -1, 1);
    if (abs(rx) < ZONA_MUERTA) rx = 0;
    ry = fmap(PS4.RStickY(), -127, 128, -1, 1);
    if (abs(ry) < ZONA_MUERTA) ry = 0;

    l1 = PS4.L1();
    r1 = PS4.R1();

    l2 = fmap(PS4.L2Value(), 0, 255, -1, 1);
    if (abs(l2) < ZONA_MUERTA) l2 = 0;
    r2 = fmap(PS4.R2Value(), 0, 255, -1, 1);
    if (abs(r2) < ZONA_MUERTA) r2 = 0;
}

void mando_on_connect() {
    Serial.println("Mando conectado!");
    PS4.setLed(255, 0, 0);
    PS4.sendToController();
}

void mando_on_disconnect() {
    Serial.println("Mando desconectado!");
}

void mando_begin() {
    PS4.attach(mando_notify);
    PS4.attachOnConnect(mando_on_connect);
    PS4.attachOnDisconnect(mando_on_disconnect);
    PS4.begin(mac_consola_ps4);
    remove_paired_devices();
}