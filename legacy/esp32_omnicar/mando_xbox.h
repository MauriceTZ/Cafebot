#pragma once
#include "config.h"
#include <BLEGamepadClient.h>

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"


XboxController xbox_controller;

void remove_paired_devices() {
    uint8_t pairedDeviceBtAddr[20][6];
    int count = esp_bt_gap_get_bond_device_num();
    esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    for (int i = 0; i < count; i++) {
        esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
    }
}

void mando_notify(XboxControlsState& s) {
    if (s.buttonX && s.buttonX != square) {
        double nuevo_punto[3] = { posicion_carro_actual(0),
                                  posicion_carro_actual(1),
                                  posicion_carro_actual(2) };

        xQueueSend(cola_navegacion, nuevo_punto, portMAX_DELAY);
        XboxVibrationsCommand cmd;
        cmd.rightTriggerMotor = 1.0;
        cmd.durationMs = 400;
        xbox_controller.write(cmd);
    }
    square = s.buttonX;
    triangle = s.buttonY;

    static bool xboxbtn = false;
    if (s.xboxButton && s.xboxButton != xboxbtn) {
        posicion_carro_actual = { 0, 0, 0 };
        velocidad_carro_actual = { 0, 0, 0 };
        double p[3];
        while (xQueueReceive(cola_navegacion, p, 0) == pdTRUE)
            ;
        XboxVibrationsCommand cmd;
        cmd.rightTriggerMotor = 1.0;
        cmd.durationMs = 400;
        xbox_controller.write(cmd);
    }
    xboxbtn = s.xboxButton;

    static bool menu_prev = false;
    if (s.menuButton && s.menuButton != menu_prev) {
        XboxVibrationsCommand cmd;
        switch (modo_actual) {
            case QUIETO:
                modo_actual = MANDO_MANUAL;
                cmd.rightMotor = 1.0;
                break;
            case MANDO_MANUAL:
                modo_actual = MANDO_SET_RUTA;
                cmd.leftMotor = 1.0;
                break;
            case MANDO_SET_RUTA:
                modo_actual = QUIETO;
                cmd.leftTriggerMotor = 1.0;
                break;
        }
        cmd.durationMs = 200;
        xbox_controller.write(cmd);
        set_velocidad({ 0, 0, 0 });
    }
    menu_prev = s.menuButton;

    // Valores ya normalizados...
    lx = s.leftStickX;
    if (abs(lx) < ZONA_MUERTA) lx = 0;
    ly = s.leftStickY;
    if (abs(ly) < ZONA_MUERTA) ly = 0;
    rx = s.rightStickX;
    if (abs(rx) < ZONA_MUERTA) rx = 0;
    ry = s.rightStickY;
    if (abs(ry) < ZONA_MUERTA) ry = 0;
    l1 = s.leftBumper;
    r1 = s.rightBumper;
    l2 = s.leftTrigger;
    if (abs(l2) < ZONA_MUERTA) l2 = 0;
    r2 = s.rightTrigger;
    if (abs(r2) < ZONA_MUERTA) r2 = 0;
}

void mando_on_connect(XboxController& ctrl) {
    Serial.printf("Mando conectado!\n");
}

void mando_on_disconnect(XboxController& ctrl) {
    Serial.printf("Mando desconectado!\n");
}

void mando_begin() {
    // remove_paired_devices();
    xbox_controller.begin();
    // xbox_controller.onConnecting(onConnecting);              // connecting started
    // xbox_controller.onConnectionFailed(onConnectionFailed);  // connection failed
    xbox_controller.onConnected(mando_on_connect);        // connection succeded
    xbox_controller.onDisconnected(mando_on_disconnect);  // disconnected
    xbox_controller.onValueChanged(mando_notify);         // value updated

    auto* pAutoScan = BLEGamepadClient::getAutoScan();
    // pAutoScan->onScanStarted(onScanStarted);
    // pAutoScan->onScanStopped(onScanStopped);
}