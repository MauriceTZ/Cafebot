# Cafebot

Robot recolector de café para la **IEEE LARC OPEN 2026** (Bogotá, 2–6 de noviembre).
El robot recoge pelotas de colores ("granos") de tres árboles, deja los verdes y lleva
los maduros a la caja roja y los sobremaduros a la azul, en rondas de 8 minutos.

**Plan y prioridades: [`docs/plan_competencia.md`](docs/plan_competencia.md)**

## Hardware

Base de 4 ruedas orientables por servo con un Arduino Mega, una Raspberry Pi 5,
un LiDAR LD06, una cámara de Raspberry Pi y un brazo de 5 GDL + garra con Dynamixel XL430.
Detalle en [`docs/hardware.md`](docs/hardware.md).

## Estructura

```
firmware/
  mega_cafebot/      Firmware del Mega (abrir mega_cafebot.ino con el Arduino IDE)
  test_host/         El MISMO firmware compilado en el PC con una planta simulada
pi/
  cafebot/drivers/   mega.py (enlace serial), ld06.py (LiDAR con CRC, en un hilo)
  cafebot/arm/       cinematica.py (IK de mínimo esfuerzo), dxl.py (bus Dynamixel "cuidadoso")
  cafebot/mundo/     cancha.py (Apéndices A/B), simulador_lidar.py
  cafebot/localizacion/ inicio.py (¿en qué sector partimos? + albercas/cajas)
  tools/             teleop.py (teclado), visor_lidar.py
  tests/             pruebas sin hardware
  system/            reglas udev (nombres fijos de puertos)
description/         URDF + mallas del brazo (Fusion 360 → URDF)
docs/                reglamento, plan, protocolo serial, brazo, cámara, hardware, localización
legacy/              código anterior, solo como referencia (no se ejecuta)
```

## Uso rápido

**Firmware.** Abrir `firmware/mega_cafebot/mega_cafebot.ino` en el Arduino IDE, elegir la
placa *Arduino Mega 2560* y subir. En el Monitor Serie (115200, "Nueva línea") escribir `H`.
Comandos en [`docs/protocolo_mega.md`](docs/protocolo_mega.md).

**Probar el firmware sin el robot** (Linux/macOS/WSL con g++):
```bash
firmware/test_host/correr.sh                 # pruebas automáticas
firmware/test_host/correr.sh --interactivo   # escribir "A 500", "W 90", "P"...
```

**Raspberry Pi:**
```bash
cd pi
python3 -m venv --system-site-packages .venv && . .venv/bin/activate
pip install -r requirements.txt
python3 -m tests.test_ld06            # sin hardware
python3 -m tests.test_mega_sim        # driver contra el firmware simulado (necesita g++)
python3 -m tests.test_cinematica      # IK de mínimo esfuerzo vs IK antigua
python3 -m tests.test_dxl_falso       # módulo Dynamixel contra un bus falso
python3 -m tests.test_inicio 100      # localización inicial en 100 rondas simuladas
python3 -m tools.localizar --sim      # o --puerto /dev/cafebot-lidar
python3 -m tools.teleop --sim         # o --puerto /dev/cafebot-mega
python3 -m tools.visor_lidar --puerto /dev/cafebot-lidar
```

## Estado (27 oct)

| Módulo | Estado |
|---|---|
| Firmware Mega v1 | Probado en el robot (27 oct): encoders x2, diámetro efectivo 50,8 mm, cableado y signos de servos de dirección verificados. Falta: calibración fina de servos, `W 360` en el piso, IR. |
| Driver Mega (Pi) | Probado contra el firmware simulado. |
| Driver LD06 | Probado con paquetes sintéticos (CRC, resincronización). Falta probar con el sensor. |
| Brazo | IK de mínimo esfuerzo y bus Dynamixel con perfiles por tiempo y monitoreo (probados sin hardware). Falta la secuencia de agarre. Ver `docs/brazo.md`. |
| Visión | Clasificador Lab por soporte en `legacy/vision_pablo`. Falta portarlo. |
| Localización inicial | Zona de partida + pose + albercas/cajas con una vuelta del LiDAR; 95/100 en simulación (`docs/localizacion.md`). Falta probar con el LD06 real. |
| Misión | Por hacer (N1 del plan). |
