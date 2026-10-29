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
  cafebot/arm/       brazo.py (brazo real), cinematica.py (IK de mínimo esfuerzo), dxl.py (bus Dynamixel "cuidadoso")
  cafebot/vision/    vision.py (cámara real para la misión), clasificador.py y camara.py (de Pablo), proyeccion.py
  cafebot/navegacion/ SLAM, odometría EKF, landmarks, seguimiento, A*, ejecutor de rutas
  cafebot/mision/     misión de la ronda + interfaces (brazo, visión, compuertas, botón)
  cafebot/config.py  geometría del robot y puertos (se sobreescribe con pi/robot_local.json)
  cafebot/mundo/     cancha.py (Apéndices A/B), simulador_lidar.py
  cafebot/localizacion/ inicio.py (¿en qué sector partimos? + albercas/cajas)
  tools/             prueba_navegacion, teleop, visor_lidar, localizar, navegacion, ir, brazo, camara, mision
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
python3 -m tools.navegacion --sim     # interfaz de navegación (mapa, rutas A→B); ver docs/navegacion.md
python3 -m tests.test_navegacion      # navegación completa en simulación
python3 -m tools.mision --sim         # una ronda completa simulada, con puntaje; ver docs/mision.md
python3 -m tools.teleop --sim         # o --puerto /dev/cafebot-mega
python3 -m tools.prueba_navegacion --puerto /dev/ttyUSB1 cuadrado 500   # recta, giro, cangrejo, ida_vuelta, consola
python3 -m tools.visor_lidar --puerto /dev/cafebot-lidar
```

## Estado (27 oct)

| Módulo | Estado |
|---|---|
| Firmware Mega v1 | Probado en el robot (27 oct): encoders x2, diámetro efectivo 52,6 mm (ruedas nuevas, 28 oct), cableado y signos de servos de dirección verificados. Falta: calibración fina de servos, `W 360` en el piso. |
| Sensores de línea | 4 × QTR-1A en A8–A11: umbrales por sensor con histéresis, calibración manual y automática, cruces ubicados en el mapa y asociados a la cancha. Herramienta `tools.ir`. Probado en simulación; falta calibrar en el robot (`docs/ir.md`). |
| Driver Mega (Pi) | Probado contra el firmware simulado. |
| Driver LD06 | Probado con paquetes sintéticos (CRC, resincronización). Falta probar con el sensor. |
| Brazo | `BrazoXL430`: IK de mínimo esfuerzo en coordenadas del robot, posturas enseñadas a mano, secuencia de agarre vigilada (choques, garra vacía). Perfil por brazo (`pi/brazos/`: ids, ceros, sentidos, límites, posturas). Herramienta `tools.brazo` con la calibración paso a paso, sin cámara. Probado con bus falso; falta calibrar en el robot (`docs/brazo.md`). |
| Visión | `VisionCamara`: clasificador Lab de Pablo + posición de cada soporte proyectada con la medida del LiDAR + color de caja. Herramienta `tools.camara`. Probado con fotos sintéticas; falta calibrar con la cámara (`docs/camara.md`). |
| Localización inicial | Zona de partida + pose + albercas/cajas con una vuelta del LiDAR; 95/100 en simulación (`docs/localizacion.md`). Falta probar con el LD06 real. |
| Navegación (Pi) | SLAM + EKF con odometría del Mega + landmarks + A* + ejecutor, con interfaz gráfica. En simulación: error de 1–2 cm tras ir a un árbol y volver (odometría sola: 13 cm). Falta probar con el LD06 real (`docs/navegacion.md`). |
| Misión | N1→N2→N3 completa en simulación con brazo y visión simulados (`docs/mision.md`). En el robot corre sin brazo ni visión hasta que existan sus clases reales. |
