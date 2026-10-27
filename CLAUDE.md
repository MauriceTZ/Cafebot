# Cafebot — contexto para Claude (y para cualquier persona nueva en el equipo)

Robot recolector de café para la **IEEE LARC OPEN 2026, Bogotá, 2–6 de noviembre de 2026**.
Reglamento: `docs/reglamento/` (v1.0 2025 y v2.0 2026, casi iguales). Plan y prioridades:
`docs/plan_competencia.md`. Responder siempre en **español**.

## Lo esencial del reglamento
- Rondas de 8 min, autonomía total, **un solo botón**, sin comunicación externa (WiFi apagado).
- Cabe en un cubo de 45 cm al partir; puede crecer después del botón.
- Maduro (rojo/naranjo/amarillo) en caja roja +7, sobremaduro (azul/negro) en caja azul +5;
  dentro del robot al final +3 / +1. Errores −5/−7. Ante la duda, **no tocar** el grano.
- Reinicio forzado: tocar línea perimetral o de alberca con las ruedas, mover una alberca >10 mm,
  sacar o botar 2+ granos a la vez (los jueces deben VER que sale uno a la vez).
- Se puede reprogramar entre rondas. Clasifican las 2 mejores de 4 rondas.

## Hardware (verificado en el robot el 27 oct salvo que diga lo contrario)
- Base 30 × 23 cm, 4 ruedas Ø53 mm (diámetro efectivo calibrado 50,8 mm) con motor + encoder
  (800 pulsos/vuelta en x1; se usa **x2**, verificado con `ED`) y servo de dirección HS-311 cada una.
- Ruedas: M1 delantera der, M2 delantera izq, M3 trasera der, M4 trasera izq.
  Servos de dirección: pin 6 = M1, 10 = M2, 13 = M3, 45 = M4, los 4 con `SIGNO_SERVO = +1`.
- Arduino Mega + shield 4Motor&9Servo: control de ruedas, dirección, odometría, IR de borde.
- Raspberry Pi 5: decisiones. LD06 adelante a ~8 cm del suelo (180° de vista).
  Cámara Pi Module 3 **estándar** fija a ~35 cm sobre un servo de inclinación (`X 2 <ángulo>`).
- Brazo 5 GDL + garra con Dynamixel XL430 (U2D2), base a 36 cm del suelo. El hombro se
  sobrecarga si el brazo queda estirado en horizontal (ver `docs/brazo.md`).
- Pendiente de confirmar: pines de los IR (22/23 digitales, A8/A9 analógicos), compuertas (44/46).

## Arquitectura
- **Mega = reflejos** (`firmware/mega_cafebot`): PI por rueda, primitivas `A` (avanzar), `W` (girar),
  `L` (cangrejo: las 4 ruedas al mismo ángulo), `G` (waypoint), watchdog, freno por borde.
  Protocolo de texto: `docs/protocolo_mega.md`.
- **Pi = decisiones** (`pi/cafebot`): drivers (`mega.py`, `ld06.py`, `imu.py`), brazo (`arm/`), mapa de la
  cancha y simulador (`mundo/`), localización inicial con LiDAR (`localizacion/`),
  navegación modular (`navegacion/`: SLAM, EKF, landmarks, seguimiento, A*, ejecutor; ver `docs/navegacion.md`).
  Geometría del robot y puertos: `pi/cafebot/config.py` (+ `pi/robot_local.json`).
- Sin ROS. El SLAM de la Pi se usa para ver y calibrar la navegación; en la cancha el mapa conocido
  (Apéndices A/B) da la referencia absoluta.
- `legacy/` es código antiguo solo de referencia; no se ejecuta.

## Cómo trabajar
- Probar sin hardware antes de pedir pruebas en el robot:
  - `firmware/test_host/correr.sh`: el firmware real compilado en el PC con una planta simulada.
  - En `pi/`: `python3 -m tests.<prueba>` (test_ld06, test_mega_sim, test_cinematica,
    test_dxl_falso, test_inicio, test_landmarks, test_navegacion).
- Lo que no se ha medido en el robot se marca `[CONFIRMAR]` / `[MEDIR]` / `[AJUSTAR]` en el código.
- Commits pequeños y descriptivos en español. Cuando el equipo prueba algo en el robot, anotar el
  resultado (fecha + qué se midió) en el comentario de la constante que se ajustó.

## Estado y siguientes pasos (actualizar al avanzar)
- Hecho: firmware del Mega probado en el robot (movimientos, encoders, dirección).
  Localización inicial: 95/100 en simulación.
- En curso, en el robot: calibración fina de servos (`Y`/`YS`/`YG`), `W 360` en el piso, IR, montaje del LiDAR.
- Hecho: navegación modular en la Pi con interfaz (`python3 -m tools.navegacion --sim`).
- Por hacer en software: IR en la navegación, misión N1 (partida → frente a un árbol), secuencia de agarre del brazo,
  visión por soporte (portar `legacy/vision_pablo/clasificador.py`), botón de inicio + arranque automático.
