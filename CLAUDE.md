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
- Base 30 × 23 cm, 4 ruedas Ø53 mm (diámetro efectivo calibrado 52,6 mm el 28 oct, ruedas nuevas) con motor + encoder
  (800 pulsos/vuelta en x1; se usa **x2**, verificado con `ED`) y servo de dirección HS-311 cada una.
- Ruedas: M1 delantera der, M2 delantera izq, M3 trasera der, M4 trasera izq.
  Servos de dirección: pin 6 = M1, 10 = M2, 13 = M3, 45 = M4, los 4 con `SIGNO_SERVO = +1`.
- Arduino Mega + shield 4Motor&9Servo: control de ruedas, dirección, odometría, sensores de línea.
- Raspberry Pi 5: decisiones. LD06 al frente, por fuera de la base y centrado, a ~5 cm del suelo (29 oct, confirmado); se
  descarta lo que ve en el cuadrado de 45 cm detrás de él (`geometria.lidar_caja_m`).
  Cámara Pi Module 3 **estándar** (29 oct) sobre un cabezal de 2 SG90 (31 oct) en la tapa de la base, a la derecha del
  brazo: inclinación con `X 2` (pin 44 = PL5; 90° = "un poquito hacia arriba"), giro con `X 3` (pin 4 = PG5,
  DESACTIVADO: falla, la cámara mira adelante). Lente a ~37 cm del suelo según el modelo de Fusion
  (`description/fusion`, `camara.cabezal` en la config) [CONFIRMAR con huincha].
- Brazo 5 GDL + garra (abre 6,5 cm) con Dynamixel XL430 (U2D2). Según el modelo de Fusion (31 oct) el eje de la base
  está 7,7 cm delante del centro y 6,3 cm a la izquierda. Altura: 31 oct, la garra horizontal con el brazo vertical
  llega a 70 cm del suelo → base a ~35,2 cm (el modelo decía 31,3): la fila alta (70,7 cm) queda justo fuera;
  con la base a ≥ 37 cm se alcanza (docs/cosecha.md). El hombro se
  sobrecarga si el brazo queda estirado en horizontal (ver `docs/brazo.md`). Ids, ceros, sentidos, límites y
  posturas de cada brazo: su perfil `pi/brazos/<nombre>.json` (`tools.brazo`, la misión usa `brazo.perfil`).
  Calibrado el 30 oct por Pablo (`pi/brazos/principal.json`, docs/calibracion_brazo.md): muñeca = id 10 (el 5 no
  responde), cero de la base girado 180° respecto del anterior, garra invertida; las canastas, girando la base por la
  izquierda (+177° / +168°). Base sin tope: encenderla mirando adelante.
- Puertos en la Pi (28 oct): Mega = clon con CH340 (1a86) → `/dev/ttyUSB1`; LD06 (CP2102) → `/dev/ttyUSB0`;
  U2D2 (FTDI) → otro ttyUSB. Con `pi/system/99-cafebot.rules`: `/dev/cafebot-mega`, `-lidar`, `-u2d2`.
- Sensores de línea: 4 × QTR-1A analógicos en A8–A11 (DI, DD, TI, TD), conector Analog IO del shield; A1/A4/A5
  son del motor M4. Montados en las 4 esquinas, a 45°, junto a cada rueda (posición = centro de la rueda,
  ±111,5 / ±108,5 mm). Calibración y umbrales: `docs/ir.md` (`python3 -m tools.ir`).
- Pendiente de confirmar: compuertas (en el firmware, `X 0`/`X 1` en 46/11; en el modelo de Fusion son Dynamixel).

## Arquitectura
- **Mega = reflejos** (`firmware/mega_cafebot`): PI por rueda, primitivas `A` (avanzar), `W` (girar),
  `L` (cangrejo: las 4 ruedas al mismo ángulo), `G` (waypoint), watchdog, freno por borde.
  Protocolo de texto: `docs/protocolo_mega.md`.
- **Pi = decisiones** (`pi/cafebot`): drivers (`mega.py`, `ld06.py`, `imu.py`), brazo (`arm/`: `BrazoXL430`),
  visión (`vision/`: `VisionCamara`, clasificador Lab de Pablo + proyección de los soportes; cosecha: modelo de
  cámara `camara_modelo.py`, árbol `arbol.py`, detección + fusión cámara/LiDAR + proyección inversa `granos.py`), mapa de la
  cancha y simulador (`mundo/`), localización inicial con LiDAR (`localizacion/`),
  navegación modular (`navegacion/`: SLAM, EKF, landmarks, seguimiento, A*, ejecutor; ver `docs/navegacion.md`),
  misión de la ronda (`mision/`: N1→N2→N3; brazo, visión, compuertas y botón detrás de interfaces; ver `docs/mision.md`).
  Geometría del robot y puertos: `pi/cafebot/config.py` (+ `pi/robot_local.json`).
- Sin ROS. El SLAM de la Pi se usa para ver y calibrar la navegación; en la cancha el mapa conocido
  (Apéndices A/B) da la referencia absoluta.
- `legacy/` es código antiguo solo de referencia; no se ejecuta.

## Cómo trabajar
- Probar sin hardware antes de pedir pruebas en el robot:
  - `firmware/test_host/correr.sh`: el firmware real compilado en el PC con una planta simulada.
  - En `pi/`: `python3 -m tests.<prueba>` (test_ld06, test_mega_sim, test_cinematica,
    test_dxl_falso, test_brazo_falso, test_calibracion_brazo, test_vision_sintetica, test_inicio, test_landmarks,
    test_navegacion, test_mision, test_ir, test_borde, test_cosecha).
  - En el robot, paso a paso: `tools.prueba_navegacion` (base), `tools.visor_lidar` / `tools.navegacion` (LiDAR),
    `tools.ir` (sensores de línea, docs/ir.md), `tools.brazo` (docs/brazo.md), `tools.camara` (docs/camara.md),
    `tools.ver_camara` (la cámara en el navegador + intrínsecos con el tablero, docs/camara.md),
    `tools.cosecha` (cámara + LiDAR + brazo frente a un árbol, docs/cosecha.md), `tools.mision` (todo junto).
- Lo que no se ha medido en el robot se marca `[CONFIRMAR]` / `[MEDIR]` / `[AJUSTAR]` en el código.
- Commits pequeños y descriptivos en español. Cuando el equipo prueba algo en el robot, anotar el
  resultado (fecha + qué se midió) en el comentario de la constante que se ajustó.

## Estado y siguientes pasos (actualizar al avanzar)
- Hecho: firmware del Mega probado en el robot (movimientos, encoders, dirección).
  Localización inicial: 95/100 en simulación.
- En curso, en el robot: calibración fina de servos (`Y`/`YS`/`YG`), `W 360` en el piso, IR, montaje del LiDAR.
- Hecho: navegación modular en la Pi con interfaz (`python3 -m tools.navegacion --sim`).
- Hecho (simulación): misión N1/N2/N3 (`python3 -m tools.mision --sim`, tecla K en la interfaz) con brazo y visión
  simulados; semilla 1 = 35 puntos en 102 s.
- 28 oct: ruedas nuevas → diámetro 52,6 mm; el motor M4 quedó invertido (`M4_INV_MOTOR true`). Desvío de ~6 cm
  a la izquierda en 1 m, por revisar de nuevo (un intento de corrección por rueda se revirtió).
- Hecho (sin hardware, probado con bus falso y fotos sintéticas): brazo real (`tools.brazo`) y cámara real
  (`tools.camara`), conectados a la misión (`tools.mision --brazo --vision camara`).
- 29 oct: sensores de línea QTR-1A integrados (firmware con umbrales/calibración/auto, `tools.ir`, cruces en la
  navegación asociados a la cancha; corrección del EKF lista pero apagada). Probado en simulación.
- 29 oct: cangrejo ESPEJADO: a ±90° dos ruedas a +90 y dos a −90 (motor al revés) para que el arrastre de las L no
  gire la base; patrón con `LE m` / `CANGREJO_ESPEJO_DEF` (por elegir en el robot, docs/hardware.md).
- 29 oct: calibración del brazo sin cámara (`tools.brazo`: escanear, identificar, cero, verificar, limites,
  articulacion, garra calibrar, fila) con un perfil por brazo. Probado con bus falso (`tests.test_calibracion_brazo`).
  Arreglado: el retiro del agarre quedaba detrás del eje de la base y la hacía girar 180° (ahora `brazo.r_min_m`).
- 29 oct: recorrido del borde (`tools.navegacion`, tecla F; `navegacion/bordes.py`): desde una esquina da la vuelta
  al rectángulo tocando los bordes con los IR y vuelve; mide la cancha y el error de cierre de odometría y LiDAR.
  Simulado con `--sim --taller 2.4x1.8` y `tests.test_borde`. Al tocar un borde ya no sigue de largo: después del
  primer toque se endereza retrocediendo y girando hacia el sensor que falta (bisección); entra ≤ 13 mm en la huincha.
- 29 oct: cosecha de prueba (`tools.cosecha`, docs/cosecha.md): pelotas detectadas por color (círculo ajustado al arco
  que deja ver el soporte), pose del árbol + corrección de la cámara fusionando con el LiDAR, posición 3D de cada grano
  (en el marco del árbol, se sigue al acercarse), color por matiz o con referencias, calibración de la cámara con el
  árbol (C) o con el brazo (M + N), agarre y canasta por categoría (postura opcional `transporte`). Probado con fotos
  sintéticas, la foto real del taller (`tests/datos/`) y una sesión simulada (`tests.test_cosecha`). La misión todavía
  clasifica con `VisionCamara` (zonas fijas): pasarla a `granos.analizar` cuando se pruebe en el robot.
- 29 oct: brazo rearmado: ceros nuevos con el brazo recto (base 3085, hombro 2045 —88° distinto—, codo1 3143,
  codo2 1532) en `arm/dxl.py` y en `legacy/codigo_brazo/main.py`; posturas `trabajo`/`descanso` con los mismos ángulos
  de antes. La MUÑECA (id 5) no respondió: revisar con `tools.brazo escanear` y medir su cero.
- 31 oct: modelo de Fusion del robot (`description/fusion/exportar_geometria.py` exporta posiciones y ejes): cámara en el
  cabezal (lente 11,1 cm delante, 5,3 a la derecha, ~37 cm; se mueve con la inclinación alrededor del eje del SG90),
  base del brazo (7,65 / +6,34 / 31,3 cm), LiDAR (x 15,75 cm). `X 2` sale de la inclinación pedida con
  `camara.servo_de_pitch` (misión, `tools.camara`); en `tools.cosecha` I/K mueven el servo y C con 2+ ángulos mide la
  relación servo → inclinación (`pitch_servo_90_deg`, `pitch_por_grado`). Firmware: servos aux 46/11/44/4, el 4 (giro)
  desactivado (`AUX_ACTIVO`): RECARGAR el Mega. Con la base del brazo a 31,3–35,2 cm la fila alta no se alcanza y la
  misión simulada baja de 35 a 14 puntos (semilla 1); con 37 cm vuelve a 35. El brazo simulado de la misión ahora
  usa la cinemática y el plan de agarre del brazo de verdad (antes, una esfera de 38 cm: era optimista).
- 31 oct: integrada la rama de Pablo: soporte sin pelota detectada = NEGRO o VACIO (nunca un color: la madera se
  confundía con AMARILLO; prueba 4b), `tools.ver_camara` (navegador) y, sin Mega, A/R/flechas dicen cuánto mover el
  árbol a mano; encima, **U** lo vuelve a ubicar con el LiDAR (sin foto, sirve de cerca; prueba 7c).
- 31 oct: la SD de la Pi murió (se usa otra, de una Pi 4). Integrada la calibración del brazo de Pablo. Lo que
  estaba en `robot_local.json` (intrínsecos de la cámara, garra, IR) quedó como valores por defecto en `config.py`.
  Arreglado: al encender, la base se lee en la vuelta que cae dentro de su rango (recortado a −76..+202°; antes, la
  más cercana a 0°: encendida pasado +180° leía una vuelta corrida); fuera de rango no se activa el torque.
- Por hacer en el robot: calibrar los IR (`tools.ir`) y confirmar el punto del piso que mira cada uno; calibrar el brazo
  (pasos de `docs/brazo.md`: cero, sentidos, límites, garra, posturas de depósito, `fila`; medir `mision.brazo_x/z`) y la
  cámara (sentido de `X 2`, `tools.camara intrinseca`, `tools.cosecha` C con 2 ángulos del servo y referencias de
  color); medir con huincha la cámara y la base del brazo (¿calza el modelo de Fusion? ¿izquierda/derecha?); medir el
  árbol de prueba (`tools.cosecha --arbol taller --crear-arbol`); recorrido inicial con las líneas; servicio systemd +
  botón GPIO.
- Por medir para la misión: base del brazo, grano delante del tablero, compuertas, GPIO del botón (`docs/mision.md`).
