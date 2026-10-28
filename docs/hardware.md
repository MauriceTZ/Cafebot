# Hardware

## Arquitectura

```
                 USB                         USB                  USB
 Arduino Mega ◄────────► Raspberry Pi 5 ◄──────────► LD06   ◄──────────► U2D2 ─► 6× XL430 (brazo)
  + shield 4M&9S          │  CSI: cámara                                (TTL, 1 Mbps)
  ├─ 4 motores + encoder  │  I2C: MPU9250 (usar giroscopio, no magnetómetro)
  ├─ 4 servos dirección   │  GPIO: botón de inicio
  ├─ 2 servos compuertas + 1 inclinación de cámara
  └─ 4 IR de borde
```

| Pieza | Responsable | Notas |
|---|---|---|
| Mega | reflejos | PI por rueda, dirección, odometría, primitivas, freno por borde, watchdog. |
| Pi 5 | decisiones | Localización con mapa conocido, visión, brazo, misión, reloj de 8 min. |

## Puertos USB en la Raspberry (vistos el 28 oct con `ls -l /dev/serial/by-id/`)

| Dispositivo | Chip USB | Sin reglas udev | Con `pi/system/99-cafebot.rules` |
|---|---|---|---|
| Arduino Mega (clon) | CH340 (`1a86`) | `/dev/ttyUSB1` | `/dev/cafebot-mega` |
| LD06 | CP2102 (`10c4`) | `/dev/ttyUSB0` | `/dev/cafebot-lidar` |
| U2D2 (brazo) | FTDI (`0403`) | otro `/dev/ttyUSB*` | `/dev/cafebot-u2d2` |

Los números `ttyUSB` dependen del orden en que se conectan: con las reglas instaladas no importa.

## Pines del Mega (`firmware/mega_cafebot/config.h`)

| Rueda | Ubicación (verificada) | Enc A | Enc B | PWM | DIR1 | DIR2 | Servo | Offset |
|---|---|---|---|---|---|---|---|---|
| M1 | delantera der | 18 | 31 | 12 | 34 | 35 | 6 | 89 |
| M2 | delantera izq | 19 | 38 | 8 | 37 | 36 | 10 | 87 |
| M3 | trasera der | 3 | 49 | 9 | 43 | 42 | 13 | 87 |
| M4 | trasera izq | 2 | A1 | 5 | A4 | A5 | 45 | 90 |

Pines provisorios, **por confirmar**: IR delanteros digitales 22/23, IR traseros
analógicos A8/A9, compuertas 44/46, inclinación de cámara 11. Los pines 20/21 (SDA/SCL) también tienen
interrupción y están libres si no se usa I2C en el Mega.

## Medidas de la base
Ruedas Ø53 mm, trocha 217 mm, entre ejes 223 mm, 800 pulsos/vuelta de rueda en el flanco
de subida de A (1600 con `ENCODER_X2`). Reductora 1:8.

## Checklist de puesta en marcha (ruedas en el aire)
1. **Encoders.** Escribir `ED` (pone en cero el diagnóstico), girar **cada rueda a mano exactamente 1 vuelta hacia adelante** y escribir `ED` otra vez. Para cada rueda se ve `subida/bajada`:
   - Ambos ≈ **+800** → cuadratura sana: se puede usar `EM 2` (doble resolución; queda guardado).
   - Ambos ≈ −800 → encoder invertido: cambiar `Mx_INV_ENC` en config.h.
   - Signos **opuestos** (p. ej. +800 / −800) → A y B no están desfasados 90° (o hay ruido/rebote en A): quedarse en `EM 1`. En x1 solo cuenta el flanco de subida, igual que v0.
   - Muy distinto de 800 → revisar `PULSOS_POR_REVOLUCION` o el cableado.
   Con las ruedas en el aire, `DONE` siempre muestra la distancia pedida: solo cuenta pulsos, no sabe cuánto avanzó de verdad. La prueba real es la vuelta a mano (o `A 1000` en el piso, con huincha).
2. `V 100 0` (repetir): las 4 ruedas hacia adelante. Si alguna va al revés, `Mx_INV_MOTOR`.
3. **Dirección, en este orden** (con las ruedas en el aire):
   1. **`L 45 200`** (no `L 90`: a ±90° una rueda girada a la izquierda y una girada a la derecha quedan en la misma línea y no se distinguen a simple vista). Vistas desde arriba (frente arriba), las 4 ruedas deben quedar **paralelas, con la punta delantera hacia la izquierda** ( \\ \\ ). La que quede en la otra diagonal ( / ) → cambiar su signo en `SIGNO_SERVO`. Si una rueda no responde como se espera, `Y i 1200` mueve solo el servo i (0 = M1 … 3 = M4) para comprobar qué rueda mueve cada pin. *No* usar `W 90` para esto.
   2. Anotar en `ESQUINA_X` / `ESQUINA_Y` dónde está físicamente M1…M4 (delantera/trasera, izquierda/derecha).
   3. `W 90`: las ruedas forman un **rombo** (cada una perpendicular a la línea hacia el centro) y el robot gira a la **izquierda** (antihorario visto desde arriba). Si quedan en X apuntando al centro → `SIGNO_GIRO -1`.
4. **Calibrar los servos de dirección (HS-311) en microsegundos** — corrige el desfase del estriado sin desarmar nada:
   - `Y` muestra la tabla actual (µs para -90°, 0°, +90° de M1..M4).
   - Para cada rueda `i` (0 = M1 … 3 = M4): `Y i 1450`, `Y i 1460`… hasta que quede **paralela al chasis** (usar una regla apoyada en el costado). Anotar el valor y fijarlo: `YS i 0 <us>`.
   - Igual para +90° (`YS i 1 <us>`) y -90° (`YS i -1 <us>`), con una escuadra contra el chasis.
   - Si un servo no llega a 90° exactos, calibrar las 4 a un ángulo menor **común** (p. ej. 88°, medido con transportador) y poner ese valor en `SERVO_ANG_CAL_DEG` (config.h): lo importante es que las 4 queden paralelas; la odometría usa el ángulo real.
   - `YG` guarda en EEPROM (sobrevive al apagado y a recargar el firmware). `YD` vuelve a los valores por defecto.
   - Después, `L 90 200`: las 4 ruedas quedan paralelas, rodando hacia la izquierda.
5. En el piso: `A 1000` + huincha → diámetro de rueda. `W 360` → `CINEMATICA_D_GIRO_Z`.
6. `J`: si oscila, bajar `kp`; si no llega a la velocidad, subir `kff` o `PWM_ARRANQUE`.

## LiDAR
Para ver **cajas (90 mm) y albercas (90 mm)** tiene que ir **por debajo de 90 mm**. Plan: al frente de la base, a ~8 cm, con ~180° de vista (`ConfigLD06.x_m = 0.16`, `fov_deg = 95`). Con solo 1 cm de margen, **montarlo nivelado**: 1° de inclinación hacia arriba sube el haz 5 cm a 3 m y deja de ver las cajas lejanas. Si se puede, 6 cm es más seguro.
Más arriba solo ve los árboles (y al público). Configurar su posición y orientación en
`ConfigLD06` (`pi/cafebot/drivers/ld06.py`).

## IMU
El magnetómetro no sirve cerca de motores y estructuras metálicas. Usar el giroscopio
(calibrar el sesgo con el robot quieto al inicio). Muchos "MPU9250" son clones MPU6500
sin magnetómetro.
