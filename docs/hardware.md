# Hardware

## Arquitectura

```
                 USB                         USB                  USB
 Arduino Mega ◄────────► Raspberry Pi 5 ◄──────────► LD06   ◄──────────► U2D2 ─► 6× XL430 (brazo)
  + shield 4M&9S          │  CSI: cámara                                (TTL, 1 Mbps)
  ├─ 4 motores + encoder  │  I2C: MPU9250 (usar giroscopio, no magnetómetro)
  ├─ 4 servos dirección   │  GPIO: botón de inicio
  ├─ 2 servos compuertas  
  └─ 4 IR de borde
```

| Pieza | Responsable | Notas |
|---|---|---|
| Mega | reflejos | PI por rueda, dirección, odometría, primitivas, freno por borde, watchdog. |
| Pi 5 | decisiones | Localización con mapa conocido, visión, brazo, misión, reloj de 8 min. |

## Pines del Mega (`firmware/mega_cafebot/config.h`)

| Rueda | Ubicación (inferida) | Enc A | Enc B | PWM | DIR1 | DIR2 | Servo | Offset |
|---|---|---|---|---|---|---|---|---|
| M1 | delantera der | 18 | 31 | 12 | 34 | 35 | 13 | 87 |
| M2 | delantera izq | 19 | 38 | 8 | 37 | 36 | 10 | 87 |
| M3 | trasera der | 3 | 49 | 9 | 43 | 42 | 45 | 90 |
| M4 | trasera izq | 2 | A1 | 5 | A4 | A5 | 6 | 89 |

Pines provisorios, **por confirmar**: IR delanteros digitales 22/23, IR traseros
analógicos A8/A9, compuertas 44/46. Los pines 20/21 (SDA/SCL) también tienen
interrupción y están libres si no se usa I2C en el Mega.

## Medidas de la base
Ruedas Ø53 mm, trocha 217 mm, entre ejes 223 mm, 800 pulsos/vuelta de rueda en el flanco
de subida de A (1600 con `ENCODER_X2`). Reductora 1:8.

## Checklist de puesta en marcha (ruedas en el aire)
1. `E` y girar cada rueda a mano hacia adelante: las 4 cuentas deben **subir**. Si no, cambiar `Mx_INV_ENC`.
2. `V 100 0` (repetir): las 4 ruedas hacia adelante. Si alguna va al revés, `Mx_INV_MOTOR`.
3. `W 90`: las ruedas forman un rombo (tangentes a un círculo) y el robot giraría a la **izquierda**.
4. `L 90 200`: las 4 ruedas apuntan a la izquierda **iguales**. Si alguna apunta a la derecha, invertir su `SIGNO_SERVO`. Si alguna no alcanza 90°, ajustar `CANGREJO_MAX_DEG`.
5. En el piso: `A 1000` + huincha → diámetro de rueda. `W 360` → `CINEMATICA_D_GIRO_Z`.
6. `J`: si oscila, bajar `kp`; si no llega a la velocidad, subir `kff` o `PWM_ARRANQUE`.

## LiDAR
Para ver **cajas (90 mm) y albercas (90 mm)** tiene que ir a **menos de ~80 mm del suelo**.
Más arriba solo ve los árboles (y al público). Configurar su posición y orientación en
`ConfigLD06` (`pi/cafebot/drivers/ld06.py`).

## IMU
El magnetómetro no sirve cerca de motores y estructuras metálicas. Usar el giroscopio
(calibrar el sesgo con el robot quieto al inicio). Muchos "MPU9250" son clones MPU6500
sin magnetómetro.
