# Protocolo serial Raspberry ↔ Arduino Mega

115200 baudios, texto ASCII, una orden por línea terminada en `\n`. Se puede usar
igual desde el Monitor Serie del Arduino IDE (fin de línea: "Nueva línea") que desde
`pi/cafebot/drivers/mega.py`.

Unidades: **mm, grados, mm/s**. Sistema del robot: x adelante, y izquierda, ángulo
antihorario (+ = izquierda). La pose interna del Mega se reporta con el ángulo en
**milirradianes** (`th_mrad`) para no usar decimales.

Al abrir el puerto USB el Mega se reinicia (≈2 s) y responde `READY cafebot-mega v1.0`.

## Movimientos encolados (responden `OK #id`, luego `DONE #id` o `ABORT #id`)

| Orden | Qué hace |
|---|---|
| `A d [v]` | Avanza `d` mm en línea recta (negativo = retrocede), manteniendo el rumbo. |
| `W a [wmax]` | Gira en el sitio `a` grados (ruedas a ±45,8°). `wmax` en grados/s. |
| `L a d [v]` | **Cangrejo**: las 4 ruedas a `a` grados (±90) y avanza `d` mm en esa dirección sin girar. `L 90 300` = 30 cm a la izquierda. |
| `G x y [th]` | Va al punto absoluto (x, y) y, si se da, termina mirando a `th` grados. |
| `T` | Ruta cuadrada de prueba de 500 mm. |

Se pueden encolar hasta 24. Al terminar cada una:

```
DONE #12 495 3 1571            -> x=495 mm, y=3 mm, th=1,571 rad
ABORT #12 BORDE 300 2 0        -> motivos: STOP, BORDE, REEMPLAZADO, WATCHDOG
```

## Velocidad directa (sin respuesta, hay que repetir cada < 500 ms)

| Orden | Qué hace |
|---|---|
| `V vx wz` | vx en mm/s y wz en **rad/s**. Si vx≈0 y wz≠0, pone las ruedas en postura de giro. |
| `K a v` | Cangrejo continuo a `a` grados con velocidad `v`. |

Si no llega otra `V`/`K` en 500 ms, el Mega se detiene y emite `EVT STOP WATCHDOG ...`.

## Estado y ajustes

| Orden | Respuesta |
|---|---|
| `S` (o `C`) | Para, vacía la cola. `OK S` |
| `P` | `POSE x y th_mrad modo=.. postura=.. cola=..` |
| `E` | `ENC m1 m2 m3 m4` (cuentas acumuladas) |
| `ED` | `ENCD subida/bajada: M1=a/b ...` lo que sumó cada flanco de A desde el último `ED` (y lo pone en cero). |
| `EK [i f]` | Escala de la rueda i (1 = M1 … 4 = M4), multiplica el diámetro. Sin argumentos la muestra; `EK 0` vuelve a `ESCALA_RUEDA_DEF` de `config.h`. Se guarda en EEPROM. |
| `EM 1\|2` | Decodificación x1 (solo flanco de subida, como v0) o x2 (ambos flancos). Solo detenido; se guarda en EEPROM. Por defecto x1. |
| `R [x y th]` | Fija la pose (la Pi corrige la odometría con LiDAR/líneas). Sin argumentos = 0 0 0. |
| `Z` | Reinicia encoders (solo detenido). |
| `M hz` | Telemetría periódica (1–50 Hz, 0 = apagada). |
| `I` | `IR mask=.. TI=.. TD=.. blanco=../.. negro=../.. seguridad=..` |
| `IW` | Calibra BLANCO de los IR traseros (robot sobre la lona). |
| `IN k` | Calibra NEGRO del sensor trasero k (0 = izq, 1 = der) puesto sobre la línea. |
| `IG` | Guarda la calibración IR en EEPROM. |
| `IS 0/1` | Seguridad de borde: con 1, al detectar una línea nueva mientras se mueve → `ABORT ... BORDE`. |
| `X k ang` | Servo auxiliar k: 0 = compuerta maduros, 1 = compuerta sobremaduros, 2 = inclinación de la cámara. |
| `J kp ki kff` | Ganancias del PI de rueda (sin argumentos: muestra las actuales). |
| `H` / `?` | Ayuda. |

## Mensajes espontáneos

```
T ms x y th_mrad vx_mm_s wz_mrad_s modo postura cola ir_mask     (telemetría, con M)
EVT IR mask x y th_mrad      (cambió algún sensor IR: sirve para corregir la pose al cruzar líneas)
EVT STOP motivo x y th_mrad  (se detuvo sin meta activa: WATCHDOG, BORDE)
```

- `modo`: 0 quieto, 1 cola, 2 velocidad, 3 cangrejo continuo.
- `postura`: 0 recto, 1 giro en el sitio, 2 cangrejo.
- `ir_mask`: bit0 delantero izq, bit1 delantero der, bit2 trasero izq, bit3 trasero der (1 = negro).
