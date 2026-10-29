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
| `L a d [v]` | **Cangrejo**: las 4 ruedas a `a` grados (±90) y avanza `d` mm en esa dirección sin girar. `L 90 300` = 30 cm a la izquierda. A ±90° las ruedas se reparten dos a +90 y dos a −90 (con el motor al revés) para que el arrastre de las L no gire la base; entre `L 90` y `L -90` las ruedas no se mueven. |
| `LE [m]` | Patrón del cangrejo lateral: 0 = las 4 igual (antes), 1 = diagonales (por defecto), 2 = izquierdas/derechas, 3 = delanteras/traseras. Sin argumento lo muestra. No se guarda: el elegido se deja en `CANGREJO_ESPEJO_DEF` (`config.h`, PASO 1b). |
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
| `EM 1\|2` | Decodificación x1 (solo flanco de subida, como v0) o x2 (ambos flancos). Solo detenido; se guarda en EEPROM. Por defecto x1. |
| `R [x y th]` | Fija la pose (la Pi corrige la odometría con LiDAR/líneas). Sin argumentos = 0 0 0. |
| `Z` | Reinicia encoders (solo detenido). |
| `M hz` | Telemetría periódica (1–50 Hz, 0 = apagada). |
| `I` | Sensores de línea (4 × QTR-1A, ver `docs/ir.md`): `IR mask=5 v=130/860/140/910 u=500/500/500/500 h=60 b=150/... n=850/... seg=0/15 auto=0` (lecturas, umbrales, histéresis, blanco, negro, freno/máscara, auto-calibración). Sensores en orden DI, DD, TI, TD. |
| `IM hz` | Lecturas crudas `IRV v0 v1 v2 v3 mask` a hz (1–100; 0 = apagado). |
| `IW` | Calibra el BLANCO de los 4 (robot sobre la lona). Recalcula los umbrales (punto medio). |
| `IN [k]` | Calibra el NEGRO del sensor k (0..3) o de los 4, sobre la línea. Sin contraste suficiente no lo guarda y avisa `EVT IRCAL contraste_bajo k lectura blanco`. |
| `IA 1` / `IA 0` | Auto-calibración: registra mín./máx. mientras se pasa sobre una línea; al terminar `OK IA calibrados=mask min=.. max=..` (blanco = mín, negro = máx, umbral = punto medio). |
| `IU [k v]` | Fija a mano el umbral del sensor k (0..1023). Sin argumentos muestra el estado. |
| `IH v` | Histéresis común (cuentas). |
| `IG` / `ID` | Guarda la calibración IR en EEPROM / vuelve a los valores de fábrica de `config.h` (sin guardar). |
| `IS 0\|1 [mascara]` | Freno de borde: con 1, al detectar una línea NUEVA mientras se mueve → `ABORT ... BORDE`. `mascara` elige los sensores (15 = todos, 3 = solo delanteros). |
| `X k ang` | Servo auxiliar k: 0 = compuerta maduros, 1 = compuerta sobremaduros, 2 = inclinación de la cámara. |
| `J kp ki kff` | Ganancias del PI de rueda (sin argumentos: muestra las actuales). |
| `H` / `?` | Ayuda. |

## Mensajes espontáneos

```
T ms x y th_mrad vx_mm_s wz_mrad_s modo postura cola ir_mask     (telemetría, con M)
EVT IR mask subida bajada x y th_mrad   (cambió algún sensor de línea; subida = entraron a negro, bajada = salieron.
                                         La pose es la del MOMENTO del cruce: ya descuenta el retraso del filtro)
IRV v0 v1 v2 v3 mask         (lecturas crudas de los IR, con IM)
EVT STOP motivo x y th_mrad  (se detuvo sin meta activa: WATCHDOG, BORDE)
```

- `modo`: 0 quieto, 1 cola, 2 velocidad, 3 cangrejo continuo.
- `postura`: 0 recto, 1 giro en el sitio, 2 cangrejo.
- `ir_mask`: bit0 delantero izq, bit1 delantero der, bit2 trasero izq, bit3 trasero der (1 = negro).
