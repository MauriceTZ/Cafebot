# Brazo: alcance y torque del hombro

Datos: tramos 107 + 107 + 67 mm + muñeca→garra 32 mm (`legacy/codigo_brazo/main.py`),
hombro 67 mm sobre la base del brazo, base del brazo a 36 cm del suelo, servos
XL430‑W250 (torque de bloqueo ≈1,4 N·m a 11,1 V, 57 g cada uno).

## Alturas de los granos (Apéndice B)

| Fila | Cara superior del soporte | Centro del grano* |
|---|---|---|
| Baja | 390 mm | ≈407 mm |
| Media | 540 mm | ≈557 mm |
| Alta | 690 mm | ≈707 mm |

\* La esfera de 42 mm apoyada en un hueco de 25 mm queda con su centro ~17 mm sobre el soporte.

El máximo medido (71 cm con la garra horizontal y el resto vertical) coincide con el
centro de la fila alta: **margen cero**, y en esa postura el brazo está estirado al
máximo, sin poder corregir nada.

## Torque estimado en el hombro (`docs/scripts/torque_hombro.py`)

| Postura | Torque | % del bloqueo |
|---|---|---|
| Brazo estirado horizontal (la prueba que falló) | ≈0,86 N·m | ≈60 % |
| Grano fila baja, 12–16 cm delante de la base | ≈0,25–0,28 N·m | ≈20 % |
| Grano fila media | ≈0,03–0,15 N·m | <11 % |
| Grano fila alta | ≈0,01–0,13 N·m | <10 % |

La falla al minuto en horizontal es lo esperable (sobrecarga/temperatura a ~60 % del
bloqueo sostenido). **Pero la tarea nunca necesita esa postura**: para recoger, el brazo
trabaja casi vertical y cerca de la base. Alargar los tramos sí empeoraría todo: el
torque crece aproximadamente con el cuadrado del largo.

## Recomendaciones (de más barata a más cara)

1. **No alargar el brazo.**
2. **Inclinar la garra hacia arriba (20–30°) en la fila alta**: el punto de agarre sube
   ~(L4 + dedos)·sen(ángulo) ≈ 3–4 cm sin tocar el hardware, y además saca el grano del
   hueco hacia arriba. Cuidado con que el dedo inferior no roce el soporte.
3. **Montar la base del brazo en el borde delantero** del robot: el árbol es un tablero
   plano, así que el robot puede quedar a pocos cm y el brazo trabaja casi vertical.
4. Si el cubo de 45 cm lo permite con el brazo plegado, un **suplemento de 3–5 cm** bajo la
   base del brazo (no agrega torque).
5. Postura de **descanso vertical** entre agarres. Nunca dejar el brazo extendido esperando
   a la cámara o a la base.
6. Resorte o elástico de compensación en el hombro (barato, ayuda mucho si el brazo
   pasa tiempo inclinado).
7. Diagnóstico: leer `Hardware Error Status` (dir. 70): bit 5 = sobrecarga, bit 2 =
   sobrecalentamiento, bit 0 = voltaje. Leer `Present Temperature` (146) y
   `Present Current` (126). Alimentar a 12 V (con 11,1 V nominal o menos baja el torque).
8. En `main.py` el hombro usa I = 150 interno **más** un integrador externo (`K_I_EXT`).
   Si la postura pedida no se alcanza (o el brazo empuja un tope), ambos integradores
   llevan el servo a corriente máxima de forma sostenida. Para competir: I interno en 0
   o bajo, y el integrador externo limitado. Así el error queda en unos pocos grados,
   pero el servo no se recalienta.
