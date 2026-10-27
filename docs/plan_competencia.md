# Plan para LARC OPEN 2026 (Bogotá, 2–6 nov)

Escrito el 27 oct. Quedan ~4 días de taller antes de viajar. **Se puede reprogramar
entre rondas** (reglamento §7), así que el robot puede seguir mejorando durante la
competencia: clasifican las **2 mejores de 4 rondas** y para la final solo hace falta
haber sumado puntos positivos en alguna.

## Regla de oro

Un grano maduro guardado dentro del robot ya vale **+3**, aunque nunca llegue a la caja.
Un error vale **−7**. Ante la duda (color "no sé", brazo inseguro), **no tocar**.

## Lo que NO entra en esta competencia

SLAM, YOLO, el segundo brazo, el gemelo digital en tiempo real (MuJoCo queda para
desarrollo) y la interfaz con clics de `legacy/integracion_juan_v1` (un operador no
puede elegir el árbol: el robot debe ser autónomo con un solo botón).

## Niveles (cada uno debe funcionar antes de empezar el siguiente)

### N0 — Base y brazo confiables (mar–jue)
- [ ] Subir `firmware/mega_cafebot`. Con ruedas en el aire: `A 300`, `W 90`, `L 90 200`. Verificar signos (motores, encoders, servos del cangrejo). Ver checklist en `docs/hardware.md`.
- [ ] En el piso: `A 1000` y medir con huincha → corregir `CINEMATICA_DIAMETRO_RUEDA`. `W 360` → corregir `CINEMATICA_D_GIRO_Z`/`W`.
- [ ] IR: `IW` sobre la lona, `IN 0`/`IN 1` sobre la línea, `IG`. Probar `IS 1` + `A 2000` hacia una línea.
- [ ] Brazo: rutina de agarre **con coordenadas fijas** (sin cámara) para un soporte en cada fila. Soltar en el depósito.
- [ ] Brazo: leer `Hardware Error Status` y temperatura (ver `docs/brazo.md`).

### N1 — Un árbol, granos dentro del robot (jue–vie)
1. Al apretar el botón: escaneo LiDAR. El capitán deja el robot **mirando a los árboles** (la orientación es libre), así que θ≈0. Los 3 árboles están en posiciones fijas: su desplazamiento lateral dice en qué sector partimos.
2. Detectar qué posiciones de alberca están ocupadas (LiDAR bajo o cámara).
3. Ruta con primitivas: `A` hasta antes de las albercas → `L` lateral hasta un hueco libre → `A` cruzando → `L` hasta alinear con el árbol → acercamiento final midiendo la distancia al tablero con el LiDAR.
4. Frente al árbol: las 16 posiciones de soporte son conocidas (Apéndice B). Para cada columna: `L` lateral para alinear el brazo, cámara clasifica cada soporte, el brazo toma solo MADURO.
5. Al acabar el tiempo o el árbol: quedarse quieto. Puntaje esperado: 3–6 maduros × 3 = 9–18.

### N2 — Entregar a la caja roja (durante la competencia)
Volver por el mismo camino, ubicar la caja roja (LiDAR para la posición, cámara para el color), acercarse de reversa hasta el borde y abrir la compuerta (`X 0 ...`). Cada maduro pasa de +3 a +7.

### N3 — Sobremaduros, más árboles (solo si sobra tiempo)

## Reparto sugerido (4 personas)
| Quién | Qué |
|---|---|
| Base | Firmware en el robot, calibración, primitivas en el piso real. |
| Brazo | Agarre con coordenadas fijas, resorte de compensación, lectura de errores Dynamixel. |
| Visión | Clasificación por soporte con posiciones conocidas (`legacy/vision_pablo/clasificador.py`), botón de inicio, arranque automático. |
| Mecánica | LiDAR bajo, brazo al borde delantero, depósitos con compuerta, cubo de 45 cm. |

## Antes de viajar
- [ ] Botón de inicio en un GPIO de la Pi + servicio systemd que arranca la misión.
- [ ] `pi/system/99-cafebot.rules` instalado (nombres fijos de puertos).
- [ ] WiFi apagado en ronda (`sudo nmcli radio wifi off`), logs a archivo.
- [ ] Fuente 5 V / 5 A para la Pi 5 (no el cargador de 20 W), baterías cargadas y de repuesto.
- [ ] El robot plegado cabe en el cubo de 45 cm **con cables**.
- [ ] microSD de respaldo con todo instalado.
