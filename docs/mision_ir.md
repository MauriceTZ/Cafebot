# Misión simplificada SIN LiDAR (rama `navegacion-ir`)

5 nov: el LiDAR y la cámara fallan por la alimentación. Se navega con los 4 IR (QTR-1A), la odometría del Mega y
la IMU, tomando como referencias las líneas negras que SE PUEDEN pisar. La cámara (solo inclinación, `X 2`; el giro
`X 3` está averiado y queda sin PWM) mira las albercas y el árbol. Los granos quedan en el robot (+3 maduro /
+1 sobremaduro): no se va a las canastas.

Código: `pi/cafebot/mision/mision_ir.py` (la ronda), `pi/cafebot/vision/vision_ir.py` (albercas y árbol con la
cámara), `pi/tools/mision_ir.py` (el programa y la simulación), parámetros en `config.mision_ir`.
Prueba sin robot: `python3 -m tests.test_mision_ir` (`--rapido` sin la ronda completa).

## Por qué no "bordear la cancha"
Los IR están junto a cada rueda (mismo punto del piso): para que un IR vea una línea, esa rueda tiene que estar
encima. Tocar con las ruedas la línea PERIMETRAL o la de una ALBERCA es reinicio forzado. Entonces el robot se guía
con las líneas interiores, que sí se pueden pisar: el **borde del beneficiadero** (x = 0,73 m) y el **borde del
cultivo** (x = 2,55 m). En cada una se cuadra con los dos IR delanteros (`Bordes.medir_adelante`: distancia ±2 mm,
ángulo ±0,5°). Entre una línea y otra manda la odometría, con el rumbo corregido por la IMU en cada tramo.

## Cómo poner el robot (lo importante)
1. Mirar dónde quedaron las albercas y elegir el árbol: el que tenga **libre la alberca de enfrente** (o un hueco a
   menos de ~40 cm de su centro) y que se pueda mirar desde el sector con **poco ángulo (≤ 18°)**. Desde los sectores
   1 y 5 eso es el árbol de su lado; desde el 3, el del medio; desde el 2 y el 4, el del medio o el del lado.
2. Poner el robot dentro de su sector, corrido de lado hacia ese árbol, con el **centro del robot apuntando al
   centro del pie del árbol**. Ojo: prolongando ese rumbo hasta el borde del beneficiadero, la rueda delantera del
   lado del árbol tiene que quedar a **≥ 3 cm de la división** del sector. Si la pisa, el IR toma la división por el
   borde y la misión calcula mal dónde está el árbol (en la simulación eso dejaba la ronda en 0).
3. El robot quieto mientras la IMU calcula su sesgo (2 s al arrancar el programa) y apretar el botón.

Lo que hace sola: avanza recto hasta el borde del beneficiadero; con el ángulo con que lo cruzó sabe a qué distancia
de lado queda el árbol (`y = (2,669 − x)·tan θ`); se endereza (si no sabe el signo de la IMU, gira ±6° para medirlo);
avanza a x = 0,85, inclina la cámara hacia abajo (−20°) y busca el azul en la franja de albercas; se corre de
cangrejo al hueco libre más cercano al árbol (con 8 cm de margen a cada lado; si no ve uno seguro, se corre 30 cm y
vuelve a mirar); cruza; se pone frente al árbol a 55 cm y saca la foto (la foto corrige la y del árbol: sin LiDAR
se sabe a ±10–15 cm: corrige hasta 18 cm); se acerca en tramos de 10 cm hasta la línea del cultivo, se cuadra y recoge. En cada estación se
vuelve a cuadrar en la línea.

### Si las albercas tapan el árbol: `--arbol`
Sin saber cuál árbol es, el hueco tiene que estar a ≤ 45 cm del árbol (más allá, si es el árbol de un costado,
podría acercarse al perímetro). Si el único paso está más lejos, decirle cuál es el árbol (1 = el de y chica,
el del lado del sector 1; 3 = el del sector 5) en `pi/system/mision.env`:
`MISION_ARGS="--brazo --imu --arbol 3"` y `sudo systemctl restart cafebot-mision`. Con eso busca el hueco por toda
la franja sin pasar a menos de 25 cm del perímetro. Volver a `--arbol 0` (o sacarlo) en la ronda siguiente.

## Qué recoge
Fila del medio y después la de abajo (la de arriba no). Primero los MADUROS de las dos filas y, si se acaban, los
SOBREMADUROS (`mision_ir.prioridad`). Solo los que la foto clasifica con confianza ≥ 0,55; ante la duda no se
tocan. La distancia al tablero se elige por fila (la baja pide alejarse: `mision.holguras_agarre_m`). Si un agarre
falla, se reintenta corrido ±1,2 cm de lado; el corrimiento que funcionó se mantiene para los siguientes. A los
12 s del final no empieza otro agarre.

## Comandos
En la Pi, con el .venv:

    python3 -m tools.mision_ir --brazo --imu --boton gpio          # la ronda de verdad (espera el botón)
    python3 -m tools.mision_ir --brazo --imu --boton enter          # igual, parte con Enter
    python3 -m tools.mision_ir --sim --semilla 7 --zona 2 --albercas 1,2   # simulación completa en el PC
    python3 -m tools.mision_ir --sim --brazo                        # simulación con el brazo REAL en el aire

El servicio (`pi/system/correr_mision.sh`, al encender) en esta rama corre `tools.mision_ir --boton gpio` con
`MISION_ARGS` de `pi/system/mision.env` (ejemplo: `mision.env.ejemplo`, `MISION_ARGS="--brazo --imu"`).
`MISION_TOOL=tools.mision` vuelve a la misión con LiDAR.

## Simulación (5 nov)
Firmware real del Mega + IR simulados + cámara dibujada (`vision/render.py`) + brazo simulado con la tolerancia de
la garra (±11 mm por eje). La "verdad" tiene 1 % de error de escala y la deriva de rumbo medida (~6 cm por metro);
la IMU simulada tiene su signo al azar. Ronda típica sin albercas en el camino: 4–5 granos, 10–11 puntos en ~100 s
(con agarres de 2 s; en el robot son más lentos), sin faltas. Errores del agarre: 2–12 mm en y.
Lote de 10 rondas (sectores 1–5, 0 a 3 albercas, partida como en "Cómo poner el robot"): todas entre 8 y 12 puntos,
ninguna falta; las 2 en que las albercas tapaban el árbol alcanzable se hicieron con `--arbol` (rodeo de 1,1 y
1,3 m).

## Por calibrar en la cancha (antes de la primera ronda)
- **IR** en la cancha real (`python3 -m tools.ir`, docs/ir.md): sin eso no ve las líneas.
- **Azul de las albercas** (`vision_ir.H_AZUL`, `S_MIN`, `V_MIN`): sacar una foto con `tools.ver_camara` mirando
  una alberca y revisar que el azul caiga en H 95..135 (OpenCV), S ≥ 90.
- **Distancia línea del cultivo → cara del árbol** (`cancha.ARBOL_X_FRENTE − CULTIVO_X0` = 11,9 cm según el
  reglamento) y la altura de los soportes: medir con huincha.
- **IMU**: que `tools.mision_ir` diga "IMU: MPU…" al arrancar; si no hay IMU sigue solo con odometría y líneas.
- La primera ronda de prueba con `--boton enter` y mirando el registro (`pi/registros/`).

## Límites conocidos
- La y del robot entre líneas es solo odometría: con 1 % de escala, un cangrejo de 50 cm agrega 5 mm de error.
- Si la foto del árbol falla dos veces, o ve el árbol corrido más de 18 cm, no recoge nada: con la y errada una
  columna (12 cm) el brazo agarraría el grano del lado (en la simulación eso llegó a meter un verde, −5).
- La puntería se lee en el borde del beneficiadero a ±2,5° (a 15°): la foto lo arregla, pero apuntar con poco ángulo
  ayuda.
- La detección del azul agranda un poco las albercas (la pared que mira a la cámara, hasta ~4 cm): es el lado
  seguro, pero puede hacer que un hueco justo no se use.
