# Misión de una ronda (N1 → N2 → N3)

Código: `pi/cafebot/mision/` (`mision.py` = máquina de estados, `interfaces.py` = contratos con brazo,
visión, compuertas y botón). Parámetros: `ConfigMision` en `pi/cafebot/config.py` (se sobreescriben en
`pi/robot_local.json`, sección `"mision"`).

## Cómo correrla

```bash
cd ~/Cafebot/pi && source .venv/bin/activate
python3 -m tools.mision --sim                       # ronda simulada completa (cancha y granos sorteados)
python3 -m tools.mision --sim --nivel 3 --semilla 7
python3 -m tools.navegacion --sim                   # con interfaz: tecla K = botón de inicio
python3 -m tests.test_mision 1 2 3                  # prueba automática (~2 min por cancha)

# En el robot
python3 -m tools.mision                             # solo navegación: va, mira y se acerca; no toca granos (botón = ENTER)
python3 -m tools.mision --brazo --vision camara     # COMPLETA: brazo XL430 + cámara (calibrarlos antes: docs/brazo.md, docs/camara.md)
python3 -m tools.mision --brazo --vision fija       # SOLO TALLER: árbol con puros maduros, para probar el brazo sin visión
python3 -m tools.mision --boton gpio ...            # botón real (GPIO 16, mision.pin_boton)
python3 -m tools.navegacion --mega /dev/ttyUSB1 --lidar /dev/ttyUSB0 --brazo --camara    # con interfaz: tecla K
```
Al terminar (entregó lo que alcanzó a recoger, o se acabó el tiempo) el programa se cierra solo: el brazo queda en su
postura de descanso y las compuertas cerradas, con torque, mientras los jueces levantan el robot.

## Prueba híbrida: ronda simulada con el BRAZO REAL (4 nov)
Todo simulado (cancha, Mega, LiDAR, visión) menos el brazo: el de verdad hace cada agarre completo (acercarse, cerrar
la garra, ir a la canasta, soltar) en los mismos puntos que tendría frente al árbol, y la simulación decide si el
grano salió del árbol y lleva el puntaje (`interfaces.BrazoHibrido`). La garra cierra en el aire: no se exige grano.
```bash
python3 -m tools.mision --sim --brazo                     # en la terminal
python3 -m tools.navegacion --sim --brazo --mision        # con el mapa y la vista de la cámara simulada
```
- El robot queda QUIETO (el Mega es simulado: las ruedas no se mueven). Dejar espacio libre adelante (~40 cm), arriba
  (~75 cm), a los lados y ATRÁS: la base gira hasta ~180° por la izquierda para ir a las canastas.
- NO poner el árbol de prueba delante: entre estaciones el robot simulado se corre de lado y el de verdad no, así
  que el brazo apuntaría a otro soporte. Para agarrar pelotas de verdad: `tools.cosecha` (docs/cosecha.md).
- Sirve para ver los movimientos reales en una ronda, el tiempo de verdad de cada agarre dentro de los 8 min, que
  estén las posturas de las canastas y la salud de los servos (°C, V). Lo que el brazo no alcanza lo dice antes de
  moverse ("fuera de alcance").
- ESC (interfaz) o Ctrl+C: la misión se detiene y el brazo termina el movimiento en curso y no empieza otro
  (`Mision.detener` → `brazo.detener`). Para una emergencia, cortar la alimentación del brazo.
- En la Pi, la simulación compila el firmware del Mega: hace falta `g++` (`sudo apt install g++`).

## Botón de inicio y servicio (para la competencia)
Botón entre el GPIO 16 (pin 36) y GND (pin 34), sin resistencia (pull-up interno): `docs/hardware.md`. Probarlo con
`python3 -m tools.boton`. Para que la misión corra sin el PC (la ronda va sin WiFi), instalar el servicio una vez:
```bash
bash ~/Cafebot/pi/system/instalar_servicio.sh     # crea pi/system/mision.env con las opciones de la misión
sudo systemctl start cafebot-mision               # una ronda: queda esperando el botón
journalctl -u cafebot-mision -f                   # ver lo que hace
sudo systemctl enable cafebot-mision              # SOLO en la competencia: arranca solo al encender la Pi
sudo systemctl disable cafebot-mision             # de vuelta al taller (si corre, ocupa el Mega, el LiDAR y el U2D2)
```
Opciones (`--nivel`, `--cajas`, `--brazo`, `--vision`): en `pi/system/mision.env`, entre rondas. El servicio no se
reinicia solo: después de una ronda, `sudo systemctl start cafebot-mision` (o apagar y encender, con `enable`).

## Compuertas (Dynamixel id 2 = maduros, id 13 = sobremaduros; `config.compuertas.ids`)
En el mismo bus que el brazo (U2D2). Quedan en modo posición, con el PWM limitado (si algo las traba no se queman) y
con sus límites grabados en el servo: aunque se pida un ángulo equivocado, no pasan de ahí. **Sin calibrar no se
mueven** (sus granos quedan en el robot: +3 / +1).

**Calibración guiada** (ángulos del servo, 0..360°), con el brazo quieto y el U2D2 conectado:
```bash
python3 -m tools.compuertas leer                       # ¿responden las dos (id 2 y 13)? modo, límites, °C
python3 -m tools.compuertas calibrar                   # las dos (o: calibrar maduros / calibrar sobremaduros)
```
1. Deja la compuerta SIN torque. Se lleva **a mano** a CERRADA (tapando la salida, sin apretarla contra un tope) mirando
   el ángulo en vivo, ENTER; después a ABIERTA (del todo: que los granos salgan solos), ENTER.
2. **Sentido**: no hay que configurarlo. Se graban los ángulos reales y el servo va de uno al otro (si "abierta" es
   menor que "cerrada", gira para el otro lado y listo).
3. Si entre cerrada y abierta se pasa por **0°/360°**, el servo (modo posición) no cruza el 0: daría la vuelta larga y
   rompería la compuerta. La herramienta no la guarda y pide **montar el cuerno girado** (soltar su tornillo y
   moverlo unos dientes) para que la compuerta trabaje cerca de 180°; después, calibrar otra vez. Correr el cero por
   software (Homing Offset) no sirve: la lectura sigue saltando en el mismo punto del eje.
4. **Límites**: `--margen` grados (5 por defecto) más allá de cerrada y de abierta, grabados en la EEPROM del servo.
5. **Prueba** con torque, despacio: cierra, abre, espera `mision.t_vaciado_s` y cierra; dice dónde quedó y cuánta
   carga hace sujetándola cerrada. Más de ~35 %: está empujando contra algo y se calienta: correr "cerrada" 2–3°
   hacia abierta.
6. Al final imprime las líneas para `ConfigCompuertas` en `pi/cafebot/config.py` (lo grabado va a
   `pi/robot_local.json`, que se pierde con la SD).

A mano, lo mismo por partes: `soltar`, `leer --seguir`, `grabar maduros cerrada|abierta`, `limites maduros MIN MAX`,
`mover maduros GRADOS`, `probar`.
Se graba en `pi/robot_local.json` (sección `compuertas`): copiarlo también a `ConfigCompuertas` en
`pi/cafebot/config.py`, que robot_local.json se pierde con la SD. `compuertas.tipo = "mega"` vuelve a las de X 0 / X 1.

## Qué hace

| Paso | Cómo | Nivel |
|---|---|---|
| Botón | `Boton.esperar()` (GPIO, ENTER o inmediato). Parte al **soltar**. | todos |
| Ubicarse | `estimar_inicio` con 3 barridos (sector, x, y, θ, albercas, sectores con caja). El mapa del SLAM tiene origen en la partida, así que con eso se pasa del mapa a la cancha. | N1 |
| Cajas | Sectores con caja: LiDAR al partir **y** el mapa después (desde la partida una caja puede tapar a la otra). Color: la cámara mira adelante, así que antes de entregar se para frente a una caja (`x_vista_cajas`, mirando al beneficiadero) y `Vision.color_caja` dice ROJA/AZUL; si hay dos cajas, la otra es del otro color. | N2 |
| Ir al árbol | El más cercano en y (si la partida es dudosa, el que el LiDAR ve; ver abajo). A* hasta quedar con el frente a `distancia_foto_m` (50 cm) del tablero, mirando +x y con la **cámara** centrada en el árbol. Si el destino estaba tapado por una alberca, al llegar se replanifica. | N1 |
| Foto | `X 2` con el ángulo que da `camara.inclinacion_arbol_deg` (`camara.servo_de_pitch`) → el LiDAR mide el tablero → `Vision.clasificar_arbol` (16 lecturas; dónde cae cada soporte en la imagen sale de esa medida). Soportes FUERA de la foto: cangrejo 10 cm hacia ese lado y otra foto. Solo se recogen las categorías de `recoger` con confianza ≥ `confianza_min`. **NO_SE = no tocar.** | N1 |
| Acercarse | Hasta tocar la **línea del cultivo** con DI y DD (ver abajo) y enderezarse con ella; después `A` recto hasta `holgura_agarre_m`. Sin línea: midiendo la cara del tablero con el LiDAR. | N1 |
| Recoger | 5 estaciones laterales (`estaciones_y`, con `L` cangrejo; después de cada cangrejo vuelve a tocar la línea); en cada una, los granos más cercanos, fila alta → baja. La posición de cada grano: distancia y ángulo del tablero **de la línea** + la odometría desde que la tocó, y la posición lateral **del pie medido por el LiDAR**; no desde la pose global. 2 intentos por grano. | N1 |
| Entregar | A* hasta `x_aproximacion_entrega` frente a la caja, mirando +x; **reversa** hasta `x_entrega`; `X 0` abre la compuerta de maduros `t_vaciado_s`, la cierra y sale. N3 hace lo mismo con la azul (`X 1`). | N2 |
| Reloj | Antes de cada árbol y de cada estación se reserva el tiempo para volver y vaciar. A los 468 s se detiene todo. Si algo falla, igual intenta entregar lo que lleva. | todos |

## Línea del cultivo con los sensores delanteros (4 nov)
La línea entre la zona intermedia y el cultivo está entre el robot y los árboles, paralela a los tableros: su borde
cercano (`CULTIVO_X0`) queda a `ARBOL_X_FRENTE − CULTIVO_X0` ≈ 11,9 cm de la cara del tablero (el reglamento pone el
tablero a 10 cm de la línea; **medirlo en la cancha** y ajustar `cancha.ARBOL_X_FRENTE`). Se puede pisar (las que no:
el perímetro y las de las albercas).
- Al acercarse al árbol (`_cuadrar_en_linea`), el robot avanza hasta que DI o DD toca la línea y sigue despacio hasta
  que la toca el otro (`Bordes.medir_adelante`). Con los dos puntos de cruce (la pose del Mega en cada evento) salen la
  distancia del robot a la línea (±2 mm) y cuánto está girado (±0,5°, `tests.test_linea_arbol`); gira eso y queda
  paralelo al tablero, con el frente a ~8 cm. Lo repite después de cada cangrejo (`linea_por_estacion`).
- Desde ahí, la distancia y el ángulo del tablero salen de la línea + la odometría (pocos cm: precisa); el LiDAR, que
  de tan cerca es menos preciso, solo da la posición lateral del pie. La pose de la cancha NO se corrige con la línea:
  un ángulo medido en 22 cm, llevado a la entrega (2–3 m más allá), correría la caja; ahí manda el SLAM.
- Si no ve la línea (IR sin calibrar en la cancha: `tools.ir`) o lo que mide no calza (más de
  `linea_max_correccion_m` o 10°), sigue como antes, con el LiDAR. Sin ver la línea no se acerca a menos de 4,5 cm del
  tablero.
- **Diagnóstico**: cada toque deja en el registro el error que tenía la pose fusionada ("la pose fusionada tenía +3 cm
  en x y +1,2°"). Si en el robot sale siempre parecido, es el montaje del LiDAR: un error fijo en x apunta a
  `geometria.lidar_x` y uno en el ángulo, a su yaw.
- Simulado con el LiDAR mal montado (1,5 cm y 1,5°): el error de cada agarre bajó de ~1,6 cm a ~0,8 cm (queda el
  error lateral del LiDAR); cuesta ~3 s por estación (semilla 1: 124 s → 137 s). `linea_cultivo = False` lo apaga.

## Si las dos cajas son del mismo color (`mision.cajas`)
4 nov: el reglamento dice una caja ROJA (maduros) y una AZUL (sobremaduros), pero las canastas que vimos en la
competencia son las dos rojas. **Primero: preguntar a los jueces cómo se distingue la de sobremaduros** (si es una
marca, un papel o una cinta, se le enseña a la cámara y deja de ser una apuesta).

| `mision.cajas` | Qué hace | Puntaje |
|---|---|---|
| `colores` (por defecto) | La cámara mira la caja más cercana y la otra es "del otro color". Si las dos son rojas, esto ya es una apuesta a la más cercana, pero perdiendo tiempo en mirar. | Lo del reglamento |
| `cercana` | Sin mirar: maduros a la caja más cercana al volver, sobremaduros a la otra (`_asignar_por_cercania`). Al azar sería la misma probabilidad (50 %), pero con un viaje más largo. Con cajas en más de 2 sectores, no apuesta. | ±7 por maduro, ±5 por sobremaduro |
| `guardar` | No entrega: los granos quedan en el robot. No reserva tiempo para el viaje (conviene con `--nivel 3`: más árboles). | +3 por maduro, +1 por sobremaduro, seguros |

Se cambia entre rondas en `robot_local.json` (`"mision": {"cajas": "cercana"}`) o con `--cajas` en `tools.mision` /
`tools.navegacion` (en `--sim`, las dos cajas se dibujan rojas en la vista de la cámara).

Cuál conviene (`tests.test_cajas` lo calcula): en UNA ronda, apostar promedia 0 y guardar da +3 por maduro, así que
guardar es mejor. Pero la clasificatoria suma las **2 mejores de 4** rondas y la final toma la **mejor de 3**: las
rondas malas se descartan y apostar sale mejor en promedio. Con 3 maduros por ronda: clasificatoria guardar 18 /
apostar 26 en promedio (42 con 69 % de probabilidad, 0 con 25 %, −42 con 6 %); final guardar 9 / apostar 16 (21 con
88 %, −21 con 12 %). Además, después de cada ronda se sabe si la apuesta salió bien: si ya hay 2 rondas buenas, las
siguientes pueden ir con `guardar`; en la última clasificatoria, si falta una buena, guardar asegura sumar.

## Resultados en simulación

(firmware real simulado, LiDAR con 3 % / 2 % de error de calibración, visión con 10 % NO_SE, brazo con 10 % de fallas
y 1 s por agarre; con el brazo real cada agarre tardará más)

| Cancha (semilla) | Partida | Albercas | Resultado | Puntaje | Tiempo |
|---|---|---|---|---|---|
| 1 | sector 5 | 5 | 5 maduros en la roja | 35 | 104 s |
| 2 | sector 3 | 3, 5 | 6 maduros en la roja | 42 | 126 s |
| 5 | sector 5 | 3, 4 | partida dudosa → reubicada en el árbol; 6 maduros en la roja | 42 | 99 s |
| 7 | sector 3 | 4, 5 | 6 maduros en la roja | 42 | 87 s |
| 5, **nivel 3** | sector 5 | 3, 4 | 3 árboles: 16 maduros en la roja, 12 sobremaduros en la azul | 172 | 237 s |

Con 1 s por agarre. Con el brazo real (~10 s por grano) un árbol toma ~1,5 min más: N2 con un árbol sigue
sobrando tiempo; para N3 el reloj decide cuántos árboles.

**Partida dudosa.** Si las albercas tapan dos árboles, el LiDAR (a 8 cm) ve un solo árbol y no sabe cuál es
(confianza < 0,15). La misión va igual hacia ese árbol (su posición en el mapa es la misma bajo cualquier
hipótesis) y, ya pasadas las albercas, repite la localización con **todo el mapa**: allí se ven los tres
árboles. En 8 de 8 canchas difíciles probadas aparte, la reubicación acertó el sector.

## Para conectar los módulos reales

- **Brazo**: una clase que herede de `interfaces.Brazo` e implemente `recoger(p_robot, categoria)`
  (sacar UN grano en `p_robot` = (x, y, z) del robot desde el suelo y dejarlo en el depósito de su categoría),
  `alcanzable`, `preparar`, `descansar`. Usa `cafebot/arm/cinematica.py` y `dxl.py`.
- **Visión**: heredar de `interfaces.Vision`: `clasificar_arbol(arbol, pose)` devuelve 16 `LecturaSoporte`.
  Con la pose y `cancha.soportes(arbol)` se sabe dónde cae cada soporte en la imagen (portar `legacy/vision_pablo`).
- Luego se agregan en `tools/mision.py` (modo real) en lugar de `BrazoNulo` / `VisionNula`.

## Por medir / confirmar

| Qué | Dónde |
|---|---|
| Posición de la base del brazo en el robot | `mision.brazo_x/y/z` |
| Centro del grano delante del tablero; desplazamiento lateral de la copa respecto del pie | `cancha.GRANO_DELANTE_M`, `cancha.COPA_DESPLAZADA_M` |
| Holgura de agarre que permite llegar a la fila alta (el brazo llega poco: `docs/brazo.md`) | `mision.holgura_agarre_m` |
| Distancia e inclinación de la cámara para ver los 16 soportes | `mision.distancia_foto_m`, `camara.inclinacion_arbol_deg` (+ `pitch_servo_90_deg` / `pitch_por_grado`, medidos con `tools.cosecha` C) |
| Ángulos y límites de las compuertas (Dynamixel 12 / 13); dónde caen los granos, a qué x retroceder | `tools.compuertas` (`compuertas.*`), `salida_x`, `x_entrega` |
| Botón: GPIO 16, pin 36 (5 nov); probar el cableado | `python3 -m tools.boton` |
| ¿Hay líneas negras en las posiciones de alberca vacías? Si las hay, el A* debe evitarlas (hoy solo evita las albercas que ve). | reglamento Apéndice A |
| IR: freno por borde desactivado en la misión hasta integrarlos (`freno_ir`). | |
