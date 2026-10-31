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
python3 -m tools.mision --boton gpio ...            # botón real en cfg.mision.pin_boton
python3 -m tools.navegacion --mega /dev/ttyUSB1 --lidar /dev/ttyUSB0 --brazo --camara    # con interfaz: tecla K
```

## Qué hace

| Paso | Cómo | Nivel |
|---|---|---|
| Botón | `Boton.esperar()` (GPIO, ENTER o inmediato). Parte al **soltar**. | todos |
| Ubicarse | `estimar_inicio` con 3 barridos (sector, x, y, θ, albercas, sectores con caja). El mapa del SLAM tiene origen en la partida, así que con eso se pasa del mapa a la cancha. | N1 |
| Cajas | Sectores con caja: LiDAR al partir **y** el mapa después (desde la partida una caja puede tapar a la otra). Color: la cámara mira adelante, así que antes de entregar se para frente a una caja (`x_vista_cajas`, mirando al beneficiadero) y `Vision.color_caja` dice ROJA/AZUL; si hay dos cajas, la otra es del otro color. | N2 |
| Ir al árbol | El más cercano en y (si la partida es dudosa, el que el LiDAR ve; ver abajo). A* hasta quedar con el frente a `distancia_foto_m` (50 cm) del tablero, mirando +x y con la **cámara** centrada en el árbol. Si el destino estaba tapado por una alberca, al llegar se replanifica. | N1 |
| Foto | `X 2` con el ángulo que da `camara.inclinacion_arbol_deg` (`camara.servo_de_pitch`) → el LiDAR mide el tablero → `Vision.clasificar_arbol` (16 lecturas; dónde cae cada soporte en la imagen sale de esa medida). Soportes FUERA de la foto: cangrejo 10 cm hacia ese lado y otra foto. Solo se recogen las categorías de `recoger` con confianza ≥ `confianza_min`. **NO_SE = no tocar.** | N1 |
| Acercarse | `A` recto, midiendo la cara del tablero con el LiDAR, hasta `holgura_agarre_m`. | N1 |
| Recoger | 5 estaciones laterales (`estaciones_y`, con `L` cangrejo); en cada una, los granos más cercanos, fila alta → baja. La posición de cada grano se calcula **desde el tablero medido por el LiDAR** (cara + bordes + ángulo), no desde la pose global: así un error de 2–3 cm de la pose no importa. 2 intentos por grano. | N1 |
| Entregar | A* hasta `x_aproximacion_entrega` frente a la caja, mirando +x; **reversa** hasta `x_entrega`; `X 0` abre la compuerta de maduros `t_vaciado_s`, la cierra y sale. N3 hace lo mismo con la azul (`X 1`). | N2 |
| Reloj | Antes de cada árbol y de cada estación se reserva el tiempo para volver y vaciar. A los 468 s se detiene todo. Si algo falla, igual intenta entregar lo que lleva. | todos |

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
| Ángulos de las compuertas, dónde caen los granos, a qué x retroceder | `compuerta_*_deg`, `salida_x`, `x_entrega` |
| GPIO del botón | `mision.pin_boton` |
| ¿Hay líneas negras en las posiciones de alberca vacías? Si las hay, el A* debe evitarlas (hoy solo evita las albercas que ve). | reglamento Apéndice A |
| IR: freno por borde desactivado en la misión hasta integrarlos (`freno_ir`). | |
