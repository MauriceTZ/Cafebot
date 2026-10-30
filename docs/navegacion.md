# Navegación en la Raspberry: SLAM + odometría + landmarks + rutas A→B

Reescritura modular de `legacy/integracion_juan_v1/Raspberry_LD06_SLAM/ld06_slam_nav_rpi_imu_integrado.py`
(un archivo de 2300 líneas) adaptada al firmware nuevo del Mega y a la forma real del robot.

## Cómo correrla

```bash
cd ~/Cafebot/pi && source .venv/bin/activate
sudo apt install python3-scipy python3-pygame python3-smbus2     # una vez
python3 -m tools.navegacion --mega /dev/ttyUSB1 --lidar /dev/ttyUSB0          # robot completo
python3 -m tools.navegacion --lidar /dev/ttyUSB0 --sin-mega                   # solo mapear (empujando)
python3 -m tools.navegacion --sim                                             # todo simulado
# agregar --imu para usar el giroscopio MPU9250
```

| Ratón / tecla | Acción |
|---|---|
| clic / Shift+clic | destino B / origen A (A vacío = el robot) |
| ENTER / ESC | ejecutar la ruta / parar |
| TAB / G | elegir landmark / aproximarse a él (queda mirándolo a `distancia_aprox_m`) |
| flechas, Q / E | mover a mano (avanzar, girar) / cangrejo izquierda, derecha |
| P, A, X | replanificar, origen = robot, borrar ruta |
| M | congelar el mapa (solo localizar) / reanudar |
| S / L | guardar / cargar mapa y landmarks |
| I | ubicar en la cancha (robot en su sector de partida): dibuja la cancha encima del mapa |
| T | mostrar/ocultar la trayectoria de solo odometría |
| R, C, O, B | reiniciar, centrar en el robot, volver al origen, recalibrar giroscopio |

**Qué mirar para calibrar:**
- **Línea verde** = trayectoria fusionada. **Amarilla** = la que daría la odometría sola. Si se separan
  mucho en una recta, revisar el diámetro de rueda; si se separan en los giros, `CINEMATICA_D_GIRO_Z`.
- **Rectángulo azul** = lo que el LiDAR considera el propio robot: los puntos que caen dentro se descartan.
  Desde el 29 oct es un **cuadrado de 45 cm detrás del LiDAR** (al frente, por fuera de la base y centrado: el
  robot no puede medir más de 450 mm por lado), `geometria.lidar_caja_m = 0.45` (0 = solo la base + margen).
  Si el LiDAR ve partes del robot fuera de él, agregar rectángulos en `geometria.extras`
  (ver `pi/robot_local.ejemplo.json`). El HUD muestra cuántos puntos propios se filtran por vuelta.
- **Punto blanco** = posición configurada del LiDAR (`lidar_x`). Si el mapa se "dobla" al girar en el sitio,
  la posición o la orientación (`lidar_yaw_deg`) del LiDAR están mal medidas.

## Recorrer el borde de la cancha (tecla **F**)

Prueba de navegación con el LiDAR y los sensores de línea juntos (`pi/cafebot/navegacion/bordes.py`):

1. Poner el robot en una **esquina**, dentro de la cancha, con un borde **detrás** y otro a la **derecha**
   (a unos 10–20 cm de cada uno; puede estar un poco torcido).
2. `python3 -m tools.navegacion` (con el Mega y el LiDAR) y tecla **F** (o `--borde` para que parta solo a los 3 s).
   **ESC** o **F** lo detienen. `--vel-borde 100` cambia la velocidad de los tramos (120 mm/s por defecto).
3. El robot:
   - toca el borde de atrás (TI/TD) y se endereza con él; toca el derecho (DD/TD) y se separa 10 cm;
   - avanza con el freno en DI/DD; cada 80 cm se acerca en cangrejo al borde derecho, lo toca, corrige el rumbo y
     se separa (así no se va desviando);
   - al tocar el borde de adelante se endereza con él, vuelve a tocar el derecho y gira 90° a la izquierda;
   - después de 4 tramos está otra vez en la esquina de partida, mirando igual.

En pantalla: los **toques** de los sensores (un color por lado) y la recta ajustada a cada lado; las esquinas
(0–4) donde quedó el robot; al terminar, el **rectángulo medido** con la pose fusionada LiDAR + odometría
(**verde**) y con la odometría sola (**amarillo**, alineado en la partida). La línea **MEDIDO** dice el tamaño de
la cancha, sus ángulos y el **error de cierre**: la esquina final ES la de partida (la ubican los bordes), así que
lo que se haya corrido cada estimación es su error. En simulación (pista de 1,5 × 1,1 m): rectángulo con ±1 cm,
cierre LiDAR + odometría 0,5 cm, odometría sola 8 cm.

Sin robot: `python3 -m tools.navegacion --sim --taller 2.4x1.8 --borde` (pista de taller: solo la huincha del
borde, dentro de una sala con paredes y muebles) y `python3 -m tests.test_borde`.

Ojo:
- Los sensores van junto a las ruedas: al tocar un borde la **rueda está sobre la línea** (en la cancha es falta).
  Es una prueba de taller. En la cancha del torneo las líneas interiores (beneficiadero, cultivo) también frenan.
- Necesita los servos de dirección funcionando (cangrejo) y los umbrales IR calibrados (`tools.ir borde`).
- El LiDAR va a ~5 cm del suelo: a esa altura ve el pie de los árboles, las cajas y las albercas; si el robot
  cabecea, puede ver el piso lejos (puntos sueltos que aparecen y desaparecen). El recorrido no depende del LiDAR (lo guían los bordes); el LiDAR da el mapa y la pose fusionada.

## Módulos (`pi/cafebot/navegacion/`)

| Módulo | Qué hace | Cambio respecto del código antiguo |
|---|---|---|
| `slam.py` | ICP barrido‑contra‑mapa + grilla log‑odds + mapa estático | Convención x adelante. El ICP parte de la odometría. La referencia es la **posición media de los impactos** de cada celda (con el centro de celda había un sesgo de 2 cm). El espacio libre se traza **desde el LiDAR**, sin el hueco delante del robot que comentaba el código antiguo. Calcula la **matriz de información** (con las normales del mapa) para saber en qué direcciones el ICP es confiable |
| `odometria.py` | EKF: predicción con la odometría del Mega (+ giroscopio), corrección con el SLAM | Usa la pose del Mega (que ya considera la postura de las ruedas: recto, giro, cangrejo) en vez de re-implementar una cinemática diferencial con ticks crudos. La corrección usa la información del ICP: donde el LiDAR ve poco, manda la odometría. Verifica solo el signo del giroscopio |
| `landmarks.py` | Agrupa el mapa estático, clasifica y da nombres estables | Antes un árbol era un **tronco cilíndrico**. A 8 cm de altura el LiDAR ve el **pie plano** del árbol: ahora se clasifica por el rectángulo envolvente (árbol ~30 cm, caja ~45 cm también vista de esquina, alberca ~72 cm). Rangos en `CLASES` |
| `seguimiento.py` | Objetos que se mueven (Kalman + asignación húngara) | Lo que el código antiguo llamaba "JPDA" es asociación GNN; se mantiene. Sin scikit‑learn |
| `planificador.py` | A* sobre el mapa (grafo de celdas) + acortado por línea de vista | Aproximación a un landmark de cara plana: se queda frente a la cara que mira al robot |
| `ejecutor.py` | Ruta → órdenes `W` (girar) y `A` (avanzar) del Mega, en tramos ≤ 0,5 m | Antes mandaba velocidades continuas pensadas para tracción diferencial. Ahora usa las primitivas del firmware y relee la pose fusionada antes de cada tramo |
| `lineas.py` | Cruces de líneas negras (4 sensores IR QTR-1A): cada cruce ubicado en el mapa, asociado a la línea conocida de la cancha, con su residuo; corrección opcional del EKF perpendicular a la línea | Nuevo (`docs/ir.md`) |
| `sistema.py` | Une todo sin interfaz (`actualizar()`, `pose()`, `ir_a()`, `aproximar()`, `fijar_cancha()`) | La misión y la interfaz usan la misma clase |

Otros: `drivers/imu.py` (MPU9250), `drivers/lidar_simulado.py` (LD06 simulado con error de calibración),
`config.py` (geometría y puertos en un solo lugar; se sobreescribe con `pi/robot_local.json`).

## Resultados en simulación (`python3 -m tests.test_navegacion`)

Se usa el firmware real del Mega simulado y un LiDAR simulado. A propósito, la odometría cree avanzar
un 3 % y girar un 2 % de más:

| Tramo | Error pose fusionada | Odometría sola |
|---|---|---|
| 0,9 m recto | 0,8 cm | 2,2 cm |
| Aproximación a un árbol (quedó a 23,6 cm; se pidieron 25) | 1,5 cm | — |
| Regreso al punto de partida | 1,3 cm, 1,9° | 13 cm, 6° |

Con el robot quieto, la pose se mantiene dentro de 0,4 cm en 15 s.

## Pendiente
- IR: ya ubicados en el mapa y asociados a la cancha (`docs/ir.md`). Falta validar los residuos en la cancha real,
  activar `ir.corregir_pose` y usar las líneas en el recorrido inicial.
- Probar con el LD06 real. Ajustar `lidar_x`, `lidar_yaw_deg`, `extras` y los rangos de `CLASES`.
- Brazo y cámara: tienen sus propios módulos (`cafebot/arm`, visión por portar). Se conectan a
  `SistemaNavegacion` a través de la misión, no dentro de la navegación.
