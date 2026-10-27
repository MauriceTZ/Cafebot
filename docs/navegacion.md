# Navegación en la Raspberry: SLAM + odometría + landmarks + rutas A→B

Reescritura modular de `legacy/integracion_juan_v1/Raspberry_LD06_SLAM/ld06_slam_nav_rpi_imu_integrado.py`
(un archivo de 2300 líneas) adaptada al firmware nuevo del Mega y a la forma real del robot.

## Cómo correrla

```bash
cd ~/Cafebot/pi && source .venv/bin/activate
sudo apt install python3-scipy python3-pygame python3-smbus2     # una vez
python3 -m tools.navegacion --mega /dev/ttyACM0 --lidar /dev/ttyUSB0          # robot completo
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
- **Rectángulo azul** = huella del robot: los puntos del LiDAR que caen dentro se descartan. Debe envolver
  el chasis. Si el LiDAR ve partes del robot fuera de él (brazo, ruedas), agregar rectángulos en
  `geometria.extras` (ver `pi/robot_local.ejemplo.json`). El HUD muestra cuántos puntos propios se filtran.
- **Punto blanco** = posición configurada del LiDAR (`lidar_x`). Si el mapa se "dobla" al girar en el sitio,
  la posición o la orientación (`lidar_yaw_deg`) del LiDAR están mal medidas.

## Módulos (`pi/cafebot/navegacion/`)

| Módulo | Qué hace | Cambio respecto del código antiguo |
|---|---|---|
| `slam.py` | ICP barrido‑contra‑mapa + grilla log‑odds + mapa estático | Convención x adelante. El ICP parte de la odometría. La referencia es la **posición media de los impactos** de cada celda (con el centro de celda había un sesgo de 2 cm). El espacio libre se traza **desde el LiDAR**, sin el hueco delante del robot que comentaba el código antiguo. Calcula la **matriz de información** (con las normales del mapa) para saber en qué direcciones el ICP es confiable |
| `odometria.py` | EKF: predicción con la odometría del Mega (+ giroscopio), corrección con el SLAM | Usa la pose del Mega (que ya considera la postura de las ruedas: recto, giro, cangrejo) en vez de re-implementar una cinemática diferencial con ticks crudos. La corrección usa la información del ICP: donde el LiDAR ve poco, manda la odometría. Verifica solo el signo del giroscopio |
| `landmarks.py` | Agrupa el mapa estático, clasifica y da nombres estables | Antes un árbol era un **tronco cilíndrico**. A 8 cm de altura el LiDAR ve el **pie plano** del árbol: ahora se clasifica por el rectángulo envolvente (árbol ~30 cm, caja ~45 cm también vista de esquina, alberca ~72 cm). Rangos en `CLASES` |
| `seguimiento.py` | Objetos que se mueven (Kalman + asignación húngara) | Lo que el código antiguo llamaba "JPDA" es asociación GNN; se mantiene. Sin scikit‑learn |
| `planificador.py` | A* sobre el mapa (grafo de celdas) + acortado por línea de vista | Aproximación a un landmark de cara plana: se queda frente a la cara que mira al robot |
| `ejecutor.py` | Ruta → órdenes `W` (girar) y `A` (avanzar) del Mega, en tramos ≤ 0,5 m | Antes mandaba velocidades continuas pensadas para tracción diferencial. Ahora usa las primitivas del firmware y relee la pose fusionada antes de cada tramo |
| `sistema.py` | Une todo sin interfaz (`actualizar()`, `pose()`, `ir_a()`, `aproximar()`) | La misión y la interfaz usan la misma clase |

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
- Integrar los IR (líneas de la cancha) como corrección absoluta y para la ruta inicial.
- Probar con el LD06 real. Ajustar `lidar_x`, `lidar_yaw_deg`, `extras` y los rangos de `CLASES`.
- Brazo y cámara: tienen sus propios módulos (`cafebot/arm`, visión por portar). Se conectan a
  `SistemaNavegacion` a través de la misión, no dentro de la navegación.
