INTEGRACION LD06 + SLAM + ODOMETRIA EN RASPBERRY + ARDUINO MEGA

CONTENIDO
- Mega_Cafebot.ino y headers: firmware para Arduino Mega 2560.
- ld06_slam_nav_rpi_imu_integrado.py: SLAM, mapa, EKF, planificación A-B, landmarks y control de ruta en Raspberry Pi.

ARQUITECTURA
1) Arduino Mega: lee los cuatro encoders mediante interrupciones, mantiene los PID de velocidad, calcula la cinemática inversa/directa necesaria para accionar los motores y servos, y acepta consignas de velocidad V.
2) El Mega mantiene contadores acumulados M1..M4, separados de los contadores por ciclo utilizados por los PID. La Raspberry solicita esos contadores con E.
3) Raspberry: calcula el desplazamiento de las ruedas, la odometría planar y el EKF; corrige la estimación con la pose del SLAM del LD06. La planificación A->B y el acercamiento a landmarks ARBOL se ejecutan en Python.
4) La Raspberry envía consignas V vx_mm_s wz_rad_s. El Mega no decide la ruta en este modo; regula los motores con los PID.
5) Watchdog de firmware: si no recibe un nuevo comando V durante 500 ms, pasa a QUIETO y frena.

PROTOCOLO SERIE (115200 baudios, una orden por línea)
- E: responde ENC M1:<ticks> M2:<ticks> M3:<ticks> M4:<ticks> S:<postura>; S=0 ruedas rectas, S=1 postura de giro.
- V <vx_mm_s> <wz_rad_s>: consigna de velocidad al controlador del Mega.
- S: parada.
- C: cancelar cola heredada y detener.
- Z: reiniciar contadores acumulados, únicamente detenido.
- P, G, D, R y T se conservan para compatibilidad/diagnóstico del firmware heredado. El programa Python nuevo no utiliza POS/G para estimar pose ni para ejecutar rutas.

TECLAS DE LA INTERFAZ PYTHON
- Clic izquierdo: destino B.
- Shift + clic izquierdo: origen A.
- Enter: ejecutar ruta A->B.
- Tab: cambiar landmark confirmado.
- G: planificar aproximación al landmark ARBOL seleccionado.
- Esc: detener.
- R: reiniciar mapa SLAM y reinicializar el EKF; debe usarse con el robot detenido.
- P: recalcular ruta; F: seguimiento/replanificación continua; X: borrar A/B; S: guardar mapa/landmarks; L: cargar mapa; M: congelar/reanudar mapa; +/- zoom.

EJECUCION EN RASPBERRY
source .venv/bin/activate
python ld06_slam_nav_rpi_imu_integrado.py --port /dev/ttyUSB0 --mega-port /dev/ttyACM0 --no-imu
Cambiar los puertos según el resultado de: ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

CALIBRACION
Los parámetros PULSES_PER_REV=800, diámetro de rueda=53 mm, ancho=217 mm y diagonal de giro basada en 217 x 223 mm provienen de config.h. Confirmar físicamente pulsos por revolución efectivos, dimensiones, signos de los encoders y dirección de giro antes de confiar en la odometría. Si los encoders reportan un sentido invertido, corregir la polaridad/signo en encoder.h.

SEGURIDAD
- No probar rutas con el robot en el suelo inicialmente. Elevar ruedas y tener un corte físico de alimentación.
- No abrir el Monitor Serie de Arduino a la vez que Python ocupa el puerto del Mega.
- La parada por watchdog es una protección adicional, no reemplaza un paro de emergencia físico.
- La clasificación ARBOL sigue siendo geométrica basada en LiDAR; la cámara aún no está fusionada.

VALIDACION
El script Python se verificó con py_compile. No se dispone de compilador Arduino ni de hardware conectado en este entorno, por lo que la compilación del sketch y el comportamiento físico aún deben validarse en la placa antes de mover el robot.


=====================================================================
NOVEDADES: AUTODETECCION, VISION DE GRANOS Y MISION DE COSECHA
=====================================================================

1) FILTRO DE AUTODETECCION DEL LIDAR (siempre activo)
- El LD06 va en el centro del chasis (30 x 23 cm). Los puntos que caen dentro de la huella del robot
  (+3 cm de holgura) se descartan al armar cada barrido, ANTES del SLAM, el mapa y el tracker.
- Convenio: angulo 0 del LD06 = +Y = frente del robot. El LARGO (30 cm) va en Y y el ANCHO (23 cm) en X.
  Si el LD06 esta montado girado 90 grados, intercambia: --robot-largo 0.23 --robot-ancho 0.30.
- El rectangulo azul dibujado alrededor del robot en el mapa ES la zona filtrada: debe envolver al chasis real.
  La linea LIDAR del HUD muestra cuantos puntos propios se descartan por vuelta.
- Ajustes: --self-margin 0.03 (m), --self-extra CX,CY,HX,HY (repetible; para un mastil, la camara, el
  brazo...), --no-self-filter (solo para comparar).
- Las celdas bajo el chasis se marcan como libres en el mapa (el robot esta ahi) y el planificador usa el
  radio que envuelve al robot (circunscrito + 1 cm = 0.20 m) en vez de 0.18 m fijo.
- Se corrigio DEFAULT_BAUD, que se usaba en --baud pero no estaba definido (NameError al arrancar).

2) VISION DE GRANOS (--vision)
- vision/ contiene, SIN modificar, camara.py, clasificador.py, detector.py y calibracion/ (camara.json,
  referencias.json) del proyecto de deteccion de colores.
- percepcion_granos.py los envuelve en un hilo (el SLAM nunca espera a la camara), con filtro de
  estabilidad (N cuadros seguidos con la misma categoria) y politica conservadora:
    MADURO (rojo/naranjo/amarillo) y SOBREMADURO (azul/negro) -> candidato a recoger
    VERDE -> se ignora (sacarlo resta 7)      NO_SE / inestable -> no se toca
- La camara solo corre cuando se necesita (al llegar al arbol o con [V]) para no quitarle CPU al SLAM.
- Se asume camara mirando al frente. dx/dy salen en mm respecto del pixel objetivo (centro de imagen).
- Opciones: --vision, --vision-imagen foto.jpg (sin camara), --vision-hz 5, --vision-n 5.

3) MISION DE COSECHA (mision_cosecha.py) - tecla [K]
  [TAB] elegir un ARBOL confirmado -> [K]:
  APROXIMANDO (ruta A*, mismo RouteExecutor) -> ESTABILIZANDO -> ESCANEANDO (camara) -> RECOGIENDO (brazo)
  -> ESPERA_RESCAN -> ESCANEANDO ... -> FIN cuando no quedan granos recogibles estables.
- Prioridad: maduros (+7) antes que sobremaduros (+5). 1 reintento por grano y luego se salta.
- [ESC] o [K] otra vez aborta (detiene ruta, brazo y apaga la camara).
- Pendiente a proposito: exploracion autonoma (faltan sensores IR del borde) y llevar los granos a las cajas.

4) BRAZO (brazo_interfaz.py)
- Por ahora BrazoNoIntegrado: la mision ve y clasifica, informa que recogeria y termina sin mover nada.
- --brazo-simulado: brazo falso para probar todo el flujo.
- Cuando exista el brazo Dynamixel: escribir una clase que herede de BrazoBase (disponible, iniciar_recogida,
  estado, reconocer, cancelar, deposito_lleno) y usarla en main() en lugar de BrazoNoIntegrado.

TECLAS NUEVAS: [K] cosecha en el arbol seleccionado | [V] camara on/off (miniatura abajo a la derecha)
AJUSTE CLAVE: --standoff (m, por defecto 0.25) = holgura entre el borde del robot y el arbol. La camara se
calibro enfocada a 0.32 m (vision/calibracion/camara.json): ajusta --standoff hasta que los granos queden a esa distancia.

EJECUCION (con camara)
  python ld06_slam_nav_rpi_imu_integrado.py --port /dev/ttyUSB0 --mega-port /dev/ttyACM0 --no-imu --vision
  (picamera2 y python3-opencv son paquetes del sistema: crear el venv con --system-site-packages)
  Sin hardware de brazo: agregar --brazo-simulado para probar el flujo completo.

PRUEBAS SIN HARDWARE (carpeta pruebas/)
  python pruebas/test_autofiltro.py       filtro del chasis con paquetes LD06 simulados
  python pruebas/test_vision_mision.py    vision real sobre imagen sintetica + maquina de estados de la mision
  python pruebas/test_smoke_main.py       programa completo en lazo cerrado (LD06, Mega y camara simulados)

=====================================================================
BRAZO ROBOTICO + GEMELO DIGITAL + SENSORES IR (INTEGRACION INICIAL)
=====================================================================

NUEVOS ARCHIVOS
- brazo_dynamixel.py: adaptador BrazoBase para los Dynamixel y la cinemática analítica de main_stress_test.py.
- main_stress_test.py: código fuente de referencia del usuario (se mantiene, no se ejecuta como programa aparte).
- brazo1_description/: descripción/URDF del brazo para MuJoCo.
- sensores_ir.py: declaración de cuatro sensores IR de borde, deshabilitados por defecto.

SECUENCIA DE COSECHA
1) El operador selecciona un landmark ARBOL confirmado con TAB y pulsa K.
2) La misión usa la ruta SLAM existente para aproximarse al árbol.
3) Al terminar la ruta, se activa la cámara y espera clasificación estable.
4) Solo MADURO/SOBREMADURO son candidatos. VERDE/NO_SE no se recogen.
5) Con --brazo-dynamixel, el adaptador usa la IK analítica del proyecto del brazo y realiza aproximación,
   apertura, cierre, elevación, depósito provisional por categoría y retorno a la postura inicial.
6) El modelo MuJoCo se abre como ventana aparte si no se usa --brazo-sin-mujoco.

IMPORTANTE: esta es una integración de software inicial, no una validación mecánica. La cámara debe tener
calibrada focal_px/distancia y la distancia al fruto debe estar dentro de --brazo-max-dist. Si dist_m no está
disponible, el adaptador bloquea el agarre físico. El radio y altura objetivo por defecto (140 mm, 200 mm)
son parámetros iniciales: ajustar la transformación cámara->base del brazo, offsets, alturas y depósitos
antes de tocar un fruto real. El desplazamiento dx corrige yaw con signo configurable en el constructor;
dy_mm no se convierte a altura automáticamente porque requiere calibración geométrica de la cámara respecto
a la base del brazo. Los dos depósitos están representados por yaw +/-90 grados como destinos provisionales;
confirmar que físicamente existan allí y que no haya colisiones.

EJECUCION SIN BRAZO FISICO (simulación de misión)
python ld06_slam_nav_rpi_imu_integrado.py --port /dev/ttyUSB0 --mega-port /dev/ttyACM0 --no-imu --vision --brazo-simulado

EJECUCION CON BRAZO DYNAMIXEL REAL (solo después de pruebas y calibración)
python ld06_slam_nav_rpi_imu_integrado.py --port /dev/ttyUSB0 --mega-port /dev/ttyACM0 --no-imu --vision --brazo-dynamixel --brazo-port /dev/ttyUSB1

Los puertos son ejemplos. LD06, Arduino Mega y Dynamixel deben tener puertos independientes. No abras el monitor
serie de otro programa sobre el mismo puerto. MuJoCo y dynamixel_sdk deben estar instalados en el entorno Python.
Primero prueba sin brazo físico con --brazo-simulado. Después, prueba el brazo real sin fruto, a velocidad/área
seguras y con parada física accesible. El cierre de la garra está basado en 65 grados del main.py del proyecto;
verificar que sea seguro para la garra real y la fruta.

SENSORES IR
sensores_ir.py declara cuatro entradas BCM provisionales: frontal_izquierdo=17, frontal_derecho=27,
trasero_izquierdo=22, trasero_derecho=23. IR_ENABLED=False: no se inicializan ni afectan navegación. Confirmar
modelo (salida digital/analógica), polaridad, niveles lógicos y cableado antes de habilitarlos. Si son sensores
analógicos, hará falta un ADC externo o lectura apropiada; este módulo asume sensores digitales.

VALIDACION REALIZADA
- py_compile pasó para el programa principal y módulos nuevos.
- pruebas/test_autofiltro.py pasó.
- pruebas/test_vision_mision.py pasó.
- pruebas/test_smoke_main.py no terminó dentro del tiempo disponible; no se declara aprobada.
- No se ha compilado en esta sesión el firmware Arduino ni probado el brazo físico. El comportamiento MuJoCo,
  las direcciones Dynamixel, la cinemática de montaje y la transformación cámara-brazo necesitan validación.


=====================================================================
REVISION 2026-10-26 (ver CAMBIOS_REVISION.md en la raiz del proyecto)
=====================================================================
- Corregida la escala de la odometria del EKF (era 1000 veces menor), el cierre/cancelacion del brazo, los IR
  (gpiozero) y un estado de servos del firmware al recibir S/C/R durante un giro.
- test_smoke_main.py SI termina (~35 s) y pasa. Nuevas pruebas: test_brazo_dynamixel.py y test_odometria_ir.py.
- PENDIENTE antes de usar el brazo real: calibrar vision/calibracion/detector.json
  (python vision/detector.py --calibrar-dist 0.32); sin focal_px el brazo rechaza todos los granos.
