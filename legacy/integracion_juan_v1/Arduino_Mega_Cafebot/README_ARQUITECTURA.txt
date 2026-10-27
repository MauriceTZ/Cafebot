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
