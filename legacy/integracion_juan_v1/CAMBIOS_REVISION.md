# Revisión del código: errores corregidos y pendientes

Fecha: 2026-10-26. Alcance: Raspberry (SLAM, misión, visión, brazo, IR) y firmware del Mega.
Sin hardware: las pruebas usan LD06, Mega, cámara y servos simulados. Nada se probó en el robot real.

## A. Errores corregidos

| # | Archivo | Problema | Corrección |
|---|---------|----------|------------|
| 1 | `ld06_slam_nav_rpi_imu_integrado.py` (`OdometryEKF.predict_encoders`) | La traslación predicha por los encoders salía **1000 veces más chica**: `PULSES/diámetro` ya daban metros y después se dividía otra vez entre 1000. Verificado: 1,0 m de ticks avanzaba la pose 0,001 m. El SLAM lo tapaba, y el ruido Q también quedaba subestimado. El giro sí estaba bien. | Todo en metros. 1 m de ticks = 1 m de pose. Ver `pruebas/test_odometria_ir.py`. |
| 2 | `brazo_dynamixel.py` | **[ESC] / cancelar no detenía el brazo**: se usaba `mover_suave_a_postura`, que no se puede interrumpir. El movimiento en curso seguía hasta 2 s hacia el fruto. | Movimiento propio a 50 Hz que revisa la cancelación en cada paso de 20 ms. El brazo se queda donde está. |
| 3 | `brazo_dynamixel.py` | **`cerrar()` soltaba el torque de golpe** con el brazo extendido (caída por gravedad). `main_stress_test.py` sí estaciona en reposo (OFFSETS_APAGARSE) antes de soltar. | `cerrar()` estaciona 3 s en reposo y luego suelta el torque. Idempotente. Se registra con `atexit`, así que también corre si el programa se cae. |
| 4 | `brazo_dynamixel.py` | El brazo no iba a la postura de trabajo al iniciar (`main_stress_test` sí). La cinemática inversa tomaba `pitch_ref` y `factor_codo2` de la postura en que estuviera al encender (reposo: 113°/0,72 en vez de 94°/0,93). | Al iniciar va a OFFSETS_PRENDERSE (2,5 s) y la IK siempre se referencia a esa postura. Desactivable con `ir_a_postura_trabajo=False`. |
| 5 | `brazo_dynamixel.py` | Errores de comunicación Dynamixel ignorados (si falla `PROFILE_VEL` el servo corría a velocidad por defecto). Fallos de TX durante el movimiento no se detectaban. IK sin validar. El torque quedaba activo si fallaba el arranque. Si MuJoCo fallaba, ya se había movido el brazo. | Se revisan las escrituras de configuración. 5 fallos de TX seguidos abortan. La IK se valida (finita y dentro de ±180°). MuJoCo se carga antes de tocar los servos. |
| 6 | `brazo_dynamixel.py` | Un fruto muy lateral hacía una corrección de yaw de hasta ±90°. | Se rechaza si la corrección supera ±45° (`max_corr_yaw_deg`). |
| 7 | `ld06_slam_nav_rpi_imu_integrado.py` (`main`) | Si el brazo fallaba al arrancar (puerto, ID, MuJoCo), el programa **se caía** sin cerrar el LD06 ni el Mega. | Se captura el error, se avisa y se continúa con `BrazoNoIntegrado`. |
| 8 | ídem | La verificación de puertos comparaba contra `args.port`, que con `--port auto` vale `'auto'`, nunca el puerto real. | Compara contra el puerto resuelto del LD06. |
| 9 | `mision_cosecha.py`, `brazo_interfaz.py` | La misión decía "El brazo rechazó #N" sin motivo (el motivo se guardaba pero nadie lo leía). | Nuevo `BrazoBase.ultimo_error()`; la bitácora y el HUD muestran el motivo. |
| 10 | `sensores_ir.py` | `DigitalInputDevice(pin, pull_up=None)` **lanza error en gpiozero** si no se da `active_state`: con `IR_ENABLED=True` habría fallado al crear los sensores. `--ir-borde` no hacía nada sin avisar. | Se usa `active_state=not IR_ACTIVE_LOW`. `leer()` devuelve True = borde. Nuevo `hay_borde()` y `estado_texto()` (HUD). Aviso si se pide `--ir-borde` con `IR_ENABLED=False`. Sigue deshabilitado por defecto. |
| 11 | `ld06_slam_nav_rpi_imu_integrado.py` + `LidarSLAM2D` | Mientras el brazo recoge puede entrar al plano del LiDAR y dejar celdas estáticas fantasma o landmarks falsos. | `slam.hold_integration` congela la integración al mapa mientras la misión está en RECOGIENDO. |
| 12 | Firmware `serial_cmd.h` / `control.h` | **S, C y R durante una rampa de servos**: `servos_en_transicion` seguía en true, `control_carro()` continuaba la rampa y devolvía los servos a 45° con el firmware creyendo estar en RECTO. Por ejemplo, ESC a menos de 1,6 s de iniciar un giro, y luego avanzar con ruedas a 45°. | Nueva `servos_a_recto_inmediato()` que deja coherente todo el estado de servos. Se verificó la sintaxis con g++ y un shim de Arduino; **no** con el toolchain real. |
| 13 | `__pycache__/` en el zip | `.pyc` de Python 3.13 incluidos. | Eliminados. |

## B. Pendientes que NO se pudieron corregir (decisión o calibración suya)

1. **Falta `vision/calibracion/detector.json`** (no está en el zip). Sin `focal_px`, `dist_m` es None para todos los granos y el brazo real **rechazará todos** (con el motivo en la bitácora). Calibrar: `python vision/detector.py --calibrar-dist 0.32` con un grano a 32 cm.
2. **El agarre no usa `dy_mm`**: siempre va a radio 140 mm y altura 200 mm (parámetros fijos). Solo se corrige el yaw con `dx_mm`. Falta la transformación cámara→base del brazo. Hoy el brazo "agarra a ciegas" en esa posición fija.
3. **Signo de encoders y escala**: con el arreglo n.º 1 la odometría ahora sí aporta traslación, así que un signo de encoder invertido, antes invisible, ahora empeoraría la pose. Probar con las ruedas en el aire y luego empujando 1 m.
4. **Pulsos parásitos al girar los servos** (el propio firmware lo dice en un comentario): se descartan en el PID, pero siguen sumando en los contadores acumulados `M1..M4` que usa la Raspberry. Si se nota deriva en giros, restar en el firmware los conteos del ciclo mientras `servos_en_transicion`.
5. **Brazo en el LiDAR**: definir con `--self-extra CX,CY,HX,HY` el rectángulo del brazo/cámara en su postura de trabajo.
6. **IR**: pines, polaridad y tipo (digital/analógico) por confirmar. Falta decidir qué hace el robot al ver el borde (hoy solo se leen y se muestran). En Raspberry Pi 5, gpiozero necesita `lgpio`. Verificado solo con un gpiozero simulado.
7. **Depósitos provisionales** (yaw ±90°), `deposito_lleno()` siempre False y sin traslado a las cajas del beneficiadero.
8. `main_stress_test.py` importa `mujoco`, `termios` y `dynamixel_sdk` al cargarse: con `--brazo-dynamixel` hacen falta incluso con `--brazo-sin-mujoco` (solo Linux).
9. `--brazo-dynamixel` sin `--vision`: el brazo se energiza pero nada lo usa (ahora hay un aviso).

## C. Pruebas

Todas pasan sin hardware (sklearn/pygame/serial simulados donde faltaban):
`test_autofiltro`, `test_vision_mision`, `test_smoke_main` (termina en ~35 s), y las nuevas `test_brazo_dynamixel` (SDK Dynamixel simulado) y `test_odometria_ir`.
Ejecutar desde `Raspberry_LD06_SLAM/`: `python pruebas/test_brazo_dynamixel.py`, etc.
