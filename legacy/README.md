# Código anterior (solo referencia)

Nada de esta carpeta se ejecuta en el robot final. Se conserva para copiar ideas o
partes al código nuevo y para no perder historia.

| Carpeta | Origen | Qué sirve todavía |
|---|---|---|
| `esp32_omnicar/` | Primera base (mecanum + ESP32) | Estructura PID/encoders (ya portada al Mega). |
| `mega_cafebot_v0/` | Primer port al Mega | Pines, medidas y cinemática de giro → `firmware/mega_cafebot/config.h`. |
| `codigo_brazo/` | IK analítica + MuJoCo + prueba de estrés | `cinematica_inversa_analitica`, offsets de los servos, secuencia de encendido/apagado. |
| `integracion_juan_v1/` | Integración LD06+SLAM+visión+brazo (26 oct) | `brazo_dynamixel.py` (movimiento cancelable, estacionar antes de soltar torque, validación de IK), filtro de huella del LiDAR. Su `vision/` es idéntica a `vision_pablo/` y no se copió. Necesita un operador (clics/teclas): no sirve como misión autónoma. |
| `vision_pablo/` | Visión y pruebas de la base mecanum (repo BlipTheClip/cafebot) | `clasificador.py` (Lab por vecino más cercano con "no sé", posiciones fijas de soportes), `camara.py` (exposición/balance fijos), `detector.py`. |
