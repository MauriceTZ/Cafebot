# Brazo: alcance, esfuerzo y cuidado de los servos

Datos: tramos 107 + 107 + 67 mm + muñeca→agarre 32 mm (`legacy/codigo_brazo/main.py`),
hombro 67 mm sobre la base del brazo, base del brazo a 36 cm del suelo.
Servos XL430‑W250‑T según el [manual de ROBOTIS](https://docs.robotis.com/docs/dxl/model_reference/x_series/xl_series/xl430-w250/):
57,2 g; torque de bloqueo 1,0 / 1,4 / 1,5 N·m a 9 / 11,1 / 12 V (1,0 / 1,3 / 1,4 A).
El manual aclara que el torque de bloqueo es un máximo **momentáneo**, no continuo.

## Alturas de los granos (Apéndice B)

| Fila | Cara del soporte | Centro del grano* | Sobre la base del brazo |
|---|---|---|---|
| Baja | 390 mm | ≈407 mm | +4,7 cm |
| Media | 540 mm | ≈557 mm | +19,7 cm |
| Alta | 690 mm | ≈707 mm | +34,7 cm |

\* Esfera de 42 mm en un hueco de 25 mm: centro ~17 mm sobre el soporte.

## Lo que muestran los cálculos (`python3 -m tests.test_cinematica`, en `pi/`)

**1. La prueba que falló era el peor caso posible.** Con el brazo estirado en horizontal,
el hombro carga ≈0,76 N·m ≈ 54 % del bloqueo, de forma sostenida. El servo tiene activa
de fábrica la protección de **sobrecarga** (Shutdown bit 5: "carga persistente mayor
que la salida máxima"). Esa protección apaga el torque y **solo se recupera reiniciando
el servo**, que es justo lo que vieron. La tarea nunca necesita esa postura.

**2. IK de mínimo esfuerzo (`pi/cafebot/arm/cinematica.py`).** Hombro, codo1 y codo2
ubican la muñeca en un plano, así que hay infinitas posturas para el mismo punto. La IK
antigua repartía el ángulo entre codo1 y codo2 en una proporción fija (0,93). La nueva
elige la postura en que el servo **más cargado** queda lo más descargado posible:

| Fila | r (cm) | IK antigua | Mín. esfuerzo, garra 90° | Mín. esfuerzo, garra libre |
|---|---|---|---|---|
| baja | 10 | 22 % | 15 % | 15 % |
| baja | 13 | 28 % | 19 % | 17 % (garra 65°) |
| baja | 16 | 33 % | 23 % | 23 % |
| media | 10–16 | 18–23 % | 16–22 % | 16–22 % |
| alta | 10 | **inalcanzable** | **inalcanzable** | 13 % (garra 58°) |

(% = torque del servo más exigido / 1,4 N·m; r = distancia horizontal del eje de la base al grano)

**3. La fila alta (la mitad de los maduros) es el verdadero límite.** Solo se alcanza
inclinando la garra hacia arriba, y muy cerca del árbol:

| Base del brazo | Distancia máx. al grano alto (con 1 cm de margen) |
|---|---|
| 36 cm | **8,5 cm** (garra a ~58°) |
| 38 cm | 14 cm |
| 40 cm | 17,5 cm |

→ **Subir la base del brazo 2–4 cm vale más que cualquier otra cosa**, si el robot plegado
sigue cabiendo en el cubo de 45 cm. **No alargar los tramos**: el torque crece
aproximadamente con el cuadrado del largo.

## Configuración que cuida los servos (`pi/cafebot/arm/dxl.py`)

| Ajuste | Antes (legacy) | Ahora | Por qué |
|---|---|---|---|
| Position I Gain | 150 + integrador externo | 0 (fábrica) | Si la postura no se alcanza, los integradores llevan el servo al 100 % hasta que salta la sobrecarga. |
| Perfil | Python interpola a 50 Hz | Perfil **por tiempo** (Drive Mode bit 2): posición final + duración | Todas las articulaciones llegan juntas, con aceleración suave → menos picos de corriente y menos tráfico en el bus. Requiere firmware ≥ v42 en los servos. |
| PWM Limit de la garra | Goal PWM 350 | PWM Limit 300 (~34 %) | Un grano pesa 12 g. Apretarlo fuerte durante el transporte es un bloqueo permanente: solo genera calor. |
| Return Delay Time | 250 (500 µs) | 0 | Lecturas del bus más rápidas. |
| Salud | — | Temperatura, voltaje, carga estimada y Hardware Error Status | Si un servo pasa de 66 °C o baja de 10,5 V, no iniciar agarres. Si saltó una protección: reboot automático. |
| Bus Watchdog | — | Opcional | Si el programa de la Pi se cuelga, el brazo se detiene. |

Además:
- **Descanso apoyado, sin torque**: diseñar un soporte donde el brazo plegado se apoye
  durante la navegación y apagar el torque. Ahí el consumo es cero, y es la mayor parte
  de la ronda.
- **Batería cargada**: a 9 V el bloqueo cae a 1,0 N·m (−29 %). Con la batería baja,
  aumenta el riesgo de sobrecarga.
- El manual pide **no subir** el límite de temperatura (72 °C) y esperar 20 min tras un
  apagado por calor: entre rondas, dejar el brazo sin torque.
- `Present Load` es una **estimación** (el XL430 no mide corriente): sirve para ver
  tendencias, no como medida exacta.
