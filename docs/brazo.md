# Brazo: calibración, alcance, esfuerzo y cuidado de los servos

## Calibrar y probar el brazo, sin cámara (`python3 -m tools.brazo`, en `pi/`)

Código: `pi/cafebot/arm/brazo.py` (clase `BrazoXL430`, la que usa la misión, y `PerfilBrazo`), `dxl.py` (bus),
`cinematica.py` (IK), `pi/tools/brazo.py` (la herramienta). Sin hardware: `python3 -m tests.test_calibracion_brazo`
y `python3 -m tests.test_brazo_falso`.

### El perfil de cada brazo

Todo lo que es propio de UN brazo físico queda en su **perfil**, `pi/brazos/<nombre>.json`: velocidad del bus, id de
cada servo, el **cero** de cada articulación (paso del encoder con el brazo recto), su **sentido**, sus **límites**,
las **posturas** con nombre y un **historial** con fecha de cada calibración. Cada brazo se calibra por separado con
`--perfil NOMBRE` (por defecto `principal`); la misión usa el de `brazo.perfil` en `pi/robot_local.json`. Después
de calibrar, **subir el perfil al repo**. Sin perfil se usan los valores del código antiguo, y las posturas de
`pi/brazo_posturas.json` (formato anterior) pasan solas al perfil la primera vez.

Los ajustes que no dependen del brazo siguen en `pi/robot_local.json` (secciones `"brazo"` y `"mision"`, ver
`ConfigBrazo` en `pi/cafebot/config.py`): posición de la base en el robot (`mision.brazo_x/y/z`), largo de la garra,
ángulos de la garra, tiempos, etc. Con el perfil calibrado, `brazo.yaw0_deg = 0` y `brazo.yaw_signo = 1`.

**Convención** (la de la IK): **0° = brazo RECTO HACIA ARRIBA** (cada tramo alineado con el anterior), la base mirando
hacia ADELANTE del robot y la garra ABIERTA (sin forzarla contra el tope). **Positivo** = hacia ADELANTE en hombro,
codos y muñeca (con el brazo recto, la punta va hacia adelante); hacia la IZQUIERDA del robot en la base; CERRANDO
en la garra.

### Antes de empezar

- Servos alimentados (12 V; con la batería baja el torque cae, ver abajo). U2D2 conectado a la Pi.
- **La primera conexión a un servo escribe su configuración y para eso le apaga el torque**: tener el brazo apoyado.
- Brazo fuera del robot (en un banco): agregar `--banco` a `fila`, `punto`, `teleop` y `agarrar`. Las coordenadas
  pasan a medirse desde la **base del brazo** (x adelante desde el eje de la base, z hacia arriba desde el plano
  donde está apoyada) y `fila` dice dónde poner el soporte de prueba para que el brazo haga lo mismo que en el robot.
  En el banco, "adelante" es hacia donde mire la base al hacer el `cero`. Al montarlo en el robot: `cero --solo base
  --rearmado` con la base mirando al frente del robot, `limites` de nuevo (en el banco no choca con el robot) y
  grabar las posturas de depósito ahí (las tolvas están en el robot).
- Las opciones (`--perfil`, `--banco`, `--puerto`, `--si`) se pueden poner antes o después del comando.

### Pasos

| Paso | Comando | Qué hacer / qué mirar |
|---|---|---|
| 1 | `escanear` | Qué servos responden y a qué velocidad (prueba 1 M, 57 600, 115 200 y 2 M). Compara con el perfil: `ok`, `NO RESPONDE`, ids que sobran. Firmware **≥ 42** (si no, usa perfil por velocidad: actualizar con Dynamixel Wizard). Si todos responden a otra velocidad, ofrece cambiarla en el perfil. |
| 1b | `identificar` | Solo si los ids no calzan con el perfil (brazo nuevo o servos cambiados): enciende el LED de un servo a la vez y preguntas qué articulación es. Luego hacer el cero de las que cambiaron con `--rearmado`. |
| 2 | `salud` | °C, V, carga, errores, pasos y ángulo de cada articulación. "Puede trabajar: SÍ". |
| 3 | `cero` (SIN torque: **sujetar el brazo**) | Poner el brazo recto hacia arriba (con una regla o escuadra a lo largo de los tramos), la base mirando adelante y la garra abierta → ENTER. Muestra cuánto se corrió cada cero; si alguno pasa de ~30°, revisar que estuviera recto. El brazo queda sostenido recto. `--solo hombro,codo1` hace solo esas. |
| 4 | `verificar` | Una por una, la articulación queda libre (las demás sostienen el brazo) y la mueves 20–30° en su sentido positivo. Si la lectura baja, confirma y se invierte el sentido en el perfil. También avisa si con el brazo recto alguna lee lejos de 0° (revisar su cero) o si se mueve otra (ids cruzados). `--solo` para repetir una. |
| 5 | `limites` (SIN torque) | Mover cada articulación por todo su recorrido **seguro** (sin chocar con el robot, otra pieza o tirar de cables). Se guarda con 3° de margen hacia adentro; las que se movieron menos de 10° quedan igual. Muestra el alcance de las 3 filas con los límites nuevos. `limites --reiniciar` vuelve a los del modelo. |
| 6 | `articulacion hombro 10` | **Primer movimiento con torque**: una sola articulación, a un ángulo (`--rel`: relativo a donde está). Avisa si el movimiento es grande. Empezar con 5–10° en cada una y comprobar que va hacia donde dice. |
| 7 | `garra calibrar` | Cierra vacía, luego con un grano (o una esfera de ~42 mm) entre los dedos, y propone `brazo.garra_vacia_deg` en el medio (y bajar `garra_cerrada_deg` si los dedos chocan entre sí antes). Guarda en `pi/robot_local.json`. |
| 8 | `leer` + tecla **G** (SIN torque) | Grabar `descanso` (plegado **detrás** del frente del robot, idealmente apoyado), `deposito_maduro` y `deposito_sobremaduro` (garra sobre cada tolva). Las de fábrica (`trabajo`, `descanso`) vienen del código antiguo: si el brazo se rearmó, volver a grabarlas. Luego `ir descanso --t 4` para ver que lleguen sin golpes. |
| 9 | `punto 0.20 0 0.45` | Medir con regla dónde quedó el centro de los dedos (error < 1 cm). La diferencia corrige `mision.brazo_x/y/z` (o `garra_largo_m`). |
| 10 | `fila baja`, `fila media`, `fila alta` (`--sin-deposito` primero) | Agarre completo con **coordenadas fijas**, como lo hace la misión frente al árbol: dice a qué distancia del frente del robot y a qué altura poner el soporte de prueba. `--lateral 0.06` prueba un soporte corrido 6 cm. Sin `--sin-deposito` lo deja en la tolva (`--categoria SOBREMADURO` para la otra). |

Otros: `perfil` (todo el perfil, el alcance y el historial, sin conectar), `ir <postura>`, `garra abrir|cerrar|<°>`,
`teleop` (W/S x, A/D y, R/F z, T/G inclinación, O/P garra, K agarrar aquí), `agarrar X Y Z`, `grabar <nombre>`,
`apagar` (descanso y soltar el torque).

**Si se cambia o se desmonta un servo**: `cambiar-id 1 <id>` si es nuevo de fábrica (id 1 a 57 600; conectar SOLO
ese servo), `identificar` si cambió de id, y después `cero --solo <articulación> --rearmado`, `verificar --solo
<articulación>` y volver a grabar las posturas. Sin `--rearmado`, `cero` corrige las posturas y los límites para que
queden físicamente iguales (sirve cuando solo se afina el cero).

**Otro brazo**: `python3 -m tools.brazo --perfil brazo2 escanear` y los mismos pasos; queda en `pi/brazos/brazo2.json`.

**Secuencia de agarre** (`recoger`): abrir → 6 cm antes del grano en la dirección del brazo → entrar en 2 pasos →
cerrar → si la garra cerró del todo, no había grano (abre y retrocede) → subir (3 cm; en la fila alta lo que alcance,
mínimo 1 cm) → retroceder (hasta 8 cm) → postura de depósito → abrir → `descanso`. Si una articulación queda a más de
6° de su meta o la carga pasa de 80 %, algo chocó: abre, retrocede y la misión sigue con el siguiente grano.
La garra **nunca se acerca a menos de `brazo.r_min_m` = 5 cm del eje de la base** (pasaría sobre el propio robot):
con `brazo_x = 0,10` y el tablero a 6 cm del frente, el grano queda a 7,5 cm del eje, así que la aproximación y el
retiro se acortan a 2,5 cm (`fila` lo avisa). Antes, el retiro de 8 cm quedaba **detrás** del eje y la base giraba
180° con el grano en la garra. Medir `brazo_x` en el robot.

**Alcance con la base a 36 cm** (calculado, `pi/tests/test_brazo_falso.py`): baja y media siempre; la **alta solo
de frente** (lateral 0, que es justo donde la misión pone el brazo para esa fila) con el frente del robot a ≤ 8 cm
del tablero, o con ±6 cm de lateral a ≤ 6 cm. Con la base a 38 cm alcanza todo hasta 10 cm. La misión usa
`mision.holgura_agarre_m = 0.06` y corrige el acercamiento con el LiDAR hasta ±1 cm; si la fila alta sale "fuera de alcance", revisar `brazo_x/z` o bajar a 0.05.

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
