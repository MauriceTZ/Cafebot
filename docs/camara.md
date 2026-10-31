# ¿Dónde va la cámara?

> **31 oct:** la cámara va en un cabezal de 2 SG90 (inclinación `X 2`, giro `X 3` desactivado), a ~37 cm del suelo,
> a la derecha del brazo (modelo de Fusion; `camara.cabezal` en la config). El ángulo de `X 2` sale de la inclinación
> pedida (`camara.servo_de_pitch`); la relación se mide con `tools.cosecha` (C con dos ángulos del servo).
>
> **29 oct:** la cámara quedó FIJA al frente del brazo, a 25 cm del suelo. La prueba de cosecha (detectar las
> pelotas, pose del árbol con cámara + LiDAR, posición 3D de cada grano, brazo y canastas) está en
> **docs/cosecha.md** (`python3 -m tools.cosecha`); ahí se calibran también la inclinación, el yaw y el roll.
> Intrínsecos: `python3 -m tools.camara intrinseca` con un tablero de ajedrez (fx ≈ 1183 a 1536×864, campo de 66°).
> El driver pide el modo del sensor de campo COMPLETO (2304×1296): sin eso, a 1536×864 libcamera elige un recorte
> del centro (~49° de campo, fx ≈ 1690) y nada calzaría. Lo de abajo es el método anterior (zonas fijas).

## Probar la cámara en el robot (`python3 -m tools.camara`, en `pi/`)

Código: `pi/cafebot/vision/` — `camara.py` y `clasificador.py` (de Pablo, portados), `proyeccion.py` (dónde cae cada
soporte en la imagen según la geometría) y `vision.py` (clase `VisionCamara`, la que usa la misión).
Sin cámara: `python3 -m tests.test_vision_sintetica`.

**Cómo encuentra los soportes:** ya no hay que marcar 16 posiciones a mano. Con la posición de la cámara en el robot
(`camara.x/y/z`), su inclinación y su distancia focal (`fx_px`) se proyecta cada soporte del Apéndice B sobre la
imagen, usando el tablero que mide el LiDAR. Así, si el robot queda 3–4 cm corrido, los círculos lo siguen. (En la
prueba sintética, sin esa corrección y con 4 cm de error, los 16 soportes salen VACIO: no se toca nada, pero se
pierden los puntos.)

| Paso | Comando | Qué hacer |
|---|---|---|
| 1 | `exposicion --dist 0.5` | Fija exposición y balance de blancos con la luz del lugar (repetir en la cancha). |
| 2 | `foto --frente 0.50` | Robot mirando un árbol de prueba, **cámara justo frente al centro del árbol**, frente del robot a 50 cm del tablero (medido). Cámara inclinada como en la misión (`--mega /dev/ttyUSB1` lo hace con `X 2`). |
| 3 | `zonas --imagen capturas/<foto>.jpg --ajustar` | Ventana: I/K inclinación, W/S altura, +/- fx, J/L lateral, hasta que los círculos caigan sobre los granos. **G** guarda en `robot_local.json`. |
| 4 | `referencias --imagen <foto> --clase ROJO --pos alta-2 media-4` | Enseñar colores, con varias fotos y TODAS las clases: ROJO NARANJO AMARILLO VERDE AZUL NEGRO VACIO (`--pos todas` si todo el árbol es de un color). |
| 5 | `revisar` | Pares de clases de distinta categoría a menos de 20 → se pueden confundir: agregar muestras o subir la luz. |
| 6 | `clasificar --frente 0.50` | Foto nueva → los 16 resultados y la imagen marcada en `pi/capturas/`. |
| 7 | `caja --dist 0.6` | Caja roja o azul delante, cámara hacia abajo → debe decir su color. |

**Distancia de la foto: 50 cm** entre el frente del robot y el tablero (no 65): más lejos el robot quedaría encima de
una alberca que esté detrás del árbol. A 50 cm, con la cámara centrada e inclinada 20°, entran los 16 soportes
(en la simulación con el modelo de cámara). Si alguno queda fuera, la misión se corre 10 cm de lado y saca otra foto.

**Color de las cajas:** la cámara mira hacia adelante y desde la partida no ve las cajas. Antes de entregar, la misión
se para mirando hacia una caja (x = 1,25 m, θ = 180°), baja la cámara y cuenta píxeles rojos/azules donde debería estar
la caja. Con ver una basta: la otra es del otro color.

Cámara del equipo: **Raspberry Pi Camera Module 3 estándar** (imx708), campo visual ≈66°×41° (H×V).
(La versión Wide tiene ≈102°×67°.)

## Decisión (27 oct)

Cámara **fija en el chasis, al lado de la base del brazo, sobre un servo de inclinación**
(servo auxiliar 2 del Mega, comando `X 2 <ángulo>`):
- **Mirando ~17° hacia arriba** para fotografiar el árbol completo desde ~65 cm.
- **Mirando hacia abajo** para las cajas (rojo/azul) y las albercas al navegar.
El cable no se flexiona (solo gira unos grados) y alcanza con el que tienen.

## Fija en el chasis, a la altura de la base del brazo (~35 cm) — RECOMENDADO para esta competencia

Ventajas: cable corto y sin flexión, calibración única, siempre sabe dónde mira.

Lo que hay que resolver:
- **No ve el árbol completo de cerca.** A 25 cm del tablero, el árbol (≈70 cm de ancho y
  filas de 41 a 71 cm de altura) necesita ~110° horizontal. Solución: **sacar la foto a
  ~60–70 cm del árbol**, clasificar los 16 soportes (posiciones conocidas del Apéndice B
  + distancia y ángulo al tablero medidos con el LiDAR) y recién entonces acercarse a
  recoger. A 65 cm, la cámara estándar inclinada ~17° hacia arriba cubre las 3 filas.
- **Las cajas (90 mm) quedan fuera de vista** con la cámara estándar inclinada hacia
  arriba: el rayo más bajo toca el suelo a varios metros. Hay dos opciones:
  1. **Camera Module 3 Wide, horizontal**: ve las 3 filas desde ~65 cm *y* las cajas desde ~40 cm. Una sola cámara, nada que se mueva.
  2. Cámara estándar sobre un **servo de inclinación** (salida libre de la shield, comando `X`): arriba para el árbol, abajo para las cajas y el piso.
- Ponerla **al lado** de la base del brazo, no detrás, para que el brazo no la tape.

## En el brazo (cámara en la mano)

Aporta:
- Mirar a cualquier parte (árbol, cajas, suelo) sin mover la base.
- **Corrección visual final**: centrar el grano entre los dedos justo antes de cerrar.
  Esto compensa errores de calibración del brazo y de la pose de la base. Es la mayor
  ventaja: la garra (~50–55 mm) deja solo ±5 mm de margen sobre un grano de 42 mm.
- Ver cada grano de cerca, con luz propia (LED), lo que reduce el problema de iluminación.

Cuesta:
- El cable plano de la cámara **no está hecho para flexionarse** en 5 articulaciones. Si
  hay que hacerlo, usar una **cámara USB** pequeña (cable redondo y flexible) en vez de la CSI.
- Calibrar la posición de la cámara respecto de la garra (cinemática directa + offset).
- Más lento: cada mirada exige mover el brazo (~1–2 s) y esperar a que se estabilice.
- Tapa parte de la vista con la propia garra.

## Recomendación con 4 días de plazo

Cámara **fija** (idealmente Wide), foto a ~65 cm, clasificación por soporte con el
clasificador Lab de `legacy/vision_pablo` y agarre con coordenadas conocidas. Compensar
la precisión con una garra **más tolerante**: dedos en forma de embudo con ~60 mm de
apertura. Entre granos vecinos hay ~78 mm libres, así que todavía no toca al vecino.
Dejar la cámara en la mano para después de la competencia.
