# Cosecha de prueba: cámara + LiDAR + brazo (`python3 -m tools.cosecha`, en `pi/`)

Robot quieto frente a UN árbol (puede avanzar y retroceder, y correrse de lado): la cámara encuentra las pelotas, el
LiDAR mide el pie del árbol, se calcula la posición 3D de cada grano y el brazo agarra el elegido (o el más cercano)
y lo deja en la canasta de su categoría.

Código: `pi/cafebot/vision/` — `camara_modelo.py` (cámara estenopeica completa: proyección y rayos), `arbol.py`
(forma del árbol y medida del pie con el LiDAR), `granos.py` (detección, ajuste, proyección inversa, color,
calibración), `sintetica.py` (fotos y LiDAR simulados). Herramienta: `pi/tools/cosecha.py`. Pruebas sin hardware:
`python3 -m tests.test_cosecha`.

## Cómo funciona (sin red neuronal: la escena es conocida)

1. **Detectar.** Píxeles saturados (madera 85–110 de saturación, pelotas 150–190; `camara.sat_min` = 130), manchas
   redondas. El soporte tapa la parte de abajo de cada pelota (la cámara está más abajo que los granos) y el brillo
   deja una muesca: el círculo se ajusta (RANSAC + ajuste geométrico) solo con los puntos del borde que están sobre la
   envolvente convexa y fuera de los tramos rectos. Error en fotos sintéticas: < 1 px en centro y radio, también
   en los casquetes de la fila alta.
2. **Asignar y ajustar.** Cada mancha va al soporte cuya proyección queda más cerca, y se ajustan por mínimos
   cuadrados la pose del árbol (x, y, ángulo) y una corrección de la cámara (inclinación, yaw, roll) con la medida
   del LiDAR como "prior": eso es la fusión cámara + LiDAR. Sin calibrar, la cámara puede corregirse ±3–4° (manda
   el LiDAR en lo que la foto no distingue, p. ej. árbol corrido vs cámara girada); calibrada, ±0,6°.
3. **Proyección inversa.** Rayo de cada pelota ∩ plano vertical de los granos (`p_rayo`). La posición final de cada
   grano es la de su soporte en el árbol ajustado, corregida hacia lo que se vio de esa pelota (sirve para un árbol
   de prueba que no calza exacto con su modelo). Se guarda en el marco del ÁRBOL: si el robot se acerca, el LiDAR
   vuelve a ubicar el pie y con eso cada grano (de cerca la cámara ya no ve el árbol).
4. **Color.** En el círculo inscrito en lo que se ve de la pelota (lejos del soporte y del fondo). Con referencias
   (clic + tecla 1..7): Lab contra las muestras, el clasificador de Pablo, con "no sé". Sin referencias: por matiz,
   provisorio, con confianza 0,6 (menor que la de la misión, 0,7: no se agarra nada hasta enseñar los colores o
   bajar `--confianza`). Un soporte sin mancha de color es VACIO (sin referencias no se distingue de NEGRO).
5. **Brazo.** `BrazoXL430.recoger`: abrir → 6 cm antes → entrar → cerrar (si cierra del todo: no había grano) →
   subir → retroceder → canasta (`deposito_maduro` / `deposito_sobremaduro`) → abrir → espera. Con la postura
   `transporte` grabada, para ir y volver de la canasta primero se pliega sin girar la base, gira plegado y recién
   ahí se estira (no barre la cámara).

## Antes de la primera prueba

| Qué | Cómo |
|---|---|
| Cámara inclinada ~28° hacia arriba | Hoy está casi horizontal: a 55 cm solo se ve la fila baja. A 25 cm del suelo y ~28°, desde 0,55–0,80 m entran los 16 soportes; bajo 0,6 m la fila alta se ve como casquetes finos (sirven, pero mejor ≥ 0,6 m). Con ~20°, sacar la foto desde ~0,8 m. |
| Intrínsecos (recomendado) | `python3 -m tools.camara intrinseca --tablero 9x6 --cuadro 0.025` con un tablero de ajedrez impreso: debe dar fx ≈ 1183 a 1536×864. Si da ~1690, la cámara está en el modo recortado del sensor (el driver ahora pide el campo completo, `MODO_SENSOR`). |
| Medidas en `pi/robot_local.json` | `camara.x/y/z` (lente), `mision.brazo_x/y/z` (eje de la base del brazo; z = base sobre el suelo), `geometria.lidar_x`. |
| Árbol de prueba | `python3 -m tools.cosecha --arbol taller --crear-arbol` y editar `pi/arboles/taller.json` (abajo). |
| Brazo | Perfil calibrado (docs/brazo.md) y posturas `descanso`, `deposito_maduro`, `deposito_sobremaduro` y, opcional, `transporte` (`python3 -m tools.brazo leer`, tecla G). Si una canasta está justo detrás (180°), ampliar el límite de la base (`tools.brazo limites`): de fábrica es ±150°. |
| Exposición | `python3 -m tools.camara exposicion --dist 0.6` con la luz del lugar. |

## Sesión típica

| Paso | Tecla | Qué mirar |
|---|---|---|
| 1. Robot frente al árbol, el frente a ~60 cm del pie | | |
| 2. Medir | ESPACIO | Círculos sobre las pelotas, "error" < 2 px. Mensajes: "la foto y el LiDAR no coinciden", "OJO: calza igual corrido una columna". |
| 3. Calibrar la cámara | C | En 3–4 posiciones del robot (moverlo ±10 cm, ESPACIO + C cada vez): se promedia y se guarda en `robot_local.json`. |
| 4. Enseñar colores | clic + 1..7 | Todas las clases, también NEGRO (6) y VACIO (7, un soporte sin grano). Se guardan en `pi/calibracion/referencias.json`. |
| 5. Acercarse | A | El frente queda a `mision.holgura_agarre_m` (6 cm) del pie; el LiDAR vuelve a ubicar el árbol. |
| 6. Probar sin agarrar | clic + P | La garra queda 6 cm antes del grano: ¿apunta a él? (V vuelve.) |
| 7. Agarrar | G / T | G: el elegido o el seguro y alcanzable más cercano. T: todos (fila alta primero). ESC detiene. |
| 8. Volver y revisar | R + ESPACIO | Los agarrados ya no están. S guarda fotos + análisis en `pi/capturas/`. |

Granos de los costados: ← / → corren el robot 2 cm de lado (cangrejo); el árbol se vuelve a ubicar solo.

**Calibración con el brazo (la más directa, opcional):** con la foto analizada y el brazo fuera de ella, O suelta el
brazo; llevar la garra a mano hasta un grano detectado, clic en ese grano y M; repetir con 4–8 granos en distintas
filas y columnas; N calcula la pose COMPLETA de la cámara (también la posición del lente) respecto del brazo. Así
los errores de montaje del LiDAR o de `brazo_x/y/z` no se suman: con la cámara calibrada con el brazo, lo que la foto
le corrige al LiDAR se sigue aplicando al acercarse (simulado: LiDAR corrido 1,5 cm, agarres a < 5 mm). Con C, en
cambio, la referencia es el LiDAR.

En la imagen: rojo = maduro, azul = sobremaduro, verde = verde, gris = vacío, amarillo = no sé (no se toca),
círculo blanco = mancha detectada y su zona de color, cruz magenta = mancha que no calza con ningún soporte,
"lejos" = el brazo no lo alcanza desde aquí. Abajo a la derecha, vista desde arriba (relleno = alcanzable).

## Alcance del brazo (cinemática del perfil de fábrica, árbol de competencia, robot centrado)

| Base del brazo | Holgura 6 cm: alta / media / baja |
|---|---|
| 25 cm (montaje del 29 oct) | ninguno / los 2 del centro / los 5 |
| 30–33 cm | ninguno / 4 del centro / los 5 |
| 36 cm | solo el del centro / 4 del centro / los 5 |

El brazo mide 31 cm desde el hombro (6,7 cm sobre la base) y la fila alta (centro del grano a 70,7 cm) queda 39 cm
sobre el hombro con la base a 25 cm: **para la fila alta hay que subir la base del brazo a ≥ 36 cm**. Los de los
costados se alcanzan corriéndose de lado (cangrejo, como `mision.estaciones_y`).

## Árbol de prueba (`pi/arboles/<nombre>.json`)

| Campo | Qué es |
|---|---|
| `z_cara` | Altura (m) de la cara de ARRIBA de los soportes de cada fila (`baja`, `media`, `alta`). |
| `columnas_y` | Posición lateral (m, + = izquierda vista desde el robot) de cada soporte, por fila. |
| `copa_y_m` | Corrimiento lateral de todos los soportes respecto del centro de lo que ve el LiDAR. |
| `grano_delante_m` | Cuánto está el centro de los granos DELANTE de la cara que ve el LiDAR (a 5 cm del suelo). Negativo si lo que ve el LiDAR (p. ej. la base con la batería) sobresale más que los granos. |
| `pie_ancho_m` | Ancho de lo que ve el LiDAR a esa altura (si no calza ±40 %, el centro lateral sale de la navegación). |
| `soporte_lado_m`, `soporte_espesor_m`, `centro_sobre_cara_m` | La placa de cada soporte (tapa la parte de abajo de la pelota) y el centro del grano sobre su cara. |
| `tablero_detras_m`, `tablero_ancho_m` | Solo para el dibujo sintético. |

## Sin robot

    python3 -m tools.cosecha --sim                               # árbol sintético; la cámara "real" tiene error de montaje
    python3 -m tools.cosecha --imagen foto.jpg --frente 0.63 --lateral 0.09   # foto guardada, pie puesto a mano
    python3 -m tools.cosecha --imagen foto.jpg --frente 0.63 --texto          # sin ventana

Con la foto del taller del 29 oct (`pi/tests/datos/arbol_taller_29oct.jpg`, cámara casi horizontal a ~63 cm): la
roja queda en `baja-3` (maduro) y la verde en `baja-4`, con 0,3 px de error; la rueda rosada del piso no calza con
ningún soporte. Sin el LiDAR (prior lateral ±5 cm) avisa que el árbol corrido una columna calza igual.

En simulación (`tests.test_cosecha`, cámara con 1–2° de error de montaje): error 3D de los granos desde la foto,
mediana 1,5–3 mm y máximo ~7 mm (sin calibrar, gracias al LiDAR; calibrada, igual o mejor); después de acercarse,
errores de agarre < 2 mm; cada grano a la canasta de su categoría; ESC detiene la secuencia. Tan cerca del árbol
(A) no se saca foto: lo medido de lejos sigue valiendo (y lo que queda fuera de una foto parcial, también).
