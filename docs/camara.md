# ¿Dónde va la cámara?

Supuesto: Raspberry Pi Camera Module 3 (imx708). Campo visual de la versión estándar ≈66°×41°
(H×V); de la **Wide** ≈102°×67°. Confirmar cuál tienen: es la decisión más importante.

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
