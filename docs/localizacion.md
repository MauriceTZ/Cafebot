# Localización inicial con el LiDAR

`pi/cafebot/localizacion/inicio.py` → `estimar_inicio(scan)`. Probar: `python3 -m tools.localizar` (LD06 real) o `--sim`.

**Supuesto:** el capitán deja el robot centrado en su sector y **mirando a los árboles**. El
reglamento deja libre la orientación, pero no permite decirle al robot en qué sector parte.

**Cómo funciona:**
1. Prueba las 5 zonas de partida, con desvíos de ±12 cm y ±12°, y elige la pose en que los
   puntos del LiDAR caen sobre el pie de los 3 árboles (siempre están en el mismo lugar).
2. Afina con mínimos cuadrados punto‑a‑segmento.
3. Con la pose conocida, cuenta los puntos en cada posición de alberca y en cada sector del
   beneficiadero (cajas). Si hay albercas, afina de nuevo usando también su cara frontal.

**Resultados en 100 rondas simuladas** (`python3 -m tests.test_inicio 100`). Sorteo como el
reglamento, error humano de colocación de ±6 cm y ±8°, ruido de 1 cm y "público" fuera de la cancha:

| | |
|---|---|
| Zona correcta | 95/100 |
| Pose con error < 3 cm y < 1,5° | 94/100 (mediana 6 mm, 0,1°) |
| Albercas exactas | 79/100: las que faltan están lejos y se ven con pocos puntos → re-escanear antes de cruzar |
| Cajas | Nunca inventa una (99/100). Ve en promedio 1,2 de 2: desde la partida una caja tapa a la otra |
| Tiempo | ~250 ms en un PC |

**Las 5 zonas equivocadas tenían todas confianza < 0,15.** En esas rondas las albercas
tapan los 3 árboles a la altura del LiDAR. Con confianza baja, la misión debe confirmar
con la cámara (los árboles se ven por encima de las albercas) antes de moverse.

**Medidas por confirmar en la cancha real** (`pi/cafebot/mundo/cancha.py`, marcadas `[MEDIR]`):
distancia del tablero del árbol a la línea, espesor del tablero y largo del soporte trasero.
