# Misión simplificada SIN LiDAR (rama `navegacion-ir`)

5 nov: el LiDAR y la cámara fallan por la alimentación. Se navega con los 4 IR (QTR-1A) y la odometría del Mega,
tomando como referencias las líneas negras que SE PUEDEN pisar, y se recoge solo la fila del medio. Los granos quedan
en el robot (+3 maduro / +1 sobremaduro): no se va a las canastas.

## Por qué no "bordear la cancha"
Los IR están junto a cada rueda (mismo punto del piso): para que un IR vea una línea, esa rueda tiene que estar
encima. Tocar con las ruedas la línea PERIMETRAL o la de una ALBERCA es reinicio forzado. Entonces el robot se guía
con las líneas interiores, que sí se pueden pisar:
- **borde del beneficiadero** (x = 0,73–0,75 m): rumbo y x al salir;
- **divisiones del beneficiadero** (líneas en y entre sectores): la y de partida (cangrejo de lado hasta tocarla);
- **borde del cultivo** (x = 2,55 m): rumbo y distancia al árbol, ±2 mm y ±0,5° (`Bordes.medir_adelante`, ya probado
  en simulación).
Entre una línea y otra manda la odometría del Mega (encoders), corregida en cada línea.

## Plan de la ronda
1. Partida en el sector Z del beneficiadero, mirando a los árboles. Ir al árbol más cercano a ese sector.
2. Avanzar hasta el borde del beneficiadero y cuadrarse en él con los dos IR delanteros (rumbo = 0, x exacta).
3. Elegir un hueco libre de la franja de albercas (5 posiciones de 72 cm en y; el robot mide 23 cm). Ir por el centro
   del hueco: ~24 cm de margen a cada lado. Las albercas se sacan de la configuración de la ronda, o la cámara mira
   hacia adelante y abajo y busca el azul en cada hueco.
4. Cruzar la zona intermedia en línea recta y cuadrarse en el borde del cultivo (rumbo y distancia al tablero).
5. Cangrejo hasta quedar frente al árbol (y por odometría). La cámara (inclinación, X 2 = 80 = al frente) clasifica
   la fila del medio y afina la posición lateral con las pelotas que ve.
6. Recoger la fila del medio: 6 soportes a 557 mm (centro de la pelota), ±6 / ±18 / ±30 cm del centro del árbol.
   Después, si queda tiempo, la de abajo. La de arriba no.
7. Fin: el programa se cierra con los granos en el robot.

## Pendiente de datos (ver la respuesta en el chat)
Sector de partida, albercas, medidas reales del árbol respecto de la línea, IMU, y qué granos recoger.
