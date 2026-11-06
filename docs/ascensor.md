# Cosecha por pisos: el brazo como ascensor (`tools.ascensor`)

6 nov. La proyección de los soportes con el modelo de la cámara no calzaba (cabezal girado, cámara torcida), así
que la cámara ahora solo dice **qué** hay. El **dónde** sale de medidas conocidas:

- **Cámara** (`cafebot/vision/pisos.py`): en la foto del árbol busca la superficie del árbol (el color que más hay
  al centro: la copa verde de la cancha o la madera del árbol del taller) y los soportes sobre ella (en el verde, lo
  que no es verde; en la madera, lo gris: borde quemado, hueco y repisa; las sombras conservan el color y no
  cuentan). Los agrupa en los 3 pisos (el de 6 soportes es el del medio; la grilla se acomoda para que ningún
  soporte que debería verse caiga fuera del árbol). En gris, en la foto anotada, todo lo que pudo ser soporte. Las columnas se cuentan de 0, de izquierda a derecha en la
  foto. Para cada soporte dice si tiene pelota y de qué color, por el matiz:
  - rojo, naranjo o amarillo = MADURO;
  - azul = SOBREMADURO;
  - negro solo con `negros`.

  La foto anotada queda en `pi/capturas/*_pisos.jpg` y la original en `*_foto.jpg`.
- **Distancia y rumbo:** los IR delanteros tocan la línea negra del árbol y el robot se endereza con ella. La cara
  del árbol queda a `linea_a_cara_m` (11,9 cm según el reglamento) del borde cercano de la línea.
- **Lado:** lo mide el LiDAR con el ancho de la base del árbol (45 cm). Sin LiDAR, el robot parte centrado y
  después cuenta la odometría.
- **Alturas:** las del reglamento (centro de la pelota: baja 40,7, media 55,7, alta 70,7 cm), ajustables.

## Cómo agarra

Para cada pelota:
1. El robot se aleja de la línea lo que pide ese piso (`retroceso_*`).
2. Se corre en cangrejo hasta tener la pelota **delante del brazo** (la base del brazo no gira).
3. El brazo **sube al piso** con la garra abierta, `brazo.aproximacion_m` antes de la pelota.
4. Entra derecho, cierra, revisa que tomó algo, sube, retrocede y la deja en el robot (`deposito_maduro` /
   `deposito_sobremaduro`).

La fila alta no se alcanza con la base del brazo a ~35 cm.

## La línea del árbol sin pisarla (6 nov)
Los IR delanteros están junto a las ruedas: cuando ven la línea, la rueda ya está en su borde. Por eso:
- La línea se toca **una vez** al llegar, despacio y **sin seguir de largo** (`Bordes.tocar`: al primer toque
  retrocede 2,5 cm, gira hacia el sensor que faltó y vuelve a tocar). Antes seguía hasta 6 cm buscando el segundo
  sensor (las ruedas entraban en la cinta). Después se despega `despegue_m` (2,5 cm).
- Los **cangrejos** se hacen con los IR a `margen_cangrejo_m` (7 cm) de la línea: al girar a ±90° cada rueda se
  corre por la L que la une a su servo. Después avanza **derecho** hasta la distancia del piso (`retroceso_*`).
- El **rumbo** que el cangrejo le cambia a la base (las ruedas la arrastran al girar) lo corrige la IMU (`--imu`:
  un giro de prueba de 6° le enseña el signo). Si hay LiDAR, corrige el lado.
- Volver a tocar la línea cada N cangrejos: `retocar_cada` (0 = nunca).
- Reglamento: reinicio solo por tocar las líneas del **perímetro** o de las **albercas** con las ruedas; la del
  cultivo no está en esa lista (confirmar con los jueces).

## Prueba y error en el robot

```
python3 -m tools.ascensor                          # Mega + LiDAR + brazo + cámara
python3 -m tools.ascensor --sin-lidar --dist 0.55  # sin LiDAR: centrado y derecho, frente a 0,55 m del pie
python3 -m tools.ascensor --sin-brazo              # solo foto + acercamiento + cangrejos
python3 -m tools.ascensor --auto --boton enter     # la ronda completa sola
```

1. Poner 3 o más pelotas de colores. Apretar **f** y revisar la tabla de la terminal y la foto en la pantalla HDMI:
   ¿los 3 pisos y cada pelota en su soporte y con su color? El `H` de cada pelota sirve para ajustar los colores
   (`pisos.COLORES_H`).
2. **a**: se cuadra con el pie, saca la foto y avanza hasta la línea. Debe decir "en la línea: la cara del árbol a
   ~8 cm del frente".
3. **2** (piso del medio) y **← →** hasta una columna con pelota. Con **p** el brazo queda en el piso con la garra
   abierta, antes de la pelota. Mirar si está a la altura de la pelota y en línea con ella, y corregir:
   - **↑ ↓** altura;
   - **j l** lado.

   El brazo se mueve con cada tecla.
4. **e** entra hasta la pelota; **i k** corrige la profundidad. **c** cierra y dice si tomó algo. **s** la saca y
   **d** la deja en el robot.
5. Cuando quede bien, **t** guarda en `robot_local.json` (sección `ascensor`, más `brazo.aproximacion_m`). Subirlo
   al repo.
6. Lo mismo con **1** (piso bajo).
7. **g** hace todo el agarre de la columna elegida y **r** la ronda completa: foto, línea, maduros del medio,
   maduros de abajo, sobremaduros.

Las demás teclas:
- **ESPACIO** para todo;
- **h** lleva el brazo a esperar;
- **v** muestra el estado;
- **+ / -** cambia la aproximación;
- **q** sale.

## Qué medir con huincha
- Del borde de la línea negra (el que está más cerca del robot) a la cara del árbol: `ascensor.linea_a_cara_m`.
- Cuánto sobresale el centro de la pelota de la cara del árbol: `ascensor.grano_delante_m` (3,5 cm).
