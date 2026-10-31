# Calibración del brazo del 30 oct

El perfil del brazo quedó en `pi/brazos/principal.json`. Se reconstruyó desde las salidas de `tools.brazo`, porque
la Pi no estaba accesible al subirlo: el historial está resumido y hay que compararlo con el archivo de la Pi.

## Qué cambió

| Articulación | ID | Cero (pasos) | Sentido | Límites |
|---|---|---|---|---|
| base | 9 | 1039 | normal | -76,3° a +352,8° |
| hombro | 8 | 2038 | invertido | -118,0° a +88,1° |
| codo1 | 11 | 3059 | invertido | -112,7° a +122,0° |
| codo2 | 7 | 1535 | invertido | -123,3° a +127,2° |
| muñeca | 10 | 4054 | invertido | -109,3° a +108,5° |
| garra | 4 | 337 | invertido | -10,8° a +59,4° |

- **Muñeca:** ahora es el ID 10; el 5 ya no responde.
- **Base:** el cero anterior estaba girado 180°.
- **Garra:** cerraba hacia ángulos negativos y el código la mandaba a +65°, contra el tope de abrir. Medida a mano:
  abierta -3,6°, con pelota +42,3°, cerrada +46,1°.
- **Límites:** salen de `tools.brazo limites`, salvo hombro mínimo y codo1 máximo, ampliados a mano para incluir las
  posturas grabadas.
- **Posturas:** `descanso`, `trabajo`, `deposito_maduro` (base +176,75°) y `deposito_sobremaduro` (base +168,49°).
  Las canastas se alcanzan girando la base por la IZQUIERDA.
- **Verificación:** `tools.brazo verificar` dio sentido correcto en las 6 articulaciones.

## Lo que no está en git

`pi/robot_local.json` es de cada máquina. Agregar a mano en otra SD (sin reemplazar lo que ya tenga):

```json
{
  "camara": {"fx_px": 1107.0, "fy_px": 1106.6, "cx_px": 767.4, "cy_px": 424.5,
             "dist": [0.08176, -0.21233, -0.00204, 0.00244, 0.1535]},
  "brazo": {"garra_abierta_deg": 0.0, "garra_cerrada_deg": 50.0, "garra_vacia_deg": 44.0},
  "ir": {"umbrales": [727, 926, 659, 715], "histeresis": 60}
}
```

La cámara está redondeada; para recalcularla hay 39 fotos buenas de 41 (RMS 0,33 px). Sin `pi/brazos/principal.json`
el brazo vuelve a los valores de `dxl.py`.

## Si en la Pi falla `git pull`

En la Pi original `pi/brazos/principal.json` existe sin estar en git, y git se niega a pisarlo:

```bash
cd ~/Cafebot/pi && cp brazos/principal.json ~/principal.json.pi && rm brazos/principal.json
git pull
diff brazos/principal.json ~/principal.json.pi    # si difieren, manda el de la Pi
```

## Riesgos

- **Cables de la base:** no hay tope mecánico. Encender con la base hacia adelante y los cables sueltos.
- **Encendido:** `normalizar_centros` supone la base entre -180° y +180°, pero su rango llega a +353°. Encendida más
  allá de +180° se leería una vuelta corrida y podría girar hacia el lado que corta los cables. Sin corregir.
- **Límites por articulación:** hay combinaciones de ángulos que pueden chocar aunque cada una esté dentro de su rango.

## Pendiente

- La garra quedó ~4 cm a la derecha en `punto 0.30 0.0 0.45`, y después se vio "chueca". De perfil, una foto calza con
  los encoders. Falta ver si es el cero de la base (~11°) o `brazo_y`: la base estaba en una esquina de la caja de prueba.
- `punto 0.20 0.0 0.70` (fila alta) y `ir deposito_maduro` no se han probado.
- `P` y `G` en la cosecha no se han vuelto a probar con esta calibración.
- Calibrar la inclinación de la cámara (tecla C) y los colores con las pelotas de PLA.
- Detección de grano: con pelota 42° y vacía 46°, solo 4° de margen.
- Bajo voltaje: la Pi recibió 4,86 V y registró avisos de bajo voltaje.
