# Sensores de línea: 4 × Pololu QTR-1A

Sirven para detectar las líneas negras de 19 mm de la cancha: el perímetro, los bordes del beneficiadero y del
cultivo y sus divisiones. Con ellas la navegación sabe por dónde pasó el robot y, más adelante, puede corregir
su pose y calibrar el recorrido.

## Conexión (shield 4Motor&9Servo, esquemático `MEGA2560_PinMap` / sección *Analog*)

El conector **Analog IO** del shield tiene una columna por entrada analógica, con 3 pines: **S** (señal),
**V** (5 V) y **G** (GND). Las 16 entradas A0–A15 están en 4 grupos de 4 (J2, J12, J15 y J23 en el esquemático).

| Sensor | Ubicación en el robot | Pin del Mega | QTR-1A → shield |
|---|---|---|---|
| DI (bit 0) | delantero izquierdo | **A8** | OUT → S, VIN → V, GND → G |
| DD (bit 1) | delantero derecho | **A9** | ídem |
| TI (bit 2) | trasero izquierdo | **A10** | ídem |
| TD (bit 3) | trasero derecho | **A11** | ídem |

A8–A11 forman el grupo J15 completo, que está libre. **No usar A1, A4 ni A5**: el shield los conecta al motor M4
(A1 = encoder B, A4/A5 = dirección del TB6612). Si se conectan en otros pines, cambiar `IR_PINES` en
`firmware/mega_cafebot/config.h`, sección 1b.

## Montaje

- **Altura: 3 mm sobre el piso (máximo 6 mm).** Según Pololu, la distancia óptima es 3 mm. Más alto, el
  contraste cae mucho: en la calibración se ve como un "contraste" bajo.
- El QTR-1A da **más voltaje con menos reflexión**: sobre la lona blanca lee bajo (~50–250) y sobre la línea
  negra, alto (~700–1000).
- Medir dónde quedó cada sensor respecto del centro del robot (x hacia adelante, y hacia la izquierda) y anotarlo
  en `pi/robot_local.json`:
  `{"ir": {"posiciones": [[0.13, 0.09], [0.13, -0.09], [-0.13, 0.09], [-0.13, -0.09]]}}` **[MEDIR]**.
  La navegación usa esas posiciones para ubicar cada cruce en el mapa.

## Umbrales: dónde se manejan

Cada sensor tiene un **blanco** y un **negro** medidos, y un **umbral** de decisión (por defecto, el punto medio),
más una **histéresis** común. El sensor pasa a NEGRO cuando su lectura supera `umbral + histéresis/2`, y vuelve a
blanco cuando baja de `umbral − histéresis/2`. Además, debe repetir 2 lecturas seguidas antes de cambiar.

| Dónde | Qué |
|---|---|
| `firmware/mega_cafebot/config.h`, sección 1b | Pines y **valores de fábrica** (`IR_BLANCO_DEF`, `IR_NEGRO_DEF`, `IR_HISTERESIS_DEF`, `IR_CONTRASTE_MIN`, `IR_MUESTRAS_CONFIRMAR`) |
| EEPROM del Mega | La **calibración en uso** (comando `IG`). Sobrevive al apagado y a recargar el firmware |
| `pi/robot_local.json`, sección `"ir"` | `umbrales` y `histeresis` **versionados en el repo**. Si están, `tools.navegacion` y `tools.mision` se los mandan al Mega al conectar. `tools.ir` los escribe al guardar |

## Calibrar (`python3 -m tools.ir`, en `pi/`)

Abre un monitor en vivo con una barra por sensor. La barra muestra la lectura, el umbral (`|`), la banda de
histéresis (`:`), el estado, el blanco y el negro calibrados y el contraste.

1. Robot sobre la lona blanca: **W**.
2. Negro, de una de estas formas:
   - **B**, y otra vez **B** para confirmar: barrido automático. El robot avanza 30 cm sobre una línea que tenga
     delante y vuelve, registrando el mínimo y el máximo de cada sensor. Para que los traseros también la crucen,
     deja la línea a unos 5 cm del frente.
   - **A** (empezar), pasar los sensores sobre la línea a mano o con **↑/↓** (2 cm por tecla), y **A** (terminar).
   - **N** con los 4 sensores sobre una línea o un papel negro, o **1–4** para calibrar uno solo.
3. Revisar el **contraste**: debe ser de al menos 300. Si es menor, bajar el sensor hacia el piso. Bajo 120, el Mega
   rechaza la calibración y avisa.
4. Ajustes finos: **TAB** elige el sensor, **+/-** mueve el umbral de a 10 y **[ ]** cambia la histéresis.
5. **G** guarda en la EEPROM del Mega y en `pi/robot_local.json`. Luego hay que subir ese archivo al repo.

Calibrar en la cancha del torneo antes de la primera ronda: la lona y la luz cambian las lecturas.

## Probar la detección (`python3 -m tools.ir prueba`)

Con el robot perpendicular a una línea que tiene unos centímetros delante, la herramienta cruza la línea a 80, 150
y 250 mm/s, ida y vuelta, y muestra para cada sensor:

- dónde entra y dónde sale;
- el **ancho** visto: ≈ 19 mm si cruza perpendicular;
- la **dispersión** entre pasadas: debe ser menor a 3 mm;
- el **retraso**: cuánto después del borde real avisa el sensor.

El Mega ya corrige ese retraso. Informa la pose del momento del cruce, no la del ciclo de control siguiente. Sin
robot: `python3 -m tools.ir --sim prueba`.

## En la navegación

- El Mega manda `EVT IR mask subida bajada x y th` en cada cambio. `pi/cafebot/navegacion/lineas.py` ubica cada
  sensor en el mapa y arma los **cruces**: una entrada y la salida siguiente del mismo sensor. El punto medio de un
  cruce queda sobre el eje de la línea.
- Con la cancha ubicada (tecla **I** en la interfaz, o automáticamente en la misión), cada cruce se asocia a la
  línea conocida más cercana (`cancha.lineas_cancha()`). Se calcula su **residuo**: a qué distancia de esa línea
  cree la navegación que estaba el sensor.
- En la interfaz, los cuadros en las esquinas del robot son los sensores (negro = ve línea). Los cruces se dibujan
  en el mapa: verde = asociado, naranjo = sin asociar. La línea **LÍNEAS** del panel muestra el residuo medio.
- `ir.corregir_pose = true` (en `robot_local.json`) usa cada cruce asociado para corregir el EKF, solo en la
  dirección perpendicular a la línea. Viene **apagado**: se activa después de validar los residuos en la cancha.
- Freno inmediato (`IS 1 [máscara]`): el robot sí cruza líneas internas (bordes del beneficiadero y del cultivo),
  así que no conviene dejarlo siempre activo.

Pruebas sin robot: `python3 -m tests.test_ir` (firmware simulado + líneas simuladas) y
`firmware/test_host/correr.sh`.

## Pendiente

- Confirmar en el Apéndice A si las albercas tienen líneas propias (`cancha.lineas_cancha` no las incluye).
- Siguiente paso: recorrido inicial (salir del sector cruzando el borde del beneficiadero) y calibración de la ruta
  con los residuos de los cruces.
