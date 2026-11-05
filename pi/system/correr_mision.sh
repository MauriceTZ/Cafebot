#!/bin/bash
# Lo ejecuta el servicio cafebot-mision al encender la Pi (o con sudo systemctl start cafebot-mision).
# Activa el .venv, espera (hasta 30 s) que aparezcan el Mega, el LiDAR y el U2D2, y corre la misión con el botón real
# (queda esperando el botón). Las opciones van en pi/system/mision.env (MISION_ARGS). Todo lo que imprime queda en
# journalctl -u cafebot-mision  y en  pi/registros/mision_<fecha>.log
cd "$(dirname "$0")/.." || exit 1
for _ in $(seq 1 30); do
    if [ -e /dev/cafebot-mega ] && [ -e /dev/cafebot-lidar ] && [ -e /dev/cafebot-u2d2 ]; then break; fi
    # sin las reglas udev: basta con que estén los 3 USB-serial (drivers/puertos.py elige cada uno por su chip)
    [ "$(ls /dev/ttyUSB* 2>/dev/null | wc -l)" -ge 3 ] && break
    sleep 1
done
if [ -f .venv/bin/activate ]; then
    # shellcheck disable=SC1091
    source .venv/bin/activate
fi
# rama navegacion-ir: la misión SIN LiDAR (docs/mision_ir.md); MISION_TOOL=tools.mision en mision.env = la de siempre
TOOL="${MISION_TOOL:-tools.mision_ir}"
mkdir -p registros
LOG="registros/mision_$(date +%Y%m%d_%H%M%S).log"
echo "cafebot-mision: $(command -v python3) -m $TOOL --boton gpio $MISION_ARGS  (registro: $LOG)"
set -o pipefail
python3 -m "$TOOL" --boton gpio $MISION_ARGS 2>&1 | tee "$LOG"
