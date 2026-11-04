#!/bin/bash
# Lo ejecuta el servicio cafebot-mision. Espera (hasta 30 s) que aparezcan el Mega y el LiDAR después de encender y
# corre la misión con el botón real. Las opciones van en pi/system/mision.env (MISION_ARGS).
cd "$(dirname "$0")/.." || exit 1
for _ in $(seq 1 30); do
    [ -e /dev/cafebot-mega ] && [ -e /dev/cafebot-lidar ] && break
    sleep 1
done
PY=python3
[ -x .venv/bin/python3 ] && PY=.venv/bin/python3
echo "cafebot-mision: $PY -m tools.mision --boton gpio $MISION_ARGS"
exec $PY -m tools.mision --boton gpio $MISION_ARGS
