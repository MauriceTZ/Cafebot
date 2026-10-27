#!/bin/bash
# Carga un firmware en la ESP32 conectada por USB a la Pi.
# Uso:  ./cargar_firmware.sh prueba_base     (prueba rueda por rueda + encoders)
#       ./cargar_firmware.sh navegacion      (firmware de navegación, para teleop.py)
# Los .bin están en firmware_bin/ (se compilan en el PC y se copian con scp).
set -e
cd "$(dirname "$0")"
NOMBRE="${1:-}"
BIN="firmware_bin/${NOMBRE}.bin"
if [ -z "$NOMBRE" ] || [ ! -f "$BIN" ]; then
  echo "Uso: $0 <nombre>   (disponibles: $(ls firmware_bin 2>/dev/null | sed 's/\.bin//' | tr '\n' ' '))"
  exit 1
fi
PUERTO=$(ls /dev/serial/by-id/* 2>/dev/null | head -n 1)
if [ -z "$PUERTO" ]; then
  echo "No hay ninguna ESP32 conectada por USB."
  exit 1
fi
echo "Cargando $BIN en $PUERTO ..."
python3 -m esptool --port "$PUERTO" --baud 460800 write-flash 0x0 "$BIN" | grep -E "Wrote|verified|rror"
echo "Listo: $NOMBRE cargado."
