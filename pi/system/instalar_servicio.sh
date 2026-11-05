#!/bin/bash
# Instala el servicio de la misión en la Raspberry:   bash pi/system/instalar_servicio.sh [--al-encender]
#   --al-encender  además lo habilita: al prender la Pi la misión arranca sola y espera el botón (la competencia)
#   sudo systemctl start cafebot-mision     una ronda: queda esperando el botón (GPIO 16)
#   journalctl -u cafebot-mision -f         ver lo que hace la misión
#   sudo systemctl stop cafebot-mision      detenerla (como Ctrl+C)
#   sudo systemctl enable cafebot-mision    que arranque solo al encender la Pi: SOLO en la competencia (mientras
#   sudo systemctl disable cafebot-mision   corre, el Mega, el LiDAR y el U2D2 están ocupados: las herramientas
#                                           de calibración no los pueden abrir)
set -e
PI_DIR="$(cd "$(dirname "$0")/.." && pwd)"
USUARIO="${SUDO_USER:-$USER}"
if [ ! -f "$PI_DIR/system/mision.env" ]; then
    cp "$PI_DIR/system/mision.env.ejemplo" "$PI_DIR/system/mision.env"
    echo "Creado $PI_DIR/system/mision.env (opciones de la misión: revisarlas)"
fi
chmod +x "$PI_DIR/system/correr_mision.sh"
sed -e "s|@USUARIO@|$USUARIO|g" -e "s|@PI@|$PI_DIR|g" "$PI_DIR/system/cafebot-mision.service" \
    | sudo tee /etc/systemd/system/cafebot-mision.service > /dev/null
sudo systemctl daemon-reload
echo "Servicio cafebot-mision instalado (usuario $USUARIO, $PI_DIR). Opciones: $PI_DIR/system/mision.env"
if [ "$1" = "--al-encender" ]; then
    sudo systemctl enable cafebot-mision
    echo "Habilitado: al encender la Pi la misión arranca sola y espera el botón."
    echo "Para calibrar con las herramientas: sudo systemctl stop cafebot-mision (o disable para quitarlo)."
fi
echo "Una ronda:  sudo systemctl start cafebot-mision   |   ver:  journalctl -u cafebot-mision -f"
