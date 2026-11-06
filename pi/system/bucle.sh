#!/bin/bash
# Reejecuta un programa cada vez que termina, bien o con error (excepción, se cayó un USB...), hasta que se
# pida parar el bucle.
#
#   pi/system/bucle.sh python3 -m tools.ascensor --auto --boton gpio --sin-lidar --dist 0.55
#   pi/system/bucle.sh python3 -m tools.mision --brazo --vision camara --nivel 1 --boton gpio
#   pi/system/bucle.sh          # sin nada: lo mismo que el servicio
#                               #   (python3 -m $MISION_TOOL --boton gpio $MISION_ARGS, de pi/system/mision.env)
#
# PARAR EL BUCLE (el programa en curso se detiene ordenado, como con Ctrl+C, y no se vuelve a lanzar):
#   - Ctrl+C en la terminal donde corre
#   - desde otra terminal (otro SSH):   pi/system/bucle.sh --parar
#   - si lo lanza un servicio:          sudo systemctl stop <servicio>   (con KillSignal=SIGINT)
#
# Cada vuelta queda en pi/registros/bucle_<fecha>_<n>.log y un resumen (inicio, código de salida, duración) en
# pi/registros/bucle.log. Si el programa se cae en menos de 5 s (falta un USB, error al partir) espera 5 s antes de
# volver a lanzarlo; si no, 2 s. Solo puede haber un bucle a la vez.
DIR_PI="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAR="/tmp/cafebot-bucle.parar"      # si existe, el bucle no vuelve a lanzar el programa
PID_BUCLE="/tmp/cafebot-bucle.pid"
PID_HIJO="/tmp/cafebot-bucle.hijo"    # el programa que está corriendo ahora

if [ "${1:-}" = "--parar" ]; then
    touch "$PARAR"
    if [ -f "$PID_HIJO" ] && kill -0 "$(cat "$PID_HIJO")" 2>/dev/null; then
        HIJO="$(cat "$PID_HIJO")"
        kill -INT "$HIJO" && echo "El programa en curso se detiene (como con Ctrl+C)..."
        # si en 10 s no terminó (colgado, o ignora Ctrl+C), TERM; y a los 5 s más, KILL
        for i in $(seq 1 15); do
            sleep 1
            kill -0 "$HIJO" 2>/dev/null || break
            [ "$i" = 10 ] && echo "No se detuvo con Ctrl+C: TERM" && kill -TERM "$HIJO" 2>/dev/null
        done
        kill -0 "$HIJO" 2>/dev/null && echo "Tampoco con TERM: KILL" && kill -KILL "$HIJO" 2>/dev/null
    fi
    if [ -f "$PID_BUCLE" ] && kill -0 "$(cat "$PID_BUCLE")" 2>/dev/null; then
        kill -TERM "$(cat "$PID_BUCLE")"
    fi
    echo "El bucle termina: no vuelve a lanzar el programa."
    exit 0
fi

cd "$DIR_PI" || exit 1
exec 9>/tmp/cafebot-bucle.lock
if ! flock -n 9; then
    echo "Ya hay un bucle corriendo (pid $(cat "$PID_BUCLE" 2>/dev/null)). Para pararlo: pi/system/bucle.sh --parar"
    exit 1
fi
echo $$ > "$PID_BUCLE"
rm -f "$PARAR" "$PID_HIJO"

if [ -f .venv/bin/activate ]; then
    # shellcheck disable=SC1091
    source .venv/bin/activate
fi
export PYTHONUNBUFFERED=1             # que la salida llegue al registro al momento

if [ $# -gt 0 ]; then
    CMD=("$@")
else
    if [ -f system/mision.env ]; then
        set -a
        # shellcheck disable=SC1091
        source system/mision.env
        set +a
    fi
    # shellcheck disable=SC2206
    CMD=(python3 -m "${MISION_TOOL:-tools.mision}" --boton gpio ${MISION_ARGS:-})
fi

PARAR_YA=0
# Ctrl+C / TERM: mientras corre el programa, la señal le llega también a él (se detiene ordenado) y bash ejecuta
# esto cuando termina; durante la espera entre vueltas corta la espera al tiro.
trap 'PARAR_YA=1' INT TERM
trap 'rm -f "$PID_BUCLE" "$PID_HIJO"' EXIT

mkdir -p registros
n=0
while :; do
    { [ "$PARAR_YA" = 1 ] || [ -e "$PARAR" ]; } && break
    n=$((n + 1))
    LOG="registros/bucle_$(date +%Y%m%d_%H%M%S)_$n.log"
    echo "=== vuelta $n ($(date '+%F %T')): ${CMD[*]}   [registro: pi/$LOG]" | tee -a registros/bucle.log
    t0=$(date +%s)
    # el programa en primer plano (lee el teclado si lo necesita); tee -i: no se corta con Ctrl+C, así guarda
    # también lo que el programa imprime al detenerse
    # (el envoltorio de python deja Ctrl+C con su efecto normal: si el bucle se lanzó con & desde un script, el
    # programa heredaría Ctrl+C ignorado y no se podría detener ordenado)
    ( echo "$BASHPID" > "$PID_HIJO"
      exec python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' "${CMD[@]}" ) 2>&1 | tee -i "$LOG"
    codigo=${PIPESTATUS[0]}
    rm -f "$PID_HIJO"
    dur=$(( $(date +%s) - t0 ))
    echo "=== vuelta $n terminó: código $codigo, ${dur} s" | tee -a registros/bucle.log
    { [ "$PARAR_YA" = 1 ] || [ -e "$PARAR" ]; } && break
    espera=2
    [ "$dur" -lt 5 ] && espera=5
    echo ">>> Se vuelve a lanzar en ${espera} s.   Para terminar: Ctrl+C  o  pi/system/bucle.sh --parar"
    sleep "$espera" &
    wait $!
    kill $! 2>/dev/null
done
rm -f "$PARAR"
echo "=== bucle terminado ($(date '+%F %T')) después de $n vuelta(s)" | tee -a registros/bucle.log
