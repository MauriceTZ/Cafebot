"""Declaración de los cuatro sensores infrarrojos de borde de la arena; deshabilitados por defecto.

Pines BCM provisionales: confirmar el cableado antes de habilitar. No se inicializa GPIO ni se
leen sensores a menos que se cumplan LAS DOS condiciones:
    1) IR_ENABLED = True en este archivo (después de confirmar pines/polaridad), y
    2) el programa se ejecuta con --ir-borde.

Semántica de leer(): True = ese sensor ve el BORDE de la arena; False = suelo normal;
None = sensores deshabilitados / sin leer.

Por ahora los IR solo se leen y se muestran en el HUD: NINGUNA parte de la navegación los usa
todavía (falta decidir qué hace el robot al ver el borde: detenerse, retroceder, marcar el mapa...).
"""
IR_ENABLED = False
IR_BCM_PINS = {
    "frontal_izquierdo": 17,
    "frontal_derecho": 27,
    "trasero_izquierdo": 22,
    "trasero_derecho": 23,
}
# True  -> la salida del módulo baja a LOW cuando detecta el borde (típico de FC-51 / TCRT5000).
# False -> la salida sube a HIGH cuando detecta el borde.
IR_ACTIVE_LOW = True


class SensoresIRBorde:
    def __init__(self, enabled=False):
        self.pedido = bool(enabled)                  # el operador pasó --ir-borde
        self.enabled = bool(enabled and IR_ENABLED)
        self._gpio = None
        if self.enabled:
            try:
                from gpiozero import DigitalInputDevice
                # pull_up=None (pin flotante, el módulo IR ya maneja el nivel) EXIGE indicar
                # active_state; sin él gpiozero lanza PinInvalidState al crear el dispositivo.
                # active_state = nivel lógico del pin cuando HAY borde.
                self._gpio = {n: DigitalInputDevice(pin, pull_up=None, active_state=not IR_ACTIVE_LOW)
                              for n, pin in IR_BCM_PINS.items()}
            except Exception as exc:
                self.cerrar()
                self.enabled = False
                raise RuntimeError(f"No se pudieron inicializar los IR: {exc}") from exc

    def leer(self):
        """{nombre: True (borde) | False (suelo) | None (deshabilitado)}."""
        if not self.enabled or not self._gpio:
            return {n: None for n in IR_BCM_PINS}
        return {n: bool(d.value) for n, d in self._gpio.items()}

    def hay_borde(self):
        """Nombres de los sensores que ven el borde ([] si ninguno o si están deshabilitados)."""
        return [n for n, v in self.leer().items() if v]

    def estado_texto(self):
        if not self.enabled:
            if self.pedido:
                return "IR BORDE  : --ir-borde pedido pero IR_ENABLED=False en sensores_ir.py (confirmar pines)"
            return "IR BORDE  : deshabilitados"
        lec = self.leer()
        return "IR BORDE  : " + " ".join(f"{n.replace('_', '-')}={'BORDE' if v else 'ok'}" for n, v in lec.items())

    def cerrar(self):
        for d in (self._gpio or {}).values():
            try:
                d.close()
            except Exception:
                pass
        self._gpio = None
