"""Calibrar con el brazo de verdad las POSES de cada piso del árbol y las posturas de las CANASTAS internas.
Lo que se guarda lo usan tools.ascensor y la misión.

    python3 -m tools.calibrar_cosecha pisos           # altura, pose retraída y adelantada de cada piso
    python3 -m tools.calibrar_cosecha pisos --mega    # además: l toca la línea sola (como tools.ascensor) y ve los IR
    python3 -m tools.calibrar_cosecha canastas        # ángulos del brazo sobre cada canasta interna

PISOS: el robot DERECHO frente al árbol con los IR delanteros TOCANDO la línea negra del árbol (justo en su borde;
con --mega, la tecla l avanza sola hasta tocarla, como tools.ascensor) y la base del brazo frente a una pelota de ese
piso (el brazo está 6,3 cm a la izquierda del centro del robot). Por piso se ajustan la ALTURA, la pose RETRAÍDA
(garra abierta, antes de entrar: no agarra), la ADELANTADA (en la pelota, lista para cerrar), el lado y la
inclinación de la garra. tools.ascensor agarra igual: toca la línea, el brazo va a la retraída, entra derecho hasta
la adelantada y cierra. Si no alcanza, dice con qué inclinaciones sí.
  1 2 3   piso BAJA / MEDIA / ALTA: el brazo va a su pose RETRAÍDA (desde el descanso, como en un agarre)
  e       ENTRAR derecho hasta la pose ADELANTADA (abre la garra antes)      r  RETROCEDER a la retraída (o v)
  ↑ ↓     altura ±5 mm          , .  altura ±1 mm
  i k     la pose donde está el brazo, hacia el árbol / hacia atrás ±5 mm   I K (mayúscula)  ±1 mm
  ← →     lado: izquierda / derecha ±5 mm
  u o     inclinación de la garra ±5°: u = la punta más arriba, o = más abajo
  c       cerrar (dice si tomó algo)    a  abrir
  s       sacar (sube y retrocede)    d  dejarla en la canasta de maduros    h  brazo a descanso
  l       (con --mega) TOCAR la línea con los IR delanteros (el brazo a descanso antes)
  t       GUARDAR el piso (altura, las 2 poses, lado, inclinación) en robot_local.json     ?  estado     q  salir

CANASTAS: el brazo va a la postura de la canasta (pasando por arriba si gira la base); se corrige articulación por
articulación (o a mano, sin torque) y se guarda en el perfil del brazo (pi/brazos/<perfil>.json).
  m n     canasta de MADUROS / SOBREMADUROS (el brazo va ahí)
  1..5    elegir articulación: 1 base  2 hombro  3 codo1  4 codo2  5 muñeca
  ↑ ↓     esa articulación ±2°      [ ]  ±10°
  x       soltar el brazo (SIN torque, sujetarlo) para llevarlo a mano / x otra vez: lo sostiene donde quedó
  a c     abrir (soltar el grano) / cerrar
  p       PROBAR: descanso -> canasta -> abrir -> descanso (como en un agarre)
  t       GUARDAR la postura actual como la de esa canasta        h  brazo a descanso      q  salir
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import traceback

import numpy as np

from cafebot.config import cargar, guardar_local
from cafebot.mundo import cancha

FILAS = {"1": "baja", "2": "media", "3": "alta"}
CANASTAS = {"m": ("deposito_maduro", cancha.MADURO), "n": ("deposito_sobremaduro", cancha.SOBREMADURO)}
ARTS = {"1": "base", "2": "hombro", "3": "codo1", "4": "codo2", "5": "muneca"}


def conectar_brazo(a):
    from tools.brazo import crear
    b = crear(argparse.Namespace(perfil=a.perfil, puerto=a.puerto, banco=False, si=True))
    b.bus.torque(True)
    comp = None
    if b.cfg.compuertas.tipo == "dxl":           # las compuertas cerradas: que lo que caiga en la canasta quede ahí
        try:
            from cafebot.arm.compuertas import DepositosDXL
            comp = DepositosDXL(b.cfg, lambda: b.bus)
            comp.preparar()
        except Exception as e:
            print(f"[compuertas] {e}")
    return b


POSES = ("retraido", "entrando", "adelantado")        # los 3 primeros pasos del plan de agarre (brazo.plan_agarre)
NOMBRE_POSE = {"retraido": "RETRAÍDA", "entrando": "entrando", "adelantado": "ADELANTADA", "sacado": "afuera (sacó)",
               None: "en descanso"}
SEP_MIN_M = 0.015          # la retraída al menos esto antes de la adelantada (si no, plan_agarre no tiene entrada)


def rangos(vals, paso=5) -> str:
    """[40, 45, 50, 100, 105] -> '40–50°, 100–105°'"""
    if not vals:
        return ""
    tramos, ini, ant = [], vals[0], vals[0]
    for v in vals[1:]:
        if v - ant > paso:
            tramos.append((ini, ant))
            ini = v
        ant = v
    tramos.append((ini, ant))
    return ", ".join(f"{a}°" if a == b else f"{a}–{b}°" for a, b in tramos)


def txt_pitch(g) -> str:
    """110 -> '110° (20° hacia abajo)'"""
    return f"{g:.0f}°" + ("" if abs(g - 90) < 0.5 else f" ({abs(g - 90):.0f}° hacia {'abajo' if g > 90 else 'arriba'})")


def conectar_mega(cfg, puerto=None):
    from cafebot.drivers.mega import Mega
    from cafebot.drivers.puertos import resolver
    mega = Mega(resolver(puerto or cfg.puertos.mega, "mega"))
    print("Conectando con el Mega...")
    mega.conectar()
    mega.telemetria(20)
    if cfg.ir.umbrales is not None or cfg.ir.histeresis is not None:
        mega.ir_aplicar(cfg.ir.umbrales, cfg.ir.histeresis)
    return mega


class Pisos:
    """Las poses del brazo en cada piso con los IR delanteros TOCANDO la línea del árbol (ver arriba). Todo en el
    sistema del robot (x adelante desde el centro, y a la izquierda, z desde el suelo)."""

    def __init__(self, b, mega=None, log=print):
        self.b, self.cfg, self.log = b, b.cfg, log
        self.ca = self.cfg.ascensor
        self.mega = mega
        self.sx = 0.5 * (self.cfg.ir.posiciones[0][0] + self.cfg.ir.posiciones[1][0])   # x de los IR delanteros
        self.x_cara = self.sx + self.ca.linea_a_cara_m     # la cara del árbol con los IR en el borde de la línea
        self.fila = "media"
        self.pose = None             # "retraido" / "entrando" / "adelantado" / "sacado" / None (descanso)
        self.plan = None
        self.pitch_plan = None       # la inclinación con que quedó el plan (la que se guarda)
        self.abierta = None

    # ------------------------------------------------------------------ valores del piso
    def x_ad(self, fila=None):
        f = fila or self.fila
        v = getattr(self.ca, f"x_adelantado_{f}_m")
        if v is not None:
            return float(v)
        return self.x_cara - self.ca.grano_delante_m + float(getattr(self.ca, f"dx_{f}_m"))   # sin calibrar

    def x_re(self, fila=None):
        f = fila or self.fila
        v = getattr(self.ca, f"x_retraido_{f}_m")
        return float(v) if v is not None else self.x_ad(f) - self.cfg.brazo.aproximacion_m

    def val(self, que):
        return float(getattr(self.ca, f"{que}_{self.fila}_m"))

    def pitch(self):
        v = getattr(self.ca, f"pitch_{self.fila}_deg")
        return None if v is None else float(v)

    def punto(self):
        """Donde se cierra la garra (pose adelantada)."""
        return np.array([self.x_ad(), self.cfg.mision.brazo_y + self.val("dy"), self.val("z")])

    def claves(self, f=None):
        f = f or self.fila
        return [f"z_{f}_m", f"dy_{f}_m", f"x_adelantado_{f}_m", f"x_retraido_{f}_m", f"pitch_{f}_deg"]

    def fijar_poses(self):
        """Lo que está por defecto (sin calibrar) pasa a valores, para ajustar una pose sin mover la otra."""
        f = self.fila
        ad, re = self.x_ad(), self.x_re()
        setattr(self.ca, f"x_adelantado_{f}_m", round(ad, 4))
        setattr(self.ca, f"x_retraido_{f}_m", round(re, 4))
        if self.pitch() is None and self.pitch_plan is not None:
            setattr(self.ca, f"pitch_{f}_deg", self.pitch_plan)

    # ------------------------------------------------------------------ plan y diagnóstico
    def armar_plan(self):
        p, a = self.punto(), self.x_ad() - self.x_re()
        if a > self.b.a_plano(p)[1] - self.b.cb.r_min_m + 1e-4:
            return None                # plan_agarre la acortaría sin avisar: la retraída no quedaría donde dice
        plan = self.b.plan_agarre(p, aproximacion=a, pitch_deg=self.pitch())
        if plan is not None:
            self.pitch_plan = round(self.b.directa_robot(plan[2][1])[1], 1)
        return plan

    def alternativas(self):
        """[(entrada en m, [inclinaciones °])] que alcanzan el agarre completo con esta altura, lado y adelantada."""
        b, p = self.b, self.punto()
        a_max = b.a_plano(p)[1] - b.cb.r_min_m
        res = []
        for a in sorted({round(self.x_ad() - self.x_re(), 3), 0.06, 0.05, 0.04, 0.03, 0.02}, reverse=True):
            if SEP_MIN_M <= a <= a_max:
                ok = [g for g in range(30, 136, 5) if b.plan_agarre(p, a, g) is not None]
                if ok:
                    res.append((a, ok))
        return res

    @staticmethod
    def mejor_pitch(ok):
        """La inclinación más cercana a 90° con margen (las de al lado también alcanzan)."""
        con_margen = [g for g in ok if g - 5 in ok and g + 5 in ok]
        return min(con_margen or ok, key=lambda g: (abs(g - 90), g))

    def sugerencia(self):
        """(entrada, inclinación): la entrada más larga que alcanza con la garra a ±20° de horizontal (si no hay, la
        de inclinación más cercana a horizontal), o None."""
        alt = [(a, self.mejor_pitch(ok)) for a, ok in self.alternativas()]
        if not alt:
            return None
        cerca = [x for x in alt if abs(x[1] - 90) <= 20]
        return max(cerca) if cerca else min(alt, key=lambda x: (abs(x[1] - 90), -x[0]))

    def por_que(self) -> str:
        """Por qué no alcanza y con qué entrada e inclinación de la garra sí."""
        b, p = self.b, self.punto()
        d, r, _ = b.a_plano(p)
        u = np.array([math.cos(d), math.sin(d), 0.0])
        a = self.x_ad() - self.x_re()
        if a < SEP_MIN_M:
            return f"la retraída tiene que quedar al menos {100 * SEP_MIN_M:.1f} cm antes de la adelantada"
        if a > r - b.cb.r_min_m:
            return (f"la retraída quedaría a menos de {100 * b.cb.r_min_m:.0f} cm del eje de la base del brazo: con la "
                    f"adelantada en x {100 * self.x_ad():.1f}, la retraída llega hasta x {100 * (p - (r - b.cb.r_min_m) * u)[0]:.1f}")
        txt = []
        g = self.pitch()
        if g is not None:
            pts = {"a la adelantada": p, "al medio": p - a / 2 * u, "a la retraída": p - a * u}
            malos = [n for n, q in pts.items() if b.ik(q, g) is None]
            txt.append(f"con la garra a {g:.0f}° no llega {', '.join(malos)}" if malos else
                       f"con la garra a {g:.0f}° llega a las 2 poses pero no puede sacar la pelota (subir y retroceder)")
        alt = self.alternativas()
        if alt:
            txt.append("sí alcanza entrando " + "; ".join(f"{100 * x:.1f} cm con la garra a {rangos(ok)}" for x, ok in alt)
                       + " (i k mueven la retraída, u o la inclinación)")
        else:
            txt.append("a esta altura y con la adelantada ahí no alcanza con ninguna entrada ni inclinación (probar la "
                       "adelantada más adelante, i, o otra altura)")
        return "; ".join(txt)

    # ------------------------------------------------------------------ estado
    def estado(self):
        f = self.fila
        cal = getattr(self.ca, f"x_adelantado_{f}_m") is not None
        ad, re = self.x_ad(), self.x_re()
        g = self.pitch()
        t = [f"[{f.upper()}] brazo: {NOMBRE_POSE.get(self.pose, self.pose)}",
             f"altura {100 * self.val('z'):.1f} cm ({100 * (self.val('z') - self.b.base[2]):+.1f} sobre la base del brazo)",
             f"adelantada x {100 * ad:.1f} cm ({100 * (ad - self.sx):.1f} delante de los IR, ~{100 * (self.x_cara - ad):.1f}"
             f" antes de la cara)", f"retraída x {100 * re:.1f} (entra {100 * (ad - re):.1f} cm)",
             f"lado {100 * self.val('dy'):+.1f} cm",
             "garra " + (txt_pitch(g) if g is not None else f"{self.pitch_plan:.0f}° (la elige la IK)" if self.pitch_plan
                         is not None else "(la elige la IK)")]
        if not cal:
            t.append("SIN CALIBRAR (del reglamento)")
        if self.abierta is False:
            t.append("garra CERRADA")
        ir = self.ir_delanteros()
        if ir is not None:
            t.append("IR delanteros: " + " ".join(f"{n} {'NEGRO' if v else 'blanco'}" for n, v in zip(("DI", "DD"), ir)))
        self.log(" | ".join(t))

    def ir_delanteros(self):
        if self.mega is None:
            return None
        try:
            m = self.mega.ir_estado().mascara
        except Exception:
            return None
        return bool(m & 1), bool(m & 2)

    # ------------------------------------------------------------------ movimientos del brazo
    def ir(self, pose):
        """Lleva el brazo a una pose del piso por el camino del agarre: retraída -> medio -> adelantada."""
        plan = self.armar_plan()
        if plan is None:
            raise RuntimeError(f"[{self.fila}] el brazo no alcanza: {self.por_que()}")
        self.plan = plan
        actual = POSES.index(self.pose) if self.pose in POSES else None
        dest = POSES.index(pose)
        if actual is None:
            pasos = list(range(dest + 1))                  # desde afuera: siempre por la retraída
        elif dest >= actual:
            pasos = list(range(actual + 1, dest + 1)) or [dest]
        else:
            pasos = list(range(actual - 1, dest - 1, -1))
        self.b.parar.clear()
        for i in pasos:
            nombre, obj, t = plan[i]
            if not self.b._mover(obj, t, nombre):
                raise RuntimeError(f"{nombre}: {self.b.ultimo_motivo}")
            self.pose = POSES[i]
        self.estado()

    def garra(self, abrir):
        ang = self.b.garra(abrir=abrir)
        self.abierta = abrir
        return ang

    def descanso(self):
        if self.pose in ("entrando", "adelantado"):
            self.ir("retraido")                            # salir de la pelota antes de plegarse
        self.b.parar.clear()
        if not self.b.ir_plegado(self.cfg.brazo.postura_espera):
            self.log(f"  {self.b.ultimo_motivo}")
        self.pose = None

    def elegir(self, fila):
        if fila == self.fila and self.pose in POSES:
            self.ir("retraido")
            return
        if self.pose is not None:
            self.descanso()                                # al otro piso desde el descanso, como en un agarre
        self.fila, self.plan, self.pitch_plan = fila, None, None
        self.log(f"\nPISO {fila.upper()}: el robot con los IR delanteros sobre la línea y la base del brazo frente a "
                 "una pelota de ese piso")
        self.fijar_inclinacion()
        self.garra(True)
        self.ir("retraido")

    def fijar_inclinacion(self):
        """La garra con la MISMA inclinación en toda la entrada (entra derecho, como la ve aquí lo hará tools.ascensor):
        sin inclinación guardada, la que elige la IK en la pelota. Un piso sin calibrar donde lo del reglamento no
        alcanza así parte de una entrada e inclinación que sí."""
        f = self.fila
        if self.pitch() is None and self.armar_plan() is not None:
            setattr(self.ca, f"pitch_{f}_deg", self.pitch_plan)
            if self.armar_plan() is None:
                setattr(self.ca, f"pitch_{f}_deg", None)
        if getattr(self.ca, f"x_adelantado_{f}_m") is not None or (self.pitch() is not None and self.armar_plan()):
            return
        sug = self.sugerencia()
        if sug is None:
            raise RuntimeError(f"[{f}] el brazo no alcanza este piso con los IR sobre la línea: {self.por_que()}")
        a, g = sug
        ad = self.x_ad()
        setattr(self.ca, f"x_adelantado_{f}_m", round(ad, 4))
        setattr(self.ca, f"x_retraido_{f}_m", round(ad - a, 4))
        setattr(self.ca, f"pitch_{f}_deg", float(g))
        self.log(f"  (sin calibrar, lo del reglamento no alcanza con la garra derecha: parto entrando {100 * a:.1f} cm "
                 f"con la garra a {txt_pitch(g)}; ajustar y guardar con t)")

    def entrar(self):
        if self.abierta is not True:
            self.garra(True)
        self.ir("adelantado")

    def retroceder(self):
        if self.pose is None:
            raise RuntimeError("primero 1 / 2 / 3")
        self.ir("retraido")

    def ajustar(self, que, d):
        """que: z / dy / x (la pose donde está el brazo) / pitch. Si el brazo está en una pose, se vuelve a mover
        ahí; si así no alcanza, queda como estaba."""
        f = self.fila
        antes = {k: getattr(self.ca, k) for k in self.claves()}
        if que == "x":
            self.fijar_poses()
            ad, re = self.x_ad(), self.x_re()
            if self.pose not in ("retraido", "adelantado"):
                ad, re = ad + d, re + d                     # el brazo no está en el piso: se mueven las 2 juntas
                self.log("  (el brazo no está en el piso: muevo las 2 poses; 1 / 2 / 3 para ir)")
            elif self.pose == "adelantado":
                ad += d
                re = min(re, ad - SEP_MIN_M)                # la retraída siempre detrás
            else:
                re = min(re + d, ad - SEP_MIN_M)
            setattr(self.ca, f"x_adelantado_{f}_m", round(ad, 4))
            setattr(self.ca, f"x_retraido_{f}_m", round(re, 4))
        elif que == "pitch":
            self.fijar_poses()
            g0 = self.pitch() if self.pitch() is not None else (self.pitch_plan if self.pitch_plan is not None else 90.0)
            setattr(self.ca, f"pitch_{f}_deg", float(min(135.0, max(30.0, g0 + d))))
        else:
            setattr(self.ca, f"{que}_{f}_m", round(self.val(que) + d, 4))
        if self.pose in ("retraido", "adelantado"):
            plan = self.armar_plan()
            if plan is None:
                motivo = self.por_que()
                for k, v in antes.items():
                    setattr(self.ca, k, v)
                self.armar_plan()
                raise RuntimeError(f"así no alcanza ({motivo}): queda como estaba")
            self.plan = plan
            nombre, obj, t = plan[POSES.index(self.pose)]
            self.b.parar.clear()
            if not self.b._mover(obj, min(t, 0.6), nombre):
                self.log(f"  {nombre}: {self.b.ultimo_motivo}")
        self.estado()

    def cerrar(self):
        ang = self.garra(False)
        self.log(f"  garra cerrada en {ang:.1f}°: " + ("VACÍA (cerró del todo)" if ang > self.cfg.brazo.garra_vacia_deg
                                                       else "¡con algo adentro!"))

    def sacar(self):
        if self.plan is None or self.pose not in POSES:
            raise RuntimeError("primero 1 / 2 / 3 y e")
        if self.pose != "adelantado":
            self.ir("adelantado")
        self.b.parar.clear()
        for nombre, obj, t in self.plan[3:]:
            if not self.b._mover(obj, t, nombre):
                raise RuntimeError(f"{nombre}: {self.b.ultimo_motivo}")
        self.pose = "sacado"
        self.estado()

    def depositar(self):
        b = self.b
        if "deposito_maduro" not in b.posturas:
            raise RuntimeError("no hay postura 'deposito_maduro': guardarla con  python3 -m tools.calibrar_cosecha canastas")
        if self.pose in ("entrando", "adelantado"):
            self.sacar()
        b.parar.clear()
        if not b.ir_plegado("deposito_maduro"):
            self.log(f"  {b.ultimo_motivo}")
        self.garra(True)
        time.sleep(0.3)
        b.ir_plegado(self.cfg.brazo.postura_espera)
        self.pose = None
        self.estado()

    # ------------------------------------------------------------------ base (con --mega)
    def tocar_linea(self):
        """Avanza despacio hasta que los IR delanteros tocan la línea y se endereza con ella, sin seguir de largo (lo
        mismo que tools.ascensor antes de agarrar). Queda ahí."""
        from cafebot.navegacion.bordes import Bordes, Detenido
        if self.mega is None:
            raise RuntimeError("sin Mega: correr con --mega (o poner el robot a mano con los IR en el borde de la línea)")
        if self.pose is not None:
            self.descanso()                                # nunca mover la base con el brazo estirado
        bo = Bordes(self.mega, self.cfg, log=self.log)
        try:
            if any(self.ir_delanteros() or ()):
                self.log("  los IR delanteros ya ven negro: retrocedo 4 cm antes")
                bo.mover("atras", 40, 40)
            res = bo.tocar("adelante", 120, 50, nombre="línea del árbol", enderezar=True, despegar_mm=25, tol_deg=0.7)
        except Detenido:
            res = None
        finally:
            bo.cerrar()
        if res is None:
            self.log("  NO tocó la línea en 12 cm (¿IR calibrados? python3 -m tools.ir)")
        else:
            self.log("  línea tocada" + (f" (estaba torcido {res.giro_deg:+.1f}°)" if res.giro_deg else " (derecho)")
                     + ": así queda el robot cuando tools.ascensor agarra")
        self.estado()

    # ------------------------------------------------------------------ guardar
    def guardar(self):
        if self.pitch() is None:
            raise RuntimeError("no guardo: falta fijar la inclinación de la garra (1 / 2 / 3 la fija; si dice que la "
                               "garra cambia al entrar, elegir una con u / o)")
        if self.armar_plan() is None:
            raise RuntimeError(f"no guardo: el brazo no alcanza ({self.por_que()})")
        self.fijar_poses()
        f = self.fila
        claves = {k: getattr(self.ca, k) for k in self.claves()}
        claves[f"retroceso_{f}_m"] = 0.0           # tools.ascensor agarra con los IR sobre la línea (como aquí)
        setattr(self.ca, f"retroceso_{f}_m", 0.0)
        ruta = guardar_local("ascensor", claves)
        ad, re = self.x_ad(), self.x_re()
        self.log(f"GUARDADO piso {f}: altura {100 * self.val('z'):.1f} cm | adelantada x {100 * ad:.1f} cm "
                 f"({100 * (ad - self.sx):.1f} delante de los IR) | retraída x {100 * re:.1f} cm (entra "
                 f"{100 * (ad - re):.1f}) | lado {100 * self.val('dy'):+.1f} cm | garra {txt_pitch(self.pitch())} -> {ruta} "
                 "(subirlo al repo). tools.ascensor agarra en este piso con los IR delanteros tocando la línea.")


class Canastas:
    def __init__(self, b, log=print):
        self.b, self.cfg, self.log = b, b.cfg, log
        self.nombre, self.cat = CANASTAS["m"]
        self.art = "base"
        self.suelto = False

    def estado(self):
        ang = self.b.angulos_nombre()
        guard = self.b.posturas.get(self.nombre, {})
        self.log(f"[{self.nombre}] articulación elegida: {self.art.upper()} | ahora: " + "  ".join(
            f"{n} {ang[n]:+.1f}°" + (f"({guard[n]:+.1f})" if n in guard and abs(guard[n] - ang[n]) > 0.5 else "")
            for n in ("base", "hombro", "codo1", "codo2", "muneca")) + ("  | SIN TORQUE" if self.suelto else "")
            + "   (entre paréntesis: lo guardado)")

    def ir(self, tecla):
        self.nombre, self.cat = CANASTAS[tecla]
        if self.nombre not in self.b.posturas:
            self.log(f"no hay postura '{self.nombre}' todavía: llevar el brazo a mano (x) sobre la canasta y guardar (t)")
            return
        self.b.parar.clear()
        if not self.b.ir_plegado(self.nombre):
            self.log(f"  no llegó bien: {self.b.ultimo_motivo}")
        self.estado()

    def mover(self, d):
        if self.suelto:
            raise RuntimeError("está SIN torque: x para sostenerlo primero")
        ang = self.b.angulos_nombre()
        self.b.parar.clear()
        if not self.b.mover_articulacion(self.art, ang[self.art] + d, t=0.4):
            self.log(f"  {self.b.ultimo_motivo}")
        self.estado()

    def soltar(self):
        b = self.b
        if not self.suelto:
            input("El brazo va a quedar SIN torque: SUJETARLO con la mano y ENTER... ")
            b.bus.torque(False, b.IDS)
            self.suelto = True
            self.log("SIN torque: llevarlo a mano sobre la canasta y apretar x para que lo sostenga ahí")
        else:
            b.bus.torque(True, b.IDS)
            self.suelto = False
            self.log("con torque: lo sostiene donde quedó")
        self.estado()

    def probar(self):
        b = self.b
        if self.suelto:
            raise RuntimeError("está SIN torque: x primero")
        if self.nombre not in b.posturas:
            raise RuntimeError(f"no hay postura '{self.nombre}': llevar el brazo sobre la canasta y guardarla (t)")
        b.parar.clear()
        b.garra(abrir=False)
        b.ir_plegado(self.cfg.brazo.postura_espera)
        if not b.ir_plegado(self.nombre):
            self.log(f"  yendo a la canasta: {b.ultimo_motivo}")
        b.garra(abrir=True)
        time.sleep(0.4)
        b.ir_plegado(self.cfg.brazo.postura_espera)
        self.log("prueba hecha: descanso -> canasta -> abrir -> descanso")

    def guardar(self):
        if self.suelto:
            self.b.bus.torque(True, self.b.IDS)
            self.suelto = False
        post = self.b.grabar_postura(self.nombre)
        self.log(f"GUARDADA '{self.nombre}' en {self.b.perfil.ruta}: "
                 + " ".join(f"{k} {v:+.1f}°" for k, v in post.items()) + " (subirlo al repo)")


def bucle(modo, obj, b):
    import termios
    import tty
    from tools.cabezal import leer_tecla
    ayuda = __doc__.split("PISOS:" if modo == "pisos" else "CANASTAS:")[1]
    ayuda = ayuda.split("CANASTAS:")[0] if modo == "pisos" else ayuda
    print(ayuda)
    viejo = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        obj.estado()
        while True:
            k = leer_tecla(0.2)
            if not k:
                continue
            if k in ("q", "Q"):
                break
            try:
                if modo == "pisos":
                    acciones = {
                        "arriba": lambda: obj.ajustar("z", 0.005), "abajo": lambda: obj.ajustar("z", -0.005),
                        ".": lambda: obj.ajustar("z", 0.001), ",": lambda: obj.ajustar("z", -0.001),
                        "i": lambda: obj.ajustar("x", 0.005), "k": lambda: obj.ajustar("x", -0.005),
                        "I": lambda: obj.ajustar("x", 0.001), "K": lambda: obj.ajustar("x", -0.001),
                        "izq": lambda: obj.ajustar("dy", 0.005), "der": lambda: obj.ajustar("dy", -0.005),
                        "u": lambda: obj.ajustar("pitch", -5.0), "o": lambda: obj.ajustar("pitch", 5.0),
                        "e": obj.entrar, "r": obj.retroceder, "v": obj.retroceder, "c": obj.cerrar,
                        "a": lambda: obj.log(f"  garra abierta en {obj.garra(True):.1f}°"),
                        "s": obj.sacar, "d": obj.depositar, "t": obj.guardar, "l": obj.tocar_linea,
                        "h": lambda: (obj.descanso(), obj.estado()), "?": obj.estado,
                    }
                    if k in FILAS:
                        obj.elegir(FILAS[k])
                    elif k in acciones:
                        acciones[k]()
                else:
                    acciones = {
                        "arriba": lambda: obj.mover(2), "abajo": lambda: obj.mover(-2),
                        "]": lambda: obj.mover(10), "[": lambda: obj.mover(-10),
                        "x": obj.soltar, "p": obj.probar, "t": obj.guardar,
                        "a": lambda: obj.log(f"  garra abierta en {b.garra(abrir=True):.1f}°"),
                        "c": lambda: obj.log(f"  garra cerrada en {b.garra(abrir=False):.1f}°"),
                        "h": lambda: (b.ir_plegado(b.cfg.brazo.postura_espera), obj.estado()),
                    }
                    if k in CANASTAS:
                        obj.ir(k)
                    elif k in ARTS:
                        obj.art = ARTS[k]
                        obj.estado()
                    elif k in acciones:
                        acciones[k]()
            except RuntimeError as e:
                print(f"  {e}")
            except Exception as e:
                print(f"  ERROR {e}")
                traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("modo", choices=("pisos", "canastas"))
    ap.add_argument("--perfil", default=None, help="otro brazo (pi/brazos/<perfil>.json)")
    ap.add_argument("--puerto", default=None, help="puerto del U2D2 si no lo encuentra solo")
    ap.add_argument("--mega", nargs="?", const="auto", default=None,
                    help="(pisos) conectar el Mega: l toca la línea sola y se ven los IR delanteros")
    a = ap.parse_args()
    b = conectar_brazo(a)
    mega = None
    try:
        if a.modo == "pisos" and a.mega:
            mega = conectar_mega(b.cfg, None if a.mega == "auto" else a.mega)
        obj = Pisos(b, mega) if a.modo == "pisos" else Canastas(b)
        bucle(a.modo, obj, b)
    finally:
        try:
            b.bus.torque(True, b.IDS)
        except Exception:
            pass
        if mega is not None:
            try:
                mega.parar()
            except Exception:
                pass
            mega.cerrar()
        print("El brazo queda con torque donde está (para descansarlo: python3 -m tools.brazo ir descanso).")
        b.cerrar()


if __name__ == "__main__":
    main()
