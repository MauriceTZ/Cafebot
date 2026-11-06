"""Misión autónoma de una ronda (niveles de docs/plan_competencia.md).

    N1  botón → ubicarse en la cancha → ir a un árbol → foto → recoger los MADUROS (quedan en el robot, +3)
    N2  + volver al beneficiadero y vaciar la compuerta de maduros en la caja roja (+7)
    N3  + sobremaduros a la caja azul (+5) y más árboles mientras alcance el tiempo

Frente al árbol (mision.cosecha): "soportes" = lo de antes (VisionCamara, estaciones laterales, la IK de cada grano);
"pisos" (--cosecha pisos) = el brazo como ASCENSOR (tools.ascensor: la foto dice qué pelota hay en cada piso, los IR
delanteros tocan la línea del árbol antes de cada agarre y el brazo usa las poses calibradas con
tools.calibrar_cosecha). En la simulación (sin cámara de verdad) siempre "soportes".

La misión corre en su propio hilo y solo usa:
    SistemaNavegacion (ir_a, pose)       Mega (A / L / X directos)       LiDAR (medir el tablero)
    Brazo / Vision / Depositos / Boton   (cafebot/mision/interfaces.py: reales o simulados)
Quien la crea debe seguir llamando nav.actualizar() a ≥ 20 Hz (lo hacen tools/mision.py y la interfaz).

Marcos: el MAPA tiene origen en la pose de partida (SLAM); la CANCHA es la del reglamento
(cafebot/mundo/cancha.py). Como el robot no se mueve antes del botón, basta con la pose de partida
en la cancha (localizacion/inicio.py) para pasar de uno a otro.
"""
from __future__ import annotations

import math
import threading
import time
import traceback

import numpy as np

from cafebot.drivers.ld06 import Scan
from cafebot.localizacion.inicio import estimar_inicio
from cafebot.mision.interfaces import Boton, Brazo, Depositos, LecturaSoporte, Vision
from cafebot.mundo import cancha
from cafebot.vision.arbol import cara_recta
from cafebot.navegacion.geometria import componer, envolver, relativo, transformar


MODOS_CAJAS = ("colores", "cercana", "guardar")     # config.ConfigMision.cajas


class Abortar(Exception):
    pass


class Mision:
    def __init__(self, cfg, nav, lidar, mega, brazo: Brazo, vision: Vision, depositos: Depositos,
                 boton: Boton | None = None, log=print):
        self.cfg, self.c = cfg, cfg.mision
        self.nav, self.lidar, self.mega = nav, lidar, mega
        self.brazo, self.vision, self.depositos, self.boton = brazo, vision, depositos, boton or Boton()
        self._log = log
        self.estado = "esperando"
        self.activa = False
        self.terminada = False
        self.t0 = None
        self.inicio = None            # pose de partida en la cancha (x, y, θ)
        self.res_inicio = None
        self.cajas = {}               # {"ROJA": zona, "AZUL": zona}
        if self.c.cajas not in MODOS_CAJAS:
            raise ValueError(f"mision.cajas = {self.c.cajas!r}: debe ser uno de {MODOS_CAJAS}")
        self.lecturas = {}            # {(arbol, fila, col): LecturaSoporte}
        self.recogidos = {cancha.MADURO: 0, cancha.SOBREMADURO: 0}
        self.eventos = []
        self.dudoso = False
        self.scan_inicio = None
        self.zonas_caja = set()       # sectores con caja (LiDAR), con o sin color conocido
        self._ref_linea = None        # (pose en la cancha, pose del Mega) al tocar la línea del cultivo (_pose_linea)
        self._asc = None              # el Ascensor mientras cosecha por pisos (detener() lo para)
        if self.c.cosecha not in ("pisos", "soportes"):
            raise ValueError(f"mision.cosecha = {self.c.cosecha!r}: debe ser 'pisos' o 'soportes'")
        self._cancelar = threading.Event()
        self._hilo = None

    # ------------------------------------------------------------------ utilidades
    def log(self, txt):
        t = self.tiempo()
        linea = f"[{t:6.1f} s] {txt}"
        self.eventos.append(linea)
        self.estado = txt
        self._log(linea)

    def tiempo(self) -> float:
        return 0.0 if self.t0 is None else time.monotonic() - self.t0

    def queda(self) -> float:
        return self.c.duracion_s - self.c.margen_final_s - self.tiempo()

    def _revisar(self):
        if self._cancelar.is_set():
            raise Abortar("cancelada")
        if self.t0 is not None and self.queda() <= 0:
            raise Abortar("se acabó el tiempo")

    def a_mapa(self, p):
        """(x, y[, θ]) de la cancha al marco del mapa."""
        q = np.array([p[0], p[1], p[2] if len(p) > 2 else 0.0])
        return relativo(self.inicio, q)

    def a_cancha(self, p):
        return componer(self.inicio, np.asarray(p, float))

    def pose_cancha(self):
        return self.a_cancha(self.nav.pose())

    def T_mapa_cancha(self):
        """Pose de la cancha en el marco del mapa (para dibujarla en la interfaz)."""
        return None if self.inicio is None else relativo(self.inicio, np.zeros(3))

    def _esperar_scan_nuevo(self, n=2):
        s = None
        for _ in range(n):
            s = self.lidar.esperar_scan(1.5)
        return s

    def _ok(self, r, que):
        if not r.ok:
            raise Abortar(f"{que}: {r.motivo}")

    def _ir(self, p_cancha, rumbo_cancha, que, max_s=60.0, tol=0.05):
        """Planificar en el mapa y esperar a que el ejecutor termine. Si el destino estaba en una zona
        aún sin mapear (tapada por una alberca), el A* lo acerca a lo conocido: se replanifica al llegar."""
        p = self.a_mapa((p_cancha[0], p_cancha[1], rumbo_cancha))
        fin = time.monotonic() + max_s
        for intento in range(4):
            self._revisar()
            if not self.nav.ir_a(p[:2], p[2]):
                self.log(f"{que}: sin ruta ({self.nav.planificador.msg}); reintento")
                time.sleep(1.0)
                continue
            while self.nav.ejecutor.activo:
                if self._cancelar.is_set() or self.queda() <= 0 or time.monotonic() > fin:
                    self.nav.detener()
                    self._revisar()
                    raise Abortar(f"{que}: demasiado lento")
                time.sleep(0.05)
            falta = float(np.hypot(*(self.nav.pose()[:2] - p[:2])))
            if self.nav.ejecutor.estado == "ruta completada" and falta <= tol:
                return True
            self.log(f"{que}: {self.nav.ejecutor.estado}, faltan {100 * falta:.0f} cm; replanifico")
        raise Abortar(f"{que}: no se pudo")

    # ------------------------------------------------------------------ hilo
    def iniciar(self):
        self.activa = True
        self._hilo = threading.Thread(target=self._correr, name="mision", daemon=True)
        self._hilo.start()

    def detener(self):
        self._cancelar.set()
        self.nav.detener()
        self.brazo.detener()                 # el brazo termina el movimiento en curso y no empieza otro
        if self._asc is not None:
            self._asc.parar()                # para el movimiento de la base en curso (cosecha por pisos)

    def esperar(self, timeout=None):
        if self._hilo:
            self._hilo.join(timeout)

    def _correr(self):
        try:
            self.estado = "esperando el botón"
            self.boton.esperar()
            self.t0 = time.monotonic()
            self.log(f"¡PARTIDA! nivel {self.c.nivel}" + ("" if self.c.cajas == "colores" else f" | cajas: {self.c.cajas}"))
            self.mega.seguridad_ir(self.c.freno_ir)
            self.depositos.cerrar_todo()
            self._localizar_reducida() if self.c.reducida else self._localizar()
            if not self.brazo.preparar():
                self.log("el brazo no está disponible: solo navegación")
            visitados = set()
            for _ in range(self.c.max_arboles if self.c.nivel >= 3 else 1):
                pendientes = [a for a in self._orden_arboles() if a not in visitados]
                if not pendientes:
                    break
                if self.queda() < self._reserva_entrega() + 40:
                    self.log("no alcanza el tiempo para otro árbol")
                    break
                visitados.add(self._arbol(pendientes[0]))
            if self.c.nivel >= 2:
                self._entregar()
            self.log("MISIÓN TERMINADA")
        except Abortar as e:
            self.log(f"misión detenida: {e}")
            if self.c.nivel >= 2 and str(e) != "cancelada" and self.queda() > 15 and "entrega" not in str(e):
                try:                              # aunque algo falle, intentar entregar lo que haya
                    self._entregar()
                except Exception as e2:
                    self.log(f"entrega final: {e2}")
        except Exception as e:
            self.log(f"ERROR: {e}")
            traceback.print_exc()
        finally:
            try:
                self.nav.detener()
                self.mega.parar()
                self.brazo.descansar()
            except Exception:
                pass
            self.activa = False
            self.terminada = True

    # ------------------------------------------------------------------ pasos
    def _localizar(self):
        self.log("ubicándome en la cancha (LiDAR)")
        # el SLAM debe estar inicializado (su origen = pose de partida) y la pose quieta
        fin = time.monotonic() + 5
        while not self.nav.ekf.inicializada and time.monotonic() < fin:
            time.sleep(0.05)
        mejores = []
        for _ in range(3):
            s = self._esperar_scan_nuevo(1)
            if s is not None:
                mejores.append(estimar_inicio(s))
        if not mejores:
            raise Abortar("el LiDAR no entrega datos")
        r = max(mejores, key=lambda r: r.confianza)
        self.scan_inicio, self.pose_scan = s, self.nav.pose()
        # El mapa empezó en la pose de partida; si el SLAM ya se movió algo, se compensa.
        self.inicio = componer(np.array([r.x, r.y, r.th]), relativo(self.nav.pose(), np.zeros(3)))
        self.res_inicio = r
        self.nav.fijar_cancha(self.T_mapa_cancha())       # los cruces de línea se asocian a la cancha
        self.log(f"partida: sector {r.zona} (x={r.x:.2f} y={r.y:.2f} θ={math.degrees(r.th):+.1f}°) "
                 f"confianza {r.confianza:.2f} | albercas {r.albercas} | cajas en {r.zonas_con_caja}")
        self.dudoso = r.confianza < 0.15
        if self.dudoso:
            self.log("confianza BAJA (las albercas tapan árboles): voy al árbol que SÍ veo y me reubico allá")
        self.zonas_caja = set(r.zonas_con_caja)
        self._actualizar_cajas()

    def _localizar_reducida(self):
        """Cancha reducida: el robot parte mirando al árbol (sin el resto de la cancha). El LiDAR mide el pie de
        adelante y con eso la "cancha" queda puesta como si fuera el árbol 2 del reglamento."""
        self.log("cancha REDUCIDA: mido el pie del árbol con el LiDAR (sin ubicarme en la cancha)")
        fin = time.monotonic() + 5
        while not self.nav.ekf.inicializada and time.monotonic() < fin:
            time.sleep(0.05)
        med = None
        for _ in range(3):
            med = self._medir_pie_adelante()
            if med is not None:
                break
        if med is None:
            if self.c.reducida_dist_m <= 0:
                raise Abortar("cancha reducida: el LiDAR no ve el pie del árbol adelante (¿a más de 90 cm?)")
            medio = self.cfg.geometria.largo / 2
            med = (self.c.reducida_dist_m + medio, 0.0, 0.0)
            self.log(f"no veo el pie con el LiDAR: supongo el frente a {100 * self.c.reducida_dist_m:.0f} cm, centrado")
        x_cara, yc, ang = med
        th = -ang                                          # el tablero mira a -x en la cancha
        yt = float(cancha.centros_arboles()[1])
        c, s = math.cos(th), math.sin(th)
        r = np.array([cancha.ARBOL_X_FRENTE - (c * x_cara - s * yc), yt - (s * x_cara + c * yc), th])
        self.inicio = componer(r, relativo(self.nav.pose(), np.zeros(3)))
        self.scan_inicio, self.pose_scan = None, self.nav.pose()
        self.res_inicio = type("Partida", (), {"zona": 0, "confianza": 1.0, "albercas": (), "zonas_con_caja": ()})()
        self.dudoso = False
        self.zonas_caja = set()
        self.log(f"pie del árbol a {100 * (x_cara - self.cfg.geometria.largo / 2):.1f} cm del frente, "
                 f"{100 * yc:+.1f} cm de lado, girado {math.degrees(ang):+.1f}°")

    def _medir_pie_adelante(self):
        """(x de la cara, y del centro, ángulo) del pie delante del robot, en su sistema; en una franja angosta
        (las paredes de los costados de una cancha chica quedan fuera). None si no se ve."""
        s = self._esperar_scan_nuevo()
        if s is None:
            return None
        x, y = s.xy()
        m = (x > 0.10) & (x < 1.10) & (np.abs(y) < cancha.ARBOL_ANCHO_PIE / 2 + 0.03)
        if m.sum() < 8:
            return None
        xc, yc = cara_recta(x[m], y[m], banda=0.03)
        if len(xc) < 4:
            return None
        ancho = float(np.ptp(yc))
        ang = math.atan(np.polyfit(yc, xc, 1)[0]) if len(yc) > 5 and ancho > 0.10 else 0.0
        centro = 0.5 * (yc.min() + yc.max()) if ancho > 0.25 else 0.0
        return float(np.median(xc)), float(centro), -ang

    def _ir_directo(self, p_cancha, que):
        """Cancha reducida (sin espacio para el A*): enderezarse, cangrejo de lado y recto adelante/atrás."""
        pc = self.pose_cancha()
        if abs(pc[2]) > math.radians(1.5):
            self._ok(self.mega.girar(-math.degrees(pc[2]), timeout=10), f"{que}: enderezarse")
            time.sleep(0.3)
            pc = self.pose_cancha()
        lado = p_cancha[1] - pc[1]
        if abs(lado) > 0.01:
            self._ok(self.mega.cangrejo(90 if lado > 0 else -90, 1000 * abs(lado), 100, timeout=15), f"{que}: cangrejo")
            time.sleep(0.3)
            pc = self.pose_cancha()
        dx = p_cancha[0] - pc[0]
        if abs(dx) > 0.01:
            self._ok(self.mega.avanzar(1000 * dx, 100, timeout=15), f"{que}: avanzar")
            time.sleep(0.3)

    def _actualizar_cajas(self):
        """Dónde están las cajas (LiDAR: inicio + mapa) y de qué color (cámara). Se llama varias veces:
        desde la partida una caja puede tapar a la otra, pero desde el árbol se ven las dos."""
        if len(self.cajas) == 2 or self.c.reducida:
            return
        pts = self.nav.slam.puntos_estaticos()
        if len(pts):
            pc = transformar(np.asarray(pts)[:, :2], self.inicio)
            for z in range(1, 6):
                if z != self.res_inicio.zona and cancha.rect_caja(z).contiene(pc[:, 0], pc[:, 1], 0.04).sum() >= 6:
                    self.zonas_caja.add(z)
        sin_color = sorted(z for z in self.zonas_caja if z not in self.cajas.values())
        if self.c.cajas != "colores":          # las dos se ven iguales: el color no sirve (ver _entregar)
            self.log(f"cajas en {sorted(self.zonas_caja)} (modo {self.c.cajas}: no miro su color)")
            return
        if sin_color:
            self.mega.comando(f"X 2 {self.cfg.camara.servo_de_pitch(self.cfg.camara.inclinacion_cajas_deg)}")
            for z, color in self.vision.color_cajas(sin_color, self.pose_cancha()).items():
                self.cajas.setdefault(color, z)
        resto = [z for z in self.zonas_caja if z not in self.cajas.values()]
        if len(self.zonas_caja) == 2 and len(self.cajas) == 1 and len(resto) == 1:   # la otra es del otro color
            self.cajas["AZUL" if "ROJA" in self.cajas else "ROJA"] = resto[0]
        self.log(f"cajas en {sorted(self.zonas_caja)}: {self.cajas or 'color desconocido'}")

    def _orden_arboles(self):
        """Primero el más cercano en y. Con la partida dudosa, el que más ve el LiDAR: aunque no se sepa
        CUÁL árbol es, sí se sabe dónde está en el mapa, así que ir hacia él es seguro."""
        ys = cancha.centros_arboles()
        if self.dudoso:
            x, y = self.scan_inicio.xy()
            pc = transformar(np.column_stack((x, y)), self.a_cancha(self.pose_scan))
            n = {a: int(np.sum((np.abs(pc[:, 0] - cancha.ARBOL_X_FRENTE) < 0.06) &
                               (np.abs(pc[:, 1] - ys[a - 1]) < cancha.ARBOL_ANCHO_PIE / 2 + 0.03))) for a in (1, 2, 3)}
            return sorted((1, 2, 3), key=lambda a: -n[a])
        yr = self.pose_cancha()[1]
        return sorted((1, 2, 3), key=lambda a: abs(ys[a - 1] - yr))

    def _reubicar(self):
        """Ya pasadas las albercas el LiDAR ve los tres árboles: se repite la localización de la partida
        con TODO el mapa (que está en el marco de partida) en vez de un solo barrido."""
        pts = self.nav.slam.puntos_estaticos()
        if len(pts) < 30:
            return False
        r = estimar_inicio(Scan(0.0, np.arctan2(pts[:, 1], pts[:, 0]), np.hypot(pts[:, 0], pts[:, 1]),
                                np.zeros(len(pts))))
        self.log(f"reubicación con el mapa: sector {r.zona}, confianza {r.confianza:.2f} "
                 f"(antes sector {self.res_inicio.zona}, {self.res_inicio.confianza:.2f})")
        if r.confianza <= self.res_inicio.confianza:
            return False
        cambio = r.zona != self.res_inicio.zona
        self.inicio = np.array([r.x, r.y, r.th])
        self.nav.fijar_cancha(self.T_mapa_cancha())
        self.res_inicio, self.dudoso = r, r.confianza < 0.15
        self._ref_linea = None
        if cambio:                                   # lo que se dedujo de las cajas estaba corrido
            self.zonas_caja, self.cajas = set(), {}
            self._actualizar_cajas()
        return cambio

    def _reserva_entrega(self) -> float:
        """Segundos a guardar para volver y vaciar (tramos a la velocidad de crucero + giros + paradas).
        Si todavía no se sabe el color, se cuenta la caja más lejana y la parada para mirarla."""
        if self.c.nivel < 2 or self.c.cajas == "guardar":
            return 0.0
        zonas = set(self.zonas_caja) | set(self.cajas.values())
        if not zonas:
            return 40.0
        v = self.cfg.navegacion.vel_mm_s / 1000
        p = self.pose_cancha()
        t = 0.0
        for color in ("ROJA", "AZUL"):
            if color == "AZUL" and self.c.nivel < 3:
                break
            candidatas = [self.cajas[color]] if color in self.cajas else sorted(zonas)
            ys = [cancha.centros_zonas_beneficiadero()[z - 1] for z in candidatas]
            yb = max(ys, key=lambda y: abs(y - p[1]))
            d = abs(p[0] - self.c.x_aproximacion_entrega) + abs(p[1] - yb)
            mirar = 15 if color not in self.cajas and self.c.cajas == "colores" else 0
            t += 1.6 * d / v + 20 + self.c.t_vaciado_s + mirar
            p = np.array([self.c.x_aproximacion_entrega, yb, 0.0])
        return t

    # ------------------------------------------------------------------ árbol
    def _medir_tablero(self, esperado_r):
        """Pie del árbol visto por el LiDAR, en el sistema del robot: (x de la cara, y del centro, ángulo).
        `esperado_r`: centro esperado (x, y) según la pose fusionada. Devuelve None si no se ve bien."""
        s = self._esperar_scan_nuevo()
        if s is None:
            return None
        x, y = s.xy()
        lado = cancha.ARBOL_ANCHO_PIE / 2 + 0.05 if self.c.reducida else 0.30    # cancha chica: paredes a ±30 cm
        m = (x > 0.05) & (x < esperado_r[0] + 0.25) & (np.abs(y - esperado_r[1]) < lado)
        if m.sum() < 8:
            return None
        xc, yc = cara_recta(x[m], y[m], banda=0.03)
        if len(xc) < 4:
            return None
        ancho = float(np.ptp(yc))
        yc_centro = 0.5 * (yc.min() + yc.max()) if ancho > 0.25 else float(esperado_r[1])
        ang = math.atan(np.polyfit(yc, xc, 1)[0]) if len(yc) > 5 and ancho > 0.10 else 0.0
        x_cara = float(np.median(xc))
        return x_cara, yc_centro, -ang, ancho

    def _arbol(self, arbol):
        try:
            return self._arbol_(arbol)
        finally:
            self._ref_linea = None                   # la línea solo vale frente a ESTE árbol

    def _arbol_(self, arbol):
        yt = cancha.centros_arboles()[arbol - 1]
        medio = self.cfg.geometria.largo / 2
        x_foto = cancha.ARBOL_X_FRENTE - self.c.distancia_foto_m - medio
        y_foto = yt - self.cfg.camara.y            # la CÁMARA (no el robot) centrada en el árbol
        self.log(f"árbol {arbol}: voy a la posición de la foto")
        if self.c.reducida:
            self._ir_directo((x_foto, y_foto), f"ir al árbol {arbol}")
        else:
            self._ir((x_foto, y_foto), 0.0, f"ir al árbol {arbol}", max_s=90)
        if self.dudoso and self._reubicar():
            ys = cancha.centros_arboles()
            arbol = int(np.argmin(np.abs(ys - self.pose_cancha()[1]))) + 1
            yt = ys[arbol - 1]
            y_foto = yt - self.cfg.camara.y
            self.log(f"en realidad estoy frente al árbol {arbol}")
            self._ir((x_foto, y_foto), 0.0, f"ir al árbol {arbol}", max_s=30)
        if self._cosecha_pisos():
            return self._arbol_pisos(arbol, yt)

        # Foto y clasificación de los 16 soportes
        self.mega.comando(f"X 2 {self.cfg.camara.servo_de_pitch(self.cfg.camara.inclinacion_arbol_deg)}")
        self.vision.preparar_arbol()
        time.sleep(0.4)
        lect = {l.clave: l for l in self._foto_arbol(arbol, yt)}
        fuera = [k for k, l in lect.items() if l.categoria == "FUERA"]
        sops = {s.clave: s for s in cancha.soportes(arbol)}
        for lado in (1, -1):                       # soportes fuera de la foto: correrse de lado y otra foto
            de_este_lado = [k for k in fuera if (sops[k].y - yt) * lado > 0]
            if not de_este_lado or self.queda() < self._reserva_entrega() + 40:
                continue
            d = self.c.desplazamiento_foto_m
            self._ok(self.mega.cangrejo(90 * lado, 1000 * d, 120, timeout=15), "cangrejo para la foto")
            for l in self._foto_arbol(arbol, yt):
                if l.clave in de_este_lado and l.categoria != "FUERA":
                    lect[l.clave] = l
            self._ok(self.mega.cangrejo(-90 * lado, 1000 * d, 120, timeout=15), "volver al centro")
        lect = list(lect.values())
        for l in lect:
            self.lecturas[l.clave] = l
        objetivo = {l.clave for l in lect if l.categoria in self.c.recoger and l.confianza >= self.c.confianza_min}
        resumen = {}
        for l in lect:
            resumen[l.categoria] = resumen.get(l.categoria, 0) + 1
        self.log(f"árbol {arbol}: visión {resumen} → recoger {len(objetivo)}")
        if not objetivo:
            return arbol

        # Acercarse de frente midiendo el tablero con el LiDAR (y corrigiendo: la fila alta queda al límite
        # del alcance del brazo, así que 1–2 cm de más la dejan fuera)
        self.brazo.descansar()
        self._cuadrar_en_linea(arbol, yt)            # rumbo y distancia exactos con la línea del cultivo (si se ve)
        for paso in range(3):
            frente, fuente = self._medir_frente(yt)
            avance = frente - self.c.holgura_agarre_m
            if paso == 0:
                self.log(f"árbol {arbol}: tablero a {100 * frente:.1f} cm del frente ({fuente}); "
                         f"avanzo {100 * avance:.1f} cm")
            if abs(avance) < 0.008 or (paso > 0 and fuente == "pose"):
                break
            self._ok(self.mega.avanzar(1000 * avance, 100 if abs(avance) > 0.05 else 60, timeout=20), "acercamiento")

        # Estaciones laterales: a cada soporte le toca la estación más cercana
        sops = {s.clave: s for s in cancha.soportes(arbol)}
        por_est = {e: [] for e in self.c.estaciones_y}
        for k in objetivo:
            s = sops[k]
            dy = s.y - yt
            e = min(self.c.estaciones_y, key=lambda e: (abs(e - dy), -e))
            por_est[e].append(s)
        orden_estaciones = sorted((e for e in por_est if por_est[e]), reverse=True)   # de izquierda a derecha
        for e in orden_estaciones:
            self._revisar()
            if self.queda() < self._reserva_entrega() + 8:
                self.log("se acaba el tiempo: dejo el árbol")
                return arbol
            # Alinear el brazo con la estación (cangrejo) según lo que ve el LiDAR
            pc = self.pose_cancha()
            esperado = relativo(pc, np.array([cancha.ARBOL_X_FRENTE, yt, 0.0]))
            med = self._medir_tablero(esperado[:2])
            yc_r = med[1] if med else esperado[1]
            lateral = yc_r + cancha.COPA_DESPLAZADA_M + e - self.c.brazo_y
            if abs(lateral) > 0.01:
                self._ok(self.mega.cangrejo(90 if lateral > 0 else -90, 1000 * abs(lateral), 100, timeout=15), "cangrejo")
                # el cangrejo puede girar o desviar la base: volver a tocar la línea y quedar a la holgura de agarre
                if self.c.linea_por_estacion and self._cuadrar_en_linea(arbol, yt):
                    d = self._frente_tablero(yt) - self.c.holgura_agarre_m
                    if abs(d) > 0.008:
                        self._ok(self.mega.avanzar(1000 * d, 60, timeout=15), "distancia de agarre")
            self._recoger_estacion([s for fila in self.c.orden_filas
                                    for s in sorted((s for s in por_est[e] if s.fila == fila), key=lambda s: -s.y)], yt)
        self.brazo.descansar()
        self.log(f"árbol {arbol}: listo | depósitos {self.brazo.estado()}")
        # Retroceder antes de planificar (lejos del tablero el A* tiene espacio)
        self._ok(self.mega.avanzar(-250, 150, timeout=15), "retroceso")
        return arbol

    # ------------------------------------------------------------------ cosecha por pisos (ascensor)
    def _cosecha_pisos(self) -> bool:
        """mision.cosecha == "pisos" y una cámara de verdad (la simulación cosecha por soportes)."""
        if self.c.cosecha != "pisos":
            return False
        from cafebot.vision.vision import VisionCamara
        if isinstance(self.vision, VisionCamara):
            return True
        self.log("cosecha por pisos sin la cámara de verdad (--vision camara): cosecho por soportes")
        return False

    def _arbol_pisos(self, arbol, yt):
        """Cosecha por PISOS (tools.ascensor, docs/ascensor.md): el LiDAR cuadra el robot con el pie, la foto dice qué
        pelota hay en cada piso y de qué color, los IR delanteros tocan la línea del árbol y, para cada pelota:
        cangrejo hasta tenerla delante del brazo, tocar la línea y el brazo como ascensor con las poses calibradas
        (tools.calibrar_cosecha). La base se mueve con la odometría del Mega (el SLAM la sigue)."""
        from tools.ascensor import Ascensor, Cancelado      # la misma cosecha que se prueba con tools.ascensor
        brazo = self.brazo if hasattr(self.brazo, "plan_agarre") else None      # el brazo real (BrazoXL430)
        if brazo is None:
            self.log(f"árbol {arbol}: sin el brazo real (--brazo): solo foto y acercamiento")
        lidar = None if getattr(self.lidar, "sin_datos", False) else self.lidar
        cam = self.vision.cam
        try:
            cam.enfocar(self.cfg.ascensor.dist_foto_m + self.cfg.geometria.largo / 2 - self.cfg.camara.x)
        except Exception:
            pass
        asc = Ascensor(self.cfg, self.mega, lidar, brazo, cam, log=self.log, usar_linea=self.c.linea_cultivo,
                       revisar_fn=self._revisar)
        rel = relativo(self.pose_cancha(), np.array([cancha.ARBOL_X_FRENTE, yt, 0.0]))
        asc.fijar_arbol(rel[0], rel[1], rel[2])              # donde lo pone la pose en la cancha: el LiDAR lo corrige
        self._asc = asc
        resto = self.c.max_granos - sum(self.recogidos.values()) if self.c.max_granos else 0
        try:
            self.log(f"árbol {arbol}: cosecha por pisos (cuadrarme, foto, línea)")
            asc.acercarse()
            if asc.pisos is None:
                self.log(f"árbol {arbol}: la foto no mostró los pisos: no recojo")
            else:
                asc.ronda(max(resto, 0), categorias=self.c.recoger,
                          seguir_fn=lambda: self.queda() > self._reserva_entrega() + 20)
        except Cancelado:
            raise Abortar("cancelada")
        except RuntimeError as e:
            self.log(f"árbol {arbol}: {e}")
        finally:
            self._asc = None
            for cat, n in asc.recogidos.items():
                self.recogidos[cat] = self.recogidos.get(cat, 0) + n
            try:
                self.mega.seguridad_ir(self.c.freno_ir)
            except Exception:
                pass
        self.log(f"árbol {arbol}: listo | en el robot {self.recogidos}")
        n = sum(self.recogidos.values())
        if 0 < self.c.max_granos <= n:
            raise Abortar(f"prueba: ya recogí {n} grano(s) (--max-granos {self.c.max_granos})")
        self._ok(self.mega.avanzar(-250, 150, timeout=15), "retroceso")      # lejos del árbol el A* tiene espacio
        return arbol

    def _foto_arbol(self, arbol, yt):
        """Clasificar los soportes con el tablero medido por el LiDAR (dice dónde cae cada uno en la foto)."""
        if getattr(self.lidar, "sin_datos", False) and hasattr(self.vision, "ubicar_arbol"):
            return self._foto_arbol_sin_lidar(arbol)
        pc = self.pose_cancha()
        esperado = relativo(pc, np.array([cancha.ARBOL_X_FRENTE, yt, 0.0]))
        med = self._medir_tablero(esperado[:2])
        return self.vision.clasificar_arbol(arbol, pc, tablero=med[:3] if med else None)

    def _foto_arbol_sin_lidar(self, arbol):
        """Sin LiDAR: las pelotas que se ven dicen dónde está el árbol (corrige la pose en la cancha) y con eso se
        clasifica. Si la foto no lo deja claro, no se recoge nada (con el árbol corrido una columna se agarraría el
        grano del lado)."""
        pc = self.pose_cancha()
        tablero, imgs, msg = self.vision.ubicar_arbol(arbol, pc)
        self.log(f"árbol {arbol}: sin LiDAR, la foto: {msg}")
        if tablero is None:
            return [LecturaSoporte(s.clave, "NO_SE", 0.0) for s in cancha.soportes(arbol)]
        # el árbol está en `tablero` (robot) y no donde decía la pose: correr la cancha entera
        from cafebot.vision.vision import tablero_desde_pose
        prior = tablero_desde_pose(arbol, pc)
        c, s_ = math.cos(pc[2]), math.sin(pc[2])
        d = np.array([tablero[0] - prior[0], tablero[1] - prior[1]])
        self.inicio = np.array([self.inicio[0] - (c * d[0] - s_ * d[1]), self.inicio[1] - (s_ * d[0] + c * d[1]),
                                self.inicio[2]])
        self.log(f"árbol {arbol}: corrijo la posición {100 * d[0]:+.1f} cm adelante, {100 * d[1]:+.1f} cm de lado")
        return self.vision.clasificar_arbol(arbol, self.pose_cancha(), tablero=tablero, imagenes=imgs)

    def _p_robot(self, sop):
        """Centro del grano en el sistema del robot, usando el tablero medido por el LiDAR (más fino que la pose).
        Si se tocó la línea del cultivo frente a este árbol, la distancia y el ángulo del tablero salen de ella (de
        cerca es más precisa que el LiDAR) y del LiDAR solo la posición lateral del pie."""
        pl = self._pose_linea()
        pc = pl if pl is not None else self.pose_cancha()
        esperado = relativo(pc, np.array([cancha.ARBOL_X_FRENTE, cancha.centros_arboles()[sop.arbol - 1], 0.0]))
        med = self._medir_tablero(esperado[:2])
        if med is None:
            r = relativo(pc, np.array([sop.x, sop.y, 0.0]))
            return np.array([r[0], r[1], sop.z])
        x_cara, yc, ang, _ = med
        if pl is not None:
            x_cara, ang = (cancha.ARBOL_X_FRENTE - pl[0]) / math.cos(pl[2]), esperado[2]
        dy = sop.y - cancha.centros_arboles()[sop.arbol - 1]           # incluye COPA_DESPLAZADA_M
        u = np.array([math.cos(ang), math.sin(ang)])                   # normal del tablero (hacia atrás)
        t = np.array([-u[1], u[0]])                                    # a lo largo del tablero (izquierda)
        p = np.array([x_cara, yc]) - cancha.GRANO_DELANTE_M * u + dy * t
        return np.array([p[0], p[1], sop.z])

    def _frente_tablero(self, yt) -> float:
        """Distancia del frente del robot a la cara del tablero (línea, LiDAR o, si no, la pose)."""
        return self._medir_frente(yt)[0]

    def _medir_frente(self, yt) -> tuple[float, str]:
        """(distancia del frente del robot a la cara del tablero, de dónde salió: "línea", "LiDAR" o "pose")."""
        medio = self.cfg.geometria.largo / 2
        pl = self._pose_linea()
        if pl is not None:                       # la recta del robot hasta la cara (x = ARBOL_X_FRENTE)
            return (cancha.ARBOL_X_FRENTE - pl[0]) / math.cos(pl[2]) - medio, "línea"
        pc = self.pose_cancha()
        esperado = relativo(pc, np.array([cancha.ARBOL_X_FRENTE, yt, 0.0]))
        med = self._medir_tablero(esperado[:2])
        if med is not None:
            return med[0] - medio, "LiDAR"
        return esperado[0] - medio, "pose"

    # ------------------------------------------------------------------ línea del cultivo (sensores delanteros)
    def _pose_linea(self):
        """Pose en la cancha según la última vez que se tocó la línea del cultivo + la odometría del Mega desde
        entonces (movimientos de pocos cm: precisa). None si no se tocó frente a este árbol."""
        if self._ref_linea is None:
            return None
        P0, M0 = self._ref_linea
        x, y, th = self.mega.pose()
        return componer(P0, relativo(M0, np.array([x / 1000.0, y / 1000.0, th])))

    def _cuadrar_en_linea(self, arbol, yt) -> bool:
        """Frente al árbol: avanzar hasta que DI y DD toquen la línea del cultivo (Bordes.medir_adelante: con los dos
        cruces, la distancia y el ángulo), enderezarse con ella y anotar la pose: x = CULTIVO_X0 − distancia (borde
        cercano de la línea), rumbo 0, y la y de la pose fusionada. Deja esa referencia para _pose_linea (la x y el
        rumbo frente al árbol). False si no la vio o si no calza con lo esperado (se sigue con el LiDAR)."""
        if not self.c.linea_cultivo:
            return False
        from cafebot.navegacion.bordes import DD, DI, Bordes, Detenido
        g = self.cfg.geometria
        sx = 0.5 * (self.cfg.ir.posiciones[DI][0] + self.cfg.ir.posiciones[DD][0])
        frente = self._frente_tablero(yt)
        d = frente + g.largo / 2 - (cancha.ARBOL_X_FRENTE - cancha.CULTIVO_X0) - sx     # de DI/DD a la línea
        if d < 0.025:                                # encima de la línea o pasados: atrás primero
            atras = 0.045 - d
            self._ok(self.mega.avanzar(-1000 * atras, 80, timeout=10), "retroceder antes de la línea")
            frente, d = frente + atras, 0.045
        # Hasta 6 cm más allá de donde debería estar la línea, pero sin ver la línea no quedar a menos de 4,5 cm del
        # tablero (según el LiDAR; el LD06 sobresale ~2,7 cm del frente). Al tocarla, el frente queda a ~8 cm.
        max_m = min(d + 0.06, frente - 0.045)
        if max_m < 0.02:
            return False
        b = Bordes(self.mega, self.cfg, log=self.log)
        try:
            med = b.medir_adelante(1000 * max_m, vel=100 if d > 0.10 else 60)
        except Detenido:
            med = None
        finally:
            b.cerrar()
            self.mega.seguridad_ir(self.c.freno_ir)
        self._ref_linea = None
        if med is None:
            self.log(f"árbol {arbol}: no vi la línea del cultivo con DI/DD (¿IR sin calibrar?): sigo con el LiDAR")
            return False
        if abs(med.angulo) > math.radians(0.5):
            self._ok(self.mega.girar(-math.degrees(med.angulo), timeout=10), "enderezar con la línea")
        time.sleep(0.3)                              # que la pose fusionada se ponga al día
        pc = self.pose_cancha()
        x = cancha.CULTIVO_X0 - med.dist_m
        dx, dth = x - pc[0], envolver(-pc[2])
        if abs(dx) > self.c.linea_max_correccion_m or abs(dth) > math.radians(10):
            self.log(f"árbol {arbol}: la línea no calza con la pose (Δx {100 * dx:+.1f} cm, Δθ "
                     f"{math.degrees(dth):+.1f}°): no la uso")
            return False
        # Solo para el trabajo frente a ESTE árbol (_pose_linea): la pose de la cancha (self.inicio) NO se toca.
        # Un rumbo medido en 22 cm de línea puede tener ±0,5°: frente al árbol es mejor que el LiDAR, pero llevado
        # a la entrega (2–3 m más allá) correría la caja varios cm; ahí manda el SLAM, que ve toda la cancha.
        P0 = np.array([x, pc[1], 0.0])
        xm, ym, thm = self.mega.pose()
        self._ref_linea = (P0, np.array([xm / 1000.0, ym / 1000.0, thm]))
        self.log(f"árbol {arbol}: línea del cultivo con DI/DD: estaba torcido {math.degrees(med.angulo):+.1f}°; la "
                 f"pose fusionada tenía {100 * -dx:+.1f} cm en x y {math.degrees(-dth):+.1f}° de error")
        return True

    def _recoger_estacion(self, pendientes: list, yt):
        """Recoge (en orden) lo que el brazo alcanza desde aquí; para lo que falta, se acerca o se aleja del tablero
        a la distancia de `holguras_agarre_m` desde la que se alcanzan más (la fila baja suele pedir alejarse y la
        alta, acercarse)."""
        probadas = set()
        while pendientes:
            ahora = [s for s in pendientes if self.brazo.alcanzable(self._p_robot(s))]
            for s in ahora:
                self._revisar()
                self._recoger(s)
            pendientes = [s for s in pendientes if s not in ahora]
            if not pendientes or self.queda() < self._reserva_entrega() + 12:
                break
            frente = self._frente_tablero(yt)
            probadas.add(round(frente, 2))
            mejor, n_mejor = None, 0
            for h in self.c.holguras_agarre_m:
                if any(abs(h - q) < 0.015 for q in probadas):
                    continue
                d = frente - h                                   # cuánto hay que avanzar
                n = sum(self.brazo.alcanzable(self._p_robot(s) - np.array([d, 0.0, 0.0])) for s in pendientes)
                if n > n_mejor:
                    mejor, n_mejor = h, n
            if mejor is None:
                break
            probadas.add(round(mejor, 2))
            d = frente - mejor
            self.log(f"  {n_mejor} grano(s) se alcanzan con el tablero a {100 * mejor:.0f} cm: me "
                     f"{'acerco' if d > 0 else 'alejo'} {100 * abs(d):.1f} cm")
            self._ok(self.mega.avanzar(1000 * d, 60, timeout=15), "distancia de agarre")
        for s in pendientes:
            p = self._p_robot(s)
            self.log(f"  {s.fila} col {s.col}: fuera de alcance (x={p[0]:.3f} y={p[1]:+.3f} z={p[2]:.3f})")

    def _recoger(self, sop):
        cat = self.lecturas[sop.clave].categoria
        for intento in (1, 2):
            p = self._p_robot(sop)
            if not self.brazo.alcanzable(p):
                self.log(f"  {sop.fila} col {sop.col}: fuera de alcance (x={p[0]:.3f} y={p[1]:+.3f} z={p[2]:.3f})")
                return False
            if self.brazo.recoger(p, cat):
                self.recogidos[cat] = self.recogidos.get(cat, 0) + 1
                self.log(f"  {sop.fila} col {sop.col}: {cat} recogido ({intento}º intento)")
                n = sum(self.recogidos.values())
                if 0 < self.c.max_granos <= n:
                    raise Abortar(f"prueba: ya recogí {n} grano(s) (--max-granos {self.c.max_granos})")
                return True
            self._revisar()
        self.log(f"  {sop.fila} col {sop.col}: no se pudo, lo dejo")
        return False

    # ------------------------------------------------------------------ entrega
    def _mirar_cajas(self, color_buscado):
        """La cámara mira hacia adelante: para saber de qué color es una caja hay que pararse frente a ella.
        Basta con ver una: la otra es del otro color."""
        y_robot = self.pose_cancha()[1]
        ys = cancha.centros_zonas_beneficiadero()
        for z in sorted((z for z in self.zonas_caja if z not in self.cajas.values()), key=lambda z: abs(ys[z - 1] - y_robot)):
            if color_buscado in self.cajas or len(self.cajas) == 2 or self.queda() < self._reserva_entrega() + 25:
                return
            yb = cancha.centros_zonas_beneficiadero()[z - 1]
            self.log(f"miro de qué color es la caja del sector {z}")
            self._ir((self.c.x_vista_cajas, yb), math.pi, f"mirar la caja {z}", max_s=60)
            self.mega.comando(f"X 2 {self.cfg.camara.servo_de_pitch(self.cfg.camara.inclinacion_cajas_deg)}")
            time.sleep(0.5)
            color = self.vision.color_caja(z, self.pose_cancha())
            self.log(f"caja del sector {z}: {color or 'no sé'}")
            if color:
                self.cajas.setdefault(color, z)
                resto = [w for w in self.zonas_caja if w not in self.cajas.values()]
                if len(self.zonas_caja) == 2 and len(self.cajas) == 1 and len(resto) == 1:
                    self.cajas["AZUL" if color == "ROJA" else "ROJA"] = resto[0]

    def _asignar_por_cercania(self):
        """mision.cajas = "cercana": las dos cajas se ven iguales y no hay cómo saber cuál cuenta como roja. Se apuesta
        (50 %) a que la más cercana es la de los maduros y la otra la de los sobremaduros: la misma probabilidad que
        al azar, pero con el viaje más corto. Si el LiDAR vio cajas en más de 2 sectores, no se apuesta."""
        if self.cajas:
            return
        ys = cancha.centros_zonas_beneficiadero()
        y = self.pose_cancha()[1]
        orden = sorted(self.zonas_caja, key=lambda z: (abs(ys[z - 1] - y), z))
        if len(orden) > 2:
            self.log(f"veo cajas en los sectores {sorted(orden)}: no sé cuáles son, los granos se quedan en el robot")
            return
        if orden:
            self.cajas["ROJA"] = orden[0]
        if len(orden) == 2:
            self.cajas["AZUL"] = orden[1]
        if orden:
            self.log(f"cajas iguales: apuesto a que la del sector {orden[0]} (la más cercana) es la de los maduros"
                     + (f" y la del {orden[1]}, la de los sobremaduros" if len(orden) == 2 else ""))

    def _entregar(self):
        if self.c.cajas == "guardar":
            self.log("cajas iguales (modo guardar): no entrego, los granos se quedan en el robot (+3 / +1)")
            return
        self._actualizar_cajas()
        if self.c.cajas == "cercana":
            self._asignar_por_cercania()
        for color, cat in (("ROJA", cancha.MADURO), ("AZUL", cancha.SOBREMADURO)):
            if self.c.cajas != "colores":
                break
            if self.recogidos.get(cat, 0) and color not in self.cajas and (color == "ROJA" or self.c.nivel >= 3):
                self._mirar_cajas(color)
        pendientes = [("ROJA", 0, cancha.MADURO)]
        if self.c.nivel >= 3:
            pendientes.append(("AZUL", 1, cancha.SOBREMADURO))
        for color, k, cat in pendientes:
            if self.recogidos.get(cat, 0) == 0:
                continue
            if color not in self.cajas:
                self.log(f"no sé dónde está la caja {color}: los {cat} se quedan en el robot")
                continue
            self._revisar()
            yb = cancha.centros_zonas_beneficiadero()[self.cajas[color] - 1]
            self.log(f"entrega: voy a la caja {color} (sector {self.cajas[color]})")
            self._ir((self.c.x_aproximacion_entrega, yb), 0.0, f"entrega {color}", max_s=90)
            # De reversa hasta que la salida quede sobre la caja
            atras = self.pose_cancha()[0] - self.c.x_entrega
            if atras > 0.01:
                self._ok(self.mega.avanzar(-1000 * atras, 100, timeout=15), "reversa a la caja")
            pc = self.pose_cancha()
            self.log(f"entrega: abro la compuerta {k} en x={pc[0]:.2f} y={pc[1]:.2f} (caja en y={yb:.2f})")
            if self.depositos.vaciar(k) is False:
                self.log(f"entrega: la compuerta {k} no se abrió bien: los {cat} pueden seguir en el robot")
            else:
                self.recogidos[cat] = 0
            self._ok(self.mega.avanzar(120, 150, timeout=10), "salir de la caja")
