"""Misión autónoma de una ronda (niveles de docs/plan_competencia.md).

    N1  botón → ubicarse en la cancha → ir a un árbol → foto → recoger los MADUROS (quedan en el robot, +3)
    N2  + volver al beneficiadero y vaciar la compuerta de maduros en la caja roja (+7)
    N3  + sobremaduros a la caja azul (+5) y más árboles mientras alcance el tiempo

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
from cafebot.mision.interfaces import Boton, Brazo, Depositos, Vision
from cafebot.mundo import cancha
from cafebot.navegacion.geometria import componer, relativo, transformar


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
        self.lecturas = {}            # {(arbol, fila, col): LecturaSoporte}
        self.recogidos = {cancha.MADURO: 0, cancha.SOBREMADURO: 0}
        self.eventos = []
        self.dudoso = False
        self.scan_inicio = None
        self.zonas_caja = set()       # sectores con caja (LiDAR), con o sin color conocido
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

    def esperar(self, timeout=None):
        if self._hilo:
            self._hilo.join(timeout)

    def _correr(self):
        try:
            self.estado = "esperando el botón"
            self.boton.esperar()
            self.t0 = time.monotonic()
            self.log(f"¡PARTIDA! nivel {self.c.nivel}")
            self.mega.seguridad_ir(self.c.freno_ir)
            self.depositos.cerrar_todo()
            self._localizar()
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

    def _actualizar_cajas(self):
        """Dónde están las cajas (LiDAR: inicio + mapa) y de qué color (cámara). Se llama varias veces:
        desde la partida una caja puede tapar a la otra, pero desde el árbol se ven las dos."""
        if len(self.cajas) == 2:
            return
        pts = self.nav.slam.puntos_estaticos()
        if len(pts):
            pc = transformar(np.asarray(pts)[:, :2], self.inicio)
            for z in range(1, 6):
                if z != self.res_inicio.zona and cancha.rect_caja(z).contiene(pc[:, 0], pc[:, 1], 0.04).sum() >= 6:
                    self.zonas_caja.add(z)
        sin_color = sorted(z for z in self.zonas_caja if z not in self.cajas.values())
        if sin_color:
            self.mega.comando(f"X 2 {self.c.camara_cajas_deg:.0f}")
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
        if cambio:                                   # lo que se dedujo de las cajas estaba corrido
            self.zonas_caja, self.cajas = set(), {}
            self._actualizar_cajas()
        return cambio

    def _reserva_entrega(self) -> float:
        """Segundos a guardar para volver y vaciar (tramos a la velocidad de crucero + giros + paradas).
        Si todavía no se sabe el color, se cuenta la caja más lejana y la parada para mirarla."""
        if self.c.nivel < 2:
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
            t += 1.6 * d / v + 20 + self.c.t_vaciado_s + (0 if color in self.cajas else 15)
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
        m = (x > 0.05) & (x < esperado_r[0] + 0.25) & (np.abs(y - esperado_r[1]) < 0.30)
        if m.sum() < 8:
            return None
        x, y = x[m], y[m]
        xf = np.percentile(x, 15)
        cara = x < xf + 0.03
        xc, yc = x[cara], y[cara]
        ancho = float(np.ptp(yc))
        yc_centro = 0.5 * (yc.min() + yc.max()) if ancho > 0.25 else float(esperado_r[1])
        ang = math.atan(np.polyfit(yc, xc, 1)[0]) if len(yc) > 5 and ancho > 0.10 else 0.0
        x_cara = float(np.median(xc))
        return x_cara, yc_centro, -ang, ancho

    def _arbol(self, arbol):
        yt = cancha.centros_arboles()[arbol - 1]
        medio = self.cfg.geometria.largo / 2
        x_foto = cancha.ARBOL_X_FRENTE - self.c.distancia_foto_m - medio
        y_foto = yt - self.cfg.camara.y            # la CÁMARA (no el robot) centrada en el árbol
        self.log(f"árbol {arbol}: voy a la posición de la foto")
        self._ir((x_foto, y_foto), 0.0, f"ir al árbol {arbol}", max_s=90)
        if self.dudoso and self._reubicar():
            ys = cancha.centros_arboles()
            arbol = int(np.argmin(np.abs(ys - self.pose_cancha()[1]))) + 1
            yt = ys[arbol - 1]
            y_foto = yt - self.cfg.camara.y
            self.log(f"en realidad estoy frente al árbol {arbol}")
            self._ir((x_foto, y_foto), 0.0, f"ir al árbol {arbol}", max_s=30)

        # Foto y clasificación de los 16 soportes
        self.mega.comando(f"X 2 {self.c.camara_arbol_deg:.0f}")
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
        for paso in range(3):
            pc = self.pose_cancha()
            esperado = relativo(pc, np.array([cancha.ARBOL_X_FRENTE, yt, 0.0]))
            med = self._medir_tablero(esperado[:2])
            frente = (med[0] if med else esperado[0]) - medio
            avance = frente - self.c.holgura_agarre_m
            if paso == 0:
                self.log(f"árbol {arbol}: tablero a {100 * frente:.1f} cm del frente; avanzo {100 * avance:.1f} cm")
            if abs(avance) < 0.008 or (paso > 0 and med is None):
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
            for fila in self.c.orden_filas:
                for s in sorted((s for s in por_est[e] if s.fila == fila), key=lambda s: -s.y):
                    self._revisar()
                    self._recoger(s)
        self.brazo.descansar()
        self.log(f"árbol {arbol}: listo | depósitos {self.brazo.estado()}")
        # Retroceder antes de planificar (lejos del tablero el A* tiene espacio)
        self._ok(self.mega.avanzar(-250, 150, timeout=15), "retroceso")
        return arbol

    def _foto_arbol(self, arbol, yt):
        """Clasificar los soportes con el tablero medido por el LiDAR (dice dónde cae cada uno en la foto)."""
        pc = self.pose_cancha()
        esperado = relativo(pc, np.array([cancha.ARBOL_X_FRENTE, yt, 0.0]))
        med = self._medir_tablero(esperado[:2])
        return self.vision.clasificar_arbol(arbol, pc, tablero=med[:3] if med else None)

    def _p_robot(self, sop):
        """Centro del grano en el sistema del robot, usando el tablero medido por el LiDAR (más fino que la pose)."""
        pc = self.pose_cancha()
        esperado = relativo(pc, np.array([cancha.ARBOL_X_FRENTE, cancha.centros_arboles()[sop.arbol - 1], 0.0]))
        med = self._medir_tablero(esperado[:2])
        if med is None:
            r = relativo(pc, np.array([sop.x, sop.y, 0.0]))
            return np.array([r[0], r[1], sop.z])
        x_cara, yc, ang, _ = med
        dy = sop.y - cancha.centros_arboles()[sop.arbol - 1]           # incluye COPA_DESPLAZADA_M
        u = np.array([math.cos(ang), math.sin(ang)])                   # normal del tablero (hacia atrás)
        t = np.array([-u[1], u[0]])                                    # a lo largo del tablero (izquierda)
        p = np.array([x_cara, yc]) - cancha.GRANO_DELANTE_M * u + dy * t
        return np.array([p[0], p[1], sop.z])

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
            self.mega.comando(f"X 2 {self.c.camara_cajas_deg:.0f}")
            time.sleep(0.5)
            color = self.vision.color_caja(z, self.pose_cancha())
            self.log(f"caja del sector {z}: {color or 'no sé'}")
            if color:
                self.cajas.setdefault(color, z)
                resto = [w for w in self.zonas_caja if w not in self.cajas.values()]
                if len(self.zonas_caja) == 2 and len(self.cajas) == 1 and len(resto) == 1:
                    self.cajas["AZUL" if color == "ROJA" else "ROJA"] = resto[0]

    def _entregar(self):
        self._actualizar_cajas()
        for color, cat in (("ROJA", cancha.MADURO), ("AZUL", cancha.SOBREMADURO)):
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
            self.depositos.vaciar(k)
            self.recogidos[cat] = 0
            self._ok(self.mega.avanzar(120, 150, timeout=10), "salir de la caja")
