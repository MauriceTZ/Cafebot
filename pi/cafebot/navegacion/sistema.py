"""El "cerebro" de navegación, SIN interfaz: lo usan la interfaz gráfica y (después) la misión.

    nav = SistemaNavegacion(cfg, lidar, mega, giroscopio)
    while ...:
        nav.actualizar()           # telemetría del Mega + barrido nuevo del LiDAR
        nav.pose()                 # pose fusionada (m, m, rad) en el marco del mapa
        nav.ir_a((x, y))           # planificar y ejecutar
        nav.aproximar("ARBOL_1")   # quedar frente a un árbol

Marco del mapa: origen en la pose del robot al arrancar (x adelante, y izquierda).
Módulos (cada uno se puede cambiar por separado):
    slam.py         mapa + ICP          odometria.py    EKF Mega(+giroscopio)+SLAM(+líneas)
    landmarks.py    árboles/cajas/...   seguimiento.py  objetos que se mueven
    planificador.py A* sobre el mapa    ejecutor.py     ruta -> órdenes W/A del Mega
    lineas.py       cruces de líneas negras (sensores IR) ubicados en el mapa y asociados a la cancha
"""
from __future__ import annotations

import math
import time

import numpy as np

from cafebot.config import Config
from cafebot.navegacion.ejecutor import EjecutorRuta
from cafebot.navegacion.landmarks import RegistroLandmarks, detectar
from cafebot.navegacion.lineas import DetectorLineas
from cafebot.navegacion.odometria import OdometriaEKF
from cafebot.navegacion.planificador import Planificador
from cafebot.navegacion.seguimiento import Seguidor
from cafebot.navegacion.slam import SlamLidar


class SistemaNavegacion:
    def __init__(self, cfg: Config, lidar, mega=None, giroscopio=None):
        self.cfg = cfg
        self.lidar = lidar
        self.mega = mega
        self.gyro = giroscopio
        self.reiniciar()

    def reiniciar(self):
        g, n = self.cfg.geometria, self.cfg.navegacion
        if getattr(self, "ejecutor", None) is not None and self.ejecutor.activo:
            self.ejecutor.detener()
        self.slam = SlamLidar(n.mapa_lado_m, n.mapa_res_m, huella=g.huella(), origen_sensor=(g.lidar_x, g.lidar_y))
        self.ekf = OdometriaEKF()
        self.seguidor = Seguidor()
        self.registro = RegistroLandmarks()
        self.planificador = Planificador(self.slam, n.plan_res_m, g.radio + n.margen_obstaculos_m, n.seguridad_m)
        self.ejecutor = EjecutorRuta(self.mega, self.pose, n.vel_mm_s, n.tramo_max_m, n.tol_llegada_m, n.tol_rumbo_deg)
        self.lineas = DetectorLineas(self.cfg.ir)
        self.cancha_en_mapa = None      # el mapa nuevo tiene otro origen: hay que volver a ubicar la cancha
        self.pts_mundo = np.empty((0, 2))
        self.pose_icp = np.zeros(3)
        self.pistas = []
        self.dinamicas = set()
        self.puntos_propios = 0
        self._ult_t_mega = None
        self._ult_scan_t = None
        self._ult_landmarks = 0.0
        self.destino = None

    # ------------------------------------------------------------------ estado
    def pose(self) -> np.ndarray:
        return self.ekf.pose if self.ekf.inicializada else self.slam.pose.copy()

    def _pose_mega(self):
        t = self.mega.tele
        return t.t_ms, np.array([t.x / 1000.0, t.y / 1000.0, t.th])

    def _quieto(self) -> bool:
        """El Mega dice que está detenido (sin metas, sin velocidad) y la telemetría es reciente."""
        if self.mega is None or not self.ekf.inicializada:
            return False
        t = self.mega.tele
        return (t.modo == 0 and abs(t.vx) < 1 and abs(t.wz) < 0.005
                and time.monotonic() - t.recibida < 0.3)

    def fijar_cancha(self, T_mapa_cancha):
        """Pose de la cancha en el marco del mapa (la da la localización inicial / la misión / la tecla I).
        Con ella, los cruces de línea se asocian a las líneas conocidas de la cancha."""
        self.cancha_en_mapa = None if T_mapa_cancha is None else np.asarray(T_mapa_cancha, float)
        self.lineas.fijar_cancha(self.cancha_en_mapa)

    def _procesar_ir(self):
        """Eventos de los sensores de línea → cruces en el mapa (+ corrección del EKF si está activada)."""
        cola = getattr(self.mega, "ir_eventos", None)
        if cola is None:
            return
        while not cola.empty():
            ev = cola.get_nowait()
            _, mega_ahora = self._pose_mega()
            for cruce, s_medio in self.lineas.procesar(ev, self.pose(), mega_ahora):
                if self.cfg.ir.corregir_pose and cruce.linea is not None and self.ekf.inicializada:
                    n, c = self.lineas.recta_en_mapa(cruce.linea)
                    cruce.corregido = self.ekf.corregir_linea(s_medio, n, c, self.cfg.ir.sigma_m)

    def _yaw_gyro(self):
        return self.gyro.leer_yaw() if self.gyro is not None else None

    # ------------------------------------------------------------------ bucle
    def actualizar(self):
        """Llamar seguido (≥ 20 Hz). No bloquea."""
        # 1) Odometría del Mega (telemetría T)
        if self.mega is not None:
            t_ms, pm = self._pose_mega()
            if t_ms and t_ms != self._ult_t_mega:
                self._ult_t_mega = t_ms
                if not self.ekf.inicializada and self.slam.inicializado:
                    self.ekf.inicializar(self.slam.pose, pm, self._yaw_gyro())
                else:
                    self.ekf.predecir(pm, self._yaw_gyro())
            self._procesar_ir()
        # 2) Barrido nuevo del LiDAR -> SLAM, corrección del EKF, seguimiento
        scan = self.lidar.ultimo_scan()
        if scan is not None and scan.t != self._ult_scan_t:
            self._ult_scan_t = scan.t
            self.puntos_propios = scan.puntos_propios
            x, y = scan.xy()
            pts = np.column_stack((x, y))
            mascara = self.slam.mascara_estatica()
            zonas = [(p.pos.copy(), p.radio + 0.2) for p in self.pistas if p.id in self.dinamicas]
            pred = self.ekf.pose if self.ekf.inicializada else None
            # quieto=False: con la referencia sub-celda y la fusión por información ya no hace falta congelar
            # la pose con el robot detenido (deriva medida < 0,4 cm en 15 s), y así se corrige el rumbo tras un giro.
            pose_icp, calidad, info = self.slam.localizar(pts, pred, zonas, quieto=False)
            self.pose_icp = pose_icp
            if self.ekf.inicializada:
                self.ekf.corregir_slam(pose_icp, calidad, info)
                pose_mapa = self.ekf.pose          # se mapea con la pose FUSIONADA
            else:
                pose_mapa = pose_icp
            self.pts_mundo, candidatos = self.slam.integrar(pts, pose_mapa, calidad, zonas)
            self.pistas = self.seguidor.actualizar(self.pts_mundo)
            self.dinamicas = {p.id for p in self.pistas if Seguidor.es_dinamica(p, self.slam, mascara)}
        # 3) Landmarks (1 Hz; cada 3 s con el mapa congelado)
        ahora = time.monotonic()
        if self.slam.inicializado and ahora - self._ult_landmarks > (3.0 if self.slam.congelado else 1.0):
            self._ult_landmarks = ahora
            self.registro.actualizar(detectar(self.slam), expirar=not self.slam.congelado)

    # ------------------------------------------------------------------ acciones
    def planificar(self, destino, origen=None) -> bool:
        self.destino = np.asarray(destino, float)
        return self.planificador.planificar(self.pose()[:2] if origen is None else np.asarray(origen), self.destino)

    def ejecutar(self, rumbo_final: float | None = None) -> bool:
        r = self.planificador.ruta
        if r is None:
            self.ejecutor.estado = "no hay ruta"
            return False
        if self.mega is None:
            self.ejecutor.estado = "ruta calculada; conecta el Mega para ejecutarla"
            return False
        return self.ejecutor.iniciar(r[1:] if len(r) > 1 else r, rumbo_final)

    def ir_a(self, destino, rumbo_final=None) -> bool:
        return self.planificar(destino) and self.ejecutar(rumbo_final)

    def aproximar(self, nombre: str) -> tuple[bool, str]:
        lm = self.registro.buscar(nombre)
        if lm is None:
            return False, f"{nombre} no existe"
        p, rumbo = self.planificador.punto_aproximacion(lm, self.pose()[:2], self.cfg.navegacion.distancia_aprox_m,
                                                        self.cfg.geometria.largo / 2)
        if p is None:
            return False, f"sin acceso libre frente a {nombre}"
        if not self.planificar(p):
            return False, f"sin ruta a {nombre}: {self.planificador.msg}"
        if not self.ejecutar(rumbo):
            return False, self.ejecutor.estado
        return True, f"aproximando a {nombre}"

    def detener(self):
        self.ejecutor.detener()
