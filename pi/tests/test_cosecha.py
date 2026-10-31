"""Cosecha (cámara + LiDAR + brazo) sin hardware:   python3 -m tests.test_cosecha

  1) Detección: el círculo de cada pelota con la parte de abajo tapada por su soporte y el brillo (fotos sintéticas)
  2) Pose del árbol + corrección de la cámara (fusión con el LiDAR) y proyección inversa: error 3D de cada grano
  3) Calibración de la cámara: con el árbol + LiDAR, y con el brazo (cinemática directa = PnP)
  4) Colores: por matiz y con referencias (negro y vacío incluidos)
  4b) Soporte vacío cuya madera el clasificador confunde con AMARILLO (30 oct): sin pelota detectada sale VACIO
  5) Foto REAL del árbol del taller (29 oct): la roja y la verde en su soporte; la rueda rosada del piso, en ninguno
  6) Intrínsecos (fx, fy, cx, cy) con fotos sintéticas de un tablero de ajedrez
  7) Sesión completa simulada (tools.cosecha): medir, calibrar, acercarse, agarrar todo lo seguro, cada grano a la
     canasta de su categoría, el camino por la postura 'transporte' y ESC
  7b) Cabezal de la cámara (servo X 2): I/K mueve el servo, el lente se mueve con la inclinación y C hecho con dos
      ángulos del servo mide la relación servo -> inclinación (la que usa la misión)
  7c) Sin Mega: A pide acercar el árbol a mano; con un error de 1,5 cm, U lo vuelve a ubicar con el LiDAR
  8) LiDAR mal montado (corrido 1,5 cm): con la cámara calibrada con el brazo (M + N) igual se agarra bien
"""
import math
import pathlib
import tempfile

import cv2
import numpy as np

from tests import bus_dxl_falso

memoria = bus_dxl_falso.instalar()

from cafebot.config import cargar  # noqa: E402
from cafebot.vision import clasificador as cl  # noqa: E402
from cafebot.vision import granos as G  # noqa: E402
from cafebot.vision.arbol import ModeloArbol, medir_pie  # noqa: E402
from cafebot.vision.camara_modelo import CamaraPinhole  # noqa: E402
from cafebot.vision.sintetica import LidarPie, foto_arbol  # noqa: E402

DATOS = pathlib.Path(__file__).resolve().parent / "datos"
CLASES5 = ["ROJO", "NARANJO", "AMARILLO", "VERDE", "AZUL"]


def colores_todos(modelo):
    return {ModeloArbol.nombre_de(c): CLASES5[i % 5] for i, (c, _, _) in enumerate(modelo.soportes())}


def pose_a(d_camara, lateral=0.0, ang_deg=0.0, modelo=None):
    """Pose del pie con el plano de los granos a d_camara del lente (cámara en x = 0,15)."""
    return (0.15 + d_camara + modelo.grano_delante_m, lateral, math.radians(ang_deg))


def prueba_deteccion(m):
    print("1) Detección (círculo ajustado al arco visible)")
    for d, incl, n_min in ((0.62, 28, 16), (0.80, 20, 16), (0.55, 28, 14), (0.55, 0, 5)):
        real = CamaraPinhole(pitch=math.radians(incl))
        pose = pose_a(d, modelo=m)
        img = foto_arbol(real, m, pose, colores_todos(m), np.random.default_rng(3))
        proy = G.proyectar_soportes(real, m, pose)
        radios = [p[2] for p in proy.values()]
        dets = G.detectar(img, 0.5 * min(radios), 1.6 * max(radios))
        asig = G.asignar(dets, proy)
        e = np.array([[dets[i].u - proy[c][0], dets[i].v - proy[c][1], dets[i].r - proy[c][2]] for c, i in asig.items()])
        print(f"  a {d:.2f} m, cámara a {incl:2d}°: {len(asig):2d} pelotas (esperadas ≥ {n_min}), error máx "
              f"|du| {np.abs(e[:, 0]).max():.1f} |dv| {np.abs(e[:, 1]).max():.1f} |dr| {np.abs(e[:, 2]).max():.1f} px")
        assert len(asig) >= n_min, (d, incl, len(asig))
        assert np.abs(e).max() < 1.5, e


def prueba_pose_y_calibracion(m):
    print("2-3) Pose del árbol (cámara + LiDAR), proyección inversa y calibración de la cámara")
    rng = np.random.default_rng(5)
    colores = colores_todos(m)
    colores["media-3"] = None
    colores["alta-2"] = "NEGRO"
    conf = CamaraPinhole(pitch=math.radians(28))
    for d, (dp, dyaw, droll, dy, dz) in ((0.62, (-2.0, 1.5, -1.0, 0.01, 0.005)), (0.80, (1.5, -1.0, 0.8, -0.005, -0.004))):
        real = conf.con(pitch=conf.pitch + math.radians(dp), yaw=math.radians(dyaw), roll=math.radians(droll),
                        pos=conf.pos + np.array([0.0, dy, dz]))
        pose = pose_a(d, 0.02, 3.0, m)
        imgs = [foto_arbol(real, m, pose, colores, rng) for _ in range(3)]
        med = medir_pie(LidarPie(m, lambda: pose, rng=rng).puntos(), m)
        assert med is not None and abs(med.y_centro - pose[1]) < 0.015 and abs(med.x_cara - pose[0]) < 0.01, med
        verdad = m.en_robot(pose)

        def error(a):
            return np.array([np.linalg.norm(g.p - verdad[k]) for k, g in a.granos.items() if g.categoria != "FUERA"])

        a = G.analizar(imgs, conf, m, med.pose, med.sigma)
        e = error(a)
        print(f"  a {d:.2f} m, cámara SIN calibrar (error de montaje {dp:+.1f}° {dyaw:+.1f}° {droll:+.1f}°, "
              f"{1000 * dy:+.0f}/{1000 * dz:+.0f} mm): {a.n_asignadas} granos, {a.error_px:.1f} px, error 3D mediana "
              f"{1000 * np.median(e):.1f} máx {1000 * e.max():.1f} mm")
        assert np.median(e) < 0.006 and e.max() < 0.012, e
        # C: calibrar con el árbol medido por el LiDAR
        asig = {dd.clave: i for i, dd in enumerate(a.detecciones) if dd.clave is not None}
        cam, rms = G.calibrar_camara(a.detecciones, asig, m, med.pose, conf)
        dif = np.degrees([cam.pitch - real.pitch, cam.yaw - real.yaw, cam.roll - real.roll])
        print(f"    calibrada con el árbol: incl/yaw/roll a {dif[0]:+.2f}/{dif[1]:+.2f}/{dif[2]:+.2f}° de la real "
              f"({rms:.1f} px)")
        assert abs(dif[0]) < 0.6 and abs(dif[2]) < 0.5 and abs(dif[1]) < 1.6, dif
        a2 = G.analizar(imgs, cam, m, med.pose, med.sigma, sig_cam=(math.radians(0.6),) * 3)
        e2 = error(a2)
        print(f"    con la cámara calibrada: error 3D mediana {1000 * np.median(e2):.1f} máx {1000 * e2.max():.1f} mm")
        assert np.median(e2) < 0.005 and e2.max() < 0.010, e2
        # N: calibrar con el brazo (los centros reales de los granos, como los daría la cinemática directa ± 1,5 mm)
        claves = list(asig)
        P = np.array([verdad[c] for c in claves]) + rng.normal(0, 0.0015, (len(claves), 3))
        obs = np.array([[a.detecciones[asig[c]].u, a.detecciones[asig[c]].v] for c in claves])
        k = [G.incertidumbre_relativa(a.detecciones[asig[c]]) for c in claves]
        cam_b, rms_b = G.calibrar_extrinsecos(P, obs, conf, k, mover_pos=True, sig_pos=0.03)
        dif = np.degrees([cam_b.pitch - real.pitch, cam_b.yaw - real.yaw, cam_b.roll - real.roll])
        dpos = 1000 * (cam_b.pos - real.pos)
        e3 = error(G.analizar(imgs, cam_b, m, med.pose, med.sigma, sig_cam=(math.radians(0.6),) * 3))
        # (con todos los puntos en un plano, un corrimiento del lente y un giro se compensan: lo que importa es
        #  dónde quedan los granos)
        print(f"    calibrada con el brazo ({len(claves)} marcas): ángulos a {np.abs(dif).max():.2f}°, lente a "
              f"{np.linalg.norm(dpos):.1f} mm ({rms_b:.1f} px) -> error 3D mediana {1000 * np.median(e3):.1f} "
              f"máx {1000 * e3.max():.1f} mm")
        assert np.abs(dif).max() < 1.5 and np.linalg.norm(dpos) < 20, (dif, dpos)
        assert np.median(e3) < 0.005 and e3.max() < 0.010, e3
        # colores por matiz: los 5 saturados bien; el negro y el vacío NO como maduros
        for k_, g in a2.granos.items():
            real_c = colores.get(g.nombre)
            if g.categoria == "FUERA":
                continue
            if real_c in CLASES5:
                assert g.clase == real_c, (g.nombre, g.clase, real_c)
            else:
                assert g.categoria not in ("MADURO", "SOBREMADURO"), (g.nombre, g.clase, real_c)


def prueba_lidar_con_basura(m):
    print("2b) LiDAR con basura pegada al robot (piso, cables): el pie se busca desde 10 cm delante del LiDAR")
    rng = np.random.default_rng(4)
    pose = pose_a(0.62, 0.03, 2.0, m)
    pts = LidarPie(m, lambda: pose, rng=rng).puntos()
    basura = np.column_stack((rng.uniform(0.18, 0.22, 25), rng.uniform(-0.12, 0.12, 25)))   # 2–7 cm delante del frente
    med = medir_pie(np.vstack([pts, basura]), m, x_min=0.16 + 0.10)
    print(f"  pie medido a x {med.x_cara:.3f} y {med.y_centro:+.3f} (real {pose[0]:.3f} {pose[1]:+.3f})")
    assert abs(med.x_cara - pose[0]) < 0.01 and abs(med.y_centro - pose[1]) < 0.015, med
    assert medir_pie(np.vstack([pts, basura]), m).x_cara < 0.25      # sin x_min se habría quedado con la basura


def prueba_inclinacion_equivocada(m):
    print("2c) Cámara montada a otro ángulo del que dice la config y árbol a otra distancia de --frente (sin LiDAR)")
    col = {"baja-1": "AMARILLO", "baja-3": "ROJO", "baja-5": "AMARILLO", "media-1": "AMARILLO", "alta-3": "AZUL",
           "alta-5": "VERDE", "alta-1": "ROJO"}
    for incl, frente in ((22, 0.78), (15, 0.65), (28, 0.60), (0, 0.60)):
        img = foto_arbol(CamaraPinhole(pitch=math.radians(incl)), m, (0.15 + frente, 0.0, 0.0), col,
                         np.random.default_rng(1))
        a = G.analizar([img], CamaraPinhole(pitch=0.0), m, (0.15 + 0.80, 0.0, 0.0), (0.02, 0.02, math.radians(3)))
        vistas = {ModeloArbol.nombre_de(d.clave): d.clase for d in a.detecciones if d.clave}
        esperadas = {k: v for k, v in col.items() if a.granos[ModeloArbol.clave_de(k)].categoria != "FUERA"}
        print(f"  real {incl:2d}° a {frente:.2f} m (config 0°, --frente 0.80): halló {math.degrees(a.dcam[0]):.1f}° a "
              f"{a.pose[0] - 0.15:.2f} m, {len(vistas)} pelotas en su soporte")
        assert vistas == esperadas and abs(math.degrees(a.dcam[0]) - incl) < 1.5 and abs(a.pose[0] - 0.15 - frente) < 0.02


def prueba_colores_con_referencias(m):
    print("4) Colores con referencias (Lab, como el clasificador de Pablo): también NEGRO y VACIO")
    rng = np.random.default_rng(11)
    cam = CamaraPinhole(pitch=math.radians(25))
    pose = pose_a(0.65, 0.0, 0.0, m)
    nombres = [ModeloArbol.nombre_de(c) for c, _, _ in m.soportes()]
    clases = CLASES5 + ["NEGRO", None]
    ense = {n: clases[i % 7] for i, n in enumerate(nombres)}
    clasif = cl.Clasificador()
    a = G.analizar([foto_arbol(cam, m, pose, ense, rng)], cam, m, pose, (0.01, 0.01, 0.03))
    img = foto_arbol(cam, m, pose, ense, rng)
    for k, g in a.granos.items():
        if g.categoria == "FUERA":
            continue
        u, v, r = g.det.zona if g.det is not None else G.zona_modelo(a.cam, m, a.pose, g.p, g.uv, g.r_px)
        from tools.cosecha import lab_zona
        clasif.agregar(ense[g.nombre] or "VACIO", lab_zona(img, u, v, r, clasif.config), g.nombre)
    # otra foto, otra distribución de colores y otra luz
    otra = {n: clases[(i * 3 + 1) % 7] for i, n in enumerate(nombres)}
    imgs = [foto_arbol(cam, m, pose, otra, np.random.default_rng(s)) for s in (21, 22, 23)]
    a = G.analizar(imgs, cam, m, pose, (0.01, 0.01, 0.03), clasif)
    mal = [(g.nombre, g.clase, otra[g.nombre]) for g in a.granos.values()
           if g.categoria != "FUERA" and g.clase != (otra[g.nombre] or "VACIO")]
    n = sum(1 for g in a.granos.values() if g.categoria != "FUERA")
    print(f"  {n - len(mal)}/{n} soportes con la clase correcta" + (f" | distintos: {mal}" if mal else ""))
    assert not mal, mal
    assert all(g.confianza > 0.7 for g in a.granos.values() if g.categoria in ("MADURO", "SOBREMADURO"))


def prueba_soporte_sin_pelota(m):
    print("4b) Soporte vacío con madera 'amarilla' (30 oct): sin pelota detectada es NEGRO o VACIO, nunca un color")
    from tools.cosecha import lab_zona
    cam = CamaraPinhole(pitch=math.radians(25))
    pose = pose_a(0.65, 0.0, 0.0, m)
    nombres = [ModeloArbol.nombre_de(c) for c, _, _ in m.soportes()]
    col = {n: ["ROJO", "AZUL", "NARANJO", None][i % 4] for i, n in enumerate(nombres)}
    vacios = [n for n in nombres if col[n] is None]
    a0 = G.analizar([foto_arbol(cam, m, pose, col, np.random.default_rng(1))], cam, m, pose, (0.01, 0.01, 0.03))
    zonas = {g.nombre: G.zona_modelo(a0.cam, m, a0.pose, g.p, g.uv, g.r_px) for g in a0.granos.values()
             if g.nombre in vacios}
    madera = (95, 160, 190)                 # BGR: amarillo apagado, saturación < sat_min (el detector no lo ve)
    imgs = []
    for s in (1, 2, 3):
        img = foto_arbol(cam, m, pose, col, np.random.default_rng(s))
        for u, v, r in zonas.values():
            cv2.circle(img, (int(u), int(v)), int(r), madera, -1)
        imgs.append(img)
    clasif = cl.Clasificador()              # referencias: la madera quedó enseñada como AMARILLO (el caso del 30 oct)
    u, v, r = zonas[vacios[0]]
    clasif.agregar("AMARILLO", lab_zona(imgs[0], u, v, r, clasif.config), vacios[0])
    for nombre, clase in (("baja-1", "ROJO"), ("baja-2", "AZUL")):
        g = a0.granos[next(k for k in a0.granos if a0.granos[k].nombre == nombre)]
        assert g.det is not None and col[nombre] == clase
        clasif.agregar(clase, lab_zona(imgs[0], *g.det.zona, clasif.config), nombre)
    crudo = {n: G.color_en(imgs, clasif, *zonas[n])[0] for n in vacios}
    assert all(c == "AMARILLO" for c in crudo.values()), crudo      # sin la regla, el brazo iría a buscarlos
    a = G.analizar(imgs, cam, m, pose, (0.01, 0.01, 0.03), clasif)
    final = {g.nombre: g.categoria for g in a.granos.values() if g.nombre in vacios}
    print(f"  {len(vacios)} soportes vacíos: el color dice {set(crudo.values())}, el análisis {set(final.values())}")
    assert set(final.values()) == {"VACIO"}, final
    assert all(g.categoria in ("MADURO", "SOBREMADURO") for g in a.granos.values() if g.nombre in col
               and col[g.nombre] in ("ROJO", "AZUL") and g.det is not None)


def prueba_foto_real(m):
    print("5) Foto REAL del taller (29 oct, cámara casi horizontal, ~63 cm)")
    img = cv2.imread(str(DATOS / "arbol_taller_29oct.jpg"))
    assert img is not None
    cam = CamaraPinhole(pitch=0.0)
    # Pie donde lo pondría el LiDAR: la roja está ~8,7 cm a la izquierda del eje de la cámara
    a = G.analizar([img], cam, m, (0.15 + 0.63 + 0.035, 0.087, 0.0), (0.02, 0.015, math.radians(3)))
    vistas = {g.nombre: g.clase for g in a.granos.values() if g.det is not None}
    sueltas = [d for d in a.detecciones if d.clave is None]
    print(f"  {vistas} | error {a.error_px:.1f} px | manchas sin soporte: {len(sueltas)} (la rueda rosada del piso)")
    assert vistas == {"baja-3": "ROJO", "baja-4": "VERDE"}, vistas
    assert a.granos[("baja", 2)].categoria == "MADURO" and a.granos[("baja", 3)].categoria == "VERDE"
    assert len(sueltas) == 1 and sueltas[0].v > 500, sueltas


def prueba_intrinsecos():
    print("6) Intrínsecos con un tablero de ajedrez (sintético, 9×6 esquinas, cuadros de 25 mm)")
    from tools.camara import calibrar_intrinsecos
    cam = CamaraPinhole(pitch=0.0, fx=1150.0, fy=1150.0, cx=760.0, cy=440.0)
    rng = np.random.default_rng(2)
    cols, filas, c = 9, 6, 0.025
    imgs = []
    for i in range(12):
        R = (np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
             for a in [math.radians(rng.uniform(-25, 25))])
        Rz = next(R)
        b = math.radians(rng.uniform(-25, 25))
        Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
        ex, ey = Rz @ Ry @ np.array([0.0, -1.0, 0.0]), Rz @ Ry @ np.array([0.0, 0.0, -1.0])   # ejes del tablero
        centro = cam.pos + np.array([rng.uniform(0.35, 0.6), rng.uniform(-0.12, 0.12), rng.uniform(-0.07, 0.07)])
        o = centro - ex * c * (cols + 1) / 2 - ey * c * (filas + 1) / 2
        img = np.full((cam.alto, cam.ancho, 3), 235, np.uint8)
        for x in range(cols + 1):
            for y in range(filas + 1):
                if (x + y) % 2:
                    continue
                P = np.array([o + ex * c * (x + dx) + ey * c * (y + dy) for dx, dy in ((0, 0), (1, 0), (1, 1), (0, 1))])
                uv, z = cam.proyectar(P)
                cv2.fillConvexPoly(img, np.round(uv * 16).astype(np.int32), (25, 25, 25), cv2.LINE_AA, 4)
        imgs.append(cv2.GaussianBlur(img, (3, 3), 0))
    vals, rms, n = calibrar_intrinsecos(imgs, cols, filas, c, cam.ancho, cam.alto)
    print(f"  {n} fotos, {rms:.2f} px: fx {vals['fx_px']} fy {vals['fy_px']} cx {vals['cx_px']} cy {vals['cy_px']} "
          "(real 1150 1150 760 440)")
    assert abs(vals["fx_px"] - 1150) < 15 and abs(vals["fy_px"] - 1150) < 15, vals
    assert abs(vals["cx_px"] - 760) < 10 and abs(vals["cy_px"] - 440) < 10, vals


def prueba_sesion_simulada():
    print("7) Sesión simulada (tools.cosecha): medir, calibrar, acercarse, agarrar")
    from cafebot.arm import brazo as mod_brazo
    from cafebot.arm.brazo import BrazoXL430
    from cafebot.drivers.mega import Mega
    from tests.sim_serial import SerialSimulado
    from tools.cosecha import Cosecha, MundoSim, posturas_sim, preparar_brazo
    cfg = cargar()
    cfg.brazo.t_mov_s = cfg.brazo.t_fino_s = 0.01
    cfg.camara.calibrada = ""
    tmp = pathlib.Path(tempfile.mkdtemp())
    mod_brazo.RAIZ_PI = tmp
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(20)
    m = ModeloArbol()
    colores = {"baja-2": "ROJO", "baja-4": "AZUL", "media-2": "NEGRO", "media-3": "NARANJO", "media-4": "VERDE",
               "media-5": "AMARILLO", "alta-3": "AZUL", "alta-2": "VERDE"}
    original = dict(colores)
    mundo = MundoSim(cfg, m, mega, 0.62, 0.02, 2.0, semilla=3, colores=colores, bus_falso=memoria)
    brazo = BrazoXL430(cfg, "falso", log=lambda t: None)
    posturas_sim(brazo)
    assert preparar_brazo(brazo, cfg, print)
    pasos = []
    mover = brazo._mover

    def espia(obj, t, que):
        pasos.append(que)
        return mover(obj, t, que)
    brazo._mover = espia
    ses = Cosecha(cfg, m, mundo.camara, mundo.lidar, mega, brazo, mundo, cl.Clasificador(), confianza_min=0.5,
                  ruta_local=str(tmp / "local.json"), ruta_referencias=str(tmp / "ref.json"), log=lambda t: None)
    try:
        ses.analizar()
        ses.calibrar_con_arbol()
        assert ses.cfg.camara.calibrada and (tmp / "local.json").exists()
        dif = np.degrees([ses.cam.pitch - mundo.cam_real.pitch, ses.cam.roll - mundo.cam_real.roll])
        ses.analizar()
        seguros = sorted(g.nombre for g in ses.seguros())
        print(f"  cámara calibrada (incl/roll a {dif[0]:+.2f}/{dif[1]:+.2f}° de la real); seguros: {seguros}")
        assert seguros == ["alta-3", "baja-2", "baja-4", "media-3", "media-5"], seguros
        ses.acercar()
        frente = ses.pose_pie[0] - cfg.geometria.largo / 2
        verdad = m.en_robot(mundo.pose_pie())
        e = max(np.linalg.norm(ses.p_robot(g) - verdad[k]) for k, g in ses.analisis.granos.items()
                if g.categoria != "FUERA")
        print(f"  acercado: frente a {100 * frente:.1f} cm del pie (pedido {100 * cfg.mision.holgura_agarre_m:.0f}); "
              f"error de los granos {1000 * e:.1f} mm")
        assert abs(frente - cfg.mision.holgura_agarre_m) < 0.015 and e < 0.008
        try:                                    # de cerca no se saca la foto (lo medido de lejos sigue valiendo)
            ses.analizar()
            raise AssertionError("debió negarse a analizar tan cerca")
        except RuntimeError as err:
            assert "demasiado cerca" in str(err)
        # ESC durante el primer agarre: no se manda nada más y no cuenta como agarrado
        n_pasos = len(pasos)
        brazo._mover = lambda obj, t, que: (brazo.parar.set(), espia(obj, t, que))[1] if len(pasos) > n_pasos else espia(obj, t, que)
        assert ses.agarrar_todos() == 0 and "detenido" in brazo.ultimo_motivo, brazo.ultimo_motivo
        brazo._mover = espia
        ses.espera()
        n = ses.agarrar_todos()
        alcanzables = sorted(ModeloArbol.nombre_de(k) for k in ses.tomados)
        print(f"  agarrados {n}: {alcanzables} | errores de agarre {np.round(1000 * np.array(mundo.errores_agarre[1:]), 1)} mm"
              f" | canastas: {brazo.depositos}")
        assert n >= 3 and max(mundo.errores_agarre) < 0.005
        maduros = sum(original[nm] in ("ROJO", "NARANJO", "AMARILLO") for nm in alcanzables)
        assert brazo.depositos["MADURO"] == maduros and brazo.depositos["SOBREMADURO"] == n - maduros
        assert all(mundo.colores[nm] is None for nm in alcanzables)            # ya no están en el árbol
        assert mundo.colores["media-4"] == "VERDE" and mundo.colores["media-2"] == "NEGRO"   # no se tocan
        # camino a la canasta: plegar sin girar, girar plegado, recién ahí estirarse
        i = pasos.index("plegar (transporte)")
        assert pasos[i + 1] == "girar plegado (transporte)" and pasos[i + 2].startswith("postura deposito_"), pasos[i:i + 3]
        print(f"  camino a la canasta: {' -> '.join(pasos[i - 1:i + 3])}")
    finally:
        mega.cerrar()


def prueba_cabezal():
    print("7b) Cabezal de la cámara: X 2 con I/K, y C con dos ángulos del servo")
    import json
    from cafebot.drivers.mega import Mega
    from cafebot.vision.camara_modelo import lente_cabezal, lente_de_referencia
    from tests.sim_serial import SerialSimulado
    from tools.cosecha import Cosecha, MundoSim
    cfg = cargar()
    cfg.camara.calibrada = ""
    cc = cfg.camara
    assert cc.cabezal
    for p in (-25.0, 5.0, 18.0, 30.0):                          # ida y vuelta del lente
        assert np.allclose(lente_de_referencia(cc, lente_cabezal(cc, p, 7.0), p, 7.0), [cc.x, cc.y, cc.z])
        assert abs(cc.pitch_de_servo(cc.servo_de_pitch(p)) - p) <= 0.5 * abs(cc.pitch_por_grado)
    tmp = pathlib.Path(tempfile.mkdtemp())
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(20)
    m = ModeloArbol()
    mundo = MundoSim(cfg, m, mega, 0.60, 0.0, 1.0, semilla=5, colores=colores_todos(m), bus_falso=memoria)
    ses = Cosecha(cfg, m, mundo.camara, mundo.lidar, mega, None, mundo, cl.Clasificador(),
                  ruta_local=str(tmp / "local.json"), ruta_referencias=str(tmp / "ref.json"), log=lambda t: None)
    try:
        s0 = ses.servo_pitch
        assert s0 == cc.servo_de_pitch(cc.inclinacion_arbol_deg), s0
        ses.analizar()
        ses.calibrar_con_arbol()
        for _ in range(4):                                       # I x4: el servo sube 8°
            ses.inclinar(+0.5)
        assert ses.servo_pitch == s0 + 8, ses.servo_pitch
        ses.analizar()
        ses.calibrar_con_arbol()
        real_90 = mundo.cc.pitch_servo_90_deg + mundo.err_pitch
        real_b = mundo.cc.pitch_por_grado * (1 + mundo.err_pendiente)
        e_inc = math.degrees(ses.cam.pitch - mundo.cam_real.pitch)
        e_pos = 1000 * np.linalg.norm(ses.cam.pos - mundo.cam_real.pos)
        print(f"  X 2 {s0} -> {ses.servo_pitch}: X 2 = 90 -> {cc.pitch_servo_90_deg:+.2f}° (real {real_90:+.2f}), "
              f"{cc.pitch_por_grado:.3f}°/° (real {real_b:.3f}) | inclinación a {e_inc:+.2f}°, lente a {e_pos:.1f} mm")
        assert abs(cc.pitch_por_grado - real_b) < 0.06 and abs(cc.pitch_servo_90_deg - real_90) < 0.6
        assert abs(e_inc) < 0.3 and e_pos < 8
        guardado = json.loads((tmp / "local.json").read_text())["camara"]
        assert {"pitch_servo_90_deg", "pitch_por_grado", "inclinacion_arbol_deg"} <= set(guardado)
        assert cc.servo_de_pitch(guardado["inclinacion_arbol_deg"]) == ses.servo_pitch   # la misión vuelve a este X 2
    finally:
        mega.cerrar()


def prueba_a_mano():
    print("7c) Sin Mega: el árbol se acerca a mano (1,5 cm menos de lo pedido) y U lo ubica con el LiDAR")
    from tools.cosecha import Cosecha, MundoSim
    cfg = cargar()
    cfg.camara.calibrada = ""
    tmp = pathlib.Path(tempfile.mkdtemp())
    m = ModeloArbol()
    mundo = MundoSim(cfg, m, None, 0.60, 0.01, 1.5, semilla=4, colores=colores_todos(m))
    ses = Cosecha(cfg, m, mundo.camara, mundo.lidar, None, None, mundo, cl.Clasificador(),
                  ruta_local=str(tmp / "local.json"), ruta_referencias=str(tmp / "ref.json"), log=lambda t: None)
    ses.analizar()
    x0 = ses.pose_pie[0]
    assert ses.acercar()
    pedido = x0 - ses.pose_pie[0]
    mundo.pie_mundo[0] -= pedido - 0.015                       # a mano quedó 1,5 cm más lejos de lo pedido

    def error():
        verdad = m.en_robot(mundo.pose_pie())
        return max(np.linalg.norm(ses.p_robot(g) - verdad[k]) for k, g in ses.analisis.granos.items()
                   if g.categoria != "FUERA")
    e0 = error()
    assert ses.reubicar()
    e1 = error()
    print(f"  pedido acercar {100 * pedido:.1f} cm; error de los granos {1000 * e0:.1f} mm -> con U {1000 * e1:.1f} mm")
    assert e0 > 0.012 and e1 < 0.006, (e0, e1)


def prueba_lidar_corrido():
    print("8) LiDAR corrido 1,5 cm de lado y 1 cm hacia adelante; cámara calibrada con el brazo (M + N)")
    from cafebot.arm import brazo as mod_brazo
    from cafebot.arm.brazo import BrazoXL430
    from cafebot.drivers.mega import Mega
    from tests.sim_serial import SerialSimulado
    from tools.cosecha import Cosecha, MundoSim, posturas_sim, preparar_brazo
    cfg = cargar()
    cfg.brazo.t_mov_s = cfg.brazo.t_fino_s = 0.01
    cfg.camara.calibrada = ""
    tmp = pathlib.Path(tempfile.mkdtemp())
    mod_brazo.RAIZ_PI = tmp
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(20)
    m = ModeloArbol()
    colores = {"baja-2": "ROJO", "baja-4": "AZUL", "media-3": "NARANJO", "media-4": "AMARILLO", "baja-3": "VERDE"}
    mundo = MundoSim(cfg, m, mega, 0.62, -0.01, -2.0, semilla=8, colores=colores, bus_falso=memoria)
    puntos = mundo.lidar.puntos
    mundo.lidar.puntos = lambda: puntos() + np.array([0.01, 0.015])      # el LiDAR "cree" que todo está corrido
    brazo = BrazoXL430(cfg, "falso", log=lambda t: None)
    posturas_sim(brazo)
    assert preparar_brazo(brazo, cfg, print)
    ses = Cosecha(cfg, m, mundo.camara, mundo.lidar, mega, brazo, mundo, cl.Clasificador(), confianza_min=0.5,
                  ruta_local=str(tmp / "local.json"), ruta_referencias=str(tmp / "ref.json"), log=lambda t: None)
    try:
        a = ses.analizar()
        # M en cada grano detectado: la garra (a mano) en el grano REAL; la cinemática directa da su posición ± 1,5 mm
        rng = np.random.default_rng(1)
        for g in a.granos.values():
            if g.det is not None:
                ses.marcas.append((mundo.grano_real(g.clave) + rng.normal(0, 0.0015, 3), (g.det.u, g.det.v), g.nombre,
                                   G.incertidumbre_relativa(g.det), a.cam.ancho))
        ses.calibrar_con_brazo()
        assert "brazo" in cfg.camara.calibrada
        ses.analizar()
        ses.acercar()
        verdad = m.en_robot(mundo.pose_pie())
        e = max(np.linalg.norm(ses.p_robot(g) - verdad[k]) for k, g in ses.analisis.granos.items()
                if g.categoria != "FUERA")
        n = ses.agarrar_todos()
        print(f"  {len(ses.marcas)} marcas; corrección del LiDAR que se mantiene al acercarse: "
              f"{100 * ses.delta_lidar[0]:+.1f} / {100 * ses.delta_lidar[1]:+.1f} cm | error de los granos ya cerca "
              f"{1000 * e:.1f} mm | agarrados {n}, errores {np.round(1000 * np.array(mundo.errores_agarre), 1)} mm")
        assert e < 0.008 and n == 4 and max(mundo.errores_agarre) < 0.008, (e, n, mundo.errores_agarre)
    finally:
        mega.cerrar()


def main():
    m = ModeloArbol()
    prueba_deteccion(m)
    prueba_pose_y_calibracion(m)
    prueba_lidar_con_basura(m)
    prueba_inclinacion_equivocada(m)
    prueba_colores_con_referencias(m)
    prueba_soporte_sin_pelota(m)
    prueba_foto_real(m)
    prueba_intrinsecos()
    prueba_sesion_simulada()
    prueba_cabezal()
    prueba_a_mano()
    prueba_lidar_corrido()
    print("OK: cosecha")


if __name__ == "__main__":
    main()
