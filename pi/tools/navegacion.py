"""Interfaz de navegación: mapa del SLAM, trayectorias, landmarks y rutas A→B (pygame).

En la Raspberry (escritorio por VNC):
    python3 -m tools.navegacion --mega /dev/ttyACM0 --lidar /dev/ttyUSB0
    python3 -m tools.navegacion --lidar /dev/ttyUSB0 --sin-mega      # solo mapear (empujando el robot)
    python3 -m tools.navegacion --sim                                 # todo simulado, sin robot
    agregar --imu para usar el giroscopio MPU9250

Ratón:  clic = destino B | Shift+clic = origen A | arrastrar = mover | rueda = zoom
Teclado (como el programa antiguo):
    ENTER ejecutar ruta | ESC parar | P replanificar | A origen = robot | X borrar ruta
    TAB elegir landmark | G aproximarse al landmark elegido
    Flechas mover a mano (↑↓ avanzar/retroceder, ←→ girar) | Q / E cangrejo izq / der
    M congelar/reanudar mapa | S guardar | L cargar | R reiniciar | C centrar | O origen
    I ubicar en la cancha (con el robot en su sector de partida) | T trayectoria de odometría
    B recalibrar giroscopio (quieto) | +/- zoom
"""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
import pygame as pg
import pygame.gfxdraw as gfx

from cafebot.config import cargar
from cafebot.mundo import cancha
from cafebot.navegacion.geometria import componer, envolver, rectangulo, relativo, transformar
from cafebot.navegacion.sistema import SistemaNavegacion

FONDO, LIBRE, TENTATIVO, ESTATICO = (12, 14, 20), (22, 30, 44), (55, 62, 85), (70, 150, 220)
COLOR_CLASE = {"ARBOL": (90, 255, 120), "CAJA": (255, 120, 90), "ALBERCA": (90, 160, 255), "OBSTACULO": (255, 170, 60)}


class Vista:
    def __init__(self, ancho, alto, escala=180.0):
        self.w, self.h, self.escala = ancho, alto, escala
        self.pan = np.array([-self.w * 0.25, 70.0])   # robot a la izquierda mirando a la derecha; bajo el panel

    def a_pantalla(self, p):
        return (int(self.w / 2 + self.pan[0] + p[0] * self.escala), int(self.h / 2 + self.pan[1] - p[1] * self.escala))

    def a_mundo(self, sx, sy):
        return np.array([(sx - self.w / 2 - self.pan[0]) / self.escala, (self.h / 2 + self.pan[1] - sy) / self.escala])

    def zoom(self, factor, centro_px):
        w = self.a_mundo(*centro_px)
        self.escala = float(np.clip(self.escala * factor, 20, 900))
        self.pan = np.array([centro_px[0] - self.w / 2 - w[0] * self.escala, centro_px[1] - self.h / 2 + w[1] * self.escala])


def dibujar_mapa(win, slam, v: Vista):
    x0, y1 = v.a_mundo(0, 0)
    x1, y0 = v.a_mundo(v.w, v.h)
    r, c, n = slam.res, slam.centro, slam.dim
    gx0, gx1 = max(int(np.floor(x0 / r + c)), 0), min(int(np.ceil(x1 / r + c)), n)
    gy0, gy1 = max(int(np.floor(y0 / r + c)), 0), min(int(np.ceil(y1 / r + c)), n)
    if gx1 <= gx0 or gy1 <= gy0:
        return
    lo = slam.log_odds[gy0:gy1, gx0:gx1]
    rgb = np.empty(lo.shape + (3,), np.uint8)
    rgb[:] = FONDO
    rgb[lo < -0.25] = LIBRE
    rgb[lo > 0.3] = TENTATIVO
    rgb[slam.mascara_estatica()[gy0:gy1, gx0:gx1]] = ESTATICO
    sup = pg.surfarray.make_surface(np.ascontiguousarray(rgb[::-1].transpose(1, 0, 2)))
    sup = pg.transform.scale(sup, (max(1, int((gx1 - gx0) * r * v.escala)), max(1, int((gy1 - gy0) * r * v.escala))))
    win.blit(sup, v.a_pantalla(((gx0 - c) * r, (gy1 - c) * r)))


def linea(win, v, pts, color, ancho=1, cerrada=False):
    if pts is not None and len(pts) > 1:
        pg.draw.lines(win, color, cerrada, [v.a_pantalla(p) for p in pts], ancho)


def dibujar_cancha(win, v, T_mapa_cancha):
    """Contorno de la cancha y objetos fijos, transformados al marco del mapa (tecla I)."""
    gris = (110, 110, 70)
    def r(cx, cy, hx, hy):
        return transformar(rectangulo(cx, cy, hx, hy), T_mapa_cancha)
    linea(win, v, r(cancha.LARGO_X / 2, cancha.LARGO_Y / 2, cancha.LARGO_X / 2, cancha.LARGO_Y / 2), gris, 1, True)
    linea(win, v, r(cancha.BENEF_X / 2, cancha.LARGO_Y / 2, cancha.BENEF_X / 2, cancha.LARGO_Y / 2), gris, 1, True)
    linea(win, v, r((cancha.ALBERCA_X0 + cancha.ALBERCA_X1) / 2, cancha.LARGO_Y / 2, 0.1, cancha.LARGO_Y / 2), gris, 1, True)
    linea(win, v, r((cancha.CULTIVO_X0 + cancha.LARGO_X) / 2, cancha.LARGO_Y / 2, (cancha.LARGO_X - cancha.CULTIVO_X0) / 2,
                    cancha.LARGO_Y / 2), gris, 1, True)
    for y in cancha.centros_zonas_beneficiadero():
        linea(win, v, r(cancha.ZONA_X_CENTRO, y, cancha.BENEF_X_INT / 2, cancha.SECTOR_BENEF_Y / 2), gris, 1, True)
    for y in cancha.centros_arboles():
        linea(win, v, r(cancha.ARBOL_X_FRENTE + 0.01, y, 0.01, cancha.ARBOL_ANCHO_PIE / 2), (200, 200, 90), 2, True)


def marcador(win, fuente, v, p, txt, color):
    sx, sy = v.a_pantalla(p)
    pg.draw.circle(win, color, (sx, sy), 10, 3)
    pg.draw.circle(win, color, (sx, sy), 3)
    win.blit(fuente.render(txt, True, color), (sx + 13, sy - 22))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mega", default=None, help="puerto del Mega (por defecto el de config)")
    ap.add_argument("--lidar", default=None, help="puerto del LD06 (por defecto el de config)")
    ap.add_argument("--sin-mega", action="store_true", help="solo mapear, sin mover el robot")
    ap.add_argument("--imu", action="store_true", help="usar el giroscopio MPU9250")
    ap.add_argument("--sim", action="store_true", help="Mega y LiDAR simulados (sin robot)")
    ap.add_argument("--zona-sim", type=int, default=3, help="sector de partida en --sim (1..5)")
    ap.add_argument("--ancho", type=int, default=1200)
    ap.add_argument("--alto", type=int, default=850)
    ap.add_argument("--ir-a", default=None, metavar="X,Y",
                    help="planificar y ejecutar una ruta a (X,Y) m apenas el mapa lo permita")
    ap.add_argument("--captura", default=None, help="(pruebas) guardar una imagen tras N s y salir: archivo.png")
    ap.add_argument("--captura-s", type=float, default=8.0)
    a = ap.parse_args()
    cfg = cargar()

    # ------------------------------------------------------------- hardware
    mega = giroscopio = None
    inicio_sim = None
    if a.sim:
        from cafebot.drivers.lidar_simulado import LD06Simulado
        from cafebot.drivers.mega import Mega
        from tests.sim_serial import SerialSimulado
        mega = Mega("sim")
        mega.conectar(serial_obj=SerialSimulado())
        esc = cancha.Escenario(a.zona_sim, 1 if a.zona_sim != 1 else 2, 5 if a.zona_sim != 5 else 4, (2, 4))
        inicio_sim = np.array([cancha.ZONA_X_CENTRO, cancha.centros_zonas_beneficiadero()[a.zona_sim - 1], 0.0])
        lidar = LD06Simulado(cfg.geometria.config_ld06(), esc.segmentos(), inicio_sim,
                             lambda: (mega.tele.x / 1000, mega.tele.y / 1000, mega.tele.th))
    else:
        from cafebot.drivers.ld06 import LD06
        lidar = LD06(a.lidar or cfg.puertos.lidar, cfg.geometria.config_ld06())
        if not a.sin_mega:
            from cafebot.drivers.mega import Mega
            mega = Mega(a.mega or cfg.puertos.mega)
            print("Conectando con el Mega (se reinicia, ~2 s)...")
            mega.conectar()
        if a.imu:
            try:
                from cafebot.drivers.imu import Giroscopio
                giroscopio = Giroscopio(cfg.puertos.imu_bus, cfg.puertos.imu_dir)
                print("Giroscopio: calibrando sesgo, NO mover el robot...")
            except Exception as e:
                print("Giroscopio DESACTIVADO:", e)
    if mega is not None:
        mega.telemetria(20)
    lidar.iniciar()
    nav = SistemaNavegacion(cfg, lidar, mega, giroscopio)

    # ------------------------------------------------------------- interfaz
    pg.init()
    win = pg.display.set_mode((a.ancho, a.alto))
    pg.display.set_caption("Cafebot - navegación (SLAM + odometría + A*)")
    fuente = pg.font.SysFont("monospace", 14, bold=True)
    reloj = pg.time.Clock()
    v = Vista(a.ancho, a.alto)
    origen_A = None
    seleccion = None
    msg = "Mueve el robot (flechas) para mapear; clic para elegir destino"
    arrastrando, recorrido = False, 0.0
    ver_odom = True
    T_mapa_cancha = None        # pose de la cancha en el marco del mapa (tecla I)
    ult_manual = 0.0
    pendiente = [np.array([float(v) for v in a.ir_a.split(",")])] if a.ir_a else []
    t0 = time.monotonic()
    corriendo = True

    while corriendo:
        for ev in pg.event.get():
            if ev.type == pg.QUIT:
                corriendo = False
            elif ev.type == pg.MOUSEBUTTONDOWN and ev.button in (1, 2, 3):
                arrastrando, recorrido = True, 0.0
            elif ev.type == pg.MOUSEBUTTONUP and ev.button in (1, 2, 3):
                if arrastrando and ev.button == 1 and recorrido < 6:
                    p = v.a_mundo(*ev.pos)
                    if pg.key.get_mods() & pg.KMOD_SHIFT:
                        origen_A = p
                        msg = "Origen A fijado"
                    else:
                        nav.planificar(p, origen_A)
                        msg = f"Ruta: {nav.planificador.msg}  (ENTER para ejecutar)"
                arrastrando = False
            elif ev.type == pg.MOUSEMOTION and arrastrando:
                v.pan += ev.rel
                recorrido += abs(ev.rel[0]) + abs(ev.rel[1])
            elif ev.type == pg.MOUSEWHEEL:
                v.zoom(1.15 if ev.y > 0 else 1 / 1.15, pg.mouse.get_pos())
            elif ev.type == pg.KEYDOWN:
                k = ev.key
                if k == pg.K_ESCAPE:
                    nav.detener()
                    msg = "STOP"
                elif k in (pg.K_RETURN, pg.K_KP_ENTER):
                    if origen_A is not None:
                        msg = "Para ejecutar, el origen debe ser el robot: tecla A y replanificar"
                    elif nav.ejecutar():
                        msg = "Ejecutando ruta"
                    else:
                        msg = nav.ejecutor.estado
                elif k == pg.K_p and nav.destino is not None:
                    nav.planificar(nav.destino, origen_A)
                    msg = f"Ruta: {nav.planificador.msg}"
                elif k == pg.K_a:
                    origen_A = None
                    msg = "Origen = robot"
                elif k == pg.K_x:
                    nav.planificador.limpiar()
                    nav.destino = origen_A = None
                elif k == pg.K_TAB:
                    nombres = [l.nombre for l in nav.registro.confirmados()]
                    if nombres:
                        seleccion = nombres[(nombres.index(seleccion) + 1) % len(nombres)] if seleccion in nombres else nombres[0]
                    else:
                        msg = "Aún no hay landmarks confirmados"
                elif k == pg.K_g:
                    if seleccion is None:
                        msg = "Elige un landmark con TAB"
                    else:
                        _, msg = nav.aproximar(seleccion)
                elif k == pg.K_m:
                    nav.slam.alternar_congelado()
                    msg = "Mapa CONGELADO (solo localiza)" if nav.slam.congelado else "Mapeando"
                elif k == pg.K_s:
                    nav.slam.guardar()
                    nav.registro.guardar()
                    msg = "Guardado: mapa_estatico.npz + landmarks.json"
                elif k == pg.K_l:
                    if nav.slam.cargar():
                        nav.registro.cargar()
                        msg = "Mapa cargado: el robot debe estar donde partió al guardarlo"
                    else:
                        msg = "No se pudo cargar mapa_estatico.npz"
                elif k == pg.K_r:
                    nav.reiniciar()
                    origen_A = seleccion = T_mapa_cancha = None
                    msg = "SLAM reiniciado"
                elif k == pg.K_c:
                    p = nav.pose()
                    v.pan = np.array([-p[0] * v.escala, p[1] * v.escala])
                elif k == pg.K_o:
                    v.pan = np.array([-v.w * 0.25, 70.0])
                elif k == pg.K_t:
                    ver_odom = not ver_odom
                elif k == pg.K_b and giroscopio is not None:
                    giroscopio.recalibrar()
                    msg = "Recalibrando giroscopio: NO mover"
                elif k == pg.K_i:
                    from cafebot.localizacion.inicio import estimar_inicio
                    s = lidar.ultimo_scan()
                    if s is not None:
                        r = estimar_inicio(s)
                        pose_cancha = np.array([r.x, r.y, r.th])
                        # T tal que T ⊕ pose_cancha = pose_mapa  ->  T = pose_mapa ⊕ (pose_cancha)^-1
                        inv = relativo(pose_cancha, np.zeros(3))
                        T_mapa_cancha = componer(nav.pose(), inv)
                        msg = (f"Ubicado: sector {r.zona}, confianza {r.confianza:.2f}, albercas {r.albercas}"
                               + ("  (BAJA: verificar)" if r.confianza < 0.15 else ""))
                elif k in (pg.K_PLUS, pg.K_EQUALS, pg.K_KP_PLUS):
                    v.zoom(1.2, (v.w // 2, v.h // 2))
                elif k in (pg.K_MINUS, pg.K_KP_MINUS):
                    v.zoom(1 / 1.2, (v.w // 2, v.h // 2))

        # Manejo manual (mantener la tecla): V/K repetidos a 10 Hz
        if mega is not None and not nav.ejecutor.activo and time.monotonic() - ult_manual > 0.1:
            t = pg.key.get_pressed()
            vel, w = cfg.navegacion.vel_mm_s, 0.6
            orden = None
            if t[pg.K_UP]:
                orden = lambda: mega.velocidad(vel, 0)
            elif t[pg.K_DOWN]:
                orden = lambda: mega.velocidad(-vel, 0)
            elif t[pg.K_LEFT]:
                orden = lambda: mega.velocidad(0, w)
            elif t[pg.K_RIGHT]:
                orden = lambda: mega.velocidad(0, -w)
            elif t[pg.K_q]:
                orden = lambda: mega.velocidad_cangrejo(90, vel)
            elif t[pg.K_e]:
                orden = lambda: mega.velocidad_cangrejo(-90, vel)
            if orden:
                orden()
                ult_manual = time.monotonic()

        nav.actualizar()
        if pendiente and nav.ekf.inicializada and time.monotonic() - t0 > 3:
            if nav.planificar(pendiente[0]) and nav.ejecutar():
                msg = f"Ruta automática a {pendiente[0]}"
                pendiente.pop(0)

        # ------------------------------------------------------------- dibujo
        win.fill(FONDO)
        dibujar_mapa(win, nav.slam, v)
        o = v.a_pantalla((0, 0))
        for rm in range(1, 8):                                  # anillos de 1 m desde la partida
            gfx.aacircle(win, o[0], o[1], int(rm * v.escala), (30, 36, 48))
        if T_mapa_cancha is not None:
            dibujar_cancha(win, v, T_mapa_cancha)
        for p in nav.pts_mundo:                                 # barrido actual
            sx, sy = v.a_pantalla(p)
            if 0 <= sx < v.w and 0 <= sy < v.h:
                win.fill((240, 240, 255), (sx - 1, sy - 1, 3, 3))
        linea(win, v, nav.planificador.ruta_cruda, (110, 40, 95), 1)
        if nav.planificador.ruta is not None:
            linea(win, v, nav.planificador.ruta, (255, 60, 200), 3)
            for p in nav.planificador.ruta:
                pg.draw.circle(win, (255, 160, 230), v.a_pantalla(p), 4)
        for lm in nav.registro.items:
            col = COLOR_CLASE.get(lm.cls, (90, 95, 110)) if lm.confirmado else (90, 95, 110)
            mitad = lm.eje * lm.largo / 2
            pg.draw.line(win, col, v.a_pantalla(lm.pos - mitad), v.a_pantalla(lm.pos + mitad), 4 if lm.confirmado else 1)
            if lm.confirmado:
                sx, sy = v.a_pantalla(lm.pos)
                if lm.nombre == seleccion:
                    pg.draw.circle(win, (255, 255, 0), (sx, sy), int(lm.largo * v.escala / 2) + 8, 2)
                win.blit(fuente.render(f"{lm.nombre} {100 * lm.largo:.0f}cm", True, col), (sx + 8, sy - 22))
        for pista in nav.pistas:                                # objetos en movimiento
            if pista.id in nav.dinamicas:
                linea(win, v, list(pista.historia), (255, 90, 60), 2)
                pg.draw.circle(win, (255, 90, 60), v.a_pantalla(pista.pos), max(int(pista.radio * v.escala), 6), 2)
        if origen_A is not None:
            marcador(win, fuente, v, origen_A, "A", (60, 255, 120))
        if nav.destino is not None:
            marcador(win, fuente, v, nav.destino, "B", (255, 70, 70))
        if ver_odom:
            linea(win, v, list(nav.ekf.tray_odom), (230, 180, 40), 1)       # solo odometría
        linea(win, v, list(nav.ekf.tray_ekf) or list(nav.slam.trayectoria), (0, 220, 130), 2)   # fusionada
        pose = nav.pose()
        esq = nav.slam.esquinas_huella(pose)
        if esq is not None:
            linea(win, v, esq, (70, 140, 255), 2, True)                     # huella filtrada
        g = cfg.geometria
        for e in g.extras:
            linea(win, v, transformar(rectangulo(*e), pose), (70, 140, 255), 1, True)
        lid = transformar(np.array([[g.lidar_x, g.lidar_y]]), pose)[0]
        pg.draw.circle(win, (255, 255, 255), v.a_pantalla(lid), 4)
        rx, ry = v.a_pantalla(pose[:2])
        hx, hy = v.a_pantalla(pose[:2] + 0.3 * np.array([math.cos(pose[2]), math.sin(pose[2])]))
        pg.draw.circle(win, (0, 255, 140), (rx, ry), 6, 2)
        pg.draw.line(win, (0, 255, 140), (rx, ry), (hx, hy), 3)

        # HUD
        st = lidar.estadisticas()
        d_odom, d_th = nav.ekf.deriva()
        conf = nav.registro.confirmados()
        rest = nav.planificador.restante(pose[:2])
        t = mega.tele if mega is not None else None
        hud = [
            f"POSE      x={pose[0]:+.3f} m  y={pose[1]:+.3f} m  θ={math.degrees(pose[2]):+.1f}°   "
            f"({'odometría Mega + SLAM' if nav.ekf.inicializada else 'solo SLAM'})",
            f"LIDAR     barridos {st['scans']} | CRC malos {st['crc_malos']} | puntos del propio robot filtrados {nav.puntos_propios}/vuelta",
            f"SLAM      {'CONGELADO' if nav.slam.congelado else 'mapeando'} | calidad ICP {nav.slam.calidad:.2f} | "
            f"barridos integrados {nav.slam.n_barridos} | celdas estáticas {int(nav.slam.mascara_estatica().sum())}",
            f"ODOMETRÍA corrección del SLAM acumulada: {100 * d_odom:.1f} cm, {d_th:+.1f}° "
            f"(amarillo = solo odometría, verde = fusionada)",
            (f"MEGA      x={t.x:.0f} y={t.y:.0f} mm θ={math.degrees(t.th):.1f}° postura={t.postura} IR={t.ir:04b}"
             if t is not None else "MEGA      sin conectar (solo mapeo)"),
            f"GIRO      {giroscopio.estado() if giroscopio else 'sin giroscopio (--imu para usarlo)'}"
            + (f" | signo {'verificado' if nav.ekf.gyro_verificado else 'por verificar (gira el robot)'}" if giroscopio else ""),
            f"LANDMARKS {', '.join(l.nombre for l in conf) or '—'} | elegido: {seleccion or '—'}",
            f"RUTA      {nav.planificador.msg}" + (f" | largo {nav.planificador.largo:.2f} m | restan {rest:.2f} m" if rest else "")
            + f" | ejecutor: {nav.ejecutor.estado}",
            f">> {msg}",
            "clic B | shift+clic A | ENTER ejecutar | ESC parar | TAB/G landmark | flechas, Q/E mover | "
            "M congelar | S/L guardar/cargar | I cancha | T odometría | R reset",
        ]
        panel = pg.Surface((v.w, 14 + 18 * len(hud)), pg.SRCALPHA)
        panel.fill((8, 10, 16, 205))
        win.blit(panel, (0, 0))
        for i, txt in enumerate(hud):
            win.blit(fuente.render(txt, True, (220, 230, 240)), (12, 8 + 18 * i))
        pg.display.flip()
        reloj.tick(30)

        if a.captura and time.monotonic() - t0 > a.captura_s:
            pg.image.save(win, a.captura)
            corriendo = False

    nav.detener()
    lidar.detener()
    if mega is not None:
        mega.cerrar()
    if giroscopio is not None:
        giroscopio.cerrar()
    pg.quit()


if __name__ == "__main__":
    main()
