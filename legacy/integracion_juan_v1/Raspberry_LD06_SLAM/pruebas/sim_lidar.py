"""Simulador mínimo de LD06: ray casting en una sala con un 'árbol' cilíndrico, más el chasis
del propio robot (que es lo que causa la autodetección). Genera paquetes LD06 reales de 47 bytes."""
import math
import numpy as np

CRC = None
SECTORES_CHASIS = [(30, 50), (130, 150), (210, 230), (310, 330), (84, 96), (264, 276)]


def _crc(packet46):
    import ld06_slam_nav_rpi_imu_integrado as M
    c = 0
    for b in packet46:
        c = M.crc_table[(c ^ b) & 0xFF]
    return c


def ray_scan(pose, n=450, sala=(4.0, 3.0), arboles=(((1.0, 1.2), 0.06),), chasis=None, rng=None):
    """pose=(x,y,theta) con theta=0 mirando a +Y (convenio del SLAM). Devuelve (ang_deg, dist_mm).
    chasis=(semi_x, semi_y): rectángulo del propio robot que el LiDAR 've' desde adentro."""
    x0, y0, th = pose
    ang = np.arange(n) * 360.0 / n
    dist = np.empty(n)
    for i, a in enumerate(ang):
        al = math.radians(a)
        # dirección en el marco local (x=sin, y=cos) -> mundo con rotación th
        lx, ly = math.sin(al), math.cos(al)
        dx = lx * math.cos(th) - ly * math.sin(th)
        dy = lx * math.sin(th) + ly * math.cos(th)
        best = 12.0
        # paredes de la sala centrada en (0.5, 0.5) aprox: x in [-W/2+.., ...]
        xmin, xmax, ymin, ymax = -2.0, 2.0, -1.5, 3.0
        for (p, d, lo, hi, other, olo, ohi) in ((x0, dx, xmin, xmax, y0, ymin, ymax),
                                                (y0, dy, ymin, ymax, x0, xmin, xmax)):
            if abs(d) < 1e-9:
                continue
            for wall in (lo, hi):
                t = (wall - p) / d
                if t > 0:
                    o = other + t * (dy if d is dx else dx)
                    if olo - 1e-6 <= o <= ohi + 1e-6:
                        best = min(best, t)
        for (cx, cy), r in arboles:
            fx, fy = x0 - cx, y0 - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if t > 0:
                    best = min(best, t)
        # ruido
        if rng is not None:
            best += rng.normal(0, 0.004)
        dist[i] = best
        # El LiDAR va encima del chasis: solo "ve" el propio robot en ciertos sectores (postes de las
        # esquinas, ruedas, cables); en el resto ve el entorno.
        if chasis is not None and any(lo <= a <= hi for lo, hi in SECTORES_CHASIS):
            hx, hy = chasis
            # intersección del rayo local con el rectángulo del chasis desde el centro
            tx = hx / abs(lx) if abs(lx) > 1e-9 else 1e9
            ty = hy / abs(ly) if abs(ly) > 1e-9 else 1e9
            dist[i] = min(dist[i], min(tx, ty))
    return ang, dist * 1000.0


def packets_from_scan(ang_deg, dist_mm, conf=200, per=12):
    """Convierte una revolución en paquetes LD06 de 47 bytes (12 puntos c/u)."""
    out = []
    n = len(ang_deg)
    for s in range(0, n - per + 1, per):
        a0 = ang_deg[s]
        a1 = ang_deg[s + per - 1]
        pkt = bytearray(47)
        pkt[0] = 0x54
        pkt[1] = 0x2C
        pkt[4:6] = int(round(a0 * 100)).to_bytes(2, "little")
        for k in range(per):
            off = 6 + 3 * k
            pkt[off:off + 2] = int(dist_mm[s + k]).to_bytes(2, "little")
            pkt[off + 2] = conf
        pkt[42:44] = int(round(a1 * 100) % 36000).to_bytes(2, "little")
        pkt[46] = _crc(bytes(pkt[:46]))
        out.append(bytes(pkt))
    return out
