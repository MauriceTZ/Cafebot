"""Visión SIMPLE para la cosecha por pisos (tools.ascensor, docs/ascensor.md): sin modelo de cámara.

6 nov: la proyección de los soportes con el modelo de la cámara no calzaba (cabezal girado, cámara torcida). La cámara
ahora solo dice QUÉ hay: en la foto del árbol busca los 3 PISOS (filas de soportes) y en cada piso qué soportes
tienen pelota y de qué color. Dónde está cada soporte lo dicen la línea del cultivo (distancia), el LiDAR (lado) y
las alturas conocidas de cada piso.

    1. Copa: la mancha VERDE más grande (camara.excluir_h), rellenada por dentro.
    2. Pelotas: manchas de color saturado y redondas dentro de la copa (granos.detectar, sin el verde).
    3. Soportes: lo que dentro de la copa NO es verde ni pelota (madera café, el hueco negro), con tamaño de soporte.
    4. Pisos: los soportes agrupados por altura en la foto (corrigiendo la inclinación de la foto); el de 6 soportes
       es el del medio. Columnas: de IZQUIERDA a derecha en la foto (= de +y a −y en el robot), 0..4 o 0..5.
    5. Cada pelota va al soporte que tiene justo debajo; su color por el matiz (rojo/naranjo/amarillo = MADURO,
       azul = SOBREMADURO). Negras: opcional (una sombra se parece).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from cafebot.mundo import cancha

N_COLUMNAS = {"alta": 5, "media": 6, "baja": 5}

# matiz OpenCV (0..180) -> color; [AJUSTAR con fotos de la cancha: la terminal muestra el H de cada pelota]
COLORES_H = (("ROJO", 0, 9), ("NARANJO", 9, 19), ("AMARILLO", 19, 36), ("AZUL", 90, 135), ("ROJO", 160, 181))
CATEGORIA = {"ROJO": cancha.MADURO, "NARANJO": cancha.MADURO, "AMARILLO": cancha.MADURO,
             "AZUL": cancha.SOBREMADURO, "NEGRO": cancha.SOBREMADURO}


@dataclass
class Soporte:
    fila: str
    col: int
    u: float
    v: float
    ancho: float
    alto: float
    inferido: bool = False       # no se vio: puesto por la grilla de los demás


@dataclass
class Pelota:
    u: float
    v: float
    r: float
    color: str                   # ROJO NARANJO AMARILLO AZUL NEGRO ?
    h: float                     # matiz medio (para ajustar COLORES_H)
    s: float
    fila: str | None = None
    col: int | None = None

    @property
    def categoria(self) -> str | None:
        return CATEGORIA.get(self.color)

    @property
    def clave(self):
        return (self.fila, self.col)


@dataclass
class Pisos:
    soportes: dict = field(default_factory=dict)     # fila -> [Soporte] por columna
    pelotas: list = field(default_factory=list)      # [Pelota]
    paso_px: float = 0.0                             # distancia entre columnas en la foto (12 cm)
    inclinacion: float = 0.0                         # dv/du de las filas (cámara torcida)
    centro_u: float = 0.0                            # columna central del árbol en la foto
    superficie: str = ""                             # de qué color se vio el árbol
    s0: float = 0.0                                  # saturación y brillo típicos de esa superficie
    candidatos: list = field(default_factory=list)   # (u, v, ancho, alto, área) de cada mancha que pudo ser soporte
    v0: float = 0.0
    mensajes: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return "media" in self.soportes and len(self.soportes) >= 2

    def lateral_camara_m(self, ancho_img: int) -> float:
        """Cuánto está el árbol a la IZQUIERDA del eje de la cámara (m), por la escala de las columnas (12 cm). Solo
        diagnóstico: incluye el giro del cabezal."""
        if self.paso_px <= 0:
            return 0.0
        return (ancho_img / 2 - self.centro_u) * 0.12 / self.paso_px

    def tabla(self) -> str:
        out = []
        for fila in ("alta", "media", "baja"):
            if fila not in self.soportes:
                out.append(f"  {fila:5s}: (no se ve)")
                continue
            celdas = []
            for s in self.soportes[fila]:
                p = next((q for q in self.pelotas if q.clave == (fila, s.col)), None)
                txt = "·" if p is None else f"{p.color}(H{p.h:.0f})"
                celdas.append(f"{s.col}:{txt}{'*' if s.inferido else ''}")
            out.append(f"  {fila:5s}: " + "  ".join(celdas))
        return "\n".join(out)


def color_de(h: float, s: float, v: float) -> str:
    if v < 70 and s < 110:
        return "NEGRO"
    for nombre, h0, h1 in COLORES_H:
        if h0 <= h < h1:
            return nombre
    return "?"


def _dist_h(h, h0):
    d = np.abs(h.astype(np.int16) - int(h0))
    return np.minimum(d, 180 - d)


def _superficie(hsv, excluir_h, sat_min):
    """La SUPERFICIE del árbol: el color que más hay en el centro de la foto (el verde de la cancha o la madera del
    árbol del taller). Devuelve (máscara de los píxeles de ese color, incluidas sus sombras: la saturación casi no
    cambia con la sombra; matices que NO son pelota (si la superficie es tan saturada como una pelota); texto)."""
    H, W = hsv.shape[:2]
    c = hsv[int(0.15 * H):int(0.85 * H), int(0.2 * W):int(0.8 * W)].reshape(-1, 3)
    c = c[(c[:, 1] >= 35) & (c[:, 2] >= 50)]
    if len(c) < 0.05 * (0.7 * H) * (0.6 * W):
        return None, None, "no veo una superficie de color en el centro de la foto (¿el árbol?)", (0.0, 0.0)
    hist = np.bincount(c[:, 0] // 6, minlength=30).astype(float)
    hist = hist + np.roll(hist, 1) + np.roll(hist, -1)
    h0 = int(np.argmax(hist)) * 6 + 3
    sel = c[_dist_h(c[:, 0], h0) <= 9]
    s0, v0 = float(np.median(sel[:, 1])), float(np.median(sel[:, 2]))
    if excluir_h and excluir_h[0] <= h0 <= excluir_h[1]:        # el verde de la cancha: su rango configurado
        hc, med = 0.5 * (excluir_h[0] + excluir_h[1]), 0.5 * (excluir_h[1] - excluir_h[0])
        nombre = "VERDE"
    else:
        hc, med = h0, 12
        nombre = "CAFÉ" if 5 <= h0 <= 30 else f"de matiz {h0}"
    mascara = ((_dist_h(hsv[..., 0], hc) <= med) & (hsv[..., 1] >= 0.6 * s0) & (hsv[..., 2] >= 0.3 * v0))
    excl = (int(hc - med), int(hc + med)) if s0 >= 0.75 * sat_min and hc - med >= 0 and hc + med <= 180 else None
    return mascara.astype(np.uint8), excl, f"árbol {nombre} (H {h0}, S {s0:.0f}, V {v0:.0f})", (s0, v0)


def _agrupar_filas(us, vs, n_filas=3, iter_=3):
    """Índices de cada fila (de arriba a abajo) y la inclinación común dv/du, cortando por los saltos en v."""
    pend = 0.0
    grupos = [list(range(len(us)))]
    for _ in range(iter_):
        vc = np.asarray(vs) - pend * np.asarray(us)
        orden = np.argsort(vc)
        saltos = np.diff(vc[orden])
        if len(saltos) == 0:
            return [list(orden)], pend
        k = min(n_filas - 1, len(saltos))
        cortes = sorted(np.argsort(saltos)[-k:])
        # un salto chico no separa filas: menos de 1/3 del mayor
        cortes = [c for c in cortes if saltos[c] > max(saltos) / 3]
        grupos, ini = [], 0
        for c in cortes:
            grupos.append([int(i) for i in orden[ini:c + 1]])
            ini = c + 1
        grupos.append([int(i) for i in orden[ini:]])
        # pendiente común: dentro de cada fila, v contra u
        num = den = 0.0
        for g in grupos:
            if len(g) >= 2:
                u = np.asarray(us)[g]
                v = np.asarray(vs)[g]
                du = u - u.mean()
                num += float(du @ (v - v.mean()))
                den += float(du @ du)
        nueva = num / den if den > 0 else 0.0
        if abs(nueva - pend) < 1e-3:
            pend = nueva
            break
        pend = nueva
    return grupos, pend


def analizar(img_bgr, excluir_h=(40, 88), sat_min: int = 130, val_min: int = 60, negros: bool = False) -> Pisos:
    import cv2
    from cafebot.vision import granos
    res = Pisos()
    excluir_h = excluir_h or (40, 88)          # sin excluir el verde, la copa entera sería una mancha
    H, W = img_bgr.shape[:2]
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    sup, excl, info, (res.s0, res.v0) = _superficie(hsv, excluir_h, sat_min)
    if sup is None:
        res.mensajes.append(info)
        return res
    res.superficie = info
    sup = cv2.morphologyEx(sup, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(sup)
    if n < 2:
        res.mensajes.append(f"no veo el árbol ({info})")
        return res
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < 0.05 * W * H:
        res.mensajes.append(f"la mancha más grande del color del árbol es chica ({100 * st[k, cv2.CC_STAT_AREA] / (W * H):.0f} %"
                            f" de la foto; {info}): ¿el árbol está en la foto?")
        return res
    cont, _ = cv2.findContours((lab == k).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    copa = np.zeros((H, W), np.uint8)
    cv2.drawContours(copa, cont, -1, 1, thickness=-1)
    copa_llena = copa.copy()
    # tamaños según la resolución (1536 px de ancho: 5 / 13 / 38)
    k_abrir, k_cerrar, k_anillo = max(3, round(W / 320)), max(5, round(W / 120)), max(9, round(W / 40))
    copa = cv2.erode(copa, np.ones((k_cerrar, k_cerrar), np.uint8))

    # pelotas: color saturado y redondas, dentro de la copa
    dets = granos.detectar(img_bgr, max(6.0, 0.008 * W), 0.06 * W, sat_min, val_min, excl)
    for d in dets:
        if not copa[min(H - 1, int(d.v)), min(W - 1, int(d.u))]:
            continue
        m = np.zeros((H, W), np.uint8)
        cv2.circle(m, (int(d.u), int(d.v)), max(2, int(0.6 * d.r)), 1, -1)
        px = hsv[m.astype(bool)]
        hh = px[:, 0].astype(float)
        if np.mean((hh < 30) | (hh > 150)) > 0.5:            # rojo: el matiz da la vuelta en 180
            hh = np.where(hh > 90, hh - 180, hh)
        h = float(np.median(hh)) % 180
        s, v = float(np.median(px[:, 1])), float(np.median(px[:, 2]))
        res.pelotas.append(Pelota(d.u, d.v, d.r, color_de(h, s, v), h, s))

    # soportes: dentro del árbol, lo que NO es del color de su superficie (las sombras sí lo son: la saturación no
    # cambia con la sombra) o es OSCURO y sin color (bordes quemados y hueco de los soportes del árbol de madera; las
    # sombras son oscuras pero conservan el color). En un anillo alrededor del árbol, solo lo oscuro: los soportes de
    # la fila de arriba quedan en el borde del contorno.
    # gris: sin el color de la superficie y no más claro que ella (la pared y una toalla son claras), o muy oscuro
    S_, V_ = hsv[..., 1], hsv[..., 2]
    oscuro = ((S_ < 0.65 * res.s0) & (V_ < 0.85 * res.v0)) | ((V_ < 0.55 * res.v0) & (S_ < 0.8 * res.s0))
    anillo = cv2.dilate(copa_llena, np.ones((k_anillo, k_anillo), np.uint8)).astype(bool) & ~copa.astype(bool)
    if res.s0 >= 100:      # árbol de color fuerte (el verde): soporte = lo que no es de ese color (madera, hueco)
        dentro = ~sup.astype(bool) | oscuro
    else:                  # árbol de madera: la veta y la luz cambian su brillo; soporte = lo gris (borde quemado,
        dentro = oscuro    # hueco, la repisa vista por debajo). Las pelotas no son grises: no hay que borrarlas
    objetos = ((copa.astype(bool) & dentro) | (anillo & oscuro)).astype(np.uint8)
    if res.s0 >= 100:
        for p in res.pelotas:
            cv2.circle(objetos, (int(p.u), int(p.v)), int(1.05 * p.r), 0, -1)
    objetos = cv2.morphologyEx(objetos, cv2.MORPH_CLOSE, np.ones((k_cerrar, k_cerrar), np.uint8))
    objetos = cv2.morphologyEx(objetos, cv2.MORPH_OPEN, np.ones((k_abrir, k_abrir), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(objetos)
    cand = []
    for i in range(1, n):
        a, w, h = st[i, cv2.CC_STAT_AREA], st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT]
        if a < 0.0003 * W * H or a > 0.03 * W * H or w > 0.2 * W or h > 0.2 * H or w < 0.9 * h:
            continue                  # (un soporte se ve más ancho que alto: es una repisa)
        cand.append((float(cen[i][0]), float(cen[i][1]), float(w), float(h), int(a)))
    if cand:
        med = float(np.median([c[4] for c in cand]))
        cand = [c for c in cand if c[4] > 0.2 * med]
    res.candidatos = cand
    if len(cand) < 4:
        res.mensajes.append(f"veo {len(cand)} soportes sobre el árbol: muy pocos ({info})")
        return res

    grupos, pend = _agrupar_filas([c[0] for c in cand], [c[1] for c in cand])
    res.inclinacion = pend
    # paso entre columnas: la mediana de las distancias entre vecinos de cada fila (sin los huecos de 2 pasos ni
    # pedazos de un mismo soporte: entre dos soportes hay al menos su ancho)
    ancho_med = float(np.median([c[2] for c in cand]))
    difs = []
    for g in grupos:
        us = sorted(cand[i][0] for i in g)
        difs += [d for d in np.diff(us) if d > 1.2 * ancho_med]
    paso = float(np.median(difs)) if difs else 0.0
    difs = [d for d in difs if d < 1.5 * paso]
    paso = float(np.median(difs)) if difs else paso
    res.paso_px = paso
    if paso <= 0:
        res.mensajes.append("no pude medir la distancia entre columnas")
        return res
    v0 = H / 2
    b_fila = [float(np.median([cand[i][1] - pend * cand[i][0] for i in g])) for g in grupos]   # v = b + pend·u

    def grilla(c0, b, nc):
        """Columnas 0..nc-1 de una fila (v = b + pend·u) con el centro del árbol en c0 (a la altura v0); las
        columnas se inclinan igual que la foto (u = ... − pend·(v − v0))."""
        a = c0 + (np.arange(nc) - (nc - 1) / 2) * paso
        u = (a - pend * (b - v0)) / (1 + pend * pend)
        return u, b + pend * u

    def puntaje(c0, nombres):
        """+1 cada soporte de la grilla que calza con uno visto; −1 si debía verse (dentro de la foto y de la copa)
        y no está; −2 si cae dentro de la foto pero fuera de la copa (imposible); −0,5 cada mancha que sobra."""
        total, asign = 0.0, {}
        for f, g, b in zip(nombres, grupos, b_fila):
            us, vs = grilla(c0, b, N_COLUMNAS[f])
            libres = set(g)
            for col, (u, v) in enumerate(zip(us, vs)):
                i = min(libres, key=lambda j: abs(cand[j][0] - u) + abs(cand[j][1] - v), default=None)
                if i is not None and abs(cand[i][0] - u) < 0.4 * paso and abs(cand[i][1] - v) < 0.5 * paso:
                    libres.discard(i)
                    asign[(f, col)] = i
                    total += 1
                elif 0.2 * paso < u < W - 0.2 * paso and 0 <= v < H:
                    total -= 1 if copa_llena[int(v), int(u)] else 2
            total -= 0.5 * len(libres)
        return total, asign

    opciones = {3: [("alta", "media", "baja")], 2: [("alta", "media"), ("media", "baja")],
                1: [("alta",), ("media",), ("baja",)]}.get(len(grupos), [])
    if not opciones:
        res.mensajes.append(f"{len(grupos)} grupos de soportes ({[len(g) for g in grupos]}): esperaba 3 pisos")
        return res
    mejor = None
    for nombres in opciones:
        for f, g, b in zip(nombres, grupos, b_fila):
            nc = N_COLUMNAS[f]
            for i in g:                             # cada soporte visto puede ser cualquier columna de su fila
                for col in range(nc):
                    a = cand[i][0] * (1 + pend * pend) + pend * (b - v0)       # inversa de grilla()
                    c0 = a - (col - (nc - 1) / 2) * paso
                    pt, asign = puntaje(c0, nombres)
                    if mejor is None or pt > mejor[0] + 1e-9:
                        mejor = (pt, nombres, c0, asign)
    pt, nombres, c0, asign = mejor
    res.centro_u = float(c0)
    n_vistos = sum(len(g) for g in grupos)
    if len(asign) < 0.6 * n_vistos:
        res.mensajes.append(f"solo {len(asign)} de {n_vistos} soportes calzan con la grilla de 5/6/5")
    for f, g, b in zip(nombres, grupos, b_fila):
        us, vs = grilla(c0, b, N_COLUMNAS[f])
        fila = []
        for col, (u, v) in enumerate(zip(us, vs)):
            i = asign.get((f, col))
            if i is not None:
                fila.append(Soporte(f, col, cand[i][0], cand[i][1], cand[i][2], cand[i][3]))
            else:
                fila.append(Soporte(f, col, float(u), float(v), 0.0, 0.0, inferido=True))
        res.soportes[f] = fila
        sobran = sum(1 for i in g if i not in asign.values())
        if sobran:
            res.mensajes.append(f"{f}: {sobran} mancha(s) que no calzan con la grilla de soportes (ignoradas)")

    # cada pelota, al soporte que tiene justo debajo
    for p in res.pelotas:
        mejor = None
        for f, fila in res.soportes.items():
            for s in fila:
                du, dv = s.u - p.u, s.v - p.v
                if abs(du) < 0.45 * paso and -0.5 * p.r < dv < 3.5 * p.r:
                    d = abs(du) + abs(dv - 1.2 * p.r)
                    if mejor is None or d < mejor[0]:
                        mejor = (d, f, s.col)
        if mejor is not None:
            p.fila, p.col = mejor[1], mejor[2]
        else:
            res.mensajes.append(f"pelota {p.color} en ({p.u:.0f}, {p.v:.0f}) sin soporte debajo: la ignoro")

    if negros:          # pelota negra: oscura y redonda justo encima de un soporte vacío
        ocupados = {p.clave for p in res.pelotas}
        r = 0.175 * paso
        for f, fila in res.soportes.items():
            for s in fila:
                if (f, s.col) in ocupados or s.inferido:
                    continue
                u, v = int(s.u), int(s.v - 1.2 * r)
                if not (0 <= u < W and 0 <= v < H):
                    continue
                m = np.zeros((H, W), np.uint8)
                cv2.circle(m, (u, v), max(2, int(0.6 * r)), 1, -1)
                px = hsv[m.astype(bool)]
                if np.median(px[:, 2]) < 60:
                    res.pelotas.append(Pelota(u, v, r, "NEGRO", float(np.median(px[:, 0])),
                                              float(np.median(px[:, 1])), f, s.col))
    return res


def dibujar(img_bgr, res: Pisos):
    import cv2
    out = img_bgr.copy()
    col_fila = {"alta": (255, 160, 0), "media": (0, 200, 255), "baja": (255, 0, 255)}
    for f, fila in res.soportes.items():
        for s in fila:
            c = col_fila[f]
            if s.inferido:
                cv2.circle(out, (int(s.u), int(s.v)), 10, c, 2)
            else:
                cv2.rectangle(out, (int(s.u - s.ancho / 2), int(s.v - s.alto / 2)),
                              (int(s.u + s.ancho / 2), int(s.v + s.alto / 2)), c, 3)
            cv2.putText(out, f"{f[0].upper()}{s.col}", (int(s.u - 15), int(s.v + s.alto / 2 + 28)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, c, 2)
    for (u, v, w, h, _) in res.candidatos:          # todo lo que pudo ser soporte (gris)
        cv2.rectangle(out, (int(u - w / 2), int(v - h / 2)), (int(u + w / 2), int(v + h / 2)), (160, 160, 160), 1)
    for p in res.pelotas:
        c = (0, 255, 0) if p.categoria == cancha.MADURO else (255, 255, 255) if p.categoria else (0, 0, 255)
        cv2.circle(out, (int(p.u), int(p.v)), int(p.r), c, 3)
        txt = f"{p.color} H{p.h:.0f}" + ("" if p.fila is None else f" {p.fila}-{p.col}")
        cv2.putText(out, txt, (int(p.u - p.r), int(p.v - p.r - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 5)
        cv2.putText(out, txt, (int(p.u - p.r), int(p.v - p.r - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)
    lineas = [f"pisos: {', '.join(res.soportes) or '-'} | paso {res.paso_px:.0f} px | torcida "
              f"{math.degrees(math.atan(res.inclinacion)):+.1f} grados | {res.superficie}"] + res.mensajes[:3]
    for i, t in enumerate(lineas):
        y = 35 + 34 * i
        cv2.putText(out, t, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5)
        cv2.putText(out, t, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return out
