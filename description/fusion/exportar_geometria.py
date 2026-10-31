"""Script de Fusion 360: exporta la geometría del ensamble de Cafebot para el código (posiciones de piezas, ejes de
los servos, cubo_camara, base del brazo) a una carpeta: geometria.json + STL en coordenadas del ensamble.

Cómo usarlo (una vez, con OPEN_CAFEBOT abierto y en la pose neutral: brazo recto, cámara mirando al frente):
  1. Utilidades > Complementos > Scripts y complementos (Shift+S) > pestaña Scripts > "+" > Crear script en Python
     (o "+" > vincular la carpeta description/fusion del repo y elegir exportar_geometria).
  2. Reemplazar el contenido del script por este archivo, guardar y "Ejecutar".
  3. Elegir una carpeta: se crea cafebot_export/ con geometria.json y los STL. Mandar esa carpeta (zip).

Todas las medidas del JSON están en CENTÍMETROS (unidades internas de Fusion) y en el sistema del ENSAMBLE (el de la
raíz). Las matrices son 4×4 por filas (rotación + traslación en la última columna).
"""
import json
import os
import traceback

import adsk.core
import adsk.fusion


def _p(pt):
    return [pt.x, pt.y, pt.z]


def _caja(bb):
    return {"min": _p(bb.minPoint), "max": _p(bb.maxPoint)}


def _matriz(m):
    a = list(m.asArray())
    return [a[0:4], a[4:8], a[8:12], a[12:16]]


def _geometria_junta(g):
    """JointGeometry o JointOrigin -> {origen, ejes}."""
    if g is None:
        return None
    jg = g
    if not hasattr(g, "primaryAxisVector") and hasattr(g, "geometry"):
        jg = g.geometry
    out = {}
    try:
        out["origen"] = _p(jg.origin)
    except Exception:
        pass
    for k in ("primaryAxisVector", "secondaryAxisVector", "thirdAxisVector"):
        try:
            v = getattr(jg, k)
            if v:
                out[k] = [v.x, v.y, v.z]
        except Exception:
            pass
    return out


def _movimiento(m):
    out = {"tipo": m.objectType.split("::")[-1]}
    for k in ("rotationValue", "slideValue"):
        try:
            out[k] = getattr(m, k)
        except Exception:
            pass
    try:
        v = m.rotationAxisVector
        if v:
            out["rotationAxisVector"] = [v.x, v.y, v.z]
    except Exception:
        pass
    try:
        out["rotationAxis"] = int(m.rotationAxis)
    except Exception:
        pass
    return out


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Abrir el diseño OPEN_CAFEBOT antes de ejecutar el script.")
            return
        root = design.rootComponent
        dlg = ui.createFolderDialog()
        dlg.title = "Carpeta donde dejar cafebot_export"
        if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        carpeta = os.path.join(dlg.folder, "cafebot_export")
        os.makedirs(carpeta, exist_ok=True)

        out = {"documento": app.activeDocument.name, "unidades": "cm",
               "unidades_del_documento": design.unitsManager.defaultLengthUnits,
               "parametros": [], "ocurrencias": [], "cuerpos": [], "juntas": [], "origenes_de_junta": [], "stl": []}
        for p in design.userParameters:
            out["parametros"].append({"nombre": p.name, "expresion": p.expression, "valor_cm": p.value})

        def cuerpos(contenedor, ruta):
            for b in contenedor.bRepBodies:
                d = {"ruta": ruta, "nombre": b.name, "visible": b.isVisible, "solido": b.isSolid,
                     "caja": _caja(b.boundingBox)}
                try:
                    pp = b.physicalProperties
                    d["centro_de_masa"] = _p(pp.centerOfMass)
                    d["volumen_cm3"] = pp.volume
                except Exception:
                    pass
                out["cuerpos"].append(d)

        cuerpos(root, "(raiz)")
        for occ in root.allOccurrences:
            out["ocurrencias"].append({"ruta": occ.fullPathName, "componente": occ.component.name,
                                       "matriz_mundo": _matriz(occ.transform2), "visible": occ.isVisible,
                                       "referencia_externa": occ.isReferencedComponent})
            cuerpos(occ, occ.fullPathName)

        def en_mundo(obj):
            """El objeto (junta u origen) en el contexto de la raíz: una copia por cada ocurrencia de su componente."""
            comp = obj.parentComponent
            if comp == root:
                return [(obj, "(raiz)")]
            res = []
            for occ in root.allOccurrencesByComponent(comp):
                try:
                    res.append((obj.createForAssemblyContext(occ), occ.fullPathName))
                except Exception:
                    pass
            return res

        for coleccion, tipo in ((root.allJoints, "junta"), (root.allAsBuiltJoints, "junta_como_construida")):
            for j in coleccion:
                for jj, ctx in en_mundo(j):
                    d = {"nombre": j.name, "tipo": tipo, "contexto": ctx}
                    try:
                        d["movimiento"] = _movimiento(jj.jointMotion)
                    except Exception:
                        pass
                    for k in ("geometryOrOriginOne", "geometryOrOriginTwo", "geometry"):
                        try:
                            g = getattr(jj, k)
                        except Exception:
                            continue
                        gg = _geometria_junta(g)
                        if gg:
                            d[k] = gg
                    for k in ("occurrenceOne", "occurrenceTwo"):
                        try:
                            o = getattr(jj, k)
                            d[k] = o.fullPathName if o else None
                        except Exception:
                            pass
                    out["juntas"].append(d)
        for jo in root.allJointOrigins:
            for jj, ctx in en_mundo(jo):
                out["origenes_de_junta"].append({"nombre": jo.name, "contexto": ctx,
                                                 "geometria": _geometria_junta(jj)})

        # STL: todo el ensamble y cada ocurrencia de primer nivel (en coordenadas del ensamble)
        em = design.exportManager

        def stl(geom, nombre):
            ruta = os.path.join(carpeta, nombre + ".stl")
            try:
                op = em.createSTLExportOptions(geom, ruta)
                op.isBinaryFormat = True
                op.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                em.execute(op)
                out["stl"].append(nombre + ".stl")
            except Exception as e:
                out["stl"].append(f"{nombre}.stl: ERROR {e}")

        stl(root, "00_ensamble_completo")
        for i, occ in enumerate(root.occurrences):
            limpio = "".join(c if c.isalnum() or c in "-_." else "_" for c in occ.name)[:60]
            stl(occ, f"{i + 1:02d}_{limpio}")

        with open(os.path.join(carpeta, "geometria.json"), "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1, ensure_ascii=False)
        ui.messageBox(f"Listo: {len(out['ocurrencias'])} ocurrencias, {len(out['cuerpos'])} cuerpos, "
                      f"{len(out['juntas'])} juntas.\n\n{carpeta}\n\nMandar esa carpeta (comprimida).")
    except Exception:
        if ui:
            ui.messageBox("Error:\n" + traceback.format_exc())
