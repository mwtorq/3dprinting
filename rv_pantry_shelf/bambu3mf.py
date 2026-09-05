#!/usr/bin/env python3
"""
Write a Bambu-Studio-compatible .3mf (geometry + EMBEDDED print settings).

A plain trimesh .3mf carries geometry only, so Bambu Studio shows "The 3mf is not
from Bambu Lab, load geometry data only" and you must dial in every setting by hand.
This module instead writes the same package layout Bambu Studio itself writes, so the
file opens as a real project with the process/filament/printer + per-object support
already set, no warning, nothing to re-enter.

What makes a .3mf "a Bambu project" (reverse-engineered from a real A1 export):
  • 3D/3dmodel.model — core mesh + Application metadata + production ('p') extension.
  • Metadata/project_settings.config — process/filament/printer JSON from the template
    with overrides applied (ONE profile per project; per-object keys refine parts).
  • Metadata/model_settings.config — object names, multi-plate mapping, per-object overrides.
  • Metadata/slice_info.config, [Content_Types].xml, _rels/.rels — package glue.

MULTI-PLATE: pass a list of plate dicts so ONE .3mf opens with every plate tab filled.
Plates use Bambu's PartPlate grid (cols = round(sqrt(n)), stride = bed*1.2, rows toward −Y).

Bambu drops sidecar configs if you go through lib3mf, so we assemble the ZIP by hand.
"""
import zipfile, json, uuid, os, math
from xml.sax.saxutils import escape
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, "bambu_profile_template.json")
BED = 256.0                                  # A1 bed; local plate centre at (128,128)

_MODEL_OPEN = ('<model unit="millimeter" xml:lang="en-US" '
   'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
   'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
   'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" '
   'requiredextensions="p">')

def _u():
    return str(uuid.uuid4())

def _a(s):
    """XML-escape an attribute value (&, <, >, \")."""
    return escape(str(s), {'"': "&quot;"})

def _plate_cols(n):
    """Bambu compute_colum_count(): columns for n plates (PartPlate.cpp)."""
    if n <= 0:
        return 1
    v = math.sqrt(n)
    r = math.floor(v + 0.5)   # C++ round() = half away from zero
    return int(r + 1 if v > r else max(1, r))

def _mesh_xml(mesh):
    """<mesh> with vertices + triangles, fast-formatted from numpy arrays."""
    v = np.asarray(mesh.vertices, dtype=float)
    f = np.asarray(mesh.faces,    dtype=np.int64)
    vx = np.char.mod('     <vertex x="%.6f"', v[:, 0])
    vy = np.char.mod(' y="%.6f"', v[:, 1])
    vz = np.char.mod(' z="%.6f"/>', v[:, 2])
    verts = "\n".join(np.char.add(np.char.add(vx, vy), vz))
    t1 = np.char.mod('     <triangle v1="%d"', f[:, 0])
    t2 = np.char.mod(' v2="%d"', f[:, 1])
    t3 = np.char.mod(' v3="%d"/>', f[:, 2])
    tris = "\n".join(np.char.add(np.char.add(t1, t2), t3))
    return ("   <mesh>\n    <vertices>\n%s\n    </vertices>\n"
            "    <triangles>\n%s\n    </triangles>\n   </mesh>" % (verts, tris))

def _centered(mesh, pos):
    """Centre the part in XY at `pos` and drop its lowest point to z=0 (print bed)."""
    m = mesh.copy()
    b = m.bounds
    ctr = (b[0] + b[1]) / 2.0
    m.apply_translation([pos[0] - ctr[0], pos[1] - ctr[1], -b[0][2]])
    return m

def write_bambu_3mf(path, plates, overrides, plate_name=None):
    """Write a multi-plate Bambu project .3mf (single file, all plates).

    plates: list of {"name": str, "parts": [ {name, mesh, pos(x,y), obj_settings}, ...]}.
      Each plate is a 256×256 bed. Plates layout matches Bambu's PartPlate grid so every
      part sits on its plater tab. Backward-compat: a flat list of part dicts (items with
      a "mesh" key) is treated as a single plate named via plate_name.
    overrides: project_settings.config keys (global process/filament/printer profile).
    """
    proj = json.load(open(TEMPLATE))
    proj.update(overrides)

    if plates and "mesh" in plates[0]:                 # flat parts list -> one plate
        plates = [{"name": plate_name or "Plate", "parts": plates}]

    # PartPlate grid: cols = compute_colum_count(n); stride = bed*(1+1/5);
    # plate i origin = (col*stride, -row*stride) with row,col = divmod(i, cols).
    cols = _plate_cols(len(plates))
    stride = BED * (1.0 + 0.2)                          # LOGICAL_PART_PLATE_GAP = 1/5
    objs, builds, msobjs, plate_blocks = [], [], [], []
    oid = 2                                            # object ids start at 2 (Bambu convention)
    for pi, plate in enumerate(plates):
        row, col = divmod(pi, cols)
        ox, oy = col * stride, -row * stride           # plate i's bed-origin in the scene
        instances = []
        for prt in plate["parts"]:
            m = _centered(prt["mesh"], (ox + BED/2 + prt["pos"][0], oy + BED/2 + prt["pos"][1]))
            objs.append('  <object id="%d" p:UUID="%s" type="model">\n%s\n  </object>'
                        % (oid, _u(), _mesh_xml(m)))
            builds.append('  <item objectid="%d" p:UUID="%s" transform="1 0 0 0 1 0 0 0 1 0 0 0" '
                          'printable="1"/>' % (oid, _u()))
            md = ['    <metadata key="name" value="%s"/>' % _a(prt["name"]),
                  '    <metadata key="extruder" value="1"/>']
            for k, v in prt.get("obj_settings", {}).items():
                md.append('    <metadata key="%s" value="%s"/>' % (_a(k), _a(v)))
            msobjs.append('  <object id="%d">\n%s\n  </object>' % (oid, "\n".join(md)))
            instances.append('    <model_instance>\n'
                             '      <metadata key="object_id" value="%d"/>\n'
                             '      <metadata key="instance_id" value="0"/>\n'
                             '    </model_instance>' % oid)
            oid += 1
        # Plate names must be filename-safe: no < > : / \ | ? * "
        pname = plate.get("name") or ("Plate %d" % (pi + 1))
        plate_blocks.append('  <plate>\n'
                            '    <metadata key="plater_id" value="%d"/>\n'
                            '    <metadata key="plater_name" value="%s"/>\n'
                            '    <metadata key="locked" value="false"/>\n%s\n'
                            '  </plate>'
                            % (pi + 1, _a(pname), "\n".join(instances)))

    model = ('<?xml version="1.0" encoding="UTF-8"?>\n%s\n'
             ' <metadata name="Application">BambuStudio-02.06.00.51</metadata>\n'
             ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n'
             ' <resources>\n%s\n </resources>\n'
             ' <build p:UUID="%s">\n%s\n </build>\n</model>\n'
             % (_MODEL_OPEN, "\n".join(objs), _u(), "\n".join(builds)))

    model_settings = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n%s\n%s\n</config>\n'
                      % ("\n".join(msobjs), "\n".join(plate_blocks)))

    proj_cfg = json.dumps(proj, indent=4)

    content_types = ('<?xml version="1.0" encoding="UTF-8"?>\n'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">\n'
        ' <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>\n'
        ' <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>\n'
        ' <Default Extension="png" ContentType="image/png"/>\n</Types>\n')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
        ' <Relationship Target="/3D/3dmodel.model" Id="rel-1" '
        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>\n')
    slice_info = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n  <header>\n'
        '    <header_item key="X-BBL-Client-Type" value="slicer"/>\n'
        '    <header_item key="X-BBL-Client-Version" value="02.06.00.51"/>\n'
        '  </header>\n</config>\n')

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", content_types)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)
        z.writestr("Metadata/project_settings.config", proj_cfg)
        z.writestr("Metadata/model_settings.config", model_settings)
        z.writestr("Metadata/slice_info.config", slice_info)
    return path
