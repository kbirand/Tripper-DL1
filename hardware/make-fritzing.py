#!/usr/bin/env python3
"""Generate tripper-puck.fzz — a self-contained Fritzing sketch of the DL1 puck.

    python3 hardware/make-fritzing.py

None of these boards exist in Fritzing's core or contrib libraries, so every
part is generated here and embedded in the archive: the .fzz opens on any
machine without installing anything into the parts bin. Edit PARTS/INST/NETS
below and re-run when the wiring changes — NETS is the netlist and the single
source of truth, the rest is placement.

Geometry follows Fritzing's conventions: part SVGs are authored in millimetres
(viewBox) with outer dimensions declared in inches, and sketch coordinates are
scene units at 90 per inch.

Verified end to end with Fritzing's own batch exporter:

    /Applications/Fritzing.app/Contents/MacOS/Fritzing -svg <folder>

which loads every sketch in the folder and writes an SVG per view — if that
produces all three views, the file is valid.
"""
import os, sys, zipfile, xml.etree.ElementTree as ET

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tripper-puck.fzz")

# Fritzing unpacks a sketch's custom parts into its partfactory cache keyed by
# moduleId, and reuses that copy forever. Regenerating a part under the same id
# therefore shows the *old* symbol no matter what the .fzz contains. Bump this
# whenever a part's SVG or pin list changes, so the ids are new to Fritzing.
MODULE_VERSION = "v3"

MM_PER_IN = 25.4
SCENE_PER_IN = 90.0
SCENE_PER_MM = SCENE_PER_IN / MM_PER_IN
GRID = 2.54                      # 0.1 in, in mm
PIN = 2.54                       # pin stub length, mm
PAD = 1.27                       # body padding above first / below last pin

# ---------------------------------------------------------------- part specs
# left/right are pin-name lists, top to bottom. A None entry leaves a gap.
# Each symbol carries two lines under the body: `name` is what the part IS,
# `note` is the one thing you need to know about it here. Keeping those two
# roles separate matters — mixing part numbers, modes and addresses into a
# single caption makes the sheet unreadable.
PARTS = {
    "xiao_esp32s3": dict(
        title="XIAO ESP32-S3", label="U", w=27.0,
        left=["D0", "D1", "D2", "D3", "D4", "D5", "D6"],
        right=["5V", "GND", "3V3", "D10", "D9", "D8", "D7"],
        name="XIAO ESP32-S3", note="BLE + on-chip TWAI CAN"),
    "gravity_10dof": dict(
        title="Gravity 10DOF (BNO055+BMP280)", label="U", w=30.0,
        left=["VCC", "GND", "SDA", "SCL"], right=[],
        name="Gravity 10DOF", note="BNO055 0x28 + BMP280 0x76, Wire @100k"),
    "ssd1306_oled": dict(
        title="SSD1306 OLED 128x32", label="DS", w=26.0,
        left=["VCC", "GND", "SDA", "SCK"], right=[],
        name="SSD1306 OLED 128x32", note="0x3C, on Wire1 at the handlebar"),
    "neo8m_gps": dict(
        title="u-blox NEO-8M (GYGPSV1)", label="U", w=28.0,
        left=["VCC", "RX", "TX", "GND"], right=[],
        name="u-blox NEO-8M GPS", note="UART 115200, 5 Hz, sky view"),
    "sn65hvd230": dict(
        title="SN65HVD230 CAN transceiver", label="U", w=28.0,
        left=["3V3", "GND", "CTX", "CRX"], right=["CANH", "CANL"],
        name="SN65HVD230 CAN PHY",
        note="250 kbit/s listen-only, remove its 120R"),
    "bec_5v": dict(
        title="5 V BEC 3 A", label="PS", w=24.0,
        left=["VIN", "GNDIN"], right=["VOUT", "GNDOUT"],
        name="5 V BEC, 3 A", note="steps the bike pack down to 5 V"),
    "bike": dict(
        title="E-bike (Talaria)", label="BK", w=26.0,
        left=[], right=["PACK+", "PACK-", "CAN_H", "CAN_L"],
        name="E-bike (Talaria)", note="traction pack + controller CAN bus"),
    # Both connectors are drawn as mated pass-throughs — left side is what
    # arrives, right side is what leaves — so the enclosure boundary is visible
    # on the sheet. Electrically each pin is still one node.
    "conn_m8_4p": dict(
        title="M8 4-pin (bike)", label="J", w=26.0,
        left=["1 +5V", "2 GND", "3 CAN_H", "4 CAN_L"],
        right=["1", "2", "3", "4"],
        name="M8 4-pin, bike harness",
        note="left = bike, right = rear box. Unplug before USB"),
    "conn_m12_8p": dict(
        title="M12 8-pin (handlebar)", label="J", w=28.0,
        left=["1 3V3", "2 GND", "3 SDA", "4 SCK", "5 GPS_RX", "6 GPS_TX",
              "7 BTN1", "8 BTN2"],
        right=["1", "2", "3", "4", "5", "6", "7", "8"],
        name="M12 8-pin, handlebar harness",
        note="left = rear box, right = bar box. 3.3 V only"),
    "res_2k2": dict(title="2.2 k", label="R", w=12.0,
                    left=["1"], right=["2"],
                    name="2.2 k", note="I2C pull-up to 3V3"),
    "res_10k": dict(title="10 k", label="R", w=12.0,
                    left=["1"], right=["2"],
                    name="10 k", note="button pull-up to 3V3"),
    "cap_100n": dict(title="100 nF", label="C", w=12.0,
                     left=["1"], right=["2"],
                     name="100 nF", note="button noise filter"),
    "cap_100u": dict(title="100 uF", label="C", w=12.0,
                     left=["+"], right=["-"],
                     name="100 uF", note="bulk, at the bar end"),
}


def geom(spec):
    """Body rect and per-connector pin geometry, in mm, origin at SVG 0,0."""
    n = max(len(spec["left"]), len(spec["right"]))
    bh = (n - 1) * GRID + 2 * PAD
    bw = spec["w"]
    x0, y0 = PIN, PAD          # body top-left; pins stick out left/right
    conns = []
    for i, name in enumerate(spec["left"]):
        conns.append(dict(name=name, side="L", x=0.0, y=PAD + i * GRID))
    for i, name in enumerate(spec["right"]):
        conns.append(dict(name=name, side="R", x=PIN + bw + PIN,
                          y=PAD + i * GRID))
    # extra room under the body for the two caption lines
    return dict(bw=bw, bh=bh, x0=x0, y0=y0,
                W=PIN + bw + PIN, H=bh + 2 * PAD + 2.2, conns=conns)


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def svg_header(W, H):
    return (f"<?xml version='1.0' encoding='UTF-8' standalone='no'?>\n"
            f"<svg xmlns='http://www.w3.org/2000/svg' version='1.2' "
            f"baseProfile='tiny' x='0in' y='0in' "
            f"width='{W/MM_PER_IN:.4f}in' height='{H/MM_PER_IN:.4f}in' "
            f"viewBox='0 0 {W:.4f} {H:.4f}'>\n")


def make_svg(spec, g, layer):
    """One SVG per view. Same drawing; layer group id and colours differ."""
    body_fill = "#FFFFFF" if layer == "schematic" else "#2E8B57"
    body_stroke = "#000000" if layer == "schematic" else "#1a5c38"
    txt = "#000000" if layer == "schematic" else "#FFFFFF"
    pin_col = "#787878" if layer == "schematic" else "#8C8C8C"
    s = [svg_header(g["W"], g["H"]), f"<g id='{layer}'>\n"]
    s.append(f"<rect x='{g['x0']:.4f}' y='{PAD/2:.4f}' width='{g['bw']:.4f}' "
             f"height='{g['bh']:.4f}' fill='{body_fill}' stroke='{body_stroke}' "
             f"stroke-width='0.3' rx='0.5'/>\n")
    for i, c in enumerate(g["conns"]):
        y = c["y"] + PAD / 2
        if c["side"] == "L":
            x1, x2, tx, anchor = 0.0, PIN, PIN + 0.6, "start"
            term_x = 0.0
        else:
            x1, x2 = g["W"], g["W"] - PIN
            tx, anchor = g["W"] - PIN - 0.6, "end"
            term_x = g["W"] - 0.25
        s.append(f"<line x1='{x1:.4f}' y1='{y:.4f}' x2='{x2:.4f}' y2='{y:.4f}' "
                 f"stroke='{pin_col}' stroke-width='0.25' stroke-linecap='round' "
                 f"id='connector{i}pin'/>\n")
        s.append(f"<rect x='{term_x:.4f}' y='{y-0.125:.4f}' width='0.25' "
                 f"height='0.25' fill='none' stroke='none' "
                 f"id='connector{i}terminal'/>\n")
        s.append(f"<text x='{tx:.4f}' y='{y+0.45:.4f}' font-family='Droid Sans' "
                 f"font-size='1.15' fill='{txt}' text-anchor='{anchor}'>"
                 f"{esc(c['name'])}</text>\n")
    s.append(f"<text id='label' x='{g['W']/2:.4f}' y='{g['H']-2.35:.4f}' "
             f"font-family='Droid Sans' font-size='1.45' font-weight='bold' "
             f"fill='{txt}' text-anchor='middle'>{esc(spec['name'])}</text>\n")
    s.append(f"<text x='{g['W']/2:.4f}' y='{g['H']-0.75:.4f}' "
             f"font-family='Droid Sans' font-size='1.05' "
             f"fill='{'#6b7a8c' if layer == 'schematic' else '#d8e6dd'}' "
             f"text-anchor='middle'>{esc(spec['note'])}</text>\n")
    s.append("</g>\n</svg>\n")
    return "".join(s)


def make_pcb_svg(spec, g):
    s = [svg_header(g["W"], g["H"])]
    for layer in ("copper0", "copper1"):
        s.append(f"<g id='{layer}'>\n")
        for i, c in enumerate(g["conns"]):
            y = c["y"] + PAD / 2
            cx = 0.6 if c["side"] == "L" else g["W"] - 0.6
            sfx = "" if layer == "copper1" else "0"
            s.append(f"<circle cx='{cx:.4f}' cy='{y:.4f}' r='0.7' fill='none' "
                     f"stroke='#F7BD13' stroke-width='0.5' "
                     f"id='connector{i}pin{sfx}'/>\n")
        s.append("</g>\n")
    s.append("</svg>\n")
    return "".join(s)


def make_fzp(mid, spec, g):
    s = [f"<?xml version='1.0' encoding='UTF-8'?>\n"
         f"<module fritzingVersion='1.0.3' moduleId='{module_id(mid)}'>\n"
         f" <version>1</version>\n <author>Tripper DL1</author>\n"
         f" <title>{esc(spec['title'])}</title>\n"
         f" <label>{spec['label']}</label>\n <date>2026-07-27</date>\n"
         f" <tags><tag>tripper</tag></tags>\n <properties>\n"
         f"  <property name='family'>Tripper DL1</property>\n"
         f"  <property name='part'>{esc(spec['title'])}</property>\n"
         f" </properties>\n"
         f" <description>{esc(spec['title'])} — Tripper Puck DL1</description>\n"
         f" <views>\n"
         f"  <iconView><layers image='schematic/{mid}_schem.svg'>"
         f"<layer layerId='schematic'/></layers></iconView>\n"
         f"  <breadboardView><layers image='breadboard/{mid}_bb.svg'>"
         f"<layer layerId='breadboard'/></layers></breadboardView>\n"
         f"  <schematicView><layers image='schematic/{mid}_schem.svg'>"
         f"<layer layerId='schematic'/></layers></schematicView>\n"
         f"  <pcbView><layers image='pcb/{mid}_pcb.svg'>"
         f"<layer layerId='copper0'/><layer layerId='copper1'/>"
         f"</layers></pcbView>\n </views>\n <connectors>\n"]
    for i, c in enumerate(g["conns"]):
        s.append(
            f"  <connector id='connector{i}' name='{esc(c['name'])}' type='male'>\n"
            f"   <description>{esc(c['name'])}</description>\n   <views>\n"
            f"    <breadboardView><p layer='breadboard' svgId='connector{i}pin' "
            f"terminalId='connector{i}terminal'/></breadboardView>\n"
            f"    <schematicView><p layer='schematic' svgId='connector{i}pin' "
            f"terminalId='connector{i}terminal'/></schematicView>\n"
            f"    <pcbView><p layer='copper0' svgId='connector{i}pin0'/>"
            f"<p layer='copper1' svgId='connector{i}pin'/></pcbView>\n"
            f"   </views>\n  </connector>\n")
    s.append(" </connectors>\n</module>\n")
    return "".join(s)


def module_id(mid):
    """Cache-busting module id — see MODULE_VERSION."""
    return f"tripper_{mid}_{MODULE_VERSION}"


GEO = {mid: geom(spec) for mid, spec in PARTS.items()}
IDX = {mid: {c["name"]: i for i, c in enumerate(GEO[mid]["conns"])}
       for mid in PARTS}

# ------------------------------------------------------------ sketch layout
# (instance name -> module, scene position in units at 90/inch)
# Laid out to keep each subsystem's wires short: bike/power on the left,
# the ESP32 in the middle with its passives stacked directly above it, and
# everything past the M12 connector on the right — the physical enclosure
# split reads left-to-right across the sheet.
INST = [
    ("BK1",   "bike",          30,  540),
    ("BEC1",  "bec_5v",       210,  430),
    ("J1",    "conn_m8_4p",   400,  540),
    ("U2",    "sn65hvd230",   210,  760),
    ("U3",    "gravity_10dof",400,  250),
    ("R1",    "res_2k2",      620,   40),
    ("R2",    "res_2k2",      620,   90),
    ("R3",    "res_10k",      620,  140),
    ("R4",    "res_10k",      620,  190),
    ("C1",    "cap_100n",     620,  240),
    ("C2",    "cap_100n",     620,  290),
    ("U1",    "xiao_esp32s3", 790,  360),
    ("J2",    "conn_m12_8p", 1010,  340),
    ("DS1",   "ssd1306_oled",1250,  120),
    ("U4",    "neo8m_gps",   1250,  250),
    ("C3",    "cap_100u",    1250,  380),
]
POS = {n: (m, x, y) for n, m, x, y in INST}

# ------------------------------------------------------------------- netlist
# each entry: (instanceA, pinA, instanceB, pinB)
NETS = [
    # --- bike side of the M8 connector: BEC output and the controller's bus
    ("BK1", "PACK+",  "BEC1", "VIN"),
    ("BK1", "PACK-",  "BEC1", "GNDIN"),
    ("BEC1", "VOUT",  "J1", "1 +5V"),
    ("BEC1", "GNDOUT","J1", "2 GND"),
    ("BK1", "CAN_H",  "J1", "3 CAN_H"),
    ("BK1", "CAN_L",  "J1", "4 CAN_L"),
    # --- rear-box side of the same four pins
    ("J1", "1", "U1", "5V"),
    ("J1", "2", "U1", "GND"),
    ("J1", "3", "U2", "CANH"),
    ("J1", "4", "U2", "CANL"),
    # --- CAN transceiver
    ("U2", "3V3", "U1", "3V3"),
    ("U2", "GND", "U1", "GND"),
    ("U2", "CTX", "U1", "D8"),
    ("U2", "CRX", "U1", "D9"),
    # --- rear I2C bus 0: sensors only, short
    ("U3", "VCC", "U1", "3V3"),
    ("U3", "GND", "U1", "GND"),
    ("U3", "SDA", "U1", "D4"),
    ("U3", "SCL", "U1", "D5"),
    # --- bus 1 pull-ups + button networks, all at the ESP32 end
    ("R1", "1", "U1", "D3"),   ("R1", "2", "U1", "3V3"),
    ("R2", "1", "U1", "D10"),  ("R2", "2", "U1", "3V3"),
    ("R3", "1", "U1", "D1"),   ("R3", "2", "U1", "3V3"),
    ("R4", "1", "U1", "D2"),   ("R4", "2", "U1", "3V3"),
    ("C1", "1", "U1", "D1"),   ("C1", "2", "U1", "GND"),
    ("C2", "1", "U1", "D2"),   ("C2", "2", "U1", "GND"),
    # --- rear side of the handlebar connector
    ("U1", "3V3", "J2", "1 3V3"),
    ("U1", "GND", "J2", "2 GND"),
    ("U1", "D3",  "J2", "3 SDA"),
    ("U1", "D10", "J2", "4 SCK"),
    ("U1", "D6",  "J2", "5 GPS_RX"),
    ("U1", "D7",  "J2", "6 GPS_TX"),
    ("U1", "D1",  "J2", "7 BTN1"),
    ("U1", "D2",  "J2", "8 BTN2"),
    # --- bar side of the handlebar connector
    ("J2", "1", "DS1", "VCC"),
    ("J2", "2", "DS1", "GND"),
    ("J2", "3", "DS1", "SDA"),
    ("J2", "4", "DS1", "SCK"),
    ("J2", "1", "U4", "VCC"),
    ("J2", "2", "U4", "GND"),
    ("J2", "5", "U4", "RX"),
    ("J2", "6", "U4", "TX"),
    ("J2", "1", "C3", "+"),
    ("J2", "2", "C3", "-"),
]

# Buttons live at the bar and return through the connector's GND pin. They are
# drawn as two-leg parts so the sketch shows the actual switch, not a stub.
PARTS["button"] = dict(title="Push button", label="SW", w=16.0,
                       left=["1"], right=["2"], name="Push button",
                       note="momentary, to GND through the harness")
GEO["button"] = geom(PARTS["button"])
IDX["button"] = {c["name"]: i for i, c in enumerate(GEO["button"]["conns"])}
INST += [("SW1", "button", 1250, 460), ("SW2", "button", 1250, 520)]
POS["SW1"] = ("button", 1250, 460)
POS["SW2"] = ("button", 1250, 520)
NETS += [("J2", "7", "SW1", "1"), ("SW1", "2", "J2", "2"),
         ("J2", "8", "SW2", "1"), ("SW2", "2", "J2", "2")]


def term_scene(inst, pin):
    """Absolute scene position of a connector's terminal point."""
    mid, px, py = POS[inst]
    g = GEO[mid]
    i = IDX[mid][pin]
    c = g["conns"][i]
    tx = 0.125 if c["side"] == "L" else g["W"] - 0.125
    ty = c["y"] + PAD / 2
    return (px + tx * SCENE_PER_MM, py + ty * SCENE_PER_MM), i


# --------------------------------------------------------------- build the fz
VIEWS = [("breadboardView", "breadboard", "breadboardWire"),
         ("schematicView", "schematic", "schematicTrace"),
         ("pcbView", "copper1", "copper1trace")]

model_index = {}
next_idx = 10
for name, mid, x, y in INST:
    model_index[name] = next_idx
    next_idx += 1

# connector -> list of (wire_model_index, wire_connector_id)
part_conn = {}
wires = []
for wi, (a, ap, b, bp) in enumerate(NETS):
    widx = next_idx
    next_idx += 1
    (ax, ay), ai = term_scene(a, ap)
    (bx, by), bi = term_scene(b, bp)
    wires.append(dict(idx=widx, x=ax, y=ay, dx=bx - ax, dy=by - ay,
                      a=(a, ai), b=(b, bi)))
    part_conn.setdefault((a, ai), []).append((widx, "connector0"))
    part_conn.setdefault((b, bi), []).append((widx, "connector1"))

out = ["<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
       "<module fritzingVersion=\"1.0.3\">\n",
       "  <boards/>\n  <views>\n",
       "    <view name=\"breadboardView\" backgroundColor=\"#ffffff\" "
       "gridSize=\"0.1in\" showGrid=\"1\" alignToGrid=\"0\" viewFromBelow=\"0\"/>\n",
       "    <view name=\"schematicView\" backgroundColor=\"#ffffff\" "
       "gridSize=\"0.1in\" showGrid=\"1\" alignToGrid=\"0\" viewFromBelow=\"0\"/>\n",
       "    <view name=\"pcbView\" backgroundColor=\"#333333\" "
       "gridSize=\"0.05in\" showGrid=\"1\" alignToGrid=\"0\" viewFromBelow=\"0\"/>\n",
       "  </views>\n  <instances>\n"]

for name, mid, x, y in INST:
    mi = model_index[name]
    out.append(f"    <instance moduleIdRef=\"{module_id(mid)}\" modelIndex=\"{mi}\" "
               f"path=\"part.{module_id(mid)}.fzp\">\n      <title>{name}</title>\n"
               f"      <views>\n")
    for vname, vlayer, _ in VIEWS:
        out.append(f"        <{vname} layer=\"{vlayer}\">\n"
                   f"          <geometry z=\"2.5\" x=\"{x}\" y=\"{y}\"/>\n"
                   f"          <connectors>\n")
        for i in range(len(GEO[mid]["conns"])):
            links = part_conn.get((name, i), [])
            if not links:
                continue
            out.append(f"            <connector connectorId=\"connector{i}\" "
                       f"layer=\"{vlayer}\">\n              <geometry x=\"0\" y=\"0\"/>\n"
                       f"              <connects>\n")
            for widx, wc in links:
                out.append(f"                <connect connectorId=\"{wc}\" "
                           f"modelIndex=\"{widx}\" layer=\"{VIEWS[[v[0] for v in VIEWS].index(vname)][2]}\"/>\n")
            out.append("              </connects>\n            </connector>\n")
        out.append("          </connectors>\n        </" + vname + ">\n")
    out.append("      </views>\n    </instance>\n")

for w in wires:
    out.append(f"    <instance moduleIdRef=\"WireModuleID\" "
               f"modelIndex=\"{w['idx']}\" path=\":/resources/parts/core/wire.fzp\">\n"
               f"      <title>Wire{w['idx']}</title>\n      <views>\n")
    for vname, vlayer, wlayer in VIEWS:
        col = "#404040" if vname != "pcbView" else "#f28a00"
        mils = "33.3333" if vname == "schematicView" else "22.2222"
        # Which view owns the wire decides how it draws. A wire flagged only
        # NormalFlag (64) belongs to breadboard, and schematic renders it as a
        # dotted ratsnest hint instead of a real line. SchematicTraceFlag (128)
        # plus RoutedFlag (2) makes it an actual routed schematic trace.
        flags = {"schematicView": 130, "pcbView": 68}.get(vname, 64)
        out.append(f"        <{vname} layer=\"{wlayer}\">\n"
                   f"          <geometry z=\"3.5\" x=\"{w['x']:.3f}\" y=\"{w['y']:.3f}\" "
                   f"x1=\"0\" y1=\"0\" x2=\"{w['dx']:.3f}\" y2=\"{w['dy']:.3f}\" "
                   f"wireFlags=\"{flags}\"/>\n"
                   f"          <wireExtras mils=\"{mils}\" color=\"{col}\" "
                   f"opacity=\"1\" banded=\"0\"/>\n          <connectors>\n")
        for wc, (pname, pi) in (("connector0", w["a"]), ("connector1", w["b"])):
            out.append(f"            <connector connectorId=\"{wc}\" layer=\"{wlayer}\">\n"
                       f"              <geometry x=\"0\" y=\"0\"/>\n"
                       f"              <connects>\n"
                       f"                <connect connectorId=\"connector{pi}\" "
                       f"modelIndex=\"{model_index[pname]}\" layer=\"{vlayer}\"/>\n"
                       f"              </connects>\n            </connector>\n")
        out.append("          </connectors>\n        </" + vname + ">\n")
    out.append("      </views>\n    </instance>\n")

out.append("  </instances>\n</module>\n")
fz = "".join(out)

# ------------------------------------------------------------------ validate
ET.fromstring(fz)
for mid, spec in PARTS.items():
    g = GEO[mid]
    for layer in ("schematic", "breadboard"):
        ET.fromstring(make_svg(spec, g, layer))
    ET.fromstring(make_pcb_svg(spec, g))
    ET.fromstring(make_fzp(mid, spec, g))

# This script can only regenerate the netlist and a machine placement — it
# cannot reproduce hand-tidied positions or wire routing. Once the .fzz has
# been laid out in Fritzing, that file is the source of truth and a silent
# overwrite would throw the work away, so refuse unless asked explicitly.
if os.path.exists(OUT) and "--force" not in sys.argv:
    raise SystemExit(
        f"{os.path.basename(OUT)} already exists.\n"
        "Re-running would discard any layout and routing done in Fritzing.\n"
        "Pass --force if you really want to rebuild it from scratch.")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("tripper-puck.fz", fz)
    for mid, spec in PARTS.items():
        g = GEO[mid]
        z.writestr(f"part.{module_id(mid)}.fzp", make_fzp(mid, spec, g))
        z.writestr(f"svg.schematic.{mid}_schem.svg", make_svg(spec, g, "schematic"))
        z.writestr(f"svg.breadboard.{mid}_bb.svg", make_svg(spec, g, "breadboard"))
        z.writestr(f"svg.pcb.{mid}_pcb.svg", make_pcb_svg(spec, g))

print(f"wrote {OUT}")
print(f"  parts   : {len(PARTS)}")
print(f"  symbols : {len(INST)}")
print(f"  wires   : {len(wires)}")
