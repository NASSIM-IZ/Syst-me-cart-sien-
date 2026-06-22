#!/usr/bin/env python3
"""
Local SVG converter for the Cartesian drawing interface.

Run:
  python svg_server.py

Endpoint:
  POST http://127.0.0.1:8000/convert-svg

The body must be SVG text. The response contains:
  - segments: converted XY points in machine millimeters
  - commands: Arduino commands compatible with the sketch: m..v.. and v..x..y..
  - grbl: optional GRBL-like G0/G1 coordinates for inspection
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from urllib.parse import parse_qs, urlparse


HOST = "127.0.0.1"
PORT = 8000

DEFAULT_MACHINE_W = 300.0
DEFAULT_MACHINE_H = 280.0
DEFAULT_SPEED = 50.0
DEFAULT_Z_HIGH = 80.0
DEFAULT_Z_LOW = 35.0
DEFAULT_Z_RPM = 5.0
DEFAULT_MARGIN = 0.0
DEFAULT_CURVE_SAMPLES = 24

NUMBER_RE = r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?"
TOKEN_RE = re.compile(r"[AaCcHhLlMmQqSsTtVvZz]|" + NUMBER_RE)
NUM_RE = re.compile(NUMBER_RE)


def local_name(tag):
    return tag.rsplit("}", 1)[-1].lower()


def as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def fmt_num(value):
    value = round(float(value), 2)
    if abs(value) < 0.005:
        value = 0.0
    text = f"{value:.2f}".rstrip("0").rstrip(".")
    return text if text else "0"


def parse_length(value, default=0.0):
    if value is None:
        return default
    text = str(value).strip()
    match = re.match(r"^\s*(" + NUMBER_RE + r")\s*([a-zA-Z%]*)", text)
    if not match:
        return default
    n = float(match.group(1))
    unit = match.group(2).lower()
    if unit == "cm":
        return n * 10.0
    if unit == "in":
        return n * 25.4
    if unit == "pt":
        return n * 25.4 / 72.0
    if unit == "pc":
        return n * 25.4 / 6.0
    # For px or no unit, the later fit-to-machine step removes ambiguity.
    return n


def mat_identity():
    return (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def mat_mul(a, b):
    return (
        a[0] * b[0] + a[2] * b[1],
        a[1] * b[0] + a[3] * b[1],
        a[0] * b[2] + a[2] * b[3],
        a[1] * b[2] + a[3] * b[3],
        a[0] * b[4] + a[2] * b[5] + a[4],
        a[1] * b[4] + a[3] * b[5] + a[5],
    )


def mat_apply(m, p):
    x, y = p
    return (m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5])


def mat_translate(tx, ty):
    return (1.0, 0.0, 0.0, 1.0, tx, ty)


def mat_scale(sx, sy):
    return (sx, 0.0, 0.0, sy, 0.0, 0.0)


def mat_rotate(deg):
    a = math.radians(deg)
    c = math.cos(a)
    s = math.sin(a)
    return (c, s, -s, c, 0.0, 0.0)


def parse_transform(text):
    if not text:
        return mat_identity()
    matrix = mat_identity()
    for name, args_text in re.findall(r"([a-zA-Z]+)\s*\(([^)]*)\)", text):
        args = [float(x) for x in NUM_RE.findall(args_text)]
        name = name.lower()
        part = mat_identity()
        if name == "matrix" and len(args) >= 6:
            part = tuple(args[:6])
        elif name == "translate" and args:
            part = mat_translate(args[0], args[1] if len(args) > 1 else 0.0)
        elif name == "scale" and args:
            part = mat_scale(args[0], args[1] if len(args) > 1 else args[0])
        elif name == "rotate" and args:
            if len(args) >= 3:
                part = mat_mul(mat_translate(args[1], args[2]), mat_mul(mat_rotate(args[0]), mat_translate(-args[1], -args[2])))
            else:
                part = mat_rotate(args[0])
        elif name == "skewx" and args:
            part = (1.0, 0.0, math.tan(math.radians(args[0])), 1.0, 0.0, 0.0)
        elif name == "skewy" and args:
            part = (1.0, math.tan(math.radians(args[0])), 0.0, 1.0, 0.0, 0.0)
        matrix = mat_mul(matrix, part)
    return matrix


def cubic(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (
        u**3 * p0[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * p3[0],
        u**3 * p0[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * p3[1],
    )


def quad(p0, p1, p2, t):
    u = 1.0 - t
    return (
        u**2 * p0[0] + 2 * u * t * p1[0] + t**2 * p2[0],
        u**2 * p0[1] + 2 * u * t * p1[1] + t**2 * p2[1],
    )


def arc_points(x1, y1, rx, ry, phi_deg, large_arc, sweep, x2, y2, samples):
    if rx == 0 or ry == 0:
        return [(x2, y2)]

    phi = math.radians(phi_deg)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)
    dx2 = (x1 - x2) / 2.0
    dy2 = (y1 - y2) / 2.0
    x1p = cos_phi * dx2 + sin_phi * dy2
    y1p = -sin_phi * dx2 + cos_phi * dy2

    rx = abs(rx)
    ry = abs(ry)
    lam = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry)
    if lam > 1.0:
        s = math.sqrt(lam)
        rx *= s
        ry *= s

    num = rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p
    den = rx * rx * y1p * y1p + ry * ry * x1p * x1p
    coef = 0.0 if den == 0 else math.sqrt(max(0.0, num / den))
    if large_arc == sweep:
        coef = -coef

    cxp = coef * (rx * y1p / ry)
    cyp = coef * (-ry * x1p / rx)
    cx = cos_phi * cxp - sin_phi * cyp + (x1 + x2) / 2.0
    cy = sin_phi * cxp + cos_phi * cyp + (y1 + y2) / 2.0

    def angle(ux, uy, vx, vy):
        dot = ux * vx + uy * vy
        length = math.sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy))
        if length == 0:
            return 0.0
        a = math.acos(max(-1.0, min(1.0, dot / length)))
        if ux * vy - uy * vx < 0:
            a = -a
        return a

    theta1 = angle(1.0, 0.0, (x1p - cxp) / rx, (y1p - cyp) / ry)
    dtheta = angle((x1p - cxp) / rx, (y1p - cyp) / ry, (-x1p - cxp) / rx, (-y1p - cyp) / ry)
    if not sweep and dtheta > 0:
        dtheta -= 2.0 * math.pi
    if sweep and dtheta < 0:
        dtheta += 2.0 * math.pi

    n = max(4, int(samples))
    pts = []
    for i in range(1, n + 1):
        t = theta1 + (i / n) * dtheta
        x = cx + rx * math.cos(t) * cos_phi - ry * math.sin(t) * sin_phi
        y = cy + rx * math.cos(t) * sin_phi + ry * math.sin(t) * cos_phi
        pts.append((x, y))
    return pts


def parse_path(d, samples):
    tokens = TOKEN_RE.findall(d or "")
    i = 0
    cmd = None
    cur = (0.0, 0.0)
    start = (0.0, 0.0)
    segments = []
    seg = None
    last_cubic_ctrl = None
    last_quad_ctrl = None
    last_cmd = None

    def is_cmd(tok):
        return len(tok) == 1 and tok.isalpha()

    def has_numbers(count):
        if i + count > len(tokens):
            return False
        return all(not is_cmd(tokens[i + offset]) for offset in range(count))

    def read_num():
        nonlocal i
        v = float(tokens[i])
        i += 1
        return v

    def end_seg():
        nonlocal seg
        if seg and len(seg) > 1:
            segments.append(seg)
        seg = None

    def push(p):
        nonlocal seg
        if seg is None:
            seg = [cur]
        if not seg or abs(seg[-1][0] - p[0]) > 1e-9 or abs(seg[-1][1] - p[1]) > 1e-9:
            seg.append(p)

    while i < len(tokens):
        if is_cmd(tokens[i]):
            cmd = tokens[i]
            i += 1
        if cmd is None:
            break

        c = cmd
        rel = c.islower()
        cu = c.upper()

        if cu == "M":
            if not has_numbers(2):
                break
            x, y = read_num(), read_num()
            cur = (cur[0] + x, cur[1] + y) if rel else (x, y)
            start = cur
            end_seg()
            seg = [cur]
            while has_numbers(2):
                x, y = read_num(), read_num()
                cur = (cur[0] + x, cur[1] + y) if rel else (x, y)
                push(cur)
            cmd = "l" if rel else "L"
        elif cu == "L":
            while has_numbers(2):
                x, y = read_num(), read_num()
                cur = (cur[0] + x, cur[1] + y) if rel else (x, y)
                push(cur)
        elif cu == "H":
            while has_numbers(1):
                x = read_num()
                cur = (cur[0] + x, cur[1]) if rel else (x, cur[1])
                push(cur)
        elif cu == "V":
            while has_numbers(1):
                y = read_num()
                cur = (cur[0], cur[1] + y) if rel else (cur[0], y)
                push(cur)
        elif cu == "C":
            while has_numbers(6):
                x1, y1, x2, y2, x3, y3 = [read_num() for _ in range(6)]
                p1 = (cur[0] + x1, cur[1] + y1) if rel else (x1, y1)
                p2 = (cur[0] + x2, cur[1] + y2) if rel else (x2, y2)
                p3 = (cur[0] + x3, cur[1] + y3) if rel else (x3, y3)
                for k in range(1, samples + 1):
                    push(cubic(cur, p1, p2, p3, k / samples))
                cur = p3
                last_cubic_ctrl = p2
        elif cu == "S":
            while has_numbers(4):
                if last_cmd and last_cmd.upper() in ("C", "S") and last_cubic_ctrl:
                    p1 = (2 * cur[0] - last_cubic_ctrl[0], 2 * cur[1] - last_cubic_ctrl[1])
                else:
                    p1 = cur
                x2, y2, x3, y3 = [read_num() for _ in range(4)]
                p2 = (cur[0] + x2, cur[1] + y2) if rel else (x2, y2)
                p3 = (cur[0] + x3, cur[1] + y3) if rel else (x3, y3)
                for k in range(1, samples + 1):
                    push(cubic(cur, p1, p2, p3, k / samples))
                cur = p3
                last_cubic_ctrl = p2
        elif cu == "Q":
            while has_numbers(4):
                x1, y1, x2, y2 = [read_num() for _ in range(4)]
                p1 = (cur[0] + x1, cur[1] + y1) if rel else (x1, y1)
                p2 = (cur[0] + x2, cur[1] + y2) if rel else (x2, y2)
                for k in range(1, samples + 1):
                    push(quad(cur, p1, p2, k / samples))
                cur = p2
                last_quad_ctrl = p1
        elif cu == "T":
            while has_numbers(2):
                if last_cmd and last_cmd.upper() in ("Q", "T") and last_quad_ctrl:
                    p1 = (2 * cur[0] - last_quad_ctrl[0], 2 * cur[1] - last_quad_ctrl[1])
                else:
                    p1 = cur
                x2, y2 = read_num(), read_num()
                p2 = (cur[0] + x2, cur[1] + y2) if rel else (x2, y2)
                for k in range(1, samples + 1):
                    push(quad(cur, p1, p2, k / samples))
                cur = p2
                last_quad_ctrl = p1
        elif cu == "A":
            while has_numbers(7):
                rx, ry, rot, laf, swp, x2, y2 = [read_num() for _ in range(7)]
                end = (cur[0] + x2, cur[1] + y2) if rel else (x2, y2)
                for p in arc_points(cur[0], cur[1], rx, ry, rot, bool(laf), bool(swp), end[0], end[1], samples):
                    push(p)
                cur = end
        elif cu == "Z":
            if seg is not None:
                push(start)
            cur = start
            end_seg()
        else:
            break

        if cu not in ("C", "S"):
            last_cubic_ctrl = None
        if cu not in ("Q", "T"):
            last_quad_ctrl = None
        last_cmd = c

    end_seg()
    return segments


def parse_points_attr(text):
    nums = [float(x) for x in NUM_RE.findall(text or "")]
    return [(nums[i], nums[i + 1]) for i in range(0, len(nums) - 1, 2)]


def transform_segment(seg, matrix):
    return [mat_apply(matrix, p) for p in seg]


def collect_svg_segments(svg_text, samples):
    root = ET.fromstring(svg_text)
    segments = []

    def add(seg, matrix, close=False):
        if close and seg and seg[0] != seg[-1]:
            seg = seg + [seg[0]]
        seg = transform_segment(seg, matrix)
        if len(seg) > 1:
            segments.append(seg)

    def walk(el, parent_matrix):
        matrix = mat_mul(parent_matrix, parse_transform(el.attrib.get("transform")))
        tag = local_name(el.tag)

        if tag == "path":
            for seg in parse_path(el.attrib.get("d", ""), samples):
                add(seg, matrix)
        elif tag == "polyline":
            add(parse_points_attr(el.attrib.get("points")), matrix)
        elif tag == "polygon":
            add(parse_points_attr(el.attrib.get("points")), matrix, close=True)
        elif tag == "line":
            x1 = parse_length(el.attrib.get("x1"))
            y1 = parse_length(el.attrib.get("y1"))
            x2 = parse_length(el.attrib.get("x2"))
            y2 = parse_length(el.attrib.get("y2"))
            add([(x1, y1), (x2, y2)], matrix)
        elif tag == "rect":
            x = parse_length(el.attrib.get("x"))
            y = parse_length(el.attrib.get("y"))
            w = parse_length(el.attrib.get("width"))
            h = parse_length(el.attrib.get("height"))
            if w > 0 and h > 0:
                add([(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)], matrix)
        elif tag in ("circle", "ellipse"):
            cx = parse_length(el.attrib.get("cx"))
            cy = parse_length(el.attrib.get("cy"))
            if tag == "circle":
                rx = ry = parse_length(el.attrib.get("r"))
            else:
                rx = parse_length(el.attrib.get("rx"))
                ry = parse_length(el.attrib.get("ry"))
            if rx > 0 and ry > 0:
                n = max(24, samples * 3)
                seg = []
                for k in range(n + 1):
                    a = (2.0 * math.pi * k) / n
                    seg.append((cx + rx * math.cos(a), cy + ry * math.sin(a)))
                add(seg, matrix)

        for child in list(el):
            walk(child, matrix)

    walk(root, mat_identity())
    return segments


def bounds_of(segments):
    pts = [p for seg in segments for p in seg]
    if not pts:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def normalize_segments(segments, machine_w, machine_h, margin, fit=True):
    b = bounds_of(segments)
    if b is None:
        return [], None, 1.0

    minx, miny, maxx, maxy = b
    src_w = max(maxx - minx, 1e-9)
    src_h = max(maxy - miny, 1e-9)
    usable_w = max(machine_w - 2.0 * margin, 1.0)
    usable_h = max(machine_h - 2.0 * margin, 1.0)

    if fit:
        scale = min(usable_w / src_w, usable_h / src_h)
        left = (machine_w - src_w * scale) / 2.0
        bottom = (machine_h - src_h * scale) / 2.0

        def conv(p):
            x = left + (p[0] - minx) * scale
            y = bottom + (maxy - p[1]) * scale
            return {"x": round(x, 2), "y": round(y, 2)}
    else:
        scale = 1.0

        def conv(p):
            return {"x": round(p[0], 2), "y": round(machine_h - p[1], 2)}

    out = []
    for seg in segments:
        clean = []
        for p in seg:
            q = conv(p)
            q["x"] = min(machine_w, max(0.0, q["x"]))
            q["y"] = min(machine_h, max(0.0, q["y"]))
            if not clean or abs(clean[-1]["x"] - q["x"]) > 0.01 or abs(clean[-1]["y"] - q["y"]) > 0.01:
                clean.append(q)
        if len(clean) > 1:
            out.append(clean)
    return out, {"minX": minx, "minY": miny, "maxX": maxx, "maxY": maxy}, scale


def build_commands(segments, speed, z_high, z_low, z_rpm):
    commands = []
    grbl = []
    if not segments:
        return commands, grbl

    commands.append(f"m{fmt_num(z_high)}v{fmt_num(z_rpm)}")
    grbl.append(f"G0 Z{fmt_num(z_high)}")

    for seg in segments:
        first = seg[0]
        commands.append(f"v{fmt_num(speed)}x{fmt_num(first['x'])}y{fmt_num(first['y'])}")
        commands.append(f"m{fmt_num(z_low)}v{fmt_num(z_rpm)}")
        grbl.append(f"G0 X{fmt_num(first['x'])} Y{fmt_num(first['y'])}")
        grbl.append(f"G1 Z{fmt_num(z_low)} F{fmt_num(z_rpm)}")
        for p in seg[1:]:
            commands.append(f"v{fmt_num(speed)}x{fmt_num(p['x'])}y{fmt_num(p['y'])}")
            grbl.append(f"G1 X{fmt_num(p['x'])} Y{fmt_num(p['y'])} F{fmt_num(speed * 60.0)}")
        commands.append(f"m{fmt_num(z_high)}v{fmt_num(z_rpm)}")
        grbl.append(f"G0 Z{fmt_num(z_high)}")
    return commands, grbl


def convert_svg(svg_text, params):
    machine_w = as_float(params.get("machineW", [DEFAULT_MACHINE_W])[0], DEFAULT_MACHINE_W)
    machine_h = as_float(params.get("machineH", [DEFAULT_MACHINE_H])[0], DEFAULT_MACHINE_H)
    speed = as_float(params.get("speed", [DEFAULT_SPEED])[0], DEFAULT_SPEED)
    z_high = as_float(params.get("zHigh", [DEFAULT_Z_HIGH])[0], DEFAULT_Z_HIGH)
    z_low = as_float(params.get("zLow", [DEFAULT_Z_LOW])[0], DEFAULT_Z_LOW)
    z_rpm = as_float(params.get("zRpm", [DEFAULT_Z_RPM])[0], DEFAULT_Z_RPM)
    margin = as_float(params.get("margin", [DEFAULT_MARGIN])[0], DEFAULT_MARGIN)
    samples = int(as_float(params.get("samples", [DEFAULT_CURVE_SAMPLES])[0], DEFAULT_CURVE_SAMPLES))
    fit = params.get("fit", ["1"])[0] not in ("0", "false", "False")

    raw_segments = collect_svg_segments(svg_text, samples=max(4, samples))
    segments, raw_bounds, scale = normalize_segments(raw_segments, machine_w, machine_h, margin, fit=fit)
    commands, grbl = build_commands(segments, speed, z_high, z_low, z_rpm)
    total_points = sum(len(seg) for seg in segments)

    return {
        "ok": True,
        "segments": segments,
        "commands": commands,
        "grbl": grbl,
        "summary": {
            "segments": len(segments),
            "points": total_points,
            "commands": len(commands),
            "machineW": machine_w,
            "machineH": machine_h,
            "fit": fit,
            "scale": scale,
            "rawBounds": raw_bounds,
        },
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "SvgArduinoServer/1.0"

    def _headers(self, status=200, content_type="application/json"):
        self.send_response(status)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_OPTIONS(self):
        self._headers(204)

    def do_GET(self):
        self._headers(200, "text/plain")
        self.wfile.write(b"SVG server ready. POST /convert-svg with SVG text.\n")

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path != "/convert-svg":
            self._headers(404)
            self.wfile.write(json.dumps({"ok": False, "error": "Unknown endpoint"}).encode("utf-8"))
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            svg_text = self.rfile.read(length).decode("utf-8", errors="replace")
            if "<svg" not in svg_text[:1000].lower():
                raise ValueError("Body does not look like SVG text")
            result = convert_svg(svg_text, parse_qs(parsed.query))
            self._headers(200)
            self.wfile.write(json.dumps(result, ensure_ascii=False).encode("utf-8"))
        except Exception as exc:
            self._headers(400)
            self.wfile.write(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False).encode("utf-8"))

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


def main():
    httpd = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"SVG server running on http://{HOST}:{PORT}")
    print("Press Ctrl+C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping SVG server.")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
