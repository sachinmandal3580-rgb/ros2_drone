"""
gazipin.py  —  Gazebo-local DIGIPIN codec
==========================================
Place this file at:
  src/drone_delivery_system/drone_delivery_system/gazipin.py

Re-implements the official DIGIPIN algorithm (India Post / IIT-H) with
a custom bounding box that covers YOUR Gazebo world (home.sdf) instead
of India.  Same 16-char alphabet, same hierarchical 4-way subdivision,
same encode/decode logic — only the bounding box changes.

  Official DIGIPIN  →  India  100 km × 100 km  →  cell ≈ 4 m
  GAZIPIN           →  Gazebo 100 m  × 100 m   →  cell ≈ 0.0015 m  (0.15 cm)

World origin from home.sdf <spherical_coordinates>:
  lat = 47.47895°,  lon = 19.057785°  (Budapest)
  X = East, Y = North  (ENU frame, same as Gazebo default)

Format:  XXX-XXX-XX  (8 chars + 2 hyphens, e.g. K22-772-7T)

Quick reference — home.sdf objects:
  K22-222-22  →  Origin / drone spawn    ( 0.00,  0.00) m
  K2J-F64-3M  →  Person Standing         ( 4.40,  2.40) m
  K2K-97F-PM  →  Dumpster                ( 3.71,  4.45) m
  J54-95C-6C  →  Fire Hydrant            ( 0.45, -1.66) m
  J57-K47-PC  →  Cardboard Box stack     ( 2.39, -3.68) m
  8CT-PP5-CM  →  Table                   (-6.33,  5.25) m
  K22-772-7T  →  Test point (0.50, 0.50)
  K22-222-7K  →  Test point (0.01, 0.01)

CLI usage:
  python gazipin.py 0.5 0.5         # encode  → K22-772-7T
  python gazipin.py K22-772-7T      # decode  → (0.4997, 0.4997)
"""

__all__ = ['encode', 'decode', 'validate',
           'WORLD_XMIN', 'WORLD_XMAX', 'WORLD_YMIN', 'WORLD_YMAX',
           'CELL_SIZE_M', 'CHARS']

# ── Constants ────────────────────────────────────────────────────────
CHARS       = '23456789CJKLMPFT'   # 16 symbols — identical to official DIGIPIN
LEVELS      = 8                    # subdivision steps → 8-char code
WORLD_XMIN  = -50.0                # West  boundary (metres)
WORLD_XMAX  =  50.0                # East  boundary (metres)
WORLD_YMIN  = -50.0                # South boundary (metres)
WORLD_YMAX  =  50.0                # North boundary (metres)
CELL_SIZE_M = (WORLD_XMAX - WORLD_XMIN) / (4 ** LEVELS)   # ≈ 0.00153 m


def encode(x: float, y: float) -> str:
    """
    Encode Gazebo local (x, y) metres → 8-char GAZIPIN string.

    Raises ValueError if coordinates are outside world bounds.
    """
    if not (WORLD_XMIN <= x <= WORLD_XMAX):
        raise ValueError(
            f"x={x:.4f} m is outside world bounds "
            f"[{WORLD_XMIN}, {WORLD_XMAX}]")
    if not (WORLD_YMIN <= y <= WORLD_YMAX):
        raise ValueError(
            f"y={y:.4f} m is outside world bounds "
            f"[{WORLD_YMIN}, {WORLD_YMAX}]")

    xmin, xmax = WORLD_XMIN, WORLD_XMAX
    ymin, ymax = WORLD_YMIN, WORLD_YMAX
    code = []

    for _ in range(LEVELS):
        xstep = (xmax - xmin) / 4.0
        ystep = (ymax - ymin) / 4.0
        col   = min(int((x - xmin) / xstep), 3)
        row   = min(int((y - ymin) / ystep), 3)
        xmin += col * xstep;  xmax = xmin + xstep
        ymin += row * ystep;  ymax = ymin + ystep
        code.append(CHARS[col * 4 + row])

    s = ''.join(code)
    return f"{s[0:3]}-{s[3:6]}-{s[6:8]}"


def decode(code: str) -> tuple:
    """
    Decode an 8-char GAZIPIN string → (x, y) centre of cell, in metres.

    Hyphens are ignored; input is case-insensitive.
    Raises ValueError on bad length or invalid characters.
    """
    clean = code.replace('-', '').upper()

    if len(clean) != LEVELS:
        raise ValueError(
            f"GAZIPIN must be {LEVELS} characters "
            f"(got {len(clean)} from '{code}')")
    for ch in clean:
        if ch not in CHARS:
            raise ValueError(
                f"Invalid character '{ch}' in '{code}'. "
                f"Allowed: {CHARS}")

    xmin, xmax = WORLD_XMIN, WORLD_XMAX
    ymin, ymax = WORLD_YMIN, WORLD_YMAX

    for ch in clean:
        idx   = CHARS.index(ch)
        col   = idx // 4
        row   = idx  % 4
        xstep = (xmax - xmin) / 4.0
        ystep = (ymax - ymin) / 4.0
        xmin += col * xstep;  xmax = xmin + xstep
        ymin += row * ystep;  ymax = ymin + ystep

    return (xmin + xmax) / 2.0, (ymin + ymax) / 2.0


def validate(code: str) -> tuple:
    """Return (True, '') on success or (False, error_message) on failure."""
    try:
        decode(code)
        return True, ''
    except ValueError as e:
        return False, str(e)


# ── CLI ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import sys, math

    if len(sys.argv) == 3:
        try:
            x, y  = float(sys.argv[1]), float(sys.argv[2])
            code  = encode(x, y)
            dx, dy = decode(code)
            err   = math.sqrt((dx - x) ** 2 + (dy - y) ** 2) * 100
            print(f"GAZIPIN  : {code}")
            print(f"Input    : ({x:.4f}, {y:.4f}) m")
            print(f"Snaps to : ({dx:.4f}, {dy:.4f}) m  (error {err:.3f} cm)")
        except ValueError as e:
            print(f"Error: {e}", file=sys.stderr); sys.exit(1)

    elif len(sys.argv) == 2:
        try:
            x, y = decode(sys.argv[1])
            print(f"GAZIPIN  : {sys.argv[1].upper()}")
            print(f"X        : {x:.6f} m")
            print(f"Y        : {y:.6f} m")
        except ValueError as e:
            print(f"Error: {e}", file=sys.stderr); sys.exit(1)

    else:
        print(f"Usage:")
        print(f"  python gazipin.py <x> <y>     # encode")
        print(f"  python gazipin.py <GAZIPIN>   # decode")
        print(f"Cell size : {CELL_SIZE_M*100:.4f} cm")
        print(f"Bounds    : X/Y ∈ [{WORLD_XMIN}, {WORLD_XMAX}] m")