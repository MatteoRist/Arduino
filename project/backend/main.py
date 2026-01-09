from fastapi import FastAPI
from datetime import datetime
from pydantic import BaseModel

from database import db, init_db
from proximity import distance_m
import math

app = FastAPI()


init_db()

# ============================================================
# GEOMETRY (INT, ARDUINO PORT)
# ============================================================

def point_in_triangle(p, a, b, c):
    px, py = p

    def cross(x1, y1, x2, y2):
        return x1 * y2 - y1 * x2

    crosses = 0
    edges = [(a, b), (b, c), (c, a)]

    for (x1, y1), (x2, y2) in edges:
        val = cross(
            x1 - x2,
            y1 - y2,
            px - x2,
            py - y2 
        )
        if val <= 0:
            crosses += 1

    return crosses == 0 or crosses == 3


# ============================================================
# PROXIMITY (INT, ARDUINO PORT)
# ============================================================

_cos_cache = {"lat": None, "value": None}

def proximity_violation(lat1, lon1, lat2, lon2, distance_m):
    if _cos_cache["lat"] is None or abs(_cos_cache["lat"] - lat1) > 9000:
        _cos_cache["value"] = int(
            math.cos(lat1 * 1e-7 * math.pi / 180.0) * 1024
        )
        _cos_cache["lat"] = lat1

    lon_scale = _cos_cache["value"]

    dlat = lat1 - lat2
    dlon = lon1 - lon2
    limit = distance_m * 90

    if abs(dlat) > limit:
        return False

    dlon_scaled = (dlon * lon_scale) >> 10
    if abs(dlon_scaled) > limit:
        return False

    return (dlat * dlat + dlon_scaled * dlon_scaled) < (limit * limit)

# ============================================================
# API
# ============================================================

class PositionUpdate(BaseModel):
    id: int
    lat: int
    lon: int


@app.post("/position")
def add_position(data: PositionUpdate):
    now = datetime.utcnow()

    db.execute(
        "INSERT INTO positions VALUES (?, ?, ?, ?)",
        (data.id, now, data.lat, data.lon)
    )

    db.execute("""
        INSERT INTO devices(id, lat, lon, last_update)
        VALUES (?, ?, ?, ?)
        ON CONFLICT(id) DO UPDATE SET
        lat=excluded.lat,
        lon=excluded.lon,
        last_update=excluded.last_update
    """, (data.id, data.lat, data.lon, now))

    db.commit()
    return {"status": "ok"}

@app.get("/violations/{device_id}")
def get_violations(device_id: int):
    device = db.execute(
        "SELECT * FROM devices WHERE id=?", (device_id,)
    ).fetchone()

    if not device:
        return []

    violations = []
    user_pt = (device["lat"], device["lon"])

    # ---------------- ARENAS ----------------
    arenas = db.execute("""
        SELECT a.*, r.must_be_out
        FROM arenas a
        JOIN arena_restrictions r ON a.arena_id = r.arena_id
        WHERE r.device_id=?
    """, (device_id,)).fetchall()

    should_be_in_any = False
    is_in_any = False

    for a in arenas:
        inside = point_in_triangle(
            user_pt,
            (a["ax"], a["ay"]),
            (a["bx"], a["by"]),
            (a["cx"], a["cy"])
        )

        if a["must_be_out"] == 1 and inside:
            violations.append({
                "type": "arena_exclusion",
                "arena_id": a["arena_id"]
            })

        if a["must_be_out"] == 0:
            should_be_in_any = True
            if inside:
                is_in_any = True

    if should_be_in_any and not is_in_any:
        violations.append({
            "type": "arena_inclusion"
        })

    # ---------------- PROXIMITY ----------------
    rules = db.execute(
        "SELECT * FROM proximity_restrictions WHERE device_a=?",
        (device_id,)
    ).fetchall()

    if rules:
        ids = [r["device_b"] for r in rules]
        placeholders = ",".join("?" * len(ids))

        others = db.execute(
            f"SELECT * FROM devices WHERE id IN ({placeholders})",
            ids
        ).fetchall()

        others_map = {o["id"]: o for o in others}

        for r in rules:
            other = others_map.get(r["device_b"])
            if not other:
                continue

            if proximity_violation(
                device["lat"], device["lon"],
                other["lat"], other["lon"],
                r["distance"]
            ):
                violations.append({
                    "type": "proximity",
                    "with": r["device_b"],
                    "limit": r["distance"]
                })

    return violations

@app.get("/positions/{device_id}")
def get_positions(device_id: int, limit: int = 100):
    rows = db.execute(
        """
        SELECT timestamp, lat, lon
        FROM positions
        WHERE device_id=?
        ORDER BY timestamp DESC
        LIMIT ?
        """,
        (device_id, limit)
    ).fetchall()

    return [
        {
            "timestamp": row["timestamp"],
            "lat": row["lat"],
            "lon": row["lon"]
        }
        for row in rows
    ]

@app.get("/devices")
def get_device():
    rows = db.execute(
        "SELECT id, lat, lon, last_update FROM devices"
    ).fetchall()

    return [dict(r) for r in rows]

@app.get("/policy/{device_id}")
def get_policy(device_id: int):
    # --- PROXIMITY ---
    proximity = db.execute(
        """
        SELECT device_b, distance
        FROM proximity_restrictions
        WHERE device_a=?
        """,
        (device_id,)
    ).fetchall()

    proximity_rules = [
        {
            "with": r["device_b"],
            "distance": r["distance"]
        }
        for r in proximity
    ]

    # --- ARENAS ---
    arenas = db.execute(
        """
        SELECT a.arena_id, a.ax, a.ay, a.bx, a.by, a.cx, a.cy, r.must_be_out
        FROM arenas a
        JOIN arena_restrictions r ON a.arena_id = r.arena_id
        WHERE r.device_id=?
        """,
        (device_id,)
    ).fetchall()

    arena_rules = [
        {
            "arena_id": a["arena_id"],
            "must_be_out": a["must_be_out"],
            "triangle": {
                "a": [a["ax"], a["ay"]],
                "b": [a["bx"], a["by"]],
                "c": [a["cx"], a["cy"]],
            }
        }
        for a in arenas
    ]

    return {
        "device_id": device_id,
        "proximity": proximity_rules,
        "arenas": arena_rules
    }

class ProximityRule(BaseModel):
    device_a: int
    device_b: int
    distance: int

@app.post("/proximity")
def set_proximity(rule: ProximityRule):
    db.execute(
        """
        INSERT INTO proximity_restrictions(device_a, device_b, distance)
        VALUES (?, ?, ?)
        ON CONFLICT(device_a, device_b) DO UPDATE SET
        distance=excluded.distance
        """,
        (rule.device_a, rule.device_b, rule.distance)
    )
    db.commit()
    return {"status": "ok"}

class ArenaPolygonAssign(BaseModel):
    device_id: int
    must_be_out: int
    points: list[list[int]]

@app.post("/arena/assign_polygon")
def assign_polygon(r: ArenaPolygonAssign):
    cur = db.cursor()

    triangles = []

    for i in range(1, len(r.points) - 1):
        a = r.points[0]
        b = r.points[i]
        c = r.points[i + 1]
        triangles.append((a, b, c))

    arena_ids = []

    for tri in triangles:
        a, b, c = tri

        cur.execute(
            "INSERT INTO arenas(ax, ay, bx, by, cx, cy) VALUES (?, ?, ?, ?, ?, ?)",
            (a[0], a[1], b[0], b[1], c[0], c[1])
        )
        arena_id = cur.lastrowid
        arena_ids.append(arena_id)

        cur.execute(
            "INSERT INTO arena_restrictions(device_id, arena_id, must_be_out) VALUES (?, ?, ?)",
            (r.device_id, arena_id, r.must_be_out)
        )

    db.commit()

    return {
        "status": "ok",
        "assigned_triangles": arena_ids
    }

@app.delete("/devices/{device_id}")
def delete_device(device_id: int):
    db.execute("DELETE FROM positions WHERE device_id=?", (device_id,))
    db.execute("DELETE FROM arena_restrictions WHERE device_id=?", (device_id,))
    db.execute("DELETE FROM proximity_restrictions WHERE device_a=? OR device_b=?", (device_id, device_id))
    db.execute("DELETE FROM devices WHERE id=?", (device_id,))
    db.commit()
    return {"status": "ok"}

@app.delete("/proximity")
def delete_proximity(device_a: int, device_b: int):
    db.execute(
        "DELETE FROM proximity_restrictions WHERE device_a=? AND device_b=?",
        (device_a, device_b)
    )
    db.commit()
    return {"status": "ok"}

@app.delete("/arena/{arena_id}")
def delete_arena(arena_id: int):
    db.execute("DELETE FROM arena_restrictions WHERE arena_id=?", (arena_id,))
    db.execute("DELETE FROM arenas WHERE arena_id=?", (arena_id,))
    db.commit()
    return {"status": "ok"}

