from math import sqrt

def distance_m(p1, p2):
    dx = (p1["lat"] - p2["lat"]) * 0.11132
    dy = (p1["lon"] - p2["lon"]) * 0.11132
    return sqrt(dx*dx + dy*dy) * 1000
