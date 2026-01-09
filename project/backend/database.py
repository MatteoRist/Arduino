import sqlite3

db = sqlite3.connect("data.db", check_same_thread=False)
db.row_factory = sqlite3.Row

def init_db():
    db.executescript("""
    CREATE TABLE IF NOT EXISTS devices (
        id INTEGER PRIMARY KEY,
        first_name TEXT,
        last_name TEXT,
        lat INTEGER,
        lon INTEGER,
        last_update DATETIME
    );

    CREATE TABLE IF NOT EXISTS positions (
        device_id INTEGER,
        timestamp DATETIME,
        lat INTEGER,
        lon INTEGER
    );

    CREATE TABLE IF NOT EXISTS arenas (
        arena_id INTEGER PRIMARY KEY,
        name TEXT,
        ax INTEGER, ay INTEGER,
        bx INTEGER, by INTEGER,
        cx INTEGER, cy INTEGER
    );

    CREATE TABLE IF NOT EXISTS arena_restrictions (
        device_id INTEGER,
        arena_id INTEGER,
        must_be_out INTEGER
    );

    CREATE TABLE IF NOT EXISTS proximity_restrictions (
    device_a INTEGER NOT NULL,
    device_b INTEGER NOT NULL,
    distance INTEGER NOT NULL,
    PRIMARY KEY (device_a, device_b)
);
    """)
    db.commit()
