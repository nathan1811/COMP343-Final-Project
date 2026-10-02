"""
database.py
-----------
SQLite memory layer: drones (live state), tasks (one row per command),
events (append-only log). This module owns all SQL.
"""
import os
import sqlite3
import time
from contextlib import contextmanager
from typing import List, Dict, Any

DB_PATH = "station.db"

SCHEMA = """
CREATE TABLE IF NOT EXISTS drones (
    id TEXT PRIMARY KEY, battery REAL, x INTEGER, y INTEGER, status TEXT, destination TEXT
);
CREATE TABLE IF NOT EXISTS tasks (
    task_id INTEGER PRIMARY KEY AUTOINCREMENT,
    command TEXT, drone_id TEXT, status TEXT, timestamp REAL
);
CREATE TABLE IF NOT EXISTS events (
    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
    event_type TEXT, description TEXT, timestamp REAL
);
"""


@contextmanager
def get_conn(db_path: str = DB_PATH):
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        yield conn
        conn.commit()
    finally:
        conn.close()


def init_db(db_path: str = DB_PATH, reset: bool = False):
    if reset and os.path.exists(db_path):
        os.remove(db_path)
    with get_conn(db_path) as conn:
        conn.executescript(SCHEMA)


def seed(drones: List, db_path: str = DB_PATH):
    with get_conn(db_path) as conn:
        for v in drones:
            conn.execute("INSERT OR REPLACE INTO drones (id, battery, x, y, status, destination) "
                         "VALUES (?,?,?,?,?,?)",
                         (v.id, v.battery, v.x, v.y, v.status, v.destination))


def log_event(event_type: str, description: str, db_path: str = DB_PATH):
    with get_conn(db_path) as conn:
        conn.execute("INSERT INTO events (event_type, description, timestamp) VALUES (?,?,?)",
                     (event_type, description, time.time()))


def create_task(command: str, drone_id: str, db_path: str = DB_PATH) -> int:
    with get_conn(db_path) as conn:
        cur = conn.execute("INSERT INTO tasks (command, drone_id, status, timestamp) "
                           "VALUES (?,?, 'PENDING', ?)", (command, drone_id, time.time()))
        return cur.lastrowid


def update_task_status(task_id: int, status: str, db_path: str = DB_PATH):
    with get_conn(db_path) as conn:
        conn.execute("UPDATE tasks SET status=? WHERE task_id=?", (status, task_id))


def update_drone(drone_id: str, db_path: str = DB_PATH, **fields):
    cols = ", ".join(f"{k}=?" for k in fields)
    with get_conn(db_path) as conn:
        conn.execute(f"UPDATE drones SET {cols} WHERE id=?", (*fields.values(), drone_id))


def fetch_all(table: str, db_path: str = DB_PATH) -> List[Dict[str, Any]]:
    with get_conn(db_path) as conn:
        return [dict(r) for r in conn.execute(f"SELECT * FROM {table}").fetchall()]


def recent_events(limit: int = 30, db_path: str = DB_PATH) -> List[Dict[str, Any]]:
    with get_conn(db_path) as conn:
        rows = conn.execute("SELECT * FROM events ORDER BY event_id DESC LIMIT ?",
                            (limit,)).fetchall()
        return [dict(r) for r in rows][::-1]