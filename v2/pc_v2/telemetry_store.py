import sqlite3, time, os
from typing import Iterable, Dict, Any

DDL = "
CREATE TABLE IF NOT EXISTS events (
  ts REAL,
  type TEXT,
  data TEXT
);
"

def open_db(path: str) -> sqlite3.Connection:
    db = sqlite3.connect(path, check_same_thread=False)
    db.execute("PRAGMA journal_mode=WAL")
    db.execute("PRAGMA synchronous=NORMAL")
    db.execute(DDL)
    return db

def add(db: sqlite3.Connection, typ: str, data: str):
    db.execute("INSERT INTO events VALUES (?,?,?)", (time.time(), typ, data))
    db.commit()

def prune(db: sqlite3.Connection, max_rows: int = 200_000):
    (n,) = db.execute("SELECT COUNT(*) FROM events").fetchone()
    if n > max_rows:
        to_delete = n - max_rows
        db.execute("DELETE FROM events WHERE rowid IN (SELECT rowid FROM events ORDER BY rowid ASC LIMIT ?)", (to_delete,))
        db.commit()
