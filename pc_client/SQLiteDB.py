import sqlite3
from typing import Any, Iterable, List, Tuple, Optional


# SQLite3 database class
class SQLiteDB:
    def __init__(self, db_path: str):
        self.db_path = db_path
        self.connection: Optional[sqlite3.Connection] = None

        self.connect()
        self.create_tables()


    # Oppen connection
    def connect(self):
        if self.connection is None:
            self.connection = sqlite3.connect(self.db_path)
            self.connection.row_factory = sqlite3.Row


    # Close connection
    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None


    # Automatically generates tables for the DB
    def create_tables(self):
        query = """
        CREATE TABLE IF NOT EXISTS recordings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            rawAudio BLOB NOT NULL,
            filterAudio BLOB NOT NULL,
            normalizedAudio BLOB NOT NULL,
            transcribe TEXT NOT NULL,
            created_at TIMESTAMPT DEFAULT CURRENT_TIMESTAMP
        )
        """

        self.execute(query)


    # Execute a query with data params
    def execute(self, query: str, params: Iterable[Any] = ()):
        self.connect()
        cur = self.connection.cursor()
        cur.execute(query, params)
        self.connection.commit()
        return cur.lastrowid


    # fetchone function
    def fetchone(self, query: str, params: Iterable[Any] = ()) -> List[sqlite3.Row]:
        self.connect()
        cur = self.connection.cursor()
        cur.execute(query, params)
        return cur.fetchone()


    # Fetchall function
    def fetchall(self, query: str, params: Iterable[Any] = ()):
        self.connect()
        cur = self.connection.cursor()
        cur.execute(query, params)
        return cur.fetchall()


    # Saves audio and transcribe
    def add_recording(self, raw_wav_path: str, filtered_wav_path: str, normalized_wav_path: str, transcribe: str) -> int:
        with open(raw_wav_path, "rb") as f:
            raw_bytes = f.read()

        with open(filtered_wav_path, "rb") as f:
            filtered_bytes = f.read()

        with open(normalized_wav_path, "rb") as f:
            normalized_bytes = f.read()

        query = """
        INSERT INTO recordings (rawAudio, filterAudio, normalizedAudio, transcribe)
        VALUES (?, ?, ?, ?)
        """

        return self.execute(query, (raw_bytes, filtered_bytes, normalized_bytes, transcribe))


    # Enter function
    def __enter__(self):
        self.connect()
        return self


    # Exit function
    def __exit__(self, exc_type, exc, tb):
         self.close()
