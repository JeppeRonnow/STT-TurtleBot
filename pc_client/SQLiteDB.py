import sqlite3
from typing import Any, Iterable, List, Tuple, Optional
import os


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
            self.connection = sqlite3.connect(self.db_path, check_same_thread=False)
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
            elapsedtime INT NOT NULL,
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
    def add_recording(self, raw_wav_path: str, filtered_wav_path: str, normalized_wav_path: str, transcribe: str, elapsedtime: int) -> int:
        with open(raw_wav_path, "rb") as f:
            raw_bytes = f.read()

        with open(filtered_wav_path, "rb") as f:
            filtered_bytes = f.read()

        with open(normalized_wav_path, "rb") as f:
            normalized_bytes = f.read()

        query = """
        INSERT INTO recordings (rawAudio, filterAudio, normalizedAudio, transcribe, elapsedtime)
        VALUES (?, ?, ?, ?, ?)
        """

        return self.execute(query, (raw_bytes, filtered_bytes, normalized_bytes, transcribe, elapsedtime))


    # Restore .WAV files from the database
    def restore_recording(self, entry_id: int, output_dir: str = "."):
        # Get row from row id in teh database
        row = self.fetchone("SELECT * FROM recordings WHERE id = ?", (entry_id,))

        # If there is now data for the selected id
        if row is None:
            return None

        # Create paths for the audio files
        raw_path = os.path.join(output_dir, f"Raw_{entry_id}.wav")
        filtered_path = os.path.join(output_dir, f"Filtered_{entry_id}.wav")
        normalized_path = os.path.join(output_dir, f"Normalized_{entry_id}.wav")

        # Open and write audio data from database to files
        with open(raw_path, "wb") as f:
            f.write(row["rawAudio"])

        with open(filtered_path, "wb") as f:
            f.write(row["filterAudio"])

        with open(normalized_path, "wb") as f:
            f.write(row["normalizedAudio"])

        # Return paths to files
        return raw_path, filtered_path, normalized_path


    # Enter function
    def __enter__(self):
        self.connect()
        return self


    # Exit function
    def __exit__(self, exc_type, exc, tb):
         self.close()


# If run as main works as a DB browser tool
if __name__ == '__main__':
    # Open database
    db = SQLiteDB("db.db")

    print("\n--- DATABASE ENTRIES ---")

    # Get entries in the database
    rows = db.fetchall("SELECT id, transcribe, created_at FROM recordings")

    # IF their is no entries
    if not rows:
        print("No recordings found in the database.\n")

    # Else print all entries key transcript and timestamp
    else:
        for r in rows:
            print(f"ID: {r['id']}\n Transcript: {r['transcribe']}\n Execution time: {r['elapsedtime']}ms\n Timestamp: {r['created_at']}\n")

        # Ask user which entry to restore the audio from
        choice = input("Enter an ID to restore WAV files (or press Enter to exit): ").strip()

        # If the user entered a number
        if choice.isdigit():
            entry_id = int(choice)
            result = db.restore_recording(entry_id)

            # If the entrie exists in the database
            if result:
                raw_path, filtered_path, normalized_path = result
                
                print(f"\nWAV files restored:\n {raw_path}\n {filtered_path}\n {normalized_path}\n")
            
            # If no entrie was found for the given ID
            else:
                print(f"No entry found with ID {entry_id}.")

    # Close the database
    db.close()
