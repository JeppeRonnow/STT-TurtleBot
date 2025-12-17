import sqlite3
import os
import sounddevice as sd
import soundfile as sf

# Database path
db_path = os.path.join(os.path.dirname(__file__), "../pc_client/db.db")

def play_audio(audio_data):
    """Play audio from bytes data"""
    # Write bytes to a temporary file
    temp_file = "temp_audio.wav"
    with open(temp_file, "wb") as f:
        f.write(audio_data)
    
    # Read the audio file
    data, samplerate = sf.read(temp_file)
    
    # Play the audio
    sd.play(data, samplerate)
    sd.wait()
    
    # Clean up temp file
    os.remove(temp_file)


# Connect to the database
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Fetch all rows where transcription is not empty
query = """
SELECT id, normalizedAudio, transcribe 
FROM recordings 
WHERE transcribe != ''
"""

cursor.execute(query)
rows = cursor.fetchall()

# Close the connection
conn.close()

# Initialize counters
successful = 0
unsuccessful = 0

print("=" * 50)
print("Transcription Accuracy Test")
print("=" * 50)
print(f"Total recordings to test: {len(rows)}")
print("=" * 50)
print("\nInstructions:")
print("  y - Correct transcription")
print("  n - Incorrect transcription")
print("  r - Replay audio")
print("  i - Ignore (skip this recording)")
print("=" * 50)

# Go through each recording
for idx, row in enumerate(rows, 1):
    record_id, audio_data, transcription = row
    
    print(f"\n[{idx}/{len(rows)}] Recording ID: {record_id}")
    print(f"Transcription: \"{transcription}\"")
    
    # Play the audio
    play_audio(audio_data)
    
    # Get user input
    while True:
        user_input = input("Was this correct? (y/n/r/i): ").lower().strip()
        
        if user_input == 'y':
            successful += 1
            break
        elif user_input == 'n':
            unsuccessful += 1
            break
        elif user_input == 'r':
            print("Replaying audio...")
            play_audio(audio_data)
        elif user_input == 'i':
            print("Skipping this recording...")
            break
        else:
            print("Invalid input. Please enter 'y', 'n', 'r', or 'i'.")

# Calculate success percentage
total = successful + unsuccessful
if total > 0:
    success_percentage = (successful / total) * 100
    
    print("\n" + "=" * 50)
    print("Test Results")
    print("=" * 50)
    print(f"Successful: {successful}")
    print(f"Unsuccessful: {unsuccessful}")
    print(f"Total tested: {total}")
    print(f"Success rate: {success_percentage:.2f}%")
    print("=" * 50)
else:
    print("\nNo recordings were tested.")
