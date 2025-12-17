import sqlite3
import os
import statistics
import matplotlib.pyplot as plt

# Database path
db_path = os.path.join(os.path.dirname(__file__), "../pc_client/db.db")

# Connect to the database
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Fetch all rows where payload is not "N/A"
query = """
SELECT elapsedtime 
FROM recordings 
WHERE payload != 'N/A'
"""

cursor.execute(query)
rows = cursor.fetchall()

# Close the connection
conn.close()

# Extract elapsed times and convert to floats
elapsed_times = [float(row[0]) for row in rows]

if elapsed_times:
    # Calculate min, max, and mean
    min_time = min(elapsed_times)
    max_time = max(elapsed_times)
    mean_time = statistics.mean(elapsed_times)
    median_time = statistics.median(elapsed_times)
    
    print("=" * 50)
    print("Execution Time Statistics")
    print("=" * 50)
    print(f"Total recordings: {len(elapsed_times)}")
    print(f"Min elapsed time: {min_time:.2f} ms")
    print(f"Max elapsed time: {max_time:.2f} ms")
    print(f"Mean elapsed time: {mean_time:.2f} ms")
    print(f"Median elapsed time: {median_time:.2f} ms")
    print("=" * 50)
    
    # Create boxplot
    plt.figure(figsize=(10, 6))
    plt.boxplot(elapsed_times, vert=True, patch_artist=True)
    plt.ylabel('Elapsed Time (ms)', fontsize=12)
    plt.title('Execution Time Statistics', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    
    # Add statistics text to the plot
    stats_text = f'Min: {min_time:.2f} ms\nMax: {max_time:.2f} ms\nMean: {mean_time:.2f} ms\nMedian: {median_time:.2f} ms\nN: {len(elapsed_times)}'
    plt.text(1.15, median_time, stats_text, fontsize=10, 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
             verticalalignment='center')
    
    plt.tight_layout()
    plt.show()
else:
    print("No recordings found with payload != 'N/A'")
