import pandas as pd
import datetime

# 1. Load the data
file_path = 'lidar_ground_truth_20260121_153856.csv'
df = pd.read_csv(file_path)

# 2. Convert Unix timestamp to readable Datetime
# unit='s' assumes the timestamp is in seconds (e.g., 1765338874)
df['cleaned_time'] = pd.to_datetime(df['timestamp'], unit='s')

# Optional: If you want to replace the original column completely:
# df['timestamp'] = df['cleaned_time']
# df.drop(columns=['cleaned_time'], inplace=True)

# 3. Sort the data by time to ensure order
df = df.sort_values(by='cleaned_time')

# 4. Save the cleaned file
output_path = 'cleaned_lidar_data.csv'
# formatting the date to string so it looks nice in Excel/CSV
df.to_csv(output_path, index=False, date_format='%Y-%m-%d %H:%M:%S')

print(f"File cleaned and saved to {output_path}")
print(df.head())