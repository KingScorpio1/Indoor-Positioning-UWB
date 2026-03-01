import pandas as pd
import numpy as np
from datetime import datetime

# --- CONFIGURATION ---
UWB_FILE = "k501_t1.csv"
LIDAR_FILE = "lidar_ground_truth_20260121_153856.csv"
OUTPUT_FILE = "merged_training_data.csv"

def parse_custom_uwb_time(time_str):
    """
    Parses format: '2026/01/21 15:22:01 412'
    Safe against non-string inputs.
    """
    try:
        if not isinstance(time_str, str):
            return None
        # Split into '2026/01/21 15:22:01' and '412'
        base_time, millis = time_str.rsplit(' ', 1)
        dt_obj = datetime.strptime(base_time, "%Y/%m/%d %H:%M:%S")
        timestamp = dt_obj.timestamp() + (int(millis) / 1000.0)
        return timestamp
    except Exception as e:
        return None

def main():
    print("--- 1. Loading Files ---")
    
    # LOAD UWB: Try different encodings
    try:
        df_uwb = pd.read_csv(UWB_FILE, encoding='utf-8')
    except UnicodeDecodeError:
        print("   > UTF-8 failed, trying GBK encoding...")
        df_uwb = pd.read_csv(UWB_FILE, encoding='gbk')
    except Exception:
        print("   > GBK failed, trying standard ISO...")
        df_uwb = pd.read_csv(UWB_FILE, encoding='ISO-8859-1')

    # LOAD LIDAR
    df_lidar = pd.read_csv(LIDAR_FILE)
    
    print(f"   UWB Raw Rows: {len(df_uwb)}")
    print(f"   Lidar Raw Rows: {len(df_lidar)}")

    # --- 2. Parse UWB Timestamps ---
    print("\n--- 2. Parsing UWB Timestamps ---")
    
    # Select the FIRST column by Index (0) to avoid encoding issues with '??'
    time_col_name = df_uwb.columns[0] 
    print(f"   > Detected Time Column Name: '{time_col_name}'")
    
    df_uwb['unix_time'] = df_uwb[time_col_name].apply(parse_custom_uwb_time)
    
    # Drop rows where parsing failed & SORT
    df_uwb = df_uwb.dropna(subset=['unix_time'])
    df_uwb = df_uwb.sort_values('unix_time') # Sort Left Data
    
    # --- 3. Rename Columns by Position ---
    print("   > Renaming columns by position index...")
    
    try:
        # Col 4=A, 5=B, 6=C, 7=D
        df_uwb = df_uwb.rename(columns={
            df_uwb.columns[4]: 'Dist_A',
            df_uwb.columns[5]: 'Dist_B',
            df_uwb.columns[6]: 'Dist_C',
            df_uwb.columns[7]: 'Dist_D'
        })
    except IndexError:
        print("CRITICAL ERROR: UWB CSV columns mismatch!")
        return

    # Check columns
    needed_cols = ['unix_time', 'Dist_A', 'Dist_B', 'Dist_C', 'Dist_D']
    if not all(col in df_uwb.columns for col in needed_cols):
        print("ERROR: Could not find Distance columns.")
        return

    # --- 4. Fix Time Offset ---
    print("\n--- 3. Synchronizing Clocks ---")
    
    uwb_start = df_uwb['unix_time'].iloc[0]
    lidar_start = df_lidar['timestamp'].iloc[0]
    
    print(f"   UWB Start:   {datetime.fromtimestamp(uwb_start)}")
    print(f"   Lidar Start: {datetime.fromtimestamp(lidar_start)}")
    
    # Calculate the gap
    time_offset = uwb_start - lidar_start
    print(f"   Detected Offset: {time_offset:.2f} seconds")
    print("   -> Shifting Lidar data to match UWB time...")
    
    # Apply offset to Lidar
    df_lidar['synced_time'] = df_lidar['timestamp'] + time_offset
    
    # --- FIX: FORCE SORT LIDAR DATA ---
    # The error "right keys must be sorted" happens if Lidar data isn't perfect
    df_lidar = df_lidar.sort_values('synced_time')
    
    # --- 5. Merge Data ---
    print("\n--- 4. Merging Data ---")
    merged = pd.merge_asof(
        df_uwb,
        df_lidar,
        left_on='unix_time',
        right_on='synced_time',
        direction='nearest',
        tolerance=0.2 # Match if within 200ms
    )
    
    # Clean up
    final_df = merged.dropna(subset=['x', 'y'])
    final_df = final_df[['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D', 'x', 'y']]
    
    print(f"   Matched {len(final_df)} data points.")
    
    if len(final_df) > 10:
        final_df.to_csv(OUTPUT_FILE, index=False)
        print(f"\nSUCCESS! Saved training data to '{OUTPUT_FILE}'")
        print("You can now run 'train_model.py' or 'train_nn.py'.")
    else:
        print("\nFAILURE: Not enough matches found.")

if __name__ == "__main__":
    main()