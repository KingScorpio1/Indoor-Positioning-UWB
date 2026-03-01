import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from positioning_model import UWBPositioningNN

# --- CONFIGURATION ---
DATA_FILE = 'merged_training_data.csv'  # Your synchronized file
MODEL_PATH = 'models/nn_model'          # Your trained AI
SAVE_IMAGE = 'research_comparison_plot.png'

def main():
    # 1. Load Data
    print(f"Loading data from {DATA_FILE}...")
    try:
        df = pd.read_csv(DATA_FILE)
    except FileNotFoundError:
        print("Error: File not found. Make sure 'merged_training_data.csv' is in this folder.")
        return

    # 2. Load AI Model
    print(f"Loading AI Model from {MODEL_PATH}...")
    try:
        # Initialize and load weights
        model = UWBPositioningNN(anchor_ids=['A', 'B', 'C', 'D'])
        model.feature_columns = ['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D'] # FORCE RAW FEATURES
        
        # We need to manually load the scaler and keras model because of the class structure
        # (This uses the built-in load function we fixed earlier)
        model = UWBPositioningNN.load(MODEL_PATH)
        model.feature_columns = ['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D'] # Re-apply fix after load
        
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    # 3. Generate Predictions
    print("Generating AI Predictions...")
    
    # Prepare Inputs
    X_input = df[['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D']].values
    
    # Run Prediction (Batch)
    # We access the internal Keras model directly for speed on large datasets
    X_scaled = model.scaler.transform(X_input)
    predictions = model.model.predict(X_scaled)
    
    # 4. Create the Plot
    print("Plotting results...")
    plt.style.use('seaborn-v0_8-paper') # Professional research style
    plt.figure(figsize=(10, 8))
    
    # Plot Ground Truth (Lidar) - The "Correct" Path
    plt.plot(df['x'], df['y'], 
             color='black', linewidth=2, linestyle='--', label='Ground Truth (LiDAR)', alpha=0.7)
    
    # Plot AI Prediction - The "Corrected" Path
    plt.scatter(predictions[:, 0], predictions[:, 1], 
                color='#007acc', s=10, alpha=0.5, label='AI Corrected UWB')

    # Optional: Highlight Start/End
    plt.scatter(df['x'].iloc[0], df['y'].iloc[0], c='green', s=100, marker='^', label='Start')
    plt.scatter(df['x'].iloc[-1], df['y'].iloc[-1], c='red', s=100, marker='X', label='End')

    # Formatting
    plt.title('Sensor Fusion Results: LiDAR Ground Truth vs. AI Correction', fontsize=14, fontweight='bold')
    plt.xlabel('X Coordinate (meters)', fontsize=12)
    plt.ylabel('Y Coordinate (meters)', fontsize=12)
    plt.legend(loc='upper right', frameon=True)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.axis('equal') # Ensure 1 meter looks like 1 meter
    
    # Calculate Error for the subtitle
    mae = np.mean(np.abs(predictions - df[['x', 'y']].values))
    plt.suptitle(f"Average Positioning Error: {mae*100:.1f} cm", y=0.92, fontsize=11, color='red')

    # Save
    plt.tight_layout()
    plt.savefig(SAVE_IMAGE, dpi=300)
    print(f"Graph saved to {SAVE_IMAGE}")
    plt.show()

if __name__ == "__main__":
    main()