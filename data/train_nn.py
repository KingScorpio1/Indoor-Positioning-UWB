import pandas as pd
from positioning_model import UWBPositioningNN

# --- CONFIGURATION ---
DATA_FILE = 'merged_training_data.csv'
MODEL_SAVE_PATH = 'models/nn_model'

def main():
    print("--- 1. Initializing Model ---")
    model = UWBPositioningNN(anchor_ids=['A', 'B', 'C', 'D'])
    
    # Update the model's expected inputs to match our new file
    model.feature_columns = ['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D']

    print(f"--- 2. Loading Data from '{DATA_FILE}' ---")
    
    # --- MANUAL DATA LOAD (The Fix) ---
    # We load the file directly with pandas to avoid the 'true_x' vs 'x' naming conflict
    try:
        df = pd.read_csv(DATA_FILE)
        
        # 1. Select Inputs (Features)
        X = df[['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D']].values
        
        # 2. Select Outputs (Targets)
        # We explicitly look for 'x' and 'y' which we know exist in merged_training_data.csv
        y = df[['x', 'y']].values 
        
        print(f"   > Successfully loaded {len(y)} samples.")
        print(f"   > Input Shape: {X.shape}")
        print(f"   > Target Shape: {y.shape}")

        # 3. Handle Scaling
        # The model usually expects a scaler to be fit. We do it here.
        if hasattr(model, 'scaler'):
            print("   > Fitting scaler to new data...")
            X = model.scaler.fit_transform(X)
            
    except Exception as e:
        print(f"CRITICAL ERROR loading data: {e}")
        return

    # --- 3. Train ---
    print("--- 3. Training Neural Network ---")
    # We pass the manually loaded X and y arrays directly
    model.train_model(X, y, epochs=150, batch_size=32)

    # --- 4. Save ---
    print(f"--- 4. Saving Model to '{MODEL_SAVE_PATH}' ---")
    model.save(MODEL_SAVE_PATH)
    print("SUCCESS: Model is trained and ready for the App!")

if __name__ == "__main__":
    main()