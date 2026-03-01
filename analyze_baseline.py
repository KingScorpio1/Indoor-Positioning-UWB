# analyze_baseline.py

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import mean_absolute_error, mean_squared_error
from positioning_model import UWBPositioningKNN

def analyze():
    """Loads collected data, trains a model, and evaluates its baseline performance."""
    
    DATA_FILE = 'data/uwb_training_data.csv'
    print(f"--- Starting Baseline Analysis using '{DATA_FILE}' ---")

    # 1. Load the data and initialize the model
    try:
        model = UWBPositioningKNN(anchor_ids=['A', 'B', 'C', 'D'])
        X, y_true, df = model.load_and_preprocess_data(DATA_FILE)
    except Exception as e:
        print(f"\nERROR: Could not load data or initialize model: {e}")
        print("Please ensure you have run 'data/collect_data.py' successfully.")
        return

    # 2. Train the model
    # We use the collected data itself to train the model, simulating real-world use.
    model.train_model(X, y_true)

    # 3. Make predictions on the *entire* dataset to see the overall error
    print("\n--- Evaluating Model Performance on Full Dataset ---")
    X_scaled = model.scaler.transform(X)
    y_pred = model.model.predict(X_scaled)

    # 4. Calculate Errors
    # Euclidean distance error for each point
    errors = np.sqrt(np.sum((y_true - y_pred) ** 2, axis=1))
    df['predicted_x'] = y_pred[:, 0]
    df['predicted_y'] = y_pred[:, 1]
    df['error_m'] = errors

    # 5. Calculate and Print Baseline Metrics
    mae = mean_absolute_error(y_true, y_pred)
    rmse = np.sqrt(mean_squared_error(y_true, y_pred))
    p95 = np.percentile(errors, 95)

    print(f"\n--- BASELINE METRICS (in meters) ---")
    print(f"Mean Absolute Error (MAE): {mae:.4f} m  ({mae*100:.1f} cm)")
    print(f"Root Mean Square Error (RMSE): {rmse:.4f} m  ({rmse*100:.1f} cm)  <-- THIS IS YOUR KEY METRIC")
    print(f"95th Percentile Error (P95): {p95:.4f} m  ({p95*100:.1f} cm)")
    print("--------------------------------------")

    # 6. Visualize the Error
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    fig.suptitle('Baseline Positioning Error Analysis', fontsize=16)

    # Plot 1: Error Heatmap
    ax1 = axes[0]
    # Use pivot_table to create a grid for the heatmap
    try:
        error_pivot = df.pivot_table(index='true_y', columns='true_x', values='error_m')
        sns.heatmap(error_pivot, ax=ax1, cmap='viridis', annot=True, fmt=".2f", cbar_kws={'label': 'Error (meters)'})
        ax1.set_title('Spatial Heatmap of Positioning Error')
        ax1.invert_yaxis() # Ensure origin (0,0) is at the bottom-left
    except Exception as e:
        ax1.set_title("Could not generate heatmap (data may not be a grid)")
        print(f"\nWARN: Heatmap generation failed: {e}")


    # Plot 2: Error Distribution
    ax2 = axes[1]
    sns.histplot(errors, ax=ax2, bins=20, kde=True)
    ax2.axvline(rmse, color='r', linestyle='--', label=f'RMSE: {rmse:.2f}m')
    ax2.axvline(p95, color='purple', linestyle=':', label=f'P95: {p95:.2f}m')
    ax2.set_title('Distribution of Errors')
    ax2.set_xlabel('Error (meters)')
    ax2.set_ylabel('Frequency')
    ax2.legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

if __name__ == "__main__":
    analyze()