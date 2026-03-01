import optuna
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout
from tensorflow.keras.optimizers import Adam

# --- 1. Load Data (Using your existing logic) ---
def load_data():
    # Use the merge logic we fixed previously
    # For this example, I assume you have the cleaned merged file ready
    df = pd.read_csv("merged_training_data.csv") 
    X = df[['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D']].values
    y = df[['x', 'y']].values
    return X, y

X, y = load_data()
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2)

# --- 2. Define the Objective Function ---
def objective(trial):
    # Optuna suggests values for hyperparameters
    n_layers = trial.suggest_int('n_layers', 1, 4)
    learning_rate = trial.suggest_loguniform('learning_rate', 1e-5, 1e-2)
    batch_size = trial.suggest_categorical('batch_size', [16, 32, 64, 128])
    
    model = Sequential()
    
    # Input Layer
    input_units = trial.suggest_int('input_units', 16, 256)
    model.add(Dense(input_units, input_dim=4, activation='relu'))
    
    # Hidden Layers (Dynamic Search)
    for i in range(n_layers):
        units = trial.suggest_int(f'units_layer_{i}', 16, 256)
        dropout = trial.suggest_float(f'dropout_layer_{i}', 0.0, 0.5)
        model.add(Dense(units, activation='relu'))
        model.add(Dropout(dropout))
        
    # Output Layer
    model.add(Dense(2, activation='linear'))
    
    # Compile
    optimizer = Adam(learning_rate=learning_rate)
    model.compile(loss='mse', optimizer=optimizer, metrics=['mae'])
    
    # Train
    history = model.fit(
        X_train, y_train,
        validation_data=(X_test, y_test),
        epochs=50, # Keep epochs low for speed during search
        batch_size=batch_size,
        verbose=0
    )
    
    # Return the validation loss (The metric we want to minimize)
    return history.history['val_loss'][-1]

# --- 3. Run the Optimization ---
if __name__ == "__main__":
    print("Starting Optimal Search for AI Architecture...")
    study = optuna.create_study(direction='minimize')
    study.optimize(objective, n_trials=50) # Try 50 different AI brains

    print("Best Hyperparameters found:")
    print(study.best_params)
    
    print("\nUpdate your positioning_model.py with these values!")