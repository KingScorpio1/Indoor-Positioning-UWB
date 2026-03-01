# positioning_model.py

import pandas as pd
import numpy as np
from sklearn.neighbors import KNeighborsRegressor
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import joblib  # For saving/loading models and scalers

# Import TensorFlow for the Neural Network. Install with: pip install tensorflow
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout
from tensorflow.keras.callbacks import EarlyStopping


class UWBPositioningKNN:
    def __init__(self, anchor_ids=['A', 'B', 'C', 'D']):
        self.model = None
        self.scaler = StandardScaler()
        self.is_trained = False
        self.anchor_ids = anchor_ids
        self.feature_columns = [f'dist_{aid}_median' for aid in self.anchor_ids]
        
    def load_and_preprocess_data(self, filename='data/uwb_training_data.csv'):
        """Load and prepare the training data for multiple anchors."""
        df = pd.read_csv(filename)
        
        # Check for missing columns and handle them
        for col in self.feature_columns:
            if col not in df.columns:
                raise ValueError(f"Missing required column '{col}' in {filename}. Please run data collection.")
        
        # Fill any missing distance values with a large number (or use a more sophisticated method)
        df[self.feature_columns] = df[self.feature_columns].fillna(99.0)

        X = df[self.feature_columns].values
        y = df[['true_x', 'true_y']].values
        return X, y, df
    
    def train_model(self, X, y, test_size=0.2, n_neighbors=7):
        """Train the KNN model."""
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=test_size, random_state=42
        )
        
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_test_scaled = self.scaler.transform(X_test)
        
        self.model = KNeighborsRegressor(n_neighbors=n_neighbors, weights='distance')
        self.model.fit(X_train_scaled, y_train)
        self.is_trained = True
        
        y_pred = self.model.predict(X_test_scaled)
        error = mean_squared_error(y_test, y_pred)
        
        print("KNN Model Training Complete:")
        print(f"  - Features: {self.feature_columns}")
        print(f"  - Optimal k: {n_neighbors}")
        print(f"  - Test R² Score: {self.model.score(X_test_scaled, y_test):.4f}")
        print(f"  - Test Mean Squared Error: {error:.4f}")
    
    def predict_position(self, distances):
        """Predict position from a dictionary of current distance measurements."""
        if not self.is_trained:
            raise ValueError("Model not trained yet!")
        
        # Create the feature vector in the correct order
        feature_vector = [distances.get(aid, 99.0) for aid in self.anchor_ids] # Use 99.0 for missing anchors
        
        X_new = np.array([feature_vector])
        X_new_scaled = self.scaler.transform(X_new)
        
        position = self.model.predict(X_new_scaled)[0]
        return position[0], position[1]  # x, y
    
    def evaluate_model(self, X, y):
        """Comprehensive model evaluation"""
        X_scaled = self.scaler.transform(X)
        y_pred = self.model.predict(X_scaled)
        
        errors = np.sqrt(np.sum((y - y_pred) ** 2, axis=1))
        
        plt.figure(figsize=(15, 5))
        
        plt.subplot(1, 3, 1)
        plt.scatter(y[:, 0], y[:, 1], c=errors, cmap='viridis', alpha=0.7)
        plt.colorbar(label='Position Error (m)')
        plt.xlabel('True X')
        plt.ylabel('True Y')
        plt.title('Position Errors Across Space')
        
        plt.subplot(1, 3, 2)
        plt.hist(errors, bins=30, alpha=0.7, edgecolor='black')
        plt.xlabel('Position Error (m)')
        plt.ylabel('Frequency')
        plt.title(f'Error Distribution\nMean: {errors.mean():.3f}m')
        
        plt.subplot(1, 3, 3)
        plt.scatter(y[:, 0], y_pred[:, 0], alpha=0.5, label='X')
        plt.scatter(y[:, 1], y_pred[:, 1], alpha=0.5, label='Y')
        plt.plot([y.min(), y.max()], [y.min(), y.max()], 'k--')
        plt.xlabel('True Position')
        plt.ylabel('Predicted Position')
        plt.legend()
        plt.title('True vs Predicted Positions')
        
        plt.tight_layout()
        plt.show()
        
        return errors
    
class UWBPositioningNN:
    """
    A Neural Network model that uses richer features to achieve higher accuracy.
    This model can learn non-linear relationships and the subtle signs of NLOS.
    """
    def __init__(self, anchor_ids=['A', 'B', 'C', 'D']):
        self.model = None
        self.scaler = StandardScaler()
        self.is_trained = False
        self.anchor_ids = anchor_ids
        # ** CRITICAL: We now use more features for each anchor! **
        # These features must be present in your training CSV.
        self.feature_columns = []
        for aid in self.anchor_ids:
            self.feature_columns.extend([
                f'dist_{aid}_median',      # The median distance (as before)
                f'dist_{aid}_std',         # The stability of the distance
                f'fp_power_{aid}',       # First Path Power
                f'rx_power_{aid}',       # Receive Power
                f'std_noise_{aid}'      # Standard deviation of noise
            ])
            
    def load_and_preprocess_data(self, filename='data/uwb_training_data_rich.csv'):
        """Loads data with rich features."""
        df = pd.read_csv(filename)
        for col in self.feature_columns:
            if col not in df.columns:
                raise ValueError(f"Missing required feature column '{col}' in {filename}. Please update your data collection script.")
        
        df[self.feature_columns] = df[self.feature_columns].fillna(0) # Fill NaNs for power/noise with 0
        X = df[self.feature_columns].values
        y = df[['true_x', 'true_y']].values
        return X, y, df

    def _build_model(self, input_shape):
        """Defines the Neural Network architecture."""
        model = Sequential([
            # Input layer: shape is the number of features (e.g., 4 anchors * 5 features = 20)
            Dense(128, activation='relu', input_shape=(input_shape,)),
            Dropout(0.2),
            Dense(128, activation='relu'),
            Dropout(0.2),
            Dense(64, activation='relu'),
            # Output layer: 2 neurons for X and Y coordinates. 'linear' activation for regression.
            Dense(2, activation='linear')
        ])
        
        # Compile the model with a good optimizer and loss function for regression
        optimizer = tf.keras.optimizers.Adam(learning_rate=0.001)
        model.compile(optimizer=optimizer, loss='mean_squared_error', metrics=['mean_absolute_error'])
        return model

    def train_model(self, X, y, test_size=0.2, epochs=100, batch_size=32):
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=test_size, random_state=42)
        
        # Fit the scaler ONLY on the training data
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_test_scaled = self.scaler.transform(X_test)
        
        self.model = self._build_model(X_train.shape[1])
        
        # Early stopping prevents overfitting if the validation loss stops improving
        early_stopping = EarlyStopping(monitor='val_loss', patience=10, restore_best_weights=True)
        
        print("\n--- Starting Neural Network Training ---")
        history = self.model.fit(
            X_train_scaled,
            y_train,
            validation_data=(X_test_scaled, y_test),
            epochs=epochs,
            batch_size=batch_size,
            callbacks=[early_stopping],
            verbose=2
        )
        
        self.is_trained = True
        
        # Evaluate the final model
        loss, mae = self.model.evaluate(X_test_scaled, y_test, verbose=0)
        rmse = np.sqrt(loss)
        print("\n--- NN Model Training Complete ---")
        print(f"  Test RMSE: {rmse*100:.2f} cm")
        print(f"  Test MAE:  {mae*100:.2f} cm")
        return history

    def predict_position(self, feature_vector):
        """
        Predicts position from a full feature vector.
        Args:
            feature_vector (list or np.array): A flat list of all features in the correct order.
                                               e.g., [dist_A_med, dist_A_std, ..., std_noise_D]
        """
        if not self.is_trained:
            raise ValueError("Model is not trained yet!")
            
        X_new = np.array([feature_vector])
        X_new_scaled = self.scaler.transform(X_new)
        
        position = self.model.predict(X_new_scaled)[0]
        return position[0], position[1] # x, y

    def save(self, path='models/nn_model'):
        """Saves the trained model and the scaler."""
        self.model.save(f'{path}/model.h5')
        joblib.dump(self.scaler, f'{path}/scaler.pkl')
        print(f"Model saved to {path}")

    @classmethod
    def load(cls, path='models/nn_model'):
        """Loads a trained model and its scaler."""
        instance = cls()
        instance.model = tf.keras.models.load_model(f'{path}/model.h5')
        instance.scaler = joblib.load(f'{path}/scaler.pkl')
        instance.is_trained = True
        print(f"Model loaded from {path}")
        return instance    


