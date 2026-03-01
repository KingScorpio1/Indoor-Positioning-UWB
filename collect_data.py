# data/collect_data.py

import serial
import time
import csv
import numpy as np
import os

# Import our robust, refactored modules
import data_parser 
import hardware_interface as hw

class UWBDataCollector:
    def __init__(self, port: str, slave_id: int = 1):
        # Use our new hardware interface for a reliable connection
        if not hw.connect_to_anchor(port, slave_id):
            print(f"FATAL: Could not connect to anchor on {port}. Please check the connection.")
            self.is_connected = False
        else:
            self.is_connected = True
        self.buffer = bytearray()

    def collect_sample(self, true_x: float, true_y: float, duration: int = 3) -> dict:
        """
        Collects raw UWB packets for a short duration and computes rich statistical features.
        """
        if not self.is_connected:
            return None

        # Store lists of all readings for each anchor
        all_distances = {'A': [], 'B': [], 'C': [], 'D': []}
        all_diagnostics = []
        
        # Start the positioning stream
        hw.start_positioning()
        time.sleep(0.1) # Give the anchor a moment to start streaming

        print(f"Collecting data for {duration} seconds...")
        start_time = time.time()
        while time.time() - start_time < duration:
            raw_data = hw.read_rtls_stream()
            if raw_data:
                self.buffer.extend(raw_data)
            time.sleep(0.01)
        
        # Stop the stream immediately after collection
        hw.stop_positioning()

        # --- NEW: Robust Buffer Processing ---
        while len(self.buffer) > 9: # Min possible packet size
            start_index = self.buffer.find(b'\x01\x03') # Standard MODBUS start
            if start_index == -1: break
            
            self.buffer = self.buffer[start_index:]
            if len(self.buffer) < 3: break

            byte_count = self.buffer[2]
            packet_len = byte_count + 5 # Header(3) + Payload(N) + CRC(2)

            if len(self.buffer) >= packet_len:
                packet = bytes(self.buffer[:packet_len])
                parsed_data = data_parser.parse_rtls_packet(packet)
                if parsed_data:
                    # Append distance and diagnostic data to our lists
                    for anchor_id, dist in parsed_data.get('distances', {}).items():
                        if anchor_id in all_distances:
                            all_distances[anchor_id].append(dist)
                    
                    if parsed_data.get('rx_diag'):
                        # This assumes diagnostics are from Anchor A, a common setup.
                        # If diagnostics can come from other anchors, this needs adjustment.
                        all_diagnostics['A'].append(parsed_data['rx_diag'])

                self.buffer = self.buffer[packet_len:]
            else:
                break
        
        self.buffer.clear() # Clear any remaining garbage

        # --- NEW: Calculate Rich Features ---
        summary = {'true_x': true_x, 'true_y': true_y}
        has_data = False
        
        for aid in ['A', 'B', 'C', 'D']:
            dist_list = all_distances[aid]
            diag_list = all_diagnostics.get(aid, []) # Get diagnostics for this specific anchor
            
            if dist_list:
                has_data = True
                summary[f'dist_{aid}_median'] = np.median(dist_list)
                summary[f'dist_{aid}_std'] = np.std(dist_list)
            else:
                summary[f'dist_{aid}_median'] = None
                summary[f'dist_{aid}_std'] = None
            
            # Add the new rich features, taking the mean of all values collected
            if diag_list:
                summary[f'fp_power_{aid}'] = np.mean([d.get('Fp_power', -150) for d in diag_list])
                summary[f'rx_power_{aid}'] = np.mean([d.get('Rx_power', -150) for d in diag_list])
                summary[f'std_noise_{aid}'] = np.mean([d.get('Std_noise', 0) for d in diag_list])
            else:
                summary[f'fp_power_{aid}'] = None
                summary[f'rx_power_{aid}'] = None
                summary[f'std_noise_{aid}'] = None

        return summary if has_data else None
    
    def create_grid_dataset(self, room_width, room_length, grid_spacing=0.5):
        """Systematically collect data in a grid pattern with user input."""
        dataset = []
        x_positions = np.arange(0, room_width + grid_spacing, grid_spacing)
        y_positions = np.arange(0, room_length + grid_spacing, grid_spacing)
        
        print("\n--- Starting Rich Data Collection for Neural Network ---")
        total_points = len(x_positions) * len(y_positions)
        
        for i, x in enumerate(x_positions):
            for j, y in enumerate(y_positions):
                point_num = i * len(y_positions) + j + 1
                print(f"\n--- Point {point_num}/{total_points} ---")
                input(f"Please move the tag to position ({x:.2f}, {y:.2f}) and press Enter...")
                
                los_status = ""
                while los_status not in ['los', 'nlos']:
                    los_status = input("Is the path to all anchors Line-of-Sight? (Enter 'los' or 'nlos'): ").lower()

                sample = self.collect_sample(x, y, duration=5)
                if sample:
                    sample['los_status'] = los_status
                    dataset.append(sample)
                    print("Sample collected:")
                    print(f"  - Dist A (Median): {sample.get('dist_A_median', 'N/A'):.3f}m, Std: {sample.get('dist_A_std', 'N/A'):.3f}")
                    print(f"  - Dist B (Median): {sample.get('dist_B_median', 'N/A'):.3f}m, Std: {sample.get('dist_B_std', 'N/A'):.3f}")
                    print(f"  - Dist C (Median): {sample.get('dist_C_median', 'N/A'):.3f}m, Std: {sample.get('dist_C_std', 'N/A'):.3f}")
                    print(f"  - Dist D (Median): {sample.get('dist_D_median', 'N/A'):.3f}m, Std: {sample.get('dist_D_std', 'N/A'):.3f}")
        
        self.save_dataset(dataset, filename='data/uwb_training_data.csv')
        return dataset

    def save_dataset(self, dataset, filename='uwb_training_data.csv'):
        if not dataset:
            print("No data to save.")
            return
        
        # Ensure the 'data' directory exists
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, 'w', newline='') as f:
            # Dynamically create headers from the first sample's keys
            writer = csv.DictWriter(f, fieldnames=dataset[0].keys())
            writer.writeheader()
            writer.writerows(dataset)
        print(f"\nDataset saved to {filename} with {len(dataset)} samples.")
        
    def close(self):
        if self.is_connected:
            hw.disconnect_from_anchor()
            print("Disconnected from anchor.")

if __name__ == "__main__":
    # --- IMPORTANT: CONFIGURE YOUR SETUP HERE ---
    COM_PORT = 'COM7'   # <-- CHANGE THIS to your anchor's COM port
    SLAVE_ID = 1        # The MODBUS ID of your anchor (usually 1)
    ROOM_WIDTH = 5.0    # Width of your collection area in meters
    ROOM_LENGTH = 5.0   # Length of your collection area in meters
    GRID_SPACING = 0.5  # Collect a point every 0.5 meters

    collector = UWBDataCollector(port=COM_PORT, slave_id=SLAVE_ID)
    if collector.is_connected:
        try:
            collector.create_grid_dataset(
                room_width=ROOM_WIDTH, 
                room_length=ROOM_LENGTH, 
                grid_spacing=GRID_SPACING
            )
        finally:
            collector.close()