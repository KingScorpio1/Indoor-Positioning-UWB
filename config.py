# config.py

import os

# === Project Root Directory ===
# This helps find asset files like images no matter where you run the script from
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# === Communication Settings ===
# For the Raspberry Pi TCP connection
TCP_HOST, TCP_PORT = '0.0.0.0', 52740  # 10.94.218.95 or 0.0.0.0

# For the Anchor MODBUS connection
DEFAULT_BAUDRATE = 115200

# === UWB & IMU Constants ===
ACC_FSR = 16.0      # ±16g full scale range for accelerometer
GYRO_FSR = 2000.0   # ±2000dps full scale range for gyroscope
ANCHOR_MAX_COUNT = 16

# Bitmasks for IMU data types
IMU_DATA_ACC_EN = 0x0001
IMU_DATA_GYRO_EN = 0x0002
IMU_DATA_EULER_EN = 0x0004
IMU_DATA_TEMP_EN = 0x0008
IMU_DATA_Q_EN = 0x0010
IMU_DATA_MAGN_EN = 0x0020

# Protocol flags for anchor data output
ANC_PROTOCAL_RTLS = 0
ANC_PROTOCAL_DIST = 1
ANC_PROTOCAL_RXDIAG = 2
ANC_PROTOCAL_TIMESTAMP = 3

# Protocol flags for tag data output
TAG_OUTPUT_DIST = 0
TAG_OUTPUT_RTLS = 1

# === MODBUS Constants ===
MODBUS_ADDR_POSITIONING_CONTROL = 59
MODBUS_VAL_START_POSITIONING = 4
MODBUS_VAL_STOP_POSITIONING = 0
DEFAULT_SLAVE_ID = 1

# === Visualization Settings ===
# Now, we add the 'assets' subfolder into the path.
ASSETS_DIR = os.path.join(BASE_DIR, "assets")

TARGET_IMG_PATH = os.path.join(ASSETS_DIR, "target.png")
ANCHOR_IMG_PATH = os.path.join(ASSETS_DIR, "anchor.png")
CAR_IMG_PATH = os.path.join(ASSETS_DIR, "Car_Red.png")

# A map of human-readable names to their register addresses
MODBUS_ADDR_MAP = {
    'MODBUS-ID': 0x01,
    'Kalman-Q': 0x10,
    'Kalman-R': 0x11,
    'Antenna Delay': 0x12,
    'Device Type': 0x14,
    'Device ID': 0x15,
    'Positioning Mode': 0x16,
    'Ranging Mode': 0x17,
    'UWB Channel': 0x18,
    'Data Rate': 0x19,
    'Network ID': 0x20
}

# Default anchor positions (can be overridden by calibration)
DEFAULT_ANCHOR_POSITIONS = {
    'A': {'x': 0.0, 'y': 0.0, 'z': 1.2, 'enabled': True, 'color': (0, 255, 0)},
    'B': {'x': 21.0, 'y': 686.0, 'z': 1.2, 'enabled': True, 'color': (0, 0, 255)},
    'C': {'x': 630.0, 'y': 656.0, 'z': 1.2, 'enabled': True, 'color': (255, 0, 0)},
    'D': {'x': 593.0, 'y': 0.0, 'z': 1.2, 'enabled': True, 'color': (255, 255, 0)}
}

# DEFAULT_ANCHOR_POSITIONS = {
#     'A': {'x': 0.0, 'y': 0.0, 'z': 1.2, 'enabled': True, 'color': (0, 255, 0)},
#     'B': {'x': -2.0, 'y': 454.0, 'z': 1.2, 'enabled': True, 'color': (0, 0, 255)},
#     'C': {'x': 553.0, 'y': 477.0, 'z': 1.2, 'enabled': True, 'color': (255, 0, 0)},
#     'D': {'x': 550.0, 'y': 0.0, 'z': 1.2, 'enabled': True, 'color': (255, 255, 0)}
# }
