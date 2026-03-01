# data_parser.py

import struct
import math
from config import (
    ANCHOR_MAX_COUNT, ANC_PROTOCAL_RTLS, ANC_PROTOCAL_DIST, ANC_PROTOCAL_RXDIAG,
    ANC_PROTOCAL_TIMESTAMP, ACC_FSR, GYRO_FSR, IMU_DATA_ACC_EN, IMU_DATA_GYRO_EN,
    IMU_DATA_EULER_EN, IMU_DATA_TEMP_EN, IMU_DATA_Q_EN, IMU_DATA_MAGN_EN
    
    
)
# --- Helper Functions for Diagnostics ---
def _calculate_fp_power(diag_data):
    """Calculates First Path Power in dBm. Based on C# code."""
    try:
        if diag_data.get('Rx_preambleCount', 0) == 0: return -150.0
        result = (diag_data['Fp_amp1']**2 + diag_data['Fp_amp2']**2 + diag_data['Fp_amp3']**2)
        n_square = diag_data['Rx_preambleCount']**2
        result /= n_square
        result = 10 * math.log10(result) if result > 0 else -150.0
        return round(result - 121.74, 2)
    except (KeyError, ValueError, ZeroDivisionError):
        return -150.0

def _calculate_rx_power(diag_data):
    """Calculates Receive Power in dBm. Based on C# code."""
    try:
        if diag_data.get('Rx_preambleCount', 0) == 0: return -150.0
        pow_value = 2**17  # DW1000 constant
        n_square = diag_data['Rx_preambleCount']**2
        result = diag_data['Max_growthCIR'] * pow_value / n_square
        result = 10 * math.log10(result) if result > 0 else -150.0
        return round(result - 121.74, 2)
    except (KeyError, ValueError, ZeroDivisionError):
        return -150.0

def crc16(data: bytes) -> int:
    """Calculate MODBUS CRC-16 checksum."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def decode_int16(data: bytes) -> int:
    """Helper function to decode a 16-bit signed integer from 2 bytes (big-endian)."""
    return struct.unpack('>h', data)[0]

def parse_imu_packet(payload: bytes) -> dict:
    """
    Parses a raw IMU data payload and returns a dictionary of values.
    This logic is ported from DWM_receiver.py.
    """
    try:
        ptr = 0
        print_en = (payload[ptr] << 8) | payload[ptr+1]
        ptr += 2
        
        imu_data = {}
        
        if print_en & IMU_DATA_ACC_EN:
            imu_data['acc_x'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * ACC_FSR; ptr += 2
            imu_data['acc_y'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * ACC_FSR; ptr += 2
            imu_data['acc_z'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * ACC_FSR; ptr += 2
            
        if print_en & IMU_DATA_GYRO_EN:
            imu_data['gyro_x'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * GYRO_FSR; ptr += 2
            imu_data['gyro_y'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * GYRO_FSR; ptr += 2
            imu_data['gyro_z'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * GYRO_FSR; ptr += 2
            
        if print_en & IMU_DATA_EULER_EN:
            imu_data['roll'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * 180.0; ptr += 2
            imu_data['pitch'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * 180.0; ptr += 2
            imu_data['yaw'] = decode_int16(payload[ptr:ptr+2]) / 32768.0 * 180.0; ptr += 2
        
        # --- Temperature Data ---
        if print_en & IMU_DATA_TEMP_EN:
            imu_data['temperature'] = decode_int16(payload[ptr:ptr+2]) / 340.0 + 36.53; ptr += 2
            
        # --- Quaternion Data ---
        if print_en & IMU_DATA_Q_EN:
            imu_data['q0'] = decode_int16(payload[ptr:ptr+2]) / 32768.0; ptr += 2
            imu_data['q1'] = decode_int16(payload[ptr:ptr+2]) / 32768.0; ptr += 2
            imu_data['q2'] = decode_int16(payload[ptr:ptr+2]) / 32768.0; ptr += 2
            imu_data['q3'] = decode_int16(payload[ptr:ptr+2]) / 32768.0; ptr += 2
    
        # --- Magnetometer Data ---
        if print_en & IMU_DATA_MAGN_EN:
            imu_data['magn_x'] = decode_int16(payload[ptr:ptr+2]) * 0.15; ptr += 2
            imu_data['magn_y'] = decode_int16(payload[ptr:ptr+2]) * 0.15; ptr += 2
            imu_data['magn_z'] = decode_int16(payload[ptr:ptr+2]) * 0.15; ptr += 2
            
        return imu_data
        
    except (struct.error, IndexError) as e:
        print(f"ERROR: Failed to parse IMU packet: {e}")
        return None
    
def parse_tag_position_packet(packet: bytes, tag_manager, log_callback=print) -> bool:
    try:
        if len(packet) < 20:  # Too short for header + position
            # log_callback(f"Packet too short: {len(packet)}")  # optional, less spam
            return False

        # Keep existing checks
        if packet[1] != 0x03 or packet[3:5] != b'\xca\xda':
            return False

        # Optional: check byte_count
        byte_count = packet[2]
        expected_len = byte_count + 5  # ID + func + count + CRC(2)
        if len(packet) < expected_len:
            # log_callback(f"Incomplete frame: got {len(packet)}, need {expected_len}")
            return False
        
        z_cm = struct.unpack_from('>h', packet, 17)[0]
        y_cm = struct.unpack_from('>h', packet, 15)[0]
        x_cm = struct.unpack_from('>h', packet, 13)[0]
        
        # print(f"Raw bytes at pos 9-14: {packet[9:15].hex()}")
        # print(f"Raw signed shorts: X={x_cm}, Y={y_cm}, Z={z_cm}")
        
        # print(f"Raw shorts: X_raw={struct.unpack_from('>h', packet, 13)[0]:6d} cm, "
        #       f"Y_raw={struct.unpack_from('>h', packet, 15)[0]:6d} cm, "
        #       f"Z_raw={struct.unpack_from('>h', packet, 17)[0]:6d} cm")
                
        # raw_x_cm = struct.unpack_from('>h', packet, 13)[0]
        # raw_y_cm = struct.unpack_from('>h', packet, 11)[0]
        # raw_z_cm = struct.unpack_from('>h', packet, 9)[0]
        
        # print(f"Raw shorts: X_raw={raw_x_cm} cm, "
        #       f"Y_raw={raw_y_cm} cm, "
        #       f"Z_raw={raw_z_cm} cm")
        
        #log_callback(f"Raw cm: X={raw_x_cm} Y={raw_y_cm} Z={raw_z_cm}") 

        x_m = x_cm / 100.0
        y_m = y_cm / 100.0
        z_m = z_cm / 100.0

        tag_id = 1  # or extract properly (see below)

        # CRITICAL: Use the REAL method name your TagManager has
        # Check managers.py or wherever TagManager is defined
        tag_data = {
            'id': str(tag_id),          # or str(1) if tag_id is always 1
            'x': x_m,
            'y': y_m,
            'z': z_m,
        }
        
        updated_tag = tag_manager.update_or_create_tag(tag_data)

        if updated_tag:
            log_callback(f"Parsed & Updated Tag {updated_tag.id} → "
                         f"X={updated_tag.x:+.2f}m Y={updated_tag.y:+.2f}m Z={updated_tag.z:+.2f}m")
        else:
            log_callback("Failed to update/create tag")

        return True

    except Exception as e:
        log_callback(f"Parse error: {e}")
        return False
    
# --- Main Parsing Function ---
def parse_rtls_packet(data: bytes):
    """
    Parses the main RTLS data packet from the anchor.
    Expects the full MODBUS frame (including slave ID, func code, etc.).
    Returns a dictionary with parsed data or None on error.
    """
    try:
        # A typical MODBUS RTU response for this data looks like:
        # [SlaveID][FuncCode][ByteCount][Header1][Header2][Payload...][CRC1][CRC2]
        if len(data) < 9 or data[1] != 0x03: # Must be a Read Holding Registers response
            print(f"DEBUG: Parse failed at check: len={len(data)}, func={data[1] if len(data)>1 else 'N/A'}, byte_count={data[2] if len(data)>2 else 'N/A'}, header={data[3:5].hex() if len(data)>4 else 'N/A'}")
            return None
        
        byte_count = data[2]
        if len(data) != byte_count + 5: # 1(ID)+1(Func)+1(Count)+2(CRC)
            print(f"DEBUG: Parse failed at check: len={len(data)}, func={data[1] if len(data)>1 else 'N/A'}, byte_count={data[2] if len(data)>2 else 'N/A'}, header={data[3:5].hex() if len(data)>4 else 'N/A'}")
            return None # Incomplete packet

        # Verify CRC
        received_crc = (data[-1] << 8) | data[-2]
        calculated_crc = crc16(data[:-2])
        if received_crc != calculated_crc:
            print(f"DEBUG: CRC Fail. Recv: {received_crc:04X}, Calc: {calculated_crc:04X}")
            print(f"DEBUG: Parse failed at check: len={len(data)}, func={data[1] if len(data)>1 else 'N/A'}, byte_count={data[2] if len(data)>2 else 'N/A'}, header={data[3:5].hex() if len(data)>4 else 'N/A'}")
            return None

        # Check for the specific header (0xCA 0xDA)
        if not (data[3] == 0xCA and data[4] == 0xDA):
            print(f"DEBUG: Parse failed at check: len={len(data)}, func={data[1] if len(data)>1 else 'N/A'}, byte_count={data[2] if len(data)>2 else 'N/A'}, header={data[3:5].hex() if len(data)>4 else 'N/A'}")
            return None # Not the RTLS data packet we are looking for

        payload = data[5:-2] # Extract the core data payload
        
        # --- Unpack payload fields ---
        unpacker_header = struct.Struct('>HHI') # Big-endian: ushort, ushort, uint
        output_protocol, tag_id, cal_flag = unpacker_header.unpack_from(payload, 0)
        cursor = unpacker_header.size
        
        parsed_data = {
            'tag_id': str(tag_id),
            'position': None,
            'distances': {},
            'rx_diag': None
        }
        
        # --- DEBUG INFO ---
        # This will print to your terminal so you can see what the hardware is thinking
        print(f"DEBUG: Tag={tag_id} | Proto={bin(output_protocol)} | CalFlag={bin(cal_flag)}")

        # --- Extract Position (if available) ---
        if (output_protocol & (1 << ANC_PROTOCAL_RTLS)):
            position_valid = bool((cal_flag >> 16) & 0x01)
            if position_valid:
                pos_unpacker = struct.Struct('>hhh') # 3x signed short
                raw_x, raw_y, raw_z = pos_unpacker.unpack_from(payload, cursor)
                # Convert from cm to meters
                parsed_data['position'] = {
                    'x': raw_x / 100.0,
                    'y': raw_y / 100.0,
                    'z': raw_z / 100.0
                }
            else:
                print(f"DEBUG: Position Invalid (Flag 16 is 0). Raw: {raw_x}, {raw_y}, {raw_z}")
                # Even if invalid, let's show the raw data if it's non-zero, 
                # it might just mean the geometric dilution of precision (GDOP) is bad.
                parsed_data['position'] = {
                    'x': raw_x / 100.0,
                    'y': raw_y / 100.0,
                    'z': raw_z / 100.0
                }
            cursor += 6
        else:
            pass
            # print("DEBUG: RTLS Protocol bit not set in Anchor config.")

        # --- Extract Distances (if available) ---
        if (output_protocol & (1 << ANC_PROTOCAL_DIST)):
            dist_unpacker = struct.Struct('>H') # ushort
            for i in range(ANCHOR_MAX_COUNT):
                distance_cm = dist_unpacker.unpack_from(payload, cursor)[0]
                is_valid = bool((cal_flag >> i) & 0x01)
                if is_valid or distance_cm > 0:
                    anchor_id = chr(65 + i) # A, B, C...
                    parsed_data['distances'][anchor_id] = distance_cm / 100.0 # to meters
                cursor += 2
                
        # --- Extract Diagnostics (if available) ---
        if (output_protocol & (1 << ANC_PROTOCAL_RXDIAG)):
            # Unpack 8 consecutive unsigned short (16-bit) values
            diag_unpacker = struct.Struct('>HHHHHHHH') 
            raw_diags = diag_unpacker.unpack_from(payload, cursor)
            
            rx_diag = {
                'Max_noise': raw_diags[0],
                'Std_noise': raw_diags[1],
                'Fp_amp1': raw_diags[2],
                'Fp_amp2': raw_diags[3],
                'Fp_amp3': raw_diags[4],
                'Max_growthCIR': raw_diags[5],
                'Rx_preambleCount': raw_diags[6],
                'Fp': raw_diags[7] / 64.0
            }
            # Calculate power levels and add them to the dictionary
            rx_diag['Fp_power'] = _calculate_fp_power(rx_diag)
            rx_diag['Rx_power'] = _calculate_rx_power(rx_diag)

            parsed_data['rx_diag'] = rx_diag
            cursor += diag_unpacker.size
            
        # --- Extract IMU (if enabled) ---
        if (output_protocol & (IMU_DATA_ACC_EN | IMU_DATA_GYRO_EN | IMU_DATA_EULER_EN)):  # Check bits
            # Assume IMU after diagnostics (adjust cursor/offsets from manual if needed)
            # Example: Unpack acc (3 signed shorts), gyro (3), euler (3) - adjust based on firmware
            imu_unpacker = struct.Struct('>hhhhhhhhh')  # 9 shorts for acc/gyro/euler
            raw_imu = imu_unpacker.unpack_from(payload, cursor)
            parsed_data['imu'] = {
                'acc_x': raw_imu[0] * ACC_FSR / 32768.0,  # Scale to g
                'acc_y': raw_imu[1] * ACC_FSR / 32768.0,
                'acc_z': raw_imu[2] * ACC_FSR / 32768.0,
                'gyro_x': raw_imu[3] * GYRO_FSR / 32768.0,  # dps
                'gyro_y': raw_imu[4] * GYRO_FSR / 32768.0,
                'gyro_z': raw_imu[5] * GYRO_FSR / 32768.0,
                'roll': raw_imu[6] / 100.0,  # Assume degrees * 100
                'pitch': raw_imu[7] / 100.0,
                'yaw': raw_imu[8] / 100.0
            }
            cursor += imu_unpacker.size

        print(f"DEBUG: Parsed data: {parsed_data}")
        return parsed_data

    except (struct.error, IndexError) as e:
        print(f"ERROR: Failed to parse RTLS packet: {e}")
        print(f"DEBUG: Parse failed at check: len={len(data)}, func={data[1] if len(data)>1 else 'N/A'}, byte_count={data[2] if len(data)>2 else 'N/A'}, header={data[3:5].hex() if len(data)>4 else 'N/A'}")
        return None