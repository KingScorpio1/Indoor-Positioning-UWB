# hardware_interface.py

import serial
import serial.tools.list_ports
import minimalmodbus
import time
import struct
from threading import Lock
import config

# --- Module-level Globals for the connection ---
anchor_instrument = None
anchor_lock = Lock()

def scan_for_ports(log_callback=print):
    """
    Scans for serial ports and tries to identify which ones are PGPlus devices.
    Args:
        log_callback: A function (like app.log) to send status messages to.
    """
    log_callback("Scanning for available COM ports...")
    all_ports = [port.device for port in serial.tools.list_ports.comports()]
    if not all_ports:
        log_callback("No serial ports found.")
        return []

    pg_plus_ports = []
    for port in all_ports:
        if is_pgplus_device(port, log_callback):
            pg_plus_ports.append(port)
    
    if pg_plus_ports:
        log_callback(f"Found compatible devices on: {', '.join(pg_plus_ports)}")
    else:
        log_callback("Scan complete. No responsive PGPlus devices found.")
        
    return pg_plus_ports
    
def is_pgplus_device(port: str, log_callback=print) -> bool:
    """
    Try to identify PGPlus device by sending a MODBUS query.
    Returns True if successful, False otherwise.
    Includes detailed error logging.
    """
    instrument = None
    try:
        # Initialize the modbus instrument for the check
        instrument = minimalmodbus.Instrument(port, config.DEFAULT_SLAVE_ID) # slaveaddress 1
        instrument.serial.baudrate = config.DEFAULT_BAUDRATE
        instrument.serial.timeout = 0.5  # 200ms timeout
        instrument.mode = minimalmodbus.MODE_RTU
        instrument.clear_buffers_before_each_transaction = True

        # Attempt to read a register. Register 0 is common for version info.
        # This is the line that probes the device.
        version = instrument.read_register(0, functioncode=3)        
        log_callback(f"Device on {port} responded. Anchor Firmware Version (Reg 0): {version}.\n")
        return True # The device responded successfully
        
    except (minimalmodbus.NoResponseError, minimalmodbus.InvalidResponseError):
        # This is the most common error: the port is valid but the device didn't answer the MODBUS query.
        # It's not a PGPlus device or it's in a non-responsive state.
        # No log is needed here as it's expected for non-target devices.
        return False
    except serial.SerialException as e:
        # This error means the port could not be opened (e.g., access denied, already in use).
        log_callback(f"Could not open port {port}: {e}\n")
        return False
    except Exception as e:
        # Catch any other unexpected errors during the check.
        log_callback(f"An unexpected error occurred while checking {port}: {e}\n")
        return False
    finally:
        # Ensure the serial port is closed after the check
        if instrument and instrument.serial.is_open:
            instrument.serial.close()
            
def connect_to_anchor(port: str, slave_id: int=1, log_callback=print) -> bool:
    """
    Connects to the anchor via MODBUS RTU.
    Returns the instrument object on success, None on failure.
    """
    global anchor_instrument
    with anchor_lock:
        if anchor_instrument and anchor_instrument.serial.is_open:
            return True

        try:
            instrument = minimalmodbus.Instrument(port, slave_id)
            instrument.serial.baudrate = config.DEFAULT_BAUDRATE
            instrument.serial.bytesize = 8
            instrument.serial.parity = serial.PARITY_NONE
            instrument.serial.stopbits = 1
            instrument.serial.timeout = 0.1 # 100ms timeout is a safe starting point.
            instrument.mode = minimalmodbus.MODE_RTU
            
            instrument.clear_buffers_before_each_transaction = True
            instrument.close_port_after_each_call = False
            instrument.serial.reset_input_buffer()
            instrument.serial.reset_output_buffer()
            
            version1 = instrument.read_register(0, functioncode=3)
            log_callback(f"SUCCESS: Connected to anchor (ID {slave_id}). Firmware Ver: {version1}")
            
            anchor_instrument = instrument            
            return True
        except minimalmodbus.NoResponseError:
            log_callback(f"ERROR: No response from device on {port} with slave ID {slave_id}.")
            log_callback(" -> Check wiring, power, and that the MODBUS ID is correct.")
            anchor_instrument = None
            return False
        except serial.SerialException as e:
            log_callback(f"ERROR: Serial port error on {port}. Is it in use by another program?")
            log_callback(f" -> {e}")
            anchor_instrument = None
            return False
        except Exception as e:
            log_callback(f"ERROR: An unexpected error occurred during connection: {e}")
            anchor_instrument = None
            return False

def disconnect_from_anchor():
    """Disconnects from the currently connected anchor."""
    global anchor_instrument
    with anchor_lock:
        if anchor_instrument and anchor_instrument.serial.is_open:
            try:
                anchor_instrument.serial.close()
                print(f"Disconnected from {anchor_instrument.serial.port}")
            except Exception as e:
                print(f"Error during disconnect: {e}")
            finally:
                anchor_instrument = None

def is_connected() -> bool:
    """Checks if the anchor is currently connected."""
    with anchor_lock:
        return anchor_instrument is not None and anchor_instrument.serial.is_open

def _calculate_crc(data):
    """Compute the CRC16 for Modbus RTU."""
    crc = 0xFFFF
    for char in data:
        crc ^= char
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

def start_positioning():
    """
    Starts positioning using RAW SERIAL write.
    This avoids MinimalModbus consuming the data stream while looking for an ACK.
    """
    if not is_connected(): return False
    
    # Get the current Slave ID (default 1)
    slave_id = anchor_instrument.address 
    
    # Build Modbus RTU Write Multiple Registers (FC16) Packet
    # Structure: ID(1) + FC(1) + Addr(2) + Qty(2) + ByteCnt(1) + Val(2) + CRC(2)
    # Address 59 = 0x003B, Value 4 = 0x0004
    payload = struct.pack('>BBHHB', slave_id, 16, 59, 1, 2) + struct.pack('>H', 4)
    crc = _calculate_crc(payload)
    packet = payload + crc
    
    for attempt in range(3):
        try:
            with anchor_lock:
                # Flush buffers to ensure we see new data
                anchor_instrument.serial.reset_input_buffer()
                anchor_instrument.serial.reset_output_buffer()
                
                print(f"DEBUG: Sending Raw Start Command: {packet.hex()}")
                anchor_instrument.serial.write(packet)
                
            # Wait for device to process and switch to stream mode
            # 0.5s is safer than 0.2s for hardware state changes
            time.sleep(0.5) 
            
            with anchor_lock:
                bytes_waiting = anchor_instrument.serial.in_waiting
                if bytes_waiting > 0:
                    print(f"DEBUG: SUCCESS! Data stream detected ({bytes_waiting} bytes).")
                    return True
                else:
                    print(f"DEBUG: No data stream detected after 0.5s (Attempt {attempt+1}).")
                    
        except Exception as e:
            print(f"Start Pos Attempt {attempt+1} failed: {e}")
            time.sleep(0.1)
            
    return False

# def start_positioning():
#     """Starts positioning with retries and handles immediate-stream-start behavior."""
#     if not is_connected(): return False
    
#     for attempt in range(3):
#         try:
#             with anchor_lock:
#                 anchor_instrument.serial.reset_input_buffer()
#                 anchor_instrument.serial.reset_output_buffer()
                
#                 # Send the start command
#                 # We use a very short timeout here because if it switches modes, 
#                 # we don't want to wait long for an ACK that will never come.
#                 old_timeout = anchor_instrument.serial.timeout
#                 anchor_instrument.serial.timeout = 0.2
                
#                 try:
#                     anchor_instrument.write_registers(config.MODBUS_ADDR_POSITIONING_CONTROL, [config.MODBUS_VAL_START_POSITIONING])
#                     anchor_instrument.serial.timeout = old_timeout
#                     return True # Ideal case: command acknowledged
#                 except (minimalmodbus.NoResponseError, minimalmodbus.InvalidResponseError):
#                     # Restore timeout
#                     anchor_instrument.serial.timeout = old_timeout
                    
#                     # This is the fix: The device likely switched to stream mode too fast 
#                     # to send a clean Modbus ACK. We check if data is flowing.
#                     print(f"DEBUG: Modbus ACK missing (Attempt {attempt+1}). Checking for data stream...")
#                     time.sleep(0.2) 
#                     if anchor_instrument.serial.in_waiting > 0:
#                         print(f"DEBUG: {anchor_instrument.serial.in_waiting} bytes detected in buffer. Positioning started successfully!")
#                         return True
#                     else:
#                         # Real failure, no data streaming
#                         print("DEBUG: No data stream detected.")
#                         raise Exception("No ACK and no data stream.")

#         except Exception as e:
#             print(f"Start Pos Attempt {attempt+1} failed: {e}")
#             time.sleep(0.1)
            
#     return False

def stop_positioning():
    """Stops positioning with retries."""
    if not is_connected(): return False
    
    for attempt in range(3):
        try:
            with anchor_lock:
                anchor_instrument.write_registers(config.MODBUS_ADDR_POSITIONING_CONTROL, [config.MODBUS_VAL_STOP_POSITIONING])
            return True
        except Exception as e:
            print(f"Stop Pos Attempt {attempt+1} failed: {e}")
            time.sleep(0.1)
            
    return False

def read_rtls_stream() -> bytes:
    """
    Reads all available bytes from the serial buffer.
    This is used to capture the continuous stream of positioning data.
    """
    if not is_connected():
        return None
    
    with anchor_lock:
        try:
            bytes_to_read = anchor_instrument.serial.in_waiting
            if bytes_to_read > 0:
                return anchor_instrument.serial.read(bytes_to_read)
        except Exception as e:
            print(f"Error reading RTLS data stream: {e}")
    return None

def read_config_registers(addr_map_ignored=None) -> dict:
    """Reads configuration safely by reading the first 20 registers."""
    if not is_connected(): return None
    
    
    read_data = {}
    with anchor_lock:
        try:
            # Read 20 registers (0-19)
            regs = anchor_instrument.read_registers(0, 20, functioncode=3)
            
            read_data['MODBUS-ID'] = regs[1] & 0xFF
            read_data['Ranging Mode'] = (regs[2] >> 8) & 0xFF
            read_data['Positioning Mode'] = regs[2] & 0xFF
            read_data['Device Type'] = regs[3] & 0xFF
            read_data['Device ID'] = regs[4] & 0xFF
            read_data['UWB Channel'] = (regs[5] >> 8) & 0xFF
            read_data['Data Rate'] = regs[5] & 0xFF
            read_data['Kalman-Q'] = regs[6]
            read_data['Kalman-R'] = regs[7]
            read_data['Antenna Delay'] = regs[8]
            
            if len(regs) > 16:
                read_data['Network ID'] = regs[16]
            
            return read_data
        except Exception as e:
            print(f"ERROR Reading Config: {e}")
            return None

def write_config_registers(registers: dict) -> bool:
    if not is_connected(): return False
    with anchor_lock:
        try:
            for addr, val in registers.items():
                anchor_instrument.write_register(addr, val, functioncode=6)
                time.sleep(0.02)
            return True
        except Exception as e:
            print(f"Write Error: {e}")
            return False

def write_register(register_address: int, value: int, functioncode: int = 16) -> bool:
    """Writes a single value."""
    if not is_connected(): return False
    with anchor_lock:
        try:
            anchor_instrument.write_register(register_address, value, functioncode=functioncode)
            return True
        except Exception as e:
            print(f"Error writing register {register_address}: {e}")
            return False

def get_instrument():
    with anchor_lock:
        return anchor_instrument

def read_modbus_register(register_address):
    instrument = get_instrument()
    if not instrument: return None
    with anchor_lock:
        try:
            value = instrument.read_register(register_address, functioncode=3)
            return value
        except Exception as e:
            print(f"ERROR: Reading from register {register_address}: {e}")
            return None
        
def write_full_configuration(config_bytes: bytes) -> bool:
    """Writes the full 122-byte configuration block."""
    if not is_connected(): return False
    with anchor_lock:
        try:
            # minimalmodbus handles converting bytes to 16-bit words automatically for write_registers
            # if we pass bytes, we might need to convert to integers first.
            # Let's safely convert bytes to list of int16s
            values = []
            for i in range(0, len(config_bytes), 2):
                val = (config_bytes[i] << 8) | config_bytes[i+1]
                values.append(val)
                
            anchor_instrument.write_registers(0, values)
            return True
        except Exception as e:
            print(f"Bulk Write Error: {e}")
            return False
        

            
