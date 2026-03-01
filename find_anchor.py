# find_anchor.py

import serial.tools.list_ports
import minimalmodbus

# --- CONFIGURATION ---
# IMPORTANT: Change this to the COM port your anchor is connected to!
PORT_TO_SCAN = 'COM4' 

# List of common baud rates to try
BAUD_RATES_TO_TRY = [115200, 57600, 38400, 19200, 9600]

# Range of MODBUS Slave IDs to try
SLAVE_IDS_TO_TRY = range(1, 248) # Standard range is 1-247

def find_the_anchor():
    """
    Scans a serial port with multiple baud rates and slave IDs to find a responsive anchor.
    """
    print(f"--- Starting Brute Force Scan on Port {PORT_TO_SCAN} ---")
    
    for baud in BAUD_RATES_TO_TRY:
        print(f"\n[+] Trying Baud Rate: {baud}...")
        for slave_id in SLAVE_IDS_TO_TRY:
            instrument = None
            try:
                # Set up the instrument with the current combination
                instrument = minimalmodbus.Instrument(PORT_TO_SCAN, slave_id)
                instrument.serial.baudrate = baud
                instrument.serial.timeout = 0.1 # Use a short timeout to speed up scanning
                instrument.mode = minimalmodbus.MODE_RTU
                
                # Try to read a known register. Register 0 (Firmware Version) is a good choice.
                # This is the line that will either succeed or raise an exception.
                firmware_version = instrument.read_register(0, functioncode=3)
                
                # If we reach this line, we have found it!
                print("\n" + "="*40)
                print("  ★★★ ANCHOR FOUND! ★★★")
                print(f"  Port:       {PORT_TO_SCAN}")
                print(f"  Baud Rate:  {baud}")
                print(f"  MODBUS ID:  {slave_id}")
                print(f"  Firmware:   {firmware_version}")
                print("="*40 + "\n")
                print("Use these settings in the main application to reconnect and save a good configuration.")
                return # Exit the function after finding the anchor
                
            except Exception as e:
                # This is expected for every failed attempt. We just continue.
                print(f".", end='', flush=True) # Print a dot for progress
                
            finally:
                # Make sure the serial port is closed after each attempt
                if instrument and instrument.serial.is_open:
                    instrument.serial.close()

    print("\n\n--- Scan Complete ---")
    print("Could not find a responsive anchor on any common baud rate or ID.")
    print("Proceed to Phase 2: Hardware Factory Reset.")

if __name__ == "__main__":
    find_the_anchor()