# test_start_stop.py
import minimalmodbus
instrument = minimalmodbus.Instrument('COM4', 1)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 0.5

# Start
instrument.write_register(59, 4, functioncode=16)
print("Started")

# Wait 5 sec
import time; time.sleep(5)

# Stop
instrument.write_register(59, 0, functioncode=16)
print("Stopped")