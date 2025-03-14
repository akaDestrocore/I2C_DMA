import os
import subprocess
import sys
import re
import serial
import serial.tools.list_ports


def get_serial_port():
    ports = serial.tools.list_ports.comports()
    
    # search for `STM`
    for port in ports:
        description = port.description or ""
        manufacturer = port.manufacturer or ""
        
        if "STM" in description or "STMicroelectronics" in manufacturer:
            return port.device
    
    # use default name instead
    default_port = "/dev/ttyACM0" if os.name != 'nt' else "COM3"
    print(f"Could not find STM32 UART port, defaulting to {default_port}")
    return default_port


def read_uart_output(serial_device, start_marker, end_marker):
    with serial.Serial(serial_device, 115200, timeout=0.05) as ser:
        output = ""
        start_found = False
        end_found = False

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        while True:
            line = ser.readline().decode('utf-8', errors='ignore')
            if start_marker in line:
                start_found = True
                # start from after the start marker
                line = line.split(start_marker, 1)[-1]
            elif end_marker in line and start_found:
                output += line.split(end_marker, 1)[0]
                end_found = True
                break
            elif start_found:
                output += line

        return output


firm = sys.argv[1] if len(sys.argv) > 1 else ""
uart_log_file = "uart_output.log"

if os.path.isfile(uart_log_file):
    os.remove(uart_log_file)

serial_device = get_serial_port()

flash_command = [
    "STM32_Programmer_CLI", 
    "--connect", "port=swd", 
    "--download", firm, 
    "--start"
]

# run flashing command
try:
    flashing_res = subprocess.run(flash_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
except subprocess.TimeoutExpired:
    print("Flashing process timed out")
    sys.exit(1)

if flashing_res.returncode != 0:
    print("Flashing failed")
    sys.exit(1)

start_marker = "===== Test Output Log ====="
end_marker   = "===== Test Output End ====="

uart_output = read_uart_output(serial_device, start_marker, end_marker)

# write the output to a file
with open(uart_log_file, 'w') as f:
    f.write(uart_output)

# print output to console
print(uart_output)
print("CTEST_FULL_OUTPUT")

