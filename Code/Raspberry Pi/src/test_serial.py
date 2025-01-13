import serial
import json

# Replace 'COM6' with the appropriate serial port for your system
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def main():
    try:
        # Open the serial port
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
            
            while True:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()
                
                if line:
                    try:
                        # Parse JSON-like data
                        data = json.loads(line)
                        is_pushed = data.get("is_pushed", None)
                        
                        if is_pushed is not None:
                            print(f"is_pushed: {is_pushed}")
                    except json.JSONDecodeError:
                        print(f"Failed to parse: {line}")
                        
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    main()
