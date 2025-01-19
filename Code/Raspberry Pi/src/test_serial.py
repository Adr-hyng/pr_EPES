import serial
import random

SERIAL_PORT = 'COM6'  # Adjust based on your setup
BAUD_RATE = 115200

HeaterActivated = 0
TempSelectedOptions = [65, 75, 85]
TempSelectedIndex = 0

def serial_reader_thread():
    global ContainerCap, Mode, CurTemperature, IsPushed
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        # Parse the CSV format
                        parts = line.split(',')
                        ser.write(f"0,{random.choice([65, 75, 85])},{random.randint(0, 1)},{random.randint(0, 1)},1\n".encode('utf-8'))
                        print(line)
                        if len(parts) == 5:
                            Receivable = int(parts[0])
                            if not Receivable: continue
                            ContainerCap = int(parts[1])
                            Mode = int(parts[2])
                            CurTemperature = int(parts[3])
                            IsPushed = int(parts[4])
                            # print(f"Parsed data: ContainerCap={ContainerCap}, Mode={Mode}, "f"CurTemperature={CurTemperature}, IsPushed={IsPushed}, Heater={HeaterActivated}, SelTemp={TempSelectedOptions[TempSelectedIndex]}-C")
                            # ser.write(f"1,75,1,1\n".encode('utf-8'))
                        else:
                            print(f"Unexpected data format: {line}")
                    except ValueError:
                        print(f"Failed to parse: {line}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        
if __name__ == "__main__":
  
  serial_reader_thread()