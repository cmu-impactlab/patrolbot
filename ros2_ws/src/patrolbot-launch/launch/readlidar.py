import serial
import struct
import time

SERIAL_PORT = '/dev/ttyLidarRemote'
BAUD_RATE = 9600

# SICK LMS Start Scanning Telegram: {STX, Addr, LenL, LenH, Mode, Checksum}
# Common "Start" command for LMS-200
START_COMMAND = b'\x02\x00\x02\x00\x21\x01\x32\x08' 

def parse_lms200():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        print(f"Connected to {SERIAL_PORT}.")
        
        # Step 1: Send the Start Command
        print("Sending Start Command to Lidar...")
        ser.write(START_COMMAND)
        time.sleep(1) # Give it a second to spin up
        
        while True:
            byte = ser.read(1)
            if byte == b'\x02':
                # Read Header (Addr + Length)
                header = ser.read(3)
                if len(header) < 3: continue
                addr, length = struct.unpack('<BH', header)
                
                # Read Payload + 2 bytes CRC
                payload = ser.read(length + 2)
                if len(payload) < length + 2: continue

                # Check for 0xB0 (Data Packet) or 0xA1 (Acknowledge)
                if payload[0] == 0xB0:
                    num_readings = (length - 6) // 2 
                    distances = struct.unpack(f'<{num_readings}H', payload[6:6 + num_readings * 2])
                    
                    # Print Human Readable Info
                    center = distances[len(distances)//2]
                    print(f"Lidar Active | Points: {num_readings} | Center: {center}mm")
                elif payload[0] == 0xA1:
                    print("Lidar acknowledged the command. Starting stream...")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals(): ser.close()

if __name__ == "__main__":
    parse_lms200()
