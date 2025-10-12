"""
/*******************************************************************************************
 *  UM982 Reader - Python Interface [Testing Script]
 *  
 *  Description:
 *  This script establishes a serial connection with the UM982 GNSS receiver and 
 *  continuously reads incoming data. It displays both standard NMEA messages and 
 *  UM982-specific proprietary messages in real time.
 *
 *  Usage:
 *   - Run: python um982_reader.py
 *   - The script will continuously print all messages received from the UM982.
 *   - Press Ctrl+C to stop execution.
 *
 *  Author: Athar Ahmed  <athar.a@virya.ai>
 *  Organization: Virya Autonomous Technologies Pvt. Ltd.
 *  Version: 1.0.0
 *******************************************************************************************/
"""

import serial
import time

def main():
    port = "/dev/ttyUSB0"
    baudrate = 115200

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print(f"[INFO] Listening to UM982 on {port} @ {baudrate} baud")
            while True:
                try:
                    line = ser.readline().decode(errors="ignore").strip()
                    if line:
                        print(f"[RX] {line}")
                except Exception as e:
                    print(f"[ERROR] {e}")
                    break
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    except Exception as e:
        print(f"[ERROR] Could not open port: {e}")

if __name__ == "__main__":
    main()
