"""
/*******************************************************************************************
 *  UM982 Writer - Python Interface [Testing Script]
 * 
 *  Description:
 *  Provides an interactive interface to send commands to the UM982 GNSS receiver over a 
 *  serial connection. Users can type commands manually and transmit them directly to the device.
 *
 *  Usage:
 *   - Run: python um982_writer.py
 *   - Type commands at the ">" prompt to send to the UM982
 *   - Type 'exit' or 'quit' to terminate the session
 *
 *  Author: Athar Ahmed  <athar.a@virya.ai>
 *  Organization: Virya Autonomous Technologies Pvt. Ltd.
 *  Version: 1.0
 *******************************************************************************************/
"""
import serial
def main():
    port = "/dev/ttyUSB0"
    baudrate = 115200
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print(f"[INFO] Connected to UM982 on {port} @ {baudrate} baud")
            print("Type commands to send. Type 'exit' to quit.")
            
            while True:
                try:
                    cmd = input("> ").strip()
                    if cmd.lower() in ["exit", "quit"]:
                        break
                    if cmd:
                        full_cmd = cmd + "\r\n"
                        ser.write(full_cmd.encode("ascii"))
                        print(f"[TX] {cmd}")
                except KeyboardInterrupt:
                    print("\n[INFO] Keyboard interrupt detected.")
                    break
    except Exception as e:
        print(f"[ERROR] Could not open port: {e}")

if __name__ == "__main__":
    main()