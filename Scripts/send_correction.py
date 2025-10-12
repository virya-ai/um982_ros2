"""
/*******************************************************************************************
 *  Test Script for UM982 RTK-GNSS Receiver
 *  
 *  Description:
 *  This script establishes a serial connection with the UM982 GNSS receiver.
 *  It performs the following functions:
 *    - Reads GGA messages from the UM982.
 *    - Sends the GGA data to an NTRIP caster.
 *    - Receives RTK correction data from the NTRIP caster.
 *    - Transmits the corrections back to the UM982 to achieve an RTK fix.
 *
 *  Note:
 *  Ensure that the serial port, baud rate, and NTRIP caster credentials 
 *  are configured correctly before running this script.
 *
 *  Author: Athar Ahmed  <athar.a@virya.ai>
 *  Organization: Virya Autonomous Technologies Pvt. Ltd.
 *  Version: 1.0.0
 *  
 *  Note:
 *  This software is proprietary to Virya Autonomous Technologies Pvt. Ltd.
 *  It must not be shared, copied, or distributed without prior authorization.
 *******************************************************************************************/
"""

import serial
import socket
import base64
import time
import threading

# --------------------
# NTRIP CASTER SETTINGS
# --------------------
NTRIP_HOST = "103.206.29.4"
NTRIP_PORT = 2105
MOUNTPOINT = "IFKP"
USERNAME = "pranav.k"
PASSWORD = "cors@2022"

# --------------------
# SERIAL SETTINGS
# --------------------
SERIAL_PORT = "/dev/ttyUSB0"  # UM982 port
BAUDRATE = 115200

# --------------------
# FUNCTIONS
# --------------------

def connect_ntrip():
    """Connect to NTRIP caster and return socket"""
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((NTRIP_HOST, NTRIP_PORT))
            auth = base64.b64encode(f"{USERNAME}:{PASSWORD}".encode()).decode()
            request = (
                f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
                f"User-Agent: NTRIP PythonClient\r\n"
                f"Authorization: Basic {auth}\r\n\r\n"
            )
            s.send(request.encode())

            # Wait for ICY response (HTTP headers)
            response = s.recv(1024).decode(errors='ignore')
            if "ICY 200 OK" in response or "HTTP/1.0 200 OK" in response:
                print("[INFO] Connected to NTRIP caster successfully")
                return s
            else:
                print("[WARN] Unexpected NTRIP response:", response.splitlines()[0])
                s.close()
                time.sleep(5)
        except Exception as e:
            print("[WARN] Failed to connect NTRIP:", e)
            time.sleep(5)

def parse_gga(gga_line):
    """Parse $GNGGA sentence and return lat, lon, alt, fix, satellites"""
    try:
        parts = gga_line.split(",")
        if len(parts) < 10:
            return None
        lat = float(parts[2])
        lat_dir = parts[3]
        lon = float(parts[4])
        lon_dir = parts[5]
        fix = int(parts[6])
        sats = int(parts[7])
        alt = float(parts[9])

        # Convert lat/lon to decimal degrees
        lat_deg = int(lat/100) + (lat%100)/60
        if lat_dir == "S":
            lat_deg = -lat_deg
        lon_deg = int(lon/100) + (lon%100)/60
        if lon_dir == "W":
            lon_deg = -lon_deg

        # Fix mapping according to your GGA header
        fix_str = {
            0: "No Fix",
            1: "Single Point",
            2: "DGPS Fix",
            3: "GPS PPS",
            4: "RTK Fixed",
            5: "RTK Float",
            7: "Manual",
            8: "Simulator"
        }.get(fix, str(fix))

        return lat_deg, lon_deg, alt, fix_str, sats
    except:
        return None

def read_gga_and_send_ntrip(ntrip_socket, serial_port):
    """Read GGA from GNSS, send to NTRIP, and print LLA + fix + sats"""
    while True:
        try:
            line = serial_port.readline()
            if line:
                line_str = line.decode(errors='ignore').strip()
                if line_str.startswith("$GNGGA") or line_str.startswith("$GPGGA"):
                    # send to NTRIP
                    try:
                        ntrip_socket.send((line_str + "\r\n").encode())
                    except (BrokenPipeError, OSError):
                        print("[ERROR] Broken pipe while sending GGA. Reconnecting NTRIP...")
                        ntrip_socket.close()
                        ntrip_socket = connect_ntrip()

                    # parse and print LLA + fix + sats
                    parsed = parse_gga(line_str)
                    if parsed:
                        lat, lon, alt, fix, sats = parsed
                        print(f"[LLA] Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.2f}m, Fix: {fix}, Sats: {sats}")
        except Exception as e:
            print("[ERROR] Reading GGA:", e)
            time.sleep(0.1)

def receive_rtcm_and_forward(ntrip_socket, serial_port):
    """Receive RTCM corrections from NTRIP and forward to GNSS"""
    while True:
        try:
            data = ntrip_socket.recv(1024)
            if data:
                serial_port.write(data)
        except (BrokenPipeError, OSError):
            print("[ERROR] NTRIP disconnected. Reconnecting...")
            ntrip_socket.close()
            ntrip_socket = connect_ntrip()
        except Exception as e:
            print("[ERROR] Receiving RTCM:", e)
            time.sleep(0.5)

# --------------------
# MAIN
# --------------------

def main():
    # Open serial port
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    time.sleep(2)  # allow GNSS to initialize
    print(f"[INFO] Listening to UM982 on {SERIAL_PORT} @ {BAUDRATE} baud")

    # Connect to NTRIP caster
    ntrip_socket = connect_ntrip()

    # Start threads: GGA sender & RTCM receiver
    threading.Thread(target=read_gga_and_send_ntrip, args=(ntrip_socket, ser), daemon=True).start()
    threading.Thread(target=receive_rtcm_and_forward, args=(ntrip_socket, ser), daemon=True).start()

    print("[INFO] NTRIP bridge running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[INFO] Exiting...")
        ser.close()
        ntrip_socket.close()

if __name__ == "__main__":
    main()