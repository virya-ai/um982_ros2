#!/usr/bin/env python3
"""
/*******************************************************************************************
 *  UM982 Setup - Automated Configuration Script
 *
 *  Description:
 *  This script establishes a serial connection with the UM982 GNSS receiver and 
 *  automatically sends a predefined sequence of configuration commands. 
 *  A 5-second delay is applied between each command to ensure proper execution 
 *  and device response.
 *
 *  The script automates the following configuration steps:
 *   1. System Initialization
 *      - freset - resets the board
 *      - unlog - stops logging any message which exists
 *
 *   2. Serial Port & Bidirectional Setup
 *      - config com2 115200 8 n 1 -> com port might differ please ensure to set the correct one
 *
 *   3. Rover & RTK Settings
 *      - mode rover AUTOMOTIVE - sets the um982 as rover
 *      - config rtk timeout 10 - times out rtk to 10s 
 *      - config rtk reliability 3 1 
 *      - config standalone enable - can give standalone gps readings
 *
 *   4. Dual-Antenna Heading Configuration
 *      - config heading fixlength
 *      - config heading length 98.5 1 -> 98.5cm with 1cm tolerance [this is to be measured and changed ]
 *      - config smooth heading 10
 *      - config signalgroup 4 5
 *
 *   5. GNSS Constellation & Anti-Interference
 *      - unmask gps
 *      - unmask glo
 *      - unmask gal
 *      - unmask bds
 *      - mask 5
 *      - config antijam force
 *      - config mmp enable
 *
 *   6. Performance Optimization
 *      - config pvtalg multi
 *      - config psrposbias enable
 *      - config agnss enable
 *
 *   7. NMEA & Proprietary Message Outputs - setting the hz -> times per second
 *      - gpgga 0.1
 *      - gpgsv 1
 *      - bestnava 0.1
 *      - bestnavha 0.1
 *      - pvtslna 0.1
 *      - gphpr 0.1
 *      - uniheadinga 0.1
 *      - gprmc 0.1
 *      - gpths 0.1
 *
 *   8. Quality & Status Monitoring (Optional)
 *      - gpgst 0.2
 *      - gpvtg 0.1
 *
 *   9. Save Configuration
 *      - saveconfig
 *
 *  Notes:
 *   -> Ensure that the correct distance between the antennas is configured.
 *   -> The board must be set to 115200 baud before running this script for reliable communication.
 *
 *  To Do:
 *   - Automate baud rate detection to connect at any available baud rate 
 *     and update it to the desired setting automatically.
 *
 *  Note:
 *  This software is proprietary to Virya Autonomous Technologies Pvt. Ltd.
 *  It must not be shared, copied, or distributed without prior authorization.
 *
 *  Author: Athar Ahmed  <athar.a@virya.ai>
 *  Organization: Virya Autonomous Technologies Pvt. Ltd.
 *  Version: 1.0.0
 *******************************************************************************************/
"""
import serial
import time

# Serial port settings
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
TIMEOUT = 2  # seconds

# List of setup commands in order
COMMAND_SEQUENCE = [
    # 1. System Initialization
    "freset",
    "version",
    "unlog",
    # 2. Serial Port & Bidirectional Setup
    "config com2 115200 8 n 1",
    # 3. Rover & RTK Settings
    "mode rover AUTOMOTIVE",
    "config rtk timeout 10",
    "config rtk reliability 3 1",
    "config standalone enable",
    # 4. Dual-Antenna Heading Configuration
    "config heading fixlength",
    "config heading length 98.5 1",
    "config smooth heading 10",
    "config signalgroup 4 5",
    # 5. GNSS Constellation & Anti-Interference
    "unmask gps",
    "unmask glo",
    "unmask gal",
    "unmask bds",
    "mask 5",
    "config antijam force",
    "config mmp enable",
    # 6. Performance Optimization
    "config pvtalg multi",
    "config psrposbias enable",
    "config agnss enable",
    # 7. NMEA & Proprietary Message Outputs
    "gpgga 0.1",
    "gpgsv 1",
    "bestnava 0.1",
    "bestnavha 0.1",
    "pvtslna 0.1",
    "gphpr 0.1",
    "uniheadinga 0.1",
    
    # 8. Quality & Status Monitoring [Optional]
    #"gpgst 0.2",
    #"gpvtg 0.1",
    #"rtkstatusa 1",
    #"rtcmstatusa onchanged",         # Please not this only supports ONCHANGED
    #"hwstatusa 1",
    #"satellitea 1",
    #"headingstatusa 1",
    
    # 9. Save Configuration
    "saveconfig",
]

def send_command(ser, cmd):
    """Send a single command."""
    ser.write((cmd + "\r\n").encode("ascii"))

def main():
    print(f"[INFO] Opening serial port {PORT} @ {BAUDRATE} baud...")
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT) as ser:
            print("[INFO] Connection established.\n")
            print("[INFO] Sending configuration commands...\n")

            for idx, cmd in enumerate(COMMAND_SEQUENCE, start=1):
                print(f">>> [{idx:02d}] TX: {cmd}")
                send_command(ser, cmd)
                time.sleep(5.0)  # 5-second delay between commands

            print("\n[INFO] All commands sent. Exiting.")

    except serial.SerialException as e:
        print(f"[ERROR] Could not open serial port: {e}")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")

if __name__ == "__main__":
    main()