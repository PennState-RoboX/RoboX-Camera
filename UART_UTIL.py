import math
import os
import re
import struct
import sys
import time
from typing import Optional, Tuple

import serial


# Direction of the camera-relative angles sent to the C-board. The firmware aims
# via `new_target = current_absolute_angle - cv_value`, and the physical motor
# direction is not derivable from source, so flip these on the bench if the gimbal
# drives away from the target instead of toward it.
YAW_SIGN = +1
PITCH_SIGN = +1


def encode_yaw_pitch_checksum(yaw_rad: float, pitch_rad: float) -> Tuple[str, str, str]:
    """
    Build hex strings for send_data: yaw/pitch as IEEE-754 float32, big-endian
    (4 bytes / 8 hex chars each), in RADIANS, plus a 1-byte checksum = sum of the
    8 payload bytes mod 256.

    This matches the C-board firmware parser (referee_usart_task.c: 14-byte packet,
    yaw at bytes 3-6 and pitch at bytes 7-10 assembled big-endian into a float) and
    the proven 2022 CV sender (Robomaster2022-CV/ArmorDetect.py decimalToHexSerial).
    """
    payload = struct.pack(">ff", float(yaw_rad), float(pitch_rad))
    hex_yaw = payload[:4].hex()
    hex_pitch = payload[4:].hex()
    checksum = sum(payload) % 256
    return hex_yaw, hex_pitch, f"{checksum:02x}"


def send_target_angles_deg(ser, yaw_deg: float, pitch_deg: float, detect_success: bool) -> None:
    """Encode camera-relative angles (degrees in) and transmit one packet.

    Angles are converted to radians before encoding because the firmware compares
    them against its IMU euler angles, which are in radians.
    """
    yaw_rad = math.radians(YAW_SIGN * float(yaw_deg))
    pitch_rad = math.radians(PITCH_SIGN * float(pitch_deg))
    hy, hp, cs = encode_yaw_pitch_checksum(yaw_rad, pitch_rad)
    send_data(ser, hy, hp, cs, detect_success)


def open_robot_serial(
    port: Optional[str] = None,
    baud: Optional[int] = None,
    timeout: float = 0.05,
) -> Optional[serial.Serial]:
    """
    Open the UART used by send_data. Port defaults to env ROBOX_SERIAL_PORT; if unset,
    returns None (camera scripts can run without hardware).
    """
    p = port if port is not None else os.environ.get("ROBOX_SERIAL_PORT", "").strip()
    if not p:
        return None
    b = baud if baud is not None else int(os.environ.get("ROBOX_SERIAL_BAUD", "115200"))
    try:
        return serial.Serial(p, b, timeout=timeout)
    except (serial.SerialException, OSError, ValueError) as e:
        print(f"UART open failed ({p!r}): {e}", file=sys.stderr)
        return None


# Angles are in Byte Format
def send_data(ser, hex_Yaw, hex_Pitch, checksum, detect_success: bool):
    # Convert detect_success from boolean to hex string ('01' for True, '00' for False)
    hex_detect_success = '01' if detect_success else '00'
    
    # Construct the 14-byte packet (header 2 + length 1 + yaw 4 + pitch 4 +
    # checksum 1 + detect 1 + footer 1). The length byte is kept at 0x0d to match
    # the firmware (which ignores byte[2]) and the proven 2022 sender.
    packet = 'a5'  # Header Byte
    packet += '5a'
    packet += '0d'  # Length byte (firmware ignores; kept for protocol compatibility)
    packet += hex_Yaw       # 4 bytes, IEEE-754 float32 big-endian (radians)
    packet += hex_Pitch     # 4 bytes, IEEE-754 float32 big-endian (radians)
    packet += checksum
    packet += hex_detect_success
    packet += 'ff'

    # Convert the hexadecimal string to bytes
    packet_bytes = bytes.fromhex(packet)
    # Write the bytes to the serial port
    ser.write(packet_bytes)


def get_imu(ser):
    imu_value = [0, 0, 0]
    if not ser.in_waiting or ser.in_waiting == 0:
        return imu_value

    while (True):
        # print(ser.in_waiting)
        raw_data = ser.read(100)
        data = raw_data.decode('utf-8', 'replace')
        # print(len(raw_data))  # Count bytes

        if 'A5' in data:
            try:
                start = data.index('A5') + 2  # Skip the 'A5'
                end = data.index('A5', start)  # Find the next 'A5' or end of the string
                imu_readings = data[start:end].split(',')

                # Convert the first three elements to floats and return them
                if len(imu_readings) >= 3:
                    imu_value = [float(x) for x in imu_readings[:3]]
                    # print(imu_value)
                    return imu_value
            except Exception as e:
                print(f"Error processing IMU data: {e}")
                continue
                # Handle the exception or continue the loop
    return imu_value


# # Example usage
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    while True:
        imu = get_imu(ser)
        print(imu)
        # send_data()