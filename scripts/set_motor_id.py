#!/usr/bin/env python3
import sys
import serial
import time
import rclpy
from rclpy.node import Node

def calculate_crc8_maxim(data):
    """
    CRC-8/MAXIM (LSB First, Polynomial: 0x31 reflected -> 0x8C)
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

def set_motor_id(port, new_id, logger=None):
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        
        # Structure: Sync Header (AA, 55, 53), New ID, followed by 5 bytes of 0x00, and CRC
        command = bytearray([0xAA, 0x55, 0x53, new_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        command[9] = calculate_crc8_maxim(command[:9])
        
        msg = f"[{port}] Setting motor ID to {new_id}..."
        if logger:
            logger.info(msg)
        else:
            print(msg)
        
        # Display the payload for verification
        payload_str = " ".join(f"{b:02X}" for b in command)
        msg_payload = f"Payload: {payload_str}"
        if logger:
            logger.info(msg_payload)
        else:
            print(msg_payload)

        # Must send 5 times consecutively
        for i in range(5):
            ser.write(command)
            time.sleep(0.1)
            msg_send = f"Transmitted packet {i+1}/5"
            if logger:
                logger.info(msg_send)
            else:
                print(msg_send)
            
        success_msg = "ID configuration packet sent successfully. Please POWER CYCLE the motor to apply changes."
        if logger:
            logger.info(success_msg)
        else:
            print(success_msg)
        
        ser.close()
        return True
        
    except Exception as e:
        err_msg = f"Error during communication: {e}"
        if logger:
            logger.error(err_msg)
        else:
            print(err_msg, file=sys.stderr)
        return False

class MotorIdSetterNode(Node):
    def __init__(self):
        super().__init__('set_motor_id_node')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('id', 1)
        
        port = self.get_parameter('serial_port').value
        new_id = self.get_parameter('id').value
        
        if not (1 <= new_id <= 253):
            self.get_logger().error("Motor ID must be between 1 and 253.")
            sys.exit(1)
            
        set_motor_id(port, new_id, self.get_logger())

def main(args=None):
    if len(sys.argv) > 1 and sys.argv[1] not in ["--ros-args", "-r", "-p"]:
        # Command line running without ROS wrapper
        import argparse
        parser = argparse.ArgumentParser(description="Set DDSM115 Motor ID over serial.")
        parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Serial port (e.g. /dev/ttyUSB0)")
        parser.add_argument("-i", "--id", type=int, required=True, help="New motor ID (1-253)")
        args = parser.parse_args()
        
        if not (1 <= args.id <= 253):
            print("Motor ID must be between 1 and 253.", file=sys.stderr)
            sys.exit(1)
            
        set_motor_id(args.port, args.id)
    else:
        # Standard ROS 2 run execution
        rclpy.init(args=args)
        node = MotorIdSetterNode()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
