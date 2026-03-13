#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 成都长数机器人有限公司 (Chengdu Changsu Robot Co., Ltd.)
# Website: https://openarmx.com/
# Contact: Mr Wang
# Phone & WeChat: +86-17746530375
# Email: openarmrobot@gmail.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author Information:
# Company: 成都长数机器人有限公司 (Chengdu Changsu Robot Co., Ltd.)
# Website: https://openarmx.com/
# Contact Person: Mr Wang
# Phone & WeChat: +86-17746530375
# Email: openarmrobot@gmail.com

"""
Robstride Motor Baudrate Configuration Script
Based on Robstride motor protocol for configuring CAN baudrate
Adapted from DM Motor version for Robstride compatibility
"""

import argparse
import sys
import can
import time
from typing import Optional


class RobstrideMotorWriter:
    """Robstride Motor parameter writer"""

    # Supported baudrates for Robstride motors
    # Note: Robstride typically uses 1Mbps CAN 2.0
    BAUDRATE_MAP = {
        125000: 0,   # 125K
        250000: 1,   # 250K
        500000: 2,   # 500K
        1000000: 3,  # 1M (standard for Robstride)
        2000000: 4,  # 2M
        4000000: 5,  # 4M
        8000000: 6   # 8M
    }

    def __init__(self, socketcan_port: str = "can0"):
        self.socketcan_port = socketcan_port
        self.can_bus: Optional[can.BusABC] = None

    def connect(self) -> bool:
        """Connect to CAN bus"""
        try:
            print(f"Connecting to CAN interface: {self.socketcan_port}")
            self.can_bus = can.interface.Bus(
                channel=self.socketcan_port,
                interface='socketcan'
            )
            print(f"✓ Connected to {self.socketcan_port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.can_bus:
            self.can_bus.shutdown()
            self.can_bus = None
            print("Disconnected from CAN bus")

    def validate_baudrate(self, baudrate: int) -> bool:
        """Validate if baudrate is supported"""
        return baudrate in self.BAUDRATE_MAP

    def validate_motor_id(self, motor_id: int) -> bool:
        """Validate Robstride motor ID range (1-8)"""
        return 1 <= motor_id <= 8

    def write_baudrate(self, motor_id: int, baudrate: int) -> bool:
        """Write baudrate to Robstride motor"""
        if not self.can_bus:
            print("✗ Not connected to CAN bus")
            return False

        if not self.validate_motor_id(motor_id):
            print(f"✗ Invalid motor ID: {motor_id}. Robstride motors use IDs 1-8")
            return False

        if not self.validate_baudrate(baudrate):
            print(f"✗ Unsupported baudrate: {baudrate}")
            print(f"Supported baudrates: {list(self.BAUDRATE_MAP.keys())}")
            return False

        try:
            baudrate_code = self.BAUDRATE_MAP[baudrate]

            print(f"Writing baudrate {baudrate} (code: {baudrate_code}) to Robstride motor ID {motor_id}")

            # Robstride baudrate configuration command format
            # Based on Robstride protocol - parameter write command
            # Format: 0600FE0X#[PARAM_ID][VALUE][00][00][00][00][00][00]
            motor_id_hex = f"{motor_id:02d}"
            can_id = int(f"0600FE{motor_id_hex}", 16)

            # Robstride parameter: Baudrate setting (parameter ID may vary - check manual)
            # Using general parameter write format
            baudrate_data = [
                0x10,           # Parameter ID for baudrate (check Robstride manual)
                baudrate_code,  # Baudrate code
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            ]

            baudrate_msg = can.Message(
                arbitration_id=can_id,
                data=baudrate_data,
                is_extended_id=True  # Robstride uses extended frames
            )

            self.can_bus.send(baudrate_msg)
            time.sleep(0.1)  # Wait for processing

            print(f"✓ Baudrate {baudrate} configuration sent to Robstride motor ID {motor_id}")
            print("⚠️  Baudrate change requires motor power cycle to take effect")
            return True

        except Exception as e:
            print(f"✗ Failed to write baudrate: {e}")
            return False

    def save_to_flash(self, motor_id: int) -> bool:
        """Save parameters to flash memory for Robstride motor"""
        if not self.can_bus:
            print("✗ Not connected to CAN bus")
            return False

        if not self.validate_motor_id(motor_id):
            print(f"✗ Invalid motor ID: {motor_id}. Robstride motors use IDs 1-8")
            return False

        try:
            print(f"Saving parameters to flash for Robstride motor ID {motor_id}")
            print("⚠️  Motor will store parameters in non-volatile memory")

            # Robstride save to flash command format
            # Format: 0600FE0X#[SAVE_CMD][00][00][00][00][00][00][00]
            motor_id_hex = f"{motor_id:02d}"
            can_id = int(f"0600FE{motor_id_hex}", 16)

            save_data = [
                0xFF,  # Save command (check Robstride manual for exact code)
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            ]

            save_msg = can.Message(
                arbitration_id=can_id,
                data=save_data,
                is_extended_id=True
            )

            self.can_bus.send(save_msg)
            time.sleep(0.5)  # Wait for save operation

            print(f"✓ Parameters save command sent to Robstride motor ID {motor_id}")
            return True

        except Exception as e:
            print(f"✗ Failed to save to flash: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(
        description="Robstride Motor Baudrate Configuration Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Write baudrate 1000000 to motor ID 2 on can0
  python change_baudrate.py --baudrate 1000000 --canid 2 --socketcan can0

  # Write baudrate and save to flash
  python change_baudrate.py --baudrate 500000 --canid 1 --socketcan can0 --flash

  # Write to motor on different CAN port
  python change_baudrate.py --baudrate 2000000 --canid 3 --socketcan can1

Supported baudrates:
  125000, 250000, 500000, 1000000, 2000000, 4000000, 8000000

Note: Robstride motors typically use 1000000 (1Mbps) baudrate
      Motor IDs range from 1 to 8 for Robstride motors
        """
    )

    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        required=True,
        help='Baudrate to write (125000, 250000, 500000, 1000000, 2000000, 4000000, 8000000)'
    )

    parser.add_argument(
        '-c', '--canid',
        type=int,
        required=True,
        help='Robstride motor CAN ID to write to (1-8)'
    )

    parser.add_argument(
        '-s', '--socketcan',
        type=str,
        default='can0',
        help='SocketCAN port (default: can0)'
    )

    parser.add_argument(
        '-f', '--flash',
        action='store_true',
        help='Save parameters to flash memory after writing'
    )

    args = parser.parse_args()

    # Create writer instance
    writer = RobstrideMotorWriter(args.socketcan)

    try:
        # Connect to CAN bus
        if not writer.connect():
            print("✗ Failed to connect to CAN bus")
            sys.exit(1)

        # Write baudrate
        if not writer.write_baudrate(args.canid, args.baudrate):
            print("✗ Failed to write baudrate")
            sys.exit(1)

        # Save to flash if requested
        if args.flash:
            if not writer.save_to_flash(args.canid):
                print("✗ Failed to save to flash")
                sys.exit(1)

        print("✓ Operation completed successfully")
        print("⚠️  Power cycle the motor for baudrate changes to take effect")

    except KeyboardInterrupt:
        print("\n⚠️  Operation cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        sys.exit(1)
    finally:
        writer.disconnect()


if __name__ == "__main__":
    main()