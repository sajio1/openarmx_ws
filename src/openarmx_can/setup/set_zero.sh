#!/bin/bash
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

set -eu

# Robstride Motor Zero Position Setup Script
# Usage: setup/set_zero.sh <CAN_IF> [CAN_ID] [--all]

# Function to display usage
usage() {
    echo "Usage: $0 <CAN_IF> [CAN_ID] [--all]"
    echo "  CAN_IF: CAN interface name (e.g., can0)"
    echo "  CAN_ID: Motor CAN ID (1-8) - not needed with --all"
    echo "  --all: Send to all motor IDs from 1 to 8"
    echo ""
    echo "Examples:"
    echo "  $0 can0 5                   # Set zero for motor ID 5"
    echo "  $0 can0 --all               # Set zero for all motors (1-8)"
    echo ""
    echo "Note: This script uses Robstride motor protocol"
}

# Function to check if CAN interface is up and get bitrate
check_can_interface() {
    local interface=$1

    # Check if interface exists and is up
    if ! ip link show "$interface" &>/dev/null; then
        echo "Error: CAN interface $interface does not exist"
        return 1
    fi

    # Check if interface is up
    local state
    state=$(ip link show "$interface" | grep -o "state [A-Z]*" | cut -d' ' -f2)
    if [ "$state" != "UP" ]; then
        echo "Error: CAN interface $interface is not UP (current state: $state)"
        return 1
    fi

    echo "CAN interface $interface is UP"

    # Get bitrate information
    local bitrate
    bitrate=$(ip -details link show "$interface" 2>/dev/null | grep -o "bitrate [0-9]*" | cut -d' ' -f2)
    if [ -n "$bitrate" ]; then
        echo "Bitrate: ${bitrate} bps"
    fi

    return 0
}

# Function to send Robstride CAN messages for a single ID
send_robstride_zero_commands() {
    local MOTOR_ID=$1
    local CAN_IF=$2

    echo "Setting zero position for Robstride motor ID: $MOTOR_ID on interface: $CAN_IF"

    # Robstride protocol: Set zero position command format
    # Format: 0600FD0X#0000000000000000 (where X is motor ID)
    local MOTOR_ID_HEX
    MOTOR_ID_HEX=$(printf "%02d" "$MOTOR_ID")

    # Send set zero command
    local SET_ZERO_ID="0600FD${MOTOR_ID_HEX}"
    echo "Sending set zero command: cansend $CAN_IF ${SET_ZERO_ID}#0000000000000000"
    cansend "$CAN_IF" "${SET_ZERO_ID}#0000000000000000"
    sleep 0.1

    echo "Zero position command sent for motor ID: $MOTOR_ID"
    echo ""
}

# Main script logic
main() {
    # Check for minimum arguments
    if [ $# -lt 1 ]; then
        usage
        exit 1
    fi

    local CAN_IF=$1
    local CAN_ID=""
    local all_flag=false

    # Parse arguments
    if [ $# -ge 2 ]; then
        if [ "$2" = "--all" ]; then
            all_flag=true
        else
            CAN_ID=$2
        fi
    fi

    # Validate CAN_IF
    if [ -z "$CAN_IF" ]; then
        usage
        exit 1
    fi

    # Validate CAN_ID only if --all flag is not set
    if [ "$all_flag" = false ] && [ -z "$CAN_ID" ]; then
        echo "Error: CAN_ID is required when --all flag is not used"
        usage
        exit 1
    fi

    # Validate CAN_ID range
    if [ "$all_flag" = false ] && ([ "$CAN_ID" -lt 1 ] || [ "$CAN_ID" -gt 8 ]); then
        echo "Error: CAN_ID must be between 1 and 8 for Robstride motors"
        exit 1
    fi

    # Check if cansend command is available
    if ! command -v cansend &>/dev/null; then
        echo "Error: cansend command not found. Please install can-utils package."
        echo "  sudo apt-get install can-utils"
        exit 1
    fi

    # Check CAN interface status
    if ! check_can_interface "$CAN_IF"; then
        exit 1
    fi

    echo ""

    # Execute based on flags
    if [ "$all_flag" = true ]; then
        echo "Setting zero position for all Robstride motors (IDs 1-8)"
        echo "========================================================"
        for i in {1..8}; do
            send_robstride_zero_commands "$i" "$CAN_IF"
        done
    else
        send_robstride_zero_commands "$CAN_ID" "$CAN_IF"
    fi

    echo "=== Zero Position Setup Completed ==="
    echo "All specified Robstride motors have been set to zero position."
    echo ""
    echo "Next steps:"
    echo "1. You can now enable motors and start control operations"
    echo "2. Use './robstride-demo' to test motor functionality"
    echo "3. Use 'candump $CAN_IF' to monitor CAN traffic"
}

# Run main function with all arguments
main "$@"