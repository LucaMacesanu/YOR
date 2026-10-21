#!/bin/bash

# echo "Enter the name of the can interface (can0, e.g.):"
# read interface

interface=can0

# Check if the interface exists
if ! ip link show "$interface" &> /dev/null; then
    echo "Interface $interface does not exist."
    echo "If you have a physical CAN adapter, make sure it's connected."
    echo "If you want to create a virtual CAN interface for testing, use:"
    echo "sudo ip link add dev $interface type vcan"
    exit 1
fi

# Try to set the interface type and bitrate
if sudo ip link set $interface type can bitrate 1000000; then
    echo "Set $interface type to CAN with 1Mbps bitrate."
else
    echo "Failed to set $interface type and bitrate."
    echo "If this is a virtual CAN interface, you can skip this step."
fi

# Bring up the interface
if sudo ip link set $interface up; then
    echo "Brought up $interface."
else
    echo "Failed to bring up $interface."
    exit 1
fi

# Set txqueuelen
if sudo ip link set $interface txqueuelen 1000; then
    echo "Set txqueuelen to 1000."
else
    echo "Failed to set txqueuelen."
    exit 1
fi

echo "CAN interface $interface setup complete."

sleep 0.1

# can_left / can_right are expected to already be named by
# /etc/udev/rules.d/80-can-yor.rules (pinned by USB serial, since
# kernel-assigned can0/can1/can2 index order is not guaranteed stable
# across boots when multiple identical USB-CAN adapters share a hub).
#
# Both arms are installed (2026-10-19). Set REQUIRE_LEFT_ARM=0 to run
# without the left arm's adapter.
REQUIRE_LEFT_ARM=${REQUIRE_LEFT_ARM:-1}

setup_arm_can() {
    local iface=$1 required=$2
    if ! ip link show "$iface" &> /dev/null; then
        if [ "$required" = 1 ]; then
            echo "ERROR: $iface does not exist."
            echo "Expected it to be renamed by udev rule 80-can-yor.rules based on USB serial."
            echo "Check 'dmesg | grep can' and 'ip -br link show' to see what interfaces are present,"
            echo "and confirm the udev rule's ATTRS{serial} values match the connected adapters."
            exit 1
        fi
        echo "$iface not present (optional), skipping."
        return
    fi
    sudo ip link set "$iface" down
    if ! sudo ip link set "$iface" type can bitrate 1000000; then
        echo "Failed to set $iface type and bitrate."
        exit 1
    fi
    if ! sudo ip link set "$iface" up; then
        echo "Failed to bring up $iface."
        exit 1
    fi
    sleep 0.1
}

# Check a Nero arm is actually on the bus. A bus ACK alone is not enough
# (other devices can ACK), so ask the arm to start feedback streaming the same
# way nerolib's set_normal_mode() does (0x470 + 0x151 move_mode=0xFF,
# enable_can_push=1; motors are NOT enabled) and look for joint-state frames
# 0x251-0x257.
check_arm_on_bus() {
    local iface=$1
    command -v candump &> /dev/null || { echo "can-utils not found; skipping $iface arm check."; return 0; }
    local log
    log=$(mktemp)
    timeout 1.5 candump "$iface" > "$log" 2>/dev/null &
    sleep 0.2
    cansend "$iface" 470#0000000000000000
    sleep 0.05
    cansend "$iface" 151#01FF320000000100
    wait
    local n
    n=$(awk '$2 ~ /^25[1-7]$/' "$log" | wc -l)
    rm -f "$log"
    if [ "$n" -gt 0 ]; then
        echo "$iface: Nero arm detected ($n joint-state frames)."
        return 0
    fi
    echo "WARNING: $iface: no Nero arm feedback. Is the arm powered and cabled to this adapter?"
    return 1
}

setup_arm_can can_right 1
setup_arm_can can_left "$REQUIRE_LEFT_ARM"

check_arm_on_bus can_right
if ip link show can_left &> /dev/null; then
    check_arm_on_bus can_left
fi


#python3 home_gripper.py
sleep 0.1

#python3 piper_reset.py
