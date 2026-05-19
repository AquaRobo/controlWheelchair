#!/usr/bin/env python3
"""
Hoverboard UART Communication Script
=====================================
Sends speed commands to the hoverboard STM32 directly via Pi GPIO UART.

Wiring (Pi GPIO → Hoverboard UART_R connector):
  GPIO 14 (TX) →  UART_R_RX  (pin 4 on UART_R connector)
  GPIO 15 (RX) →  UART_R_TX  (pin 3 on UART_R connector)
  GND          →  GND        (pin 1 on UART_R connector)
  ** Do NOT connect 5V/3.3V — hoverboard is self-powered **

Protocol (matches hoverboard-firmware-hack-FOC):
  Frame is 8 bytes packed as little-endian struct:
    uint16  START    = 0xABCD
    int16   steer    (range: -1000 to +1000)
    int16   speed    (range: -1000 to +1000)
    uint16  checksum = START ^ steer ^ speed   (XOR of the three uint16 words)

  IMPORTANT: The hoverboard requires frames continuously every ~100ms.
  If it stops receiving, it cuts power to the motors.

Usage:
  python3 hoverboard_uart.py                        # interactive mode
  python3 hoverboard_uart.py --speed 200            # straight forward
  python3 hoverboard_uart.py --speed 200 --steer 50
  python3 hoverboard_uart.py --demo
"""

import serial
import struct
import time
import argparse
import sys
import threading

# ── Protocol constants ──────────────────────────────────────────────────────
START_FRAME = 0xABCD    # uint16 start marker
BAUD_RATE   = 115200    # Must match firmware (hoverboard-firmware-hack-FOC)
SEND_RATE   = 0.1       # Send a frame every 100ms — matches ESP32 TIME_SEND
MAX_SPEED   = 1000
MIN_SPEED   = -1000


def build_frame(steer: int, speed: int) -> bytes:
    """
    Build an 8-byte command frame matching the ESP32 firmware protocol.

    Packed as little-endian struct { uint16 start; int16 steer; int16 speed; uint16 checksum }
    Checksum = START ^ steer ^ speed  (XOR of the raw 16-bit words, same as C firmware)
    """
    steer = max(MIN_SPEED, min(MAX_SPEED, steer))
    speed = max(MIN_SPEED, min(MAX_SPEED, speed))
    checksum = START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
    return struct.pack('<HhhH', START_FRAME, steer, speed, checksum)


# ── Continuous sender ───────────────────────────────────────────────────────

class HoverboardSender:
    """
    Sends the current steer/speed command at 10Hz in a background thread.
    The hoverboard will cut motors if it stops receiving frames.
    """
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.steer = 0
        self.speed = 0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set(self, steer: int, speed: int):
        with self._lock:
            self.steer = steer
            self.speed = speed

    def stop(self):
        """Send zero then stop the background thread."""
        self.set(0, 0)
        time.sleep(SEND_RATE * 2)   # Let one last zero frame go out
        self._stop_event.set()
        self._thread.join()

    def _loop(self):
        while not self._stop_event.is_set():
            with self._lock:
                frame = build_frame(self.steer, self.speed)
            try:
                self.ser.write(frame)
            except serial.SerialException:
                break
            time.sleep(SEND_RATE)


# ── High-level helpers ──────────────────────────────────────────────────────

def ramp(sender: HoverboardSender, target_speed: int, steer: int = 0,
         step: int = 20, delay: float = 0.1):
    """Gradually ramp to target speed to avoid sudden jolts."""
    current = sender.speed
    direction = 1 if target_speed > current else -1
    while current != target_speed:
        current += direction * step
        if direction == 1 and current > target_speed:
            current = target_speed
        if direction == -1 and current < target_speed:
            current = target_speed
        sender.set(steer, current)
        print(f"  steer={steer:+5d}  speed={current:+5d}  "
              f"frame={build_frame(steer, current).hex(' ').upper()}")
        time.sleep(delay)


def demo_sequence(sender: HoverboardSender):
    print("\n── Demo sequence ──────────────────────────────")

    print("\n[1] Ramp forward to 300...")
    ramp(sender, 300)
    time.sleep(2)

    print("\n[2] Ramp back to 0...")
    ramp(sender, 0)
    time.sleep(1)

    print("\n[3] Ramp reverse to -200...")
    ramp(sender, -200)
    time.sleep(2)

    print("\n[4] Ramp back to 0...")
    ramp(sender, 0)
    time.sleep(1)

    print("\n[5] Forward with right steer...")
    ramp(sender, 250, steer=200)
    time.sleep(2)

    print("\n[6] Final stop.")
    ramp(sender, 0, steer=0)
    print("\n── Demo complete ──────────────────────────────\n")


def interactive_mode(sender: HoverboardSender):
    print("\nInteractive mode — type  'steer speed'  or 'q' to quit.")
    print("Commands are sent continuously until you change them.")
    print("Example: '0 300' = straight forward,  '0 0' = stop\n")

    while True:
        try:
            line = input("steer speed > ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if line.lower() in ("q", "quit", "exit"):
            break

        parts = line.split()
        if len(parts) != 2:
            print("  Enter two integers: steer speed")
            continue
        try:
            steer, speed = int(parts[0]), int(parts[1])
        except ValueError:
            print("  Invalid numbers.")
            continue

        sender.set(steer, speed)
        frame = build_frame(steer, speed)
        print(f"  → steer={steer:+5d}  speed={speed:+5d}  "
              f"frame={frame.hex(' ').upper()}")


# ── Entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Send UART commands to hoverboard")
    parser.add_argument("--port",  default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud",  type=int, default=BAUD_RATE, help=f"Baud rate (default: {BAUD_RATE})")
    parser.add_argument("--speed", type=int, default=None, help="Speed value and hold until Ctrl-C")
    parser.add_argument("--steer", type=int, default=0,    help="Steer value to use with --speed (default: 0)")
    parser.add_argument("--demo",  action="store_true",    help="Run built-in demo sequence")
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(
            port=args.port, baudrate=args.baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=1
        )
    except serial.SerialException as e:
        print(f"ERROR: Could not open port: {e}")
        print("Tip: ls /dev/ttyAMA*  or  ls /dev/serial*")
        sys.exit(1)

    print(f"Connected on {ser.name}  (sending every {int(SEND_RATE*1000)}ms)\n")

    sender = HoverboardSender(ser)

    try:
        if args.speed is not None:
            print(f"Holding steer={args.steer}, speed={args.speed} — Ctrl-C to stop")
            ramp(sender, args.speed, steer=args.steer)
            while True:
                time.sleep(1)
        elif args.demo:
            demo_sequence(sender)
        else:
            interactive_mode(sender)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Stopping...")
        sender.stop()
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
