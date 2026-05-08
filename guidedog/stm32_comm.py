# stm32_comm.py

import re
import time
import queue
import threading

import serial


class STM32Comm:
    """
    Background serial communication with STM32.

    - Reads ultrasonic sensor data continuously in a separate thread.
    - Stores only the latest distance value.
    - Sends commands to STM32 without blocking the YOLO loop.
    """

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=0.01):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser = None
        self.running = False
        self.thread = None

        self.lock = threading.Lock()
        self.latest_line = None
        self.latest_distance_cm = None

        self.command_queue = queue.Queue()

    def start(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )

        # Some STM32 boards reset when serial opens.
        time.sleep(2)

        self.running = True
        self.thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.thread.start()
        print("STM32 communication thread started.")

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1)

        if self.ser is not None and self.ser.is_open:
            self.ser.close()

        print("STM32 communication stopped.")

    def send_command(self, command):
        """Queue a command to send to STM32."""
        if command:
            self.command_queue.put(str(command))

    def get_latest_distance(self):
        """Return the most recent ultrasonic distance in cm."""
        with self.lock:
            return self.latest_distance_cm

    def get_latest_line(self):
        """Return the most recent raw STM32 line."""
        with self.lock:
            return self.latest_line

    def _serial_loop(self):
        while self.running:
            try:
                self._read_once()
                self._write_pending_commands()
            except serial.SerialException as e:
                print("STM32 serial error:", e)
                time.sleep(0.1)
            except Exception as e:
                print("STM32 communication error:", e)
                time.sleep(0.1)

    def _read_once(self):
        if self.ser is None:
            return

        line = self.ser.readline().decode(errors="ignore").strip()
        if not line:
            return

        distance = self._parse_distance_cm(line)

        with self.lock:
            self.latest_line = line
            if distance is not None:
                self.latest_distance_cm = distance

    def _write_pending_commands(self):
        if self.ser is None:
            return

        # Send all commands currently waiting, but do not block.
        while not self.command_queue.empty():
            command = self.command_queue.get_nowait()
            self.ser.write((command + "\n").encode())

    @staticmethod
    def _parse_distance_cm(line):
        """
        Extract the first number from STM32 text.

        Works with examples like:
        - "23.5"
        - "DIST: 23.5"
        - "distance=23 cm"
        - "Ultrasonic: 23cm"
        """
        match = re.search(r"[-+]?\d*\.?\d+", line)
        if match is None:
            return None

        try:
            return float(match.group())
        except ValueError:
            return None
