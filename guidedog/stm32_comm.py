# stm32_comm.py

import re
import time
import queue
import threading

import serial


class STM32Comm:
    """
    Background serial communication with STM32.

    - STM32에서 초음파 거리값 계속 읽기
    - 가장 최근 거리값만 저장하기
    - YOLO loop 막히는 일 없이 STM32로 명령 보내기
    """

    #STM32가 연결된 포트, 통신 속도, 읽기 대기 시간
    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=0.01):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser = None
        self.running = False
        self.thread = None

        #STM32 통신 스레드와 YOLO 메인 루프가 동시에 같은 값을 읽거나 쓰기 lock로 방지
        self.lock = threading.Lock()
        self.latest_line = None
        self.latest_distance_cm = None
        #STM32로 보낼 명령을 저장하는 큐
        self.command_queue = queue.Queue()

    #STM32 통신 시작
    def start(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )

        # Some STM32 boards reset when serial opens.
        time.sleep(2)

        #별도 스레드 시작   
        self.running = True
        self.thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.thread.start()
        print("STM32 communication thread started.")

    #프로그램이 종료될 때 STM32 통신도 종료
    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1)

        if self.ser is not None and self.ser.is_open:
            self.ser.close()

        print("STM32 communication stopped.")


    #STM32로 보낼 명령 큐에 넣기
    def send_command(self, command):
        """Queue a command to send to STM32."""
        if command:
            self.command_queue.put(str(command))

    #가장 최근에 STM32에서 받은 초음파 거리값 가져오기
    def get_latest_distance(self):
        """Return the most recent ultrasonic distance in cm."""
        with self.lock:
            return self.latest_distance_cm

    def get_latest_line(self):
        """Return the most recent raw STM32 line."""
        with self.lock:
            return self.latest_line

    """
    def _serial_loop(self):
    백그라운드 스레드에서 계속 실행
    
    1. self._read_once()
    -STM32에서 온 데이터를 한 줄 읽기

    2. self._write_pending_commands()
    -STM32로 보낼 명령이 있으면 보냄

    """

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

        #STM32에서 한 줄을 읽기
        line = self.ser.readline().decode(errors="ignore").strip()
        if not line:
            return

        #문자열에서 숫자만 추출
        distance = self._parse_distance_cm(line)

        #읽은 원본 문자열과 거리값을 저장
        with self.lock:
            self.latest_line = line
            if distance is not None:
                self.latest_distance_cm = distance

    def _write_pending_commands(self):
        if self.ser is None:
            return

        # 큐에 쌓인 명령을 STM32로 보내기
        while not self.command_queue.empty():
            command = self.command_queue.get_nowait()
            self.ser.write((command + "\n").encode())

    @staticmethod
    def _parse_distance_cm(line):
        """
        문자열에서 첫 번째 숫자 찾기
        예:
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
