# main_threaded_stm32.py

import time
import cv2

from camera import setup_camera, release_camera
from detector import ObjectDetector
from visualizer import setup_window, draw_detections, draw_fps, show_frame
from fps_counter import FPSCounter
from stm32_comm import STM32Comm


# Change these command strings to match your STM32 firmware.
CMD_FORWARD = "FORWARD"
CMD_STOP = "STOP"

# Stop if ultrasonic distance is closer than this value.
STOP_DISTANCE_CM = 50.0

# Do not send the same command too often.
COMMAND_INTERVAL_SEC = 0.2


def decide_command(distance_cm, detections):
    """
    Decide what command to send to STM32.

    Current simple logic:
    - If ultrasonic sensor says something is closer than STOP_DISTANCE_CM: STOP
    - Otherwise: FORWARD

    Later, you can add YOLO class/position logic here.
    """
    if distance_cm is not None and distance_cm < STOP_DISTANCE_CM:
        return CMD_STOP

    return CMD_FORWARD


def draw_distance(frame, distance_cm):
    if distance_cm is None:
        text = "Distance: -- cm"
    else:
        text = f"Distance: {distance_cm:.1f} cm"

    cv2.putText(
        frame,
        text,
        (20, 85),
        cv2.FONT_HERSHEY_PLAIN,
        2,
        (255, 255, 255),
        2
    )
    return frame


def main():
    cap = setup_camera()
    detector = ObjectDetector()
    fps_counter = FPSCounter()

    stm32 = STM32Comm(
        port="/dev/ttyACM0",
        baudrate=115200,
        timeout=0.01
    )
    stm32.start()

    setup_window()

    last_command = None
    last_command_time = 0

    try:
        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                print("Camera frame not received.")
                break

            # 1. YOLO detection stays in the main loop.
            detections = detector.detect(frame)

            # 2. Get latest STM32 ultrasonic value without waiting.
            distance_cm = stm32.get_latest_distance()

            # 3. Decide command using sensor data + YOLO result.
            command = decide_command(distance_cm, detections)

            # 4. Send command only when changed or after interval.
            now = time.time()
            if command != last_command and now - last_command_time >= COMMAND_INTERVAL_SEC:
                stm32.send_command(command)
                last_command = command
                last_command_time = now
                print("Sent command:", command, "distance:", distance_cm)

            # Print detection information.
            for det in detections:
                print(
                    det["class_name"],
                    det["confidence"],
                    det["box"],
                    "center:",
                    det["center"]
                )

            # Draw results.
            frame = draw_detections(frame, detections)
            frame = draw_distance(frame, distance_cm)

            # FPS.
            fps = fps_counter.calculate()
            frame = draw_fps(frame, fps)

            # Show frame.
            show_frame(frame)

            key = cv2.waitKey(1)
            if key == ord("q"):
                break

    finally:
        stm32.stop()
        release_camera(cap)


if __name__ == "__main__":
    main()
