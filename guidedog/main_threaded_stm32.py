# main_threaded_stm32.py

import time
import cv2

from camera import setup_camera, release_camera
from detector import ObjectDetector
from visualizer import setup_window, draw_detections, draw_fps, show_frame
from fps_counter import FPSCounter
from stm32_comm import STM32Comm


# Raspberry Pi가 STM32로 보낼 명령
CMD_FORWARD = "FORWARD"
CMD_STOP = "STOP"

# 초음파 센서 거리값이 50cm보다 작으면 멈추기
STOP_DISTANCE_CM = 50.0

# 최소 0.2초 간격을 두고 명령 보내기
COMMAND_INTERVAL_SEC = 0.2

#센서값과 YOLO 탐지 결과를 이용해서 STM32에 어떤 명령을 보낼지
def decide_command(distance_cm, detections):
    """
    현재 로직
    - STOP_DISTANCE_CM 보다 가깝다: 멈춰
    - 아니면 가라
    """
    if distance_cm is not None and distance_cm < STOP_DISTANCE_CM:
        return CMD_STOP

    # if person detected in center and distance < 100:
    #     STOP
    # elif obstacle on left:
    #     TURN_RIGHT
    # elif obstacle on right:
    #     TURN_LEFT
    # else:
    #     FORWARD

    return CMD_FORWARD


#카메라 화면에 초음파 거리값을 출력
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
