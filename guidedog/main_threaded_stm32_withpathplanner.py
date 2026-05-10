# main_threaded_stm32.py

import time
import cv2

from camera import setup_camera, release_camera
from detector import ObjectDetector
from visualizer import setup_window, draw_detections, draw_fps, show_frame
from fps_counter import FPSCounter
from stm32_comm import STM32Comm
from path_planner import PathPlanner
from config import COMMAND_INTERVAL_SEC


# 카메라 화면에 초음파 거리값을 출력
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


def draw_command(frame, command, danger):
    """
    현재 이동 명령과 위험도를 화면에 출력.
    """
    cv2.putText(
        frame,
        f"CMD: {command}",
        (20, 120),
        cv2.FONT_HERSHEY_PLAIN,
        2,
        (0, 255, 255),
        2
    )

    danger_text = (
        f"L:{danger['LEFT']:.1f} "
        f"C:{danger['CENTER']:.1f} "
        f"R:{danger['RIGHT']:.1f}"
    )

    cv2.putText(
        frame,
        danger_text,
        (20, 155),
        cv2.FONT_HERSHEY_PLAIN,
        1.5,
        (255, 255, 0),
        2
    )

    return frame


def main():
    cap = setup_camera()
    detector = ObjectDetector()
    fps_counter = FPSCounter()
    path_planner = PathPlanner()

    # STM32과 Serial 통신을 시작
    stm32 = STM32Comm(
        port="/dev/ttyACM0",
        baudrate=115200,
        timeout=0.01
    )

    stm32.start()
    setup_window()

    # 명령 중복 전송 방지 변수
    last_command = None
    last_command_time = 0

    try:
        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                print("Camera frame not received.")
                break

            # 1. 현재 프레임에서 객체 탐지
            detections = detector.detect(frame)

            # 2. STM32에서 가장 최근 초음파 거리값 가져오기
            distance_cm = stm32.get_latest_distance()

            # 3. path planner 모듈로 이동 명령 결정
            command, danger = path_planner.decide_command(
                detections=detections,
                distance_cm=distance_cm
            )

            # 4. 명령이 바뀌었고, 최소 전송 간격이 지났을 때만 STM32로 전송
            now = time.time()
            if command != last_command and now - last_command_time >= COMMAND_INTERVAL_SEC:
                stm32.send_command(command)
                last_command = command
                last_command_time = now
                print("Sent command:", command, "distance:", distance_cm, "danger:", danger)

            # 디버깅 출력
            for det in detections:
                print(
                    det["class_name"],
                    det["confidence"],
                    det["box"],
                    "center:",
                    det["center"]
                )

            # 화면에 결과 그리기
            frame = draw_detections(frame, detections)
            frame = draw_distance(frame, distance_cm)
            frame = draw_command(frame, command, danger)

            # FPS 출력
            fps = fps_counter.calculate()
            frame = draw_fps(frame, fps)

            show_frame(frame)

            key = cv2.waitKey(1)
            if key == ord("q"):
                break

    finally:
        stm32.stop()
        release_camera(cap)


if __name__ == "__main__":
    main()
