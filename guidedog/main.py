# main.py

import cv2

from camera import setup_camera, release_camera
from detector import ObjectDetector
from visualizer import setup_window, draw_detections, draw_fps, show_frame
from fps_counter import FPSCounter




def main():
    cap = setup_camera()
    detector = ObjectDetector()
    fps_counter = FPSCounter()

    setup_window()

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            print("Camera frame not received.")
            break

        # Detect objects
        detections = detector.detect(frame)

        # Print detection information
        for det in detections:
            print(
                det["class_name"],
                det["confidence"],
                det["box"],
                "center:",
                det["center"]
            )

        # Draw results
        frame = draw_detections(frame, detections)

        # FPS
        fps = fps_counter.calculate()
        frame = draw_fps(frame, fps)

        # Show frame
        show_frame(frame)

        key = cv2.waitKey(10)
        if key == ord("q"):
            break

    release_camera(cap)


if __name__ == "__main__":
    main()