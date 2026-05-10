
from config import (
    FRAME_WIDTH,
    STOP_DISTANCE_CM,
    AVOID_DISTANCE_CM,
    CENTER_DANGER_WEIGHT,
    BOX_AREA_SCALE,
)


class PathPlanner:
    """
    YOLO detection + ultrasonic distance를 이용해서
    로봇이 어떤 방향으로 움직일지 결정하는 모듈.

    입력:
        detections: detector.py에서 나온 detection list
        distance_cm: STM32 초음파 거리값

    출력:
        command: "FORWARD", "STOP", "TURN_LEFT", "TURN_RIGHT",
                 "SLIGHT_LEFT", "SLIGHT_RIGHT"
        danger: 왼쪽/중앙/오른쪽 위험도 dictionary
    """

    def __init__(self):
        self.image_width = FRAME_WIDTH
        self.stop_distance_cm = STOP_DISTANCE_CM
        self.avoid_distance_cm = AVOID_DISTANCE_CM
        self.center_danger_weight = CENTER_DANGER_WEIGHT
        self.box_area_scale = BOX_AREA_SCALE

    def get_zone(self, center_x):
        """
        object center_x 좌표를 이용해서 LEFT / CENTER / RIGHT 구역 판단.
        """
        left_boundary = self.image_width / 3
        right_boundary = self.image_width * 2 / 3

        if center_x < left_boundary:
            return "LEFT"
        elif center_x < right_boundary:
            return "CENTER"
        else:
            return "RIGHT"

    def get_box_area(self, box):
        """
        box = [x1, y1, x2, y2]
        박스가 클수록 가까운 물체일 가능성이 높으므로 위험도에 반영.
        """
        x1, y1, x2, y2 = box
        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        return width * height

    def calculate_danger(self, detections):
        """
        YOLO 탐지 결과만 이용해서 LEFT / CENTER / RIGHT 위험도 계산.
        """
        danger = {
            "LEFT": 0.0,
            "CENTER": 0.0,
            "RIGHT": 0.0,
        }

        for det in detections:
            box = det.get("box")
            center = det.get("center")

            if box is None or center is None:
                continue

            center_x = center[0]
            zone = self.get_zone(center_x)

            box_area = self.get_box_area(box)
            danger_score = box_area / self.box_area_scale

            # 중앙 물체는 로봇 진행 방향과 겹치므로 더 위험하게 처리
            if zone == "CENTER":
                danger["CENTER"] += danger_score * self.center_danger_weight
            else:
                danger[zone] += danger_score

        return danger

    def decide_command(self, detections, distance_cm):
        """
        YOLO 위험도 + 초음파 거리값으로 최종 command 결정.
        """
        danger = self.calculate_danger(detections)

        # 1. 초음파 기준 너무 가까우면 무조건 정지
        if distance_cm is not None and distance_cm < self.stop_distance_cm:
            return "STOP", danger

        # 2. 초음파가 회피 거리 안에 들어오면 중앙 위험도 증가
        # 초음파 센서가 전방을 보고 있다고 가정하기 때문
        if distance_cm is not None and distance_cm < self.avoid_distance_cm:
            danger["CENTER"] += 10.0

        # 3. 중앙이 막혔으면 왼쪽/오른쪽 중 더 안전한 쪽으로 회전
        if danger["CENTER"] > 0:
            if danger["LEFT"] < danger["RIGHT"]:
                return "TURN_LEFT", danger
            elif danger["RIGHT"] < danger["LEFT"]:
                return "TURN_RIGHT", danger
            else:
                # 양쪽 위험도가 같으면 기본적으로 오른쪽으로 회피
                return "TURN_RIGHT", danger

        # 4. 왼쪽에 장애물이 있으면 오른쪽으로 살짝 이동
        if danger["LEFT"] > danger["RIGHT"]:
            return "SLIGHT_RIGHT", danger

        # 5. 오른쪽에 장애물이 있으면 왼쪽으로 살짝 이동
        if danger["RIGHT"] > danger["LEFT"]:
            return "SLIGHT_LEFT", danger

        # 6. 위험이 없으면 직진
        return "FORWARD", danger
