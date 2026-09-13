#!/usr/bin/env python3
"""face_tracker.py -- Fase 6 "camera-gerichtheid": ROS2-node die gezichten
detecteert in de live Astra-kleurenstroom en de horizontale richting
(bearing, graden t.o.v. het midden van het beeld) publiceert.

BEWUST ALLEEN WAARNEMING -- deze node stuurt zelf GEEN enkel bewegings-
commando. Het daadwerkelijk laten draaien van de robot op basis van deze
bearing is een aparte, later te bouwen en apart aan te kondigen stap (zelfde
scope-grens als Fase 5's WANDER-modus).

FaceDetector-klasse is een directe overname (zelfde MediaPipe-aanpak) van
Yahboom's bestaande yahboomcar_mediapipe/07_FaceDetection.py -- dat bestand
zelf is geen ROS2-node en opent zijn eigen camera (cv.VideoCapture(0)), hier
alleen de kale detectielogica hergebruikt tegen de echte /camera/color/
image_raw-topic (astra_camera, Fase 6 12 sep 2026: pas voor het eerst
gebouwd/aangezet, zie muto_fase6-memory -- vergde een uvc_product_id-fix,
0x0501 (launch-default) -> 0x050f (dit specifieke toestel)).
"""
import json
import math

import cv2 as cv
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge


class FaceDetector:
    """Vrijwel ongewijzigd overgenomen van Yahboom's 07_FaceDetection.py."""

    def __init__(self, min_detection_con=0.5):
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(
            min_detection_confidence=min_detection_con)

    def find_faces(self, frame):
        img_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.face_detection.process(img_rgb)
        bboxes = []
        if results.detections:
            ih, iw, _ = frame.shape
            for detection in results.detections:
                bbox_c = detection.location_data.relative_bounding_box
                bbox = (int(bbox_c.xmin * iw), int(bbox_c.ymin * ih),
                        int(bbox_c.width * iw), int(bbox_c.height * ih))
                bboxes.append((bbox, detection.score[0]))
        return bboxes


class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.bridge = CvBridge()
        self.detector = FaceDetector(min_detection_con=0.6)
        self.fx = None  # uit camera_info, voor pixel->graden

        self.pub = self.create_publisher(String, 'face_bearing', 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._info_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self._image_cb, 10)

        self._last_log_found = None
        self.get_logger().info('face_tracker gestart (alleen waarneming, geen beweging)')

    def _info_cb(self, msg: CameraInfo):
        if self.fx is None and msg.k[0] > 0:
            self.fx = msg.k[0]
            self.get_logger().info(f'camera fx={self.fx:.1f}px, HFOV~{math.degrees(2 * math.atan(msg.width / (2 * self.fx))):.1f} graden')

    def _image_cb(self, msg: Image):
        if self.fx is None:
            return  # wacht op camera_info voor we een bearing kunnen berekenen
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_w = frame.shape[1]
        bboxes = self.detector.find_faces(frame)

        if not bboxes:
            if self._last_log_found is not False:
                self.get_logger().info('geen gezicht (meer) gedetecteerd')
                self._last_log_found = False
            self.pub.publish(String(data=json.dumps({"face_found": False})))
            return

        # Grootste (dichtstbijzijnde) gezicht kiezen, niet per se het meest zekere.
        (x, y, w, h), score = max(bboxes, key=lambda b: b[0][2] * b[0][3])
        face_center_x = x + w / 2.0
        offset_px = face_center_x - img_w / 2.0
        bearing_deg = math.degrees(math.atan2(offset_px, self.fx))

        if self._last_log_found is not True:
            self.get_logger().info('gezicht gedetecteerd')
            self._last_log_found = True

        self.pub.publish(String(data=json.dumps({
            "face_found": True,
            "bearing_deg": round(bearing_deg, 1),
            "confidence": round(float(score), 2),
            "bbox_width_px": w,
        })))


def main():
    rclpy.init()
    node = FaceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
