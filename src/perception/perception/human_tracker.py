import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesis
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
import math
import numpy as np
from scipy.optimize import linear_sum_assignment


class TrackState:
    """ByteTrack track state machine."""
    TENTATIVE = 1
    CONFIRMED = 2
    LOST = 3


class KalmanBoxTracker:
    """Simple Kalman filter for 2D bounding box tracking (constant velocity model).

    State: [cx, cy, s, r, vx, vy, vs]
      - cx, cy: center x, y (pixels)
      - s: area (width * height)
      - r: aspect ratio (width / height) — assumed constant
      - vx, vy, vs: velocities
    """

    def __init__(self, bbox: tuple[float, float, float, float], track_id: int):
        # bbox: (x1, y1, x2, y2) in pixel coords
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w / 2.0
        cy = y1 + h / 2.0
        s = w * h  # area
        r = w / h if h > 0 else 1.0  # aspect ratio

        self.track_id = track_id
        self.hits = 1
        self.missed = 0
        self.state = 'tentative'  # tentative -> confirmed -> lost

        # State: [cx, cy, s, r, vx, vy, vs]
        self.x = np.array([cx, cy, s, r, 0.0, 0.0, 0.0], dtype=np.float64)
        # State covariance
        self.P = np.eye(7) * 10.0
        # State transition matrix (constant velocity)
        self.F = np.eye(7)
        # Measurement matrix
        self.H = np.eye(4, 7)  # measure [cx, cy, s, r]
        # Measurement noise
        self.R = np.eye(4) * 1.0
        # Process noise
        self.Q = np.eye(7) * 0.01
        # Motion noise
        self.F[0, 4] = 1.0  # cx += vx
        self.F[1, 5] = 1.0  # cy += vy
        self.F[2, 6] = 1.0  # s += vs

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, bbox):
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w / 2.0
        cy = y1 + h / 2.0
        s = w * h
        r = w / h if h > 0 else 1.0
        z = np.array([cx, cy, s, r])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(7) - K @ self.H) @ self.P
        self.hits += 1
        self.missed = 0

    def get_bbox(self):
        cx, cy, s, r = self.x[0], self.x[1], self.x[2], self.x[3]
        w = math.sqrt(s * r)
        h = s / w if w > 0 else 0
        x1 = cx - w / 2.0
        y1 = cy - h / 2.0
        x2 = cx + w / 2.0
        y2 = cy + h / 2.0
        return (x1, y1, x2, y2)

    def get_center(self):
        return (self.x[0], self.x[1])


def iou(bbox1, bbox2):
    """Compute IOU between two bounding boxes (x1, y1, x2, y2)."""
    x1 = max(bbox1[0], bbox2[0])
    y1 = max(bbox1[1], bbox2[1])
    x2 = min(bbox1[2], bbox2[2])
    y2 = min(bbox1[3], bbox2[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
    area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0.0


def iou_distance(track_bbox, det_bbox):
    """Convert IOU to distance (1 - IOU) for cost matrix."""
    return 1.0 - iou(track_bbox, det_bbox)


class ByteTrack:
    """Simple IOU-based ByteTrack implementation.

    Uses Kalman filter for motion prediction and IOU for matching.
    Follows ByteTrack's two-stage matching: high-score detections first,
    then low-score detections matched to unmatched tracks.
    """

    def __init__(self, track_thresh: float = 0.5, match_thresh: float = 0.8,
                 track_buffer: int = 30, frame_rate: int = 30):
        self.track_thresh = track_thresh
        self.match_thresh = match_thresh
        self.track_buffer = track_buffer
        self.frame_rate = frame_rate
        self.tracks: list[KalmanBoxTracker] = []
        self.next_id = 1

    def update(self, detections: list[dict]) -> list[dict]:
        """Update tracks with new detections.

        Each detection dict: {bbox: (x1,y1,x2,y2), score: float, class_id: str}
        Returns list of active track dicts: {track_id: int, bbox: ..., score: ..., class_id: str}
        """
        # Predict all tracks
        for track in self.tracks:
            track.predict()

        # Split detections by confidence
        high_dets = [d for d in detections if d['score'] >= self.track_thresh]
        low_dets = [d for d in detections if d['score'] < self.track_thresh]

        # First match: high-score detections with existing tracks
        matched_tracks, unmatched_dets, unmatched_tracks = self._match(high_dets, self.tracks)

        # Second match: unmatched tracks with low-score detections
        unmatched_tracks2 = []
        if unmatched_tracks:
            unmatched_track_objects = [self.tracks[i] for i in unmatched_tracks]
            matched_low, _, unmatched_tracks2 = self._match(low_dets, unmatched_track_objects)
            for t, det_idx in matched_low:
                actual_idx = unmatched_tracks[t]
                self.tracks[actual_idx].update(low_dets[det_idx]['bbox'])
                self.tracks[actual_idx].class_id = low_dets[det_idx]['class_id']
                self.tracks[actual_idx].score = low_dets[det_idx]['score']

        # Update matched tracks
        for track_idx, det_idx in matched_tracks:
            self.tracks[track_idx].update(high_dets[det_idx]['bbox'])
            self.tracks[track_idx].class_id = high_dets[det_idx]['class_id']
            self.tracks[track_idx].score = high_dets[det_idx]['score']

        # Mark unmatched confirmed tracks as missed
        for t in unmatched_tracks2:
            self.tracks[unmatched_tracks[t]].missed += 1

        # Create new tracks for unmatched high-score detections
        for det_idx in unmatched_dets:
            det = high_dets[det_idx]
            tracker = KalmanBoxTracker(det['bbox'], self.next_id)
            tracker.class_id = det['class_id']
            tracker.score = det['score']
            self.tracks.append(tracker)
            self.next_id += 1

        # Remove lost tracks
        self.tracks = [t for t in self.tracks if t.missed < self.track_buffer]

        # Build output
        output = []
        for track in self.tracks:
            if track.hits >= 2:  # Only output tracks with at least 2 hits
                bbox = track.get_bbox()
                output.append({
                    'track_id': track.track_id,
                    'bbox': bbox,
                    'score': track.score,
                    'class_id': track.class_id,
                })
        return output

    def _match(self, detections, tracks):
        """Match detections to tracks using IOU + Hungarian algorithm."""
        if not detections or not tracks:
            return [], list(range(len(detections))), list(range(len(tracks)))

        # Build cost matrix (IOU distance)
        cost_matrix = np.zeros((len(tracks), len(detections)), dtype=np.float64)
        for t, track in enumerate(tracks):
            track_bbox = track.get_bbox()
            for d, det in enumerate(detections):
                cost_matrix[t, d] = iou_distance(track_bbox, det['bbox'])

        # Hungarian algorithm
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        matched = []
        unmatched_dets = list(range(len(detections)))
        unmatched_tracks = list(range(len(tracks)))

        for t, d in zip(row_ind, col_ind):
            if cost_matrix[t, d] < self.match_thresh:
                matched.append((t, d))
                if d in unmatched_dets:
                    unmatched_dets.remove(d)
                if t in unmatched_tracks:
                    unmatched_tracks.remove(t)

        return matched, unmatched_dets, unmatched_tracks


class HumanTracker(Node):
    """Subscribes to /yolo/detections, runs ByteTrack, publishes tracked humans + objects."""

    def __init__(self):
        super().__init__('human_tracker_node')
        self.get_logger().info('HumanTracker node starting')

        # QoS: sensor data — best effort, volatile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.sub = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.detection_callback, qos)
        self.tracked_humans_pub = self.create_publisher(PoseArray, '/tracked_humans', 10)
        self.classified_objects_pub = self.create_publisher(
            Detection2DArray, '/classified_objects', 10)

        self.tracker = ByteTrack(track_thresh=0.5, match_thresh=0.8, track_buffer=30)
        self.get_logger().info('HumanTracker node ready — waiting for /yolo/detections')

    def detection_callback(self, msg: Detection2DArray):
        """Process yolo detections: track people, classify all objects."""
        detections = []
        for det in msg.detections:
            bbox = det.bbox
            # Convert center+size to x1,y1,x2,y2
            x1 = bbox.center.x - bbox.size_x / 2.0
            y1 = bbox.center.y - bbox.size_y / 2.0
            x2 = bbox.center.x + bbox.size_x / 2.0
            y2 = bbox.center.y + bbox.size_y / 2.0

            # Get best hypothesis
            best_hypothesis = None
            best_score = 0.0
            for hypothesis in det.results:
                if hypothesis.score > best_score:
                    best_score = hypothesis.score
                    best_hypothesis = hypothesis

            if best_hypothesis is None:
                continue

            class_id = best_hypothesis.id
            detections.append({
                'bbox': (x1, y1, x2, y2),
                'score': best_score,
                'class_id': class_id,
            })

        # Run tracker
        tracked = self.tracker.update(detections)
        self.get_logger().debug(f'Tracked {len(tracked)} objects from {len(detections)} detections')

        # Publish /tracked_humans (PoseArray) — only persons
        humans = [t for t in tracked if t['class_id'] == '0' or t['class_id'] == 'person']
        humans_msg = PoseArray()
        humans_msg.header = msg.header
        humans_msg.header.frame_id = 'camera_link'
        for h in humans:
            cx, cy = h['bbox'][0] + (h['bbox'][2] - h['bbox'][0]) / 2.0, \
                     h['bbox'][1] + (h['bbox'][3] - h['bbox'][1]) / 2.0
            pose = Pose()
            pose.position.x = float(cx)
            pose.position.y = float(cy)
            # Use track_id as z position to pass through (for visualization)
            pose.position.z = float(h['track_id'])
            humans_msg.poses.append(pose)
        self.tracked_humans_pub.publish(humans_msg)

        # Publish /classified_objects (Detection2DArray) — all objects with labels
        classified = Detection2DArray()
        classified.header = msg.header
        for t in tracked:
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.x = t['bbox'][0] + (t['bbox'][2] - t['bbox'][0]) / 2.0
            detection.bbox.center.y = t['bbox'][1] + (t['bbox'][3] - t['bbox'][1]) / 2.0
            detection.bbox.size_x = t['bbox'][2] - t['bbox'][0]
            detection.bbox.size_y = t['bbox'][3] - t['bbox'][1]
            hypothesis = ObjectHypothesis()
            hypothesis.id = t['class_id']
            hypothesis.score = t['score']
            detection.results.append(hypothesis)
            classified.detections.append(detection)
        self.classified_objects_pub.publish(classified)


def main(args=None):
    rclpy.init(args=args)
    node = HumanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()