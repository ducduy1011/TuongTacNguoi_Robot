"""May trang thai don gian; dung odometry de do quang duong/goc quay."""
import math


class Behavior:
    CHOICES = {'Wave': 'WAVE', 'Pointing_Up': 'BACK_FORTH', 'Victory': 'CIRCLE'}
    VALID = set(CHOICES) | {'Thumb_Up', 'Open_Palm', 'Closed_Fist', 'No_Hand', 'Unknown'}

    def __init__(self):
        self.selected = None
        self.active = None
        self.reason = 'WAITING'
        self.segments = []
        self.index = 0
        self.progress = 0.0
        self.pose = None
        self.start_pose = None
        self.return_timeout = 0.0
        self.last_odom = -math.inf
        self.last_signal = -math.inf
        self.last_hand = -math.inf
        self.last_gesture = 'Unknown'
        self.thumb_used = False
        self.not_thumb_since = None
        self.segment_started = 0.0

    def stop(self, reason):
        self.selected = None
        self.active = None
        self.segments = []
        self.start_pose = None
        self.reason = reason

    def odometry(self, x, y, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        if self.pose is not None and self.active and self.active != 'RETURN':
            px, py, pa = self.pose
            v, w, _ = self.segments[self.index]
            angle = math.atan2(math.sin(yaw - pa), math.cos(yaw - pa))
            if w:
                self.progress += angle * (1 if w > 0 else -1)
            else:
                # Do tien do co dau; rung tai cho khong duoc tinh la di duoc.
                self.progress += ((x - px) * math.cos(pa) + (y - py) * math.sin(pa)) * (1 if v > 0 else -1)
        self.pose = (x, y, yaw)
        self.last_odom = now

    def gesture(self, label, now):
        self.last_signal = now
        if label not in self.VALID:
            label = 'Unknown'
        if label not in ('No_Hand', 'Unknown'):
            self.last_hand = now
        if label != 'Thumb_Up':
            if self.not_thumb_since is None:
                self.not_thumb_since = now
            if now - self.not_thumb_since >= 0.3:
                self.thumb_used = False
        else:
            self.not_thumb_since = None
        previous = self.last_gesture
        self.last_gesture = label
        if label == 'Open_Palm':
            self.stop('STOP_OPEN_PALM')
        elif label == 'Closed_Fist':
            if self.active == 'RETURN' or self.reason == 'CANCELLED_RETURNED':
                return  # Giu nam tay khong khoi dong lai qua trinh ve.
            if self.active and self.start_pose and self.pose and now - self.last_odom < 0.5:
                self.active = 'RETURN'
                self.selected = None
                self.segments = []
                self.segment_started = now
                distance = math.hypot(self.start_pose[0] - self.pose[0],
                                      self.start_pose[1] - self.pose[1])
                self.return_timeout = distance / 0.12 * 4 + 30
                self.reason = 'RETURNING TO START'
            else:
                self.stop('CANCELLED')
        elif label in self.CHOICES and not self.active:
            self.selected = self.CHOICES[label]
            self.reason = 'SELECTED ' + self.selected + ' -> Thumb_Up to confirm'
        elif label == 'Thumb_Up' and previous != label and not self.thumb_used:
            self.thumb_used = True
            if self.selected and not self.active and now - self.last_odom < 0.5:
                self.active = self.selected
                self.start_pose = self.pose
                self.selected = None
                if self.active == 'BACK_FORTH':
                    self.segments = [(v, 0.0, 0.4) for _ in range(4) for v in (0.12, -0.12)]
                elif self.active == 'CIRCLE':
                    self.segments = [(0.12, 0.30, 2 * math.pi)]
                else:
                    self.segments = [(0.0, w, a) for w, a in ((0.5, 0.4), (-0.5, 0.8), (0.5, 0.4))]
                self.index = 0
                self.progress = 0.0
                self.segment_started = now
                self.reason = 'RUNNING ' + self.active

    def tick(self, now):
        # Dong ho thuc: van dung du Gazebo bi Pause hoac webcam bi treo.
        # Dang dung cho xac nhan: cho phep doi dang tay trong 3 giay.
        # Xac nhan mot lan: No_Hand/Unknown van la heartbeat cua camera.
        # Chi mat message hoac odometry moi dung hanh vi dang chay.
        if now - self.last_signal > 0.7:
            self.stop('SIGNAL_LOST')
        elif now - self.last_odom > 0.7:
            self.stop('ODOM_LOST')
        elif self.selected and now - self.last_hand > 3.0:
            self.stop('SIGNAL_LOST')
        if not self.active:
            return 0.0, 0.0
        if self.active == 'RETURN':
            return self.return_to_start(now)
        v, w, target = self.segments[self.index]
        if now - self.segment_started > target / abs(w or v) * 3 + 3:
            self.stop('MOTION_TIMEOUT')
            return 0.0, 0.0
        if self.progress >= target:
            self.index += 1
            self.progress = 0.0
            self.segment_started = now
            if self.index == len(self.segments):
                self.stop('COMPLETED')
            else:
                self.reason = f'RUNNING {self.active} {self.index + 1}/{len(self.segments)}'
            return 0.0, 0.0
        return v, w

    def return_to_start(self, now):
        """Ve diem bat dau bang odometry: quay ve dich, di den dich, chinh huong."""
        if now - self.segment_started > self.return_timeout:
            self.stop('RETURN_TIMEOUT')
            return 0.0, 0.0
        x, y, yaw = self.pose
        sx, sy, start_yaw = self.start_pose
        dx, dy = sx - x, sy - y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx) if distance > 0.02 else start_yaw
        error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        angular = max(-0.5, min(0.5, 1.5 * error))
        if distance > 0.02:
            if abs(error) > 0.10:
                return 0.0, angular
            return min(0.12, max(0.03, distance)), angular
        if abs(error) > 0.04:
            return 0.0, angular
        self.stop('CANCELLED_RETURNED')
        return 0.0, 0.0

    def status(self):
        if self.active == 'RETURN':
            return self.reason
        if self.active:
            return f'{self.reason} | step {self.index + 1}/{len(self.segments)}'
        return self.reason
