"""Loc nhieu theo thoi gian; Wave la chuyen dong cua cac diem AI."""
import math


def wave_features(points, aspect):
    """Theo doi ca dau ngon de nhan duoc vay bang co tay.

    aspect = chieu cao / chieu rong, de do khoang cach dung ti le anh.
    Chi bo sung nhan ban tay xoe khi it nhat 3/4 ngon dai duoi.
    """
    def distance(a, b):
        return math.hypot(a.x - b.x, (a.y - b.y) * aspect)

    extended = sum(
        distance(points[tip], points[0]) > 1.12 * distance(points[pip], points[0])
        and distance(points[tip], points[mcp]) > 1.3 * distance(points[pip], points[mcp])
        for mcp, pip, tip in ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)))
    palm = sum(points[i].x for i in (0, 5, 9, 13, 17)) / 5
    tips = sum(points[i].x for i in (8, 12, 16, 20)) / 4
    return 0.4 * palm + 0.6 * tips, distance(points[5], points[17]), extended >= 3


class GestureFilter:
    def __init__(self, hold=0.3):
        self.hold = hold
        self.candidate = 'Unknown'
        self.since = 0.0
        self.reset_wave()

    def reset_wave(self):
        self.extreme = None
        self.direction = 0
        self.turns = 0
        self.wave_start = 0.0
        self.wave = False
        self.last_open = None
        self.threshold = None

    def update(self, label, x, now, palm_width=None, palm_open=False):
        # Khong ghi de cac lenh xac nhan, chon khac hay huy.
        if label == 'Unknown' and palm_open:
            label = 'Open_Palm'
        is_open = label == 'Open_Palm' and x is not None
        # AI co the mat nhan vai khung khi tay di nhanh. Chi giu bo dem,
        # van phat Unknown/No_Hand trong khoang mat nhan (watchdog van hoat dong).
        gap = self.last_open is not None and now - self.last_open > 0.35
        if gap or label not in ('Open_Palm', 'Unknown', 'No_Hand'):
            self.reset_wave()
        if is_open:
            if self.extreme is None or (now - self.wave_start > 3.0 and not self.wave):
                self.reset_wave()
                self.extreme = x
                self.wave_start = now
                self.threshold = max(0.025, min(0.07, (palm_width or 0.15) * 0.35))
            self.last_open = now
            delta = x - self.extreme
            # Nguong theo kich thuoc tay; co san 2,5% anh de loai rung nho.
            if self.direction == 0 and abs(delta) >= self.threshold:
                self.direction = 1 if delta > 0 else -1
                self.extreme = x
            elif self.direction * delta > 0:
                self.extreme = x
            elif self.direction * delta <= -self.threshold:
                self.turns += 1
                self.direction *= -1
                self.extreme = x
                if self.turns >= 2:
                    if now - self.wave_start >= 0.4:
                        self.wave = True
                    else:
                        # Rung qua nhanh: bo chuoi nay, khong cho dung yen
                        # them vai khung roi tu bien thanh lenh Wave.
                        self.turns = 0
                        self.direction = 0
                        self.wave_start = now
            if self.wave:
                label = 'Wave'
        # Dung/huy/mat tay duoc phat ngay; khong giu lai lenh cu.
        immediate = label in ('Open_Palm', 'Closed_Fist', 'No_Hand', 'Unknown', 'Wave')
        if label != self.candidate:
            self.candidate = label
            self.since = now
        return label if immediate or now - self.since >= self.hold else 'Unknown'
