from collections import deque


class FastView:
    def __init__(self):
        self.history = deque(maxlen=1200)
        self.seq = self.tick = None
        self.elapsed = 0.0
        self.received = self.missing = self.resets = 0
        self.last_progress = None
        self.dt_min = None
        self.dt_max = 0

    def accept(self, frame, now):
        tick = frame["device_sample_ms"]
        seq = frame["fast_seq"]
        dt = 0 if self.tick is None else (
            (tick - self.tick) & 0xffffffff)
        step = 1 if self.seq is None else (
            (seq - self.seq) & 0xffff)

        if self.tick is not None and dt >= 0x80000000:
            self.history.clear()
            self.elapsed = 0.0
            self.resets += 1
            self.dt_min = None
            self.dt_max = 0
            dt, step = 0, 1
        elif self.tick is not None and dt == 0:
            return

        self.elapsed += dt / 1000.0
        if dt:
            self.dt_min = dt if self.dt_min is None else (
                min(self.dt_min, dt))
            self.dt_max = max(self.dt_max, dt)
        if step != 1 or dt > 150:
            self.missing += max(0, step - 1)
            self.history.append((
                self.elapsed, float("nan"),
                float("nan"), float("nan")))

        self.seq, self.tick = seq, tick
        self.received += 1
        self.last_progress = now
        angles = frame["rpy"] if frame["att_valid"] else (
            float("nan"),) * 3
        self.history.append((self.elapsed, *angles))
