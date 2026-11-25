import csv
from datetime import datetime, timezone


class FastCsvRecorder:
    def __init__(self, path):
        self.file = open(
            path, "x", newline="", encoding="utf-8")
        try:
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                "pc_time_utc", "pc_elapsed_s",
                "device_sample_ms", "fast_seq",
                "cal", "att_valid",
                "roll_deg", "pitch_deg", "yaw_deg",
            ])
            self.file.flush()
        except Exception:
            self.file.close()
            raise

    def write(self, frame, elapsed):
        stamp = datetime.now(timezone.utc).isoformat(
            timespec="milliseconds").replace("+00:00", "Z")
        angles = frame["rpy"] if frame["att_valid"] else (
            "", "", "")
        self.writer.writerow([
            stamp, f"{elapsed:.6f}",
            frame["device_sample_ms"], frame["fast_seq"],
            frame["cal"], int(frame["att_valid"]),
            *angles,
        ])
        self.file.flush()

    def close(self):
        self.file.close()
