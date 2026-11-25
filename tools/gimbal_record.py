import csv
from datetime import datetime, timezone
from pathlib import Path


class CsvRecorder:
    def __init__(self, filename):
        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("x", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        axis_fields = (
            "state", "enabled", "ready", "feedback_mdeg",
            "pulse_us", "last_fault")
        columns = [
            "pc_time_utc", "pc_elapsed_s", "n", "cal", "att",
            "roll_deg", "pitch_deg", "yaw_deg"]
        columns += [
            f"{axis}_{field}"
            for axis in ("p", "y") for field in axis_fields]
        columns += [
            "filter_gain_milli",
            "filter_apply_status",
            "filter_apply_count"]
        columns.append("raw_line")
        self.writer.writerow(columns)
        self.file.flush()

    def write(self, frame, raw, elapsed):
        row = [
            datetime.now(timezone.utc).isoformat(
                timespec="milliseconds"),
            f"{elapsed:.3f}", frame["n"], frame["cal"],
            frame["att"]]
        row += list(frame["rpy"])
        row += list(frame["P"]) + list(frame["Y"])
        gain = frame.get("gain_milli")
        ks = frame.get("ks")
        row += [
            "" if gain is None else gain,
            *(ks if ks is not None else ("", ""))]
        row.append(raw.decode("ascii").strip())
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        self.file.close()
