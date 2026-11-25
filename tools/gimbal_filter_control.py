import time
import serial
from matplotlib.widgets import Button


class FilterControl:
    def __init__(self, fig, port):
        self.port = port
        self.frame = None
        self.pending = None
        self.last_n = None
        self.allowed = False
        self.note = "Waiting for parameter readback"
        self.label = fig.text(0.02, 0.86, "", va="top", fontsize=9)
        self.buttons = []
        for index, (gain, token) in enumerate(
                ((500, b"1"), (2000, b"2"), (5000, b"3"))):
            axis = fig.add_axes(
                [0.10 + index * 0.28, 0.75, 0.22, 0.045])
            button = Button(axis, f"Gain {gain / 1000:g}")
            button.on_clicked(
                lambda event, g=gain, t=token: self.send(g, t))
            self.buttons.append(button)
        self._enable(False)

    def _enable(self, enabled):
        self.allowed = enabled
        for button in self.buttons:
            button.set_active(enabled)
            button.color = "0.85" if enabled else "0.95"
            button.ax.set_facecolor(button.color)
            button.label.set_color("black" if enabled else "0.6")

    def send(self, gain, token):
        if not self.allowed:
            return
        self.pending = (
            gain, self.frame["ks"][1], time.monotonic() + 5.0)
        self._enable(False)
        self.note = f"Waiting for gain {gain / 1000:g}"
        try:
            if self.port.write(token) != 1:
                raise serial.SerialTimeoutException("Incomplete write")
        except (serial.SerialException, OSError) as exc:
            self.note = f"UNCONFIRMED write: {exc}"

    def accept(self, frame):
        if self.last_n is not None and frame["n"] < self.last_n:
            self.pending = None
            self.note = "MCU reset; readback refreshed"
        self.last_n = frame["n"]
        self.frame = frame

        if self.pending is not None and frame["ks"] is not None:
            gain, base_count, _ = self.pending
            expected_count = (base_count + 1) & 0xffffffff
            status, count = frame["ks"]
            if count == expected_count:
                if status == 1 and frame["gain_milli"] == gain:
                    self.pending = None
                    self.note = f"Applied gain {gain / 1000:g}"
                else:
                    self.note = "UNCONFIRMED: unexpected result"
            elif count != base_count:
                self.note = "UNCONFIRMED: application count changed"

    def update(self, now, live):
        frame = self.frame
        supported = (
            frame is not None and frame["ks"] is not None
            and frame["gain_milli"] is not None
            and 0 <= frame["gain_milli"] <= 20000)
        ready = (
            supported and live and frame["cal"] == 1
            and frame["att"] == 1
            and frame["P"][0] != 1 and frame["Y"][0] != 1
            and frame["P"][1] == 0 and frame["Y"][1] == 0)

        if self.pending is not None and now >= self.pending[2]:
            self.note = (
                "UNCONFIRMED: no matching readback; no automatic retry")

        self._enable(bool(ready and self.pending is None))
        actual = (
            f"K={frame['gain_milli'] / 1000:g} KS={frame['ks']}"
            if supported else "K/KS unavailable")
        self.label.set_text(
            ("" if live else "STALE ") + actual + "\n" + self.note)
        self.label.set_color(
            "red" if "UNCONFIRMED" in self.note
            else "black" if ready else "darkorange")
