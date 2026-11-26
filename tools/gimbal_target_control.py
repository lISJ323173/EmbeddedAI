import time

from matplotlib.widgets import Button


class TargetControl:
    def __init__(self, fig, port):
        self.port = port
        self.frame = None
        self.last_n = None
        self.pending = None
        self.stop_n = None
        self.live = False
        self.note = "Start C before selecting a target"
        self.label = fig.text(0.02, 0.68, "", fontsize=9)
        self.buttons = []
        for index, degree in enumerate((4, 6, 8)):
            button = Button(
                fig.add_axes([0.12 + index * 0.19, 0.60, 0.16, 0.045]),
                f"P {degree} deg")
            button.on_clicked(
                lambda event, value=degree: self.request(value))
            self.buttons.append(button)
        self.stop_button = Button(
            fig.add_axes([0.69, 0.60, 0.16, 0.045]), "Stop X")
        self.stop_button.on_clicked(lambda event: self.stop())
        self.update(time.monotonic(), False)

    def _running(self):
        f = self.frame
        return (
            f is not None and f["cal"] == 1 and f["att"] == 1
            and f["P"][:3] == (1, 1, 1) and f["P"][5] == 0
            and f["Y"][0] != 1 and f["Y"][1] == 0
            and f.get("pitch_target_mdeg") in (4000, 6000, 8000)
            and f.get("pitch_step") is not None
            and f["pitch_step"][0] == 0)

    def _enabled(self):
        return (
            self.live and self._running()
            and self.pending is None and self.stop_n is None)

    def _write(self, command):
        if self.port.write(command) != len(command):
            raise OSError("incomplete serial write")

    def request(self, degree):
        if degree not in (4, 6, 8) or not self._enabled():
            self.note = "Rejected locally: P control unavailable or busy"
            return
        f = self.frame
        self.pending = {
            "target": degree * 1000,
            "count": (f["pitch_step"][1] + 1) & 0xffffffff,
            "n": f["n"], "sent": time.monotonic(),
        }
        self.note = f"Waiting for target {degree} deg"
        try:
            self._write({4: b"4", 6: b"6", 8: b"8"}[degree])
        except OSError as exc:
            self.note = f"Send uncertain; awaiting readback: {exc}"
        self.update(time.monotonic(), self.live)

    def stop(self):
        self.pending = None
        self.stop_n = -1 if self.last_n is None else self.last_n
        self.note = "X sent; waiting for stopped status"
        try:
            self._write(b"X")
        except OSError as exc:
            self.note = f"X send uncertain: {exc}"
        self.update(time.monotonic(), self.live)

    def accept(self, frame):
        if self.last_n is not None and frame["n"] < self.last_n:
            self.pending = None
            self.stop_n = None
            self.note = "MCU reset; readback refreshed"
        self.frame = frame
        self.last_n = frame["n"]
        if self.stop_n is not None:
            if (frame["n"] > self.stop_n
                    and frame["P"][1] == 0 and frame["Y"][1] == 0):
                self.stop_n = None
                self.note = "Stopped"
            return
        if self.pending is None:
            return
        p = self.pending
        if not self._running():
            self.pending = None
            self.note = "Control unavailable; target unconfirmed"
        elif (frame["n"] > p["n"]
              and frame["pitch_step"][1] == p["count"]
              and frame["pitch_target_mdeg"] == p["target"]):
            self.pending = None
            self.note = f"Applied target {p['target'] / 1000:g} deg"

    def update(self, now, live):
        self.live = live
        if self.pending is not None:
            if now - self.pending["sent"] > 5.0:
                self.note = "UNCONFIRMED; use Stop X before retry"
        enabled = self._enabled()
        for button in self.buttons:
            button.set_active(enabled)
            button.ax.set_facecolor("0.85" if enabled else "0.95")
            button.label.set_color("black" if enabled else "0.6")
        target = None if self.frame is None else (
            self.frame.get("pitch_target_mdeg"))
        value = "NA" if target is None else f"{target / 1000:g}"
        prefix = "" if live else "NO LIVE DATA | "
        step = None if self.frame is None else self.frame.get("pitch_step")
        count = "NA" if step is None else str(step[1])
        self.label.set_text(
            f"{prefix}P target={value} count={count} | {self.note}")
