import time

from matplotlib.widgets import Button


class TargetControl:
    def __init__(self, fig, port, axis, row_y, label_y,
                 with_stop=False):
        self.port = port
        self.axis = axis
        self.frame = None
        self.last_n = None
        self.pending = None
        self.stop_n = None
        self.live = False
        self.peers = []
        self.note = f"Start {axis} loop before selecting a target"
        self.label = fig.text(0.02, label_y, "", fontsize=9)

        if axis == "P":
            self.degrees = (4, 6, 8)
            self.chars = {4: b"4", 6: b"6", 8: b"8"}
        else:
            self.degrees = (-2, 0, 2)
            self.chars = {-2: b"a", 0: b"b", 2: b"c"}
        self.targets = tuple(d * 1000 for d in self.degrees)

        self.buttons = []
        for index, degree in enumerate(self.degrees):
            button = Button(
                fig.add_axes([0.12 + index * 0.19, row_y, 0.16, 0.04]),
                f"{axis} {degree:+d} deg" if axis == "Y"
                else f"{axis} {degree} deg")
            button.on_clicked(
                lambda event, value=degree: self.request(value))
            self.buttons.append(button)

        self.stop_button = None
        if with_stop:
            self.stop_button = Button(
                fig.add_axes([0.69, row_y, 0.16, 0.04]), "Stop X (both)")
            self.stop_button.on_clicked(lambda event: self.stop())

        self.update(time.monotonic(), False)

    def _target(self, frame):
        if self.axis == "P":
            return frame.get("pitch_target_mdeg")
        return frame.get("yaw_target_mdeg")

    def _count(self, frame):
        if self.axis == "P":
            step = frame.get("pitch_step")
            return None if step is None else step[1]
        return frame.get("yaw_target_ack")

    def _running(self):
        f = self.frame
        if f is None or f["cal"] != 1 or f["att"] != 1:
            return False
        axis = f[self.axis]
        if axis[:3] != (1, 1, 1) or axis[5] != 0:
            return False
        if self._target(f) not in self.targets:
            return False
        if self._count(f) is None:
            return False
        if self.axis == "P":
            step = f.get("pitch_step")
            return step is not None and step[0] == 0
        return True

    def _enabled(self):
        return (
            self.live and self._running()
            and self.pending is None and self.stop_n is None)

    def _write(self, command):
        if self.port.write(command) != len(command):
            raise OSError("incomplete serial write")

    def request(self, degree):
        if degree not in self.chars or not self._enabled():
            self.note = "Rejected locally: control unavailable or busy"
            return
        f = self.frame
        count = self._count(f)
        self.pending = {
            "target": degree * 1000,
            "count": (count + 1) & 0xffffffff,
            "n": f["n"], "sent": time.monotonic(),
        }
        self.note = f"Waiting for {self.axis} target {degree:+d} deg"
        try:
            self._write(self.chars[degree])
        except OSError as exc:
            self.note = f"Send uncertain; awaiting readback: {exc}"
        self.update(time.monotonic(), self.live)

    def stop(self):
        controls = [self, *self.peers]
        for control in controls:
            control.pending = None
            control.stop_n = -1 if control.last_n is None else control.last_n
            control.note = "X sent; waiting for stopped status"
        try:
            self._write(b"X")
        except OSError as exc:
            for control in controls:
                control.note = f"X send uncertain: {exc}"
        for control in controls:
            control.update(time.monotonic(), control.live)

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
                self.note = "Stopped (both axes)"
            return

        if self.pending is None:
            return
        p = self.pending
        if not self._running():
            self.pending = None
            self.note = "Control unavailable; target unconfirmed"
        elif (frame["n"] > p["n"]
              and self._count(frame) == p["count"]
              and self._target(frame) == p["target"]):
            self.pending = None
            self.note = (
                f"Applied {self.axis} target "
                f"{p['target'] / 1000:+g} deg")

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

        target = None if self.frame is None else self._target(self.frame)
        value = "NA" if target is None else f"{target / 1000:+g}"
        count = None if self.frame is None else self._count(self.frame)
        count_text = "NA" if count is None else str(count)
        prefix = "" if live else "NO LIVE DATA | "
        self.label.set_text(
            f"{prefix}{self.axis} target={value} count={count_text} "
            f"| {self.note}")
