def parse_line(raw):
    fields = {}
    for token in raw.decode("ascii").strip().split():
        key, value = token.split("=", 1)
        if key in fields:
            raise ValueError("duplicate field")
        fields[key] = value

    def vector(key, count):
        values = tuple(int(v) for v in fields[key].split(","))
        if len(values) != count:
            raise ValueError(f"invalid {key}")
        return values

    result = {
        "n": int(fields["N"]),
        "cal": int(fields["CAL"]),
        "att": int(fields["ATT"]),
        "rpy": tuple(v / 1000.0 for v in vector("RPY", 3)),
    }
    for key in ("P", "Y"):
        values = vector(key, 6)
        state, enabled, ready, feedback, pulse, fault = values
        if state not in range(5):
            raise ValueError("invalid axis state")
        if enabled not in (0, 1) or ready not in (0, 1):
            raise ValueError("invalid axis flags")
        result[key] = values
    gain = int(fields["K"]) if "K" in fields else None
    ks = vector("KS", 2) if "KS" in fields else None

    if gain is not None and not 0 <= gain <= 0xffffffff:
        raise ValueError("invalid K")
    if ks is not None:
        if gain is None:
            raise ValueError("KS without K")
        if ks[0] not in range(5) or not 0 <= ks[1] <= 0xffffffff:
            raise ValueError("invalid KS")

    result["gain_milli"] = gain
    result["ks"] = ks

    sample_ms = int(fields["T"]) if "T" in fields else None
    if sample_ms is not None and not 0 <= sample_ms <= 0xffffffff:
        raise ValueError("invalid T")
    result["device_sample_ms"] = sample_ms

    target = int(fields["PT"]) if "PT" in fields else None
    step = vector("PS", 3) if "PS" in fields else None

    if (target is None) != (step is None):
        raise ValueError("incomplete PT/PS")
    if target is not None and not -0x80000000 <= target <= 0x7fffffff:
        raise ValueError("invalid PT")
    if step is not None:
        if step[0] not in range(5):
            raise ValueError("invalid PS state")
        if not all(0 <= value <= 0xffffffff for value in step[1:]):
            raise ValueError("invalid PS counter/time")

    result["pitch_target_mdeg"] = target
    result["pitch_step"] = step

    return result
