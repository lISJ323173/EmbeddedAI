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

    return result
