#!/usr/bin/env python3
# Independent checks for discoverCompose R2. Reads the native program's output (ROW / SINK / PKT lines).
#   check_r2.py payloads  out.txt            SINK lines vs an independent formatter
#   check_r2.py packets   out.txt            every PKT PUBLISH recomputed by paho-mqtt's encoder (needs paho)
#   check_r2.py capture   out.txt sub.txt    mosquitto_sub capture ("topic payload" lines) vs the same expectation
# The expectation comes from the simulated devices' register values, formatted with Python's json module; only the
# row numbering is taken from the ROW lines (the table under test).
import json
import sys

# physical truth of mockTwi.h: (driver, channel of the bus it sits on, -1 = root) -> {capability: (raw, decimals)}
TRUTH = {
    ("SensorB", -1): {"temperature": (0x00BB, 1), "humidity": (64, 0)},   # regs {0xB2, 0x00, 0xBB, 64}: 18.7 C, 64 %
    ("SensorA", 0):  {"temperature": (0x00D7, 1)},                        # regs {0xA1, 0x00, 0xD7, 0}: 21.5 C
    ("SensorA", 1):  {"temperature": (0x00FD, 1)},                        # regs {0xA1, 0x00, 0xFD, 0}: 25.3 C
}
CAP_ORDER = ["temperature", "humidity"]     # a driver emits in this order


def parse_rows(lines):
    rows = {}
    for ln in lines:
        p = ln.split()
        if p and p[0] == "ROW":
            rows[int(p[1])] = dict(busId=int(p[2]), parent=int(p[3]), kind=p[4], drv=p[5])
    return rows


def expected(rows):
    """{topic: [payload, ...]} in pump order (row order, capabilities in emission order)."""
    out = {}
    for r in sorted(rows):
        row = rows[r]
        if row["kind"] != "dev":
            continue
        parent = rows[row["parent"]]
        chan = -1 if row["parent"] == 0 else parent["busId"]
        truth = TRUTH.get((row["drv"], chan))
        if not truth:
            continue
        for cap in CAP_ORDER:
            if cap not in truth:
                continue
            raw, dec = truth[cap]
            value = raw / (10 ** dec) if dec else raw
            out.setdefault("iop/json/" + cap, []).append(
                json.dumps({"row": r, "value": value}, separators=(",", ":")))
            out.setdefault("iop/csv/" + cap, []).append("%d,%s" % (r, value))
    return out


def by_topic(pairs):
    d = {}
    for t, p in pairs:
        d.setdefault(t, []).append(p)
    return d


def compare(got, want, what):
    ok = True
    for t in sorted(set(got) | set(want)):
        if got.get(t, []) != want.get(t, []):
            ok = False
            print("FAIL %s topic %s:\n  got  %s\n  want %s" % (what, t, got.get(t), want.get(t)))
    return ok


def main():
    mode, path = sys.argv[1], sys.argv[2]
    lines = open(path).read().splitlines()
    rows = parse_rows(lines)
    want = expected(rows)
    n_want = sum(len(v) for v in want.values())

    if mode == "payloads":
        got = by_topic((l.split(" ", 2)[1], l.split(" ", 2)[2]) for l in lines if l.startswith("SINK "))
        ok = compare(got, want, "sink") and n_want == 8
        # two identical sensors on different channels: distinct row ids and distinct values
        temps = [json.loads(p) for p in got.get("iop/json/temperature", [])]
        ok = ok and len({t["row"] for t in temps}) == len(temps) == 3 and len({t["value"] for t in temps}) == 3
        print("%s: %d payloads (JSON+CSV) byte-exact against the independent formatter; rows/values distinct" %
              ("OK" if ok else "FAIL", n_want))
        sys.exit(0 if ok else 1)

    if mode == "capture":
        cap = [l.split(" ", 1) for l in open(sys.argv[3]).read().splitlines() if l]
        got = by_topic((t, p) for t, p in cap)
        ok = compare(got, want, "broker capture") and len(cap) == n_want == 8
        print("%s: mosquitto_sub captured %d messages, byte-exact against the independent formatter" %
              ("OK" if ok else "FAIL", len(cap)))
        sys.exit(0 if ok else 1)

    if mode == "packets":
        import paho.mqtt.client as mqtt
        from paho.mqtt.enums import CallbackAPIVersion
        captured = []

        def spy(command, packet, mid, qos, info=None, *a, **kw):
            captured.append(bytes(packet))
            return mqtt.MQTTErrorCode.MQTT_ERR_SUCCESS

        class Dummy:
            def close(self):
                pass

        c = mqtt.Client(CallbackAPIVersion.VERSION2, client_id="r2", protocol=mqtt.MQTTv311)
        c._packet_queue = spy
        c._sock = Dummy()
        n = 0
        ok = True
        for l in lines:
            if not l.startswith("PKT "):
                continue
            topic, payload_hex, pkt_hex = l[4:].split("\t")
            captured.clear()
            c._send_publish(0, topic.encode(), bytes.fromhex(payload_hex), qos=0, retain=False, dup=False,
                            info=mqtt.MQTTMessageInfo(0))
            n += 1
            if captured != [bytes.fromhex(pkt_hex)]:
                ok = False
                print("FAIL packet %s\n  ours %s\n  paho %s" % (topic, pkt_hex, captured[0].hex() if captured else None))
        ok = ok and n == 8
        print("%s: %d PUBLISH packets byte-exact against paho-mqtt %s" % ("OK" if ok else "FAIL", n, mqtt.__version__ if hasattr(mqtt, "__version__") else ""))
        sys.exit(0 if ok else 1)

    sys.exit(2)


main()
