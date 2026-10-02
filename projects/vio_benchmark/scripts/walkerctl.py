#!/usr/bin/env python3
"""
walkerctl.py -- a small client for walkerd, the supervisor inside the
dotflysim2 container, so scripts can drive the same units walker's dashboard
does (sim, bridge, project, ...). Runs INSIDE the container (lib.sh calls it).

    walkerctl.py ping
    walkerctl.py state [UNIT]                 whole snapshot, or one unit's state
    walkerctl.py select world=powerline drone=m4e cameras=fisheye
    walkerctl.py start UNIT                   sim | bridge | ...
    walkerctl.py stop UNIT
    walkerctl.py project PACKAGE [k=v ...] [--rebuild|--no-rebuild]
                                              start a mission with ROS parameters
    walkerctl.py wait UNIT STATE[,STATE] [TIMEOUT_S]
                                              exit 0 once the unit is in a state

Protocol (walkerd/__main__.py handle()): one JSON object per line on
/run/walker/walkerd.sock, a reply carrying the request's "id"; events walkerd
broadcasts to every client in between are skipped.
"""
import json
import os
import socket
import sys
import time

SOCKET = os.environ.get("WALKERD_SOCKET", "/run/walker/walkerd.sock")


def request(req: dict, timeout: float = 30.0) -> dict:
    req = dict(req, id=int(time.time() * 1000) % 1_000_000_000)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect(SOCKET)
        s.sendall((json.dumps(req) + "\n").encode())
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                if msg.get("id") == req["id"]:
                    return msg
    raise TimeoutError(f"no reply from walkerd to {req.get('op')}")


def unit_state(name: str) -> str:
    snap = request({"op": "state"})
    for u in snap.get("units", []):
        if u.get("name") == name:
            return u.get("state", "?")
    return "?"


def kv(args):
    out = {}
    for a in args:
        if "=" not in a:
            raise SystemExit(f"expected key=value, got '{a}'")
        k, v = a.split("=", 1)
        out[k] = v
    return out


def main(argv) -> int:
    if not argv:
        print(__doc__)
        return 2
    cmd, rest = argv[0], argv[1:]
    try:
        if cmd == "ping":
            print(json.dumps(request({"op": "ping"})))
            return 0
        if cmd == "state":
            if rest:
                print(unit_state(rest[0]))
            else:
                print(json.dumps(request({"op": "state"}), indent=1))
            return 0
        if cmd == "select":
            reply = request({"op": "select", "args": kv(rest)})
        elif cmd in ("start", "stop"):
            reply = request({"op": cmd, "unit": rest[0]})
        elif cmd == "project":
            rebuild = "--no-rebuild" not in rest
            params = kv([a for a in rest[1:] if not a.startswith("--")])
            reply = request({"op": "start", "unit": "project",
                             "args": {"package": rest[0], "params": params, "rebuild": rebuild}})
        elif cmd == "wait":
            unit, wanted = rest[0], set(rest[1].split(","))
            deadline = time.monotonic() + float(rest[2] if len(rest) > 2 else 120)
            while time.monotonic() < deadline:
                state = unit_state(unit)
                if state in wanted:
                    print(state)
                    return 0
                time.sleep(1.0)
            print(f"timeout: {unit} is {unit_state(unit)}, wanted {','.join(sorted(wanted))}", file=sys.stderr)
            return 1
        else:
            print(__doc__)
            return 2
    except (OSError, TimeoutError) as exc:
        print(f"walkerctl: cannot talk to walkerd at {SOCKET}: {exc} -- is it up? (./walker up)", file=sys.stderr)
        return 3
    print(json.dumps(reply))
    return 0 if reply.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
