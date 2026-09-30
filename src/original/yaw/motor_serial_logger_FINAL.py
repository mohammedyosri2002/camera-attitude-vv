#!/usr/bin/env python3
"""
motor_serial_logger_FINAL.py
============================
Production serial logger for the 10-run IEEE Aerospace yaw campaign.
Ubuntu / Linux.  Arduino on /dev/ttyACM0 @ 115200.

    python3 motor_serial_logger_FINAL.py --port /dev/ttyACM0 --run 1 --dir CW

What it does
------------
 1. Opens the Arduino (which resets on port open) and waits for "READY".
 2. Creates  runs/run01_CW/  automatically.
 3. Prompts: "Confirm CAMERA and IMU are recording, then press ENTER."
 4. Sends  START,1,CW  itself -- no manual 'c'.
 5. Logs every received line with time.time_ns() and time.perf_counter_ns().
 6. Parses "#S,<event>,<micros>,<value>[,<extra>]" into motor_events.csv.
 7. On "#S,RUN_END,..." it finishes the files, closes the port and exits
    by itself -- no Ctrl-C needed for a normal run.
 8. Immediately fits the Arduino <-> PC affine clock map and prints
    slope, ppm, offset, residual SD in ms, and the event count.
 9. Prints a large RUN INVALID banner if too few sync events were seen.

No analysis beyond the clock fit is performed here. Raw lines are never
filtered, reordered or discarded.

Output
------
    runs/run01_CW/motor_raw.csv
        t_wall_ns, t_perf_ns, t_wall_s, line
    runs/run01_CW/motor_events.csv
        event, t_arduino_us, t_wall_ns, t_perf_ns, t_wall_s, value, extra
    runs/run01_CW/motor_clockfit.json
"""

import argparse
import csv
import json
import os
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial is missing.  pip3 install pyserial")


# ------------------------------------------------------------------
# Expected number of "#S," events for the full profile:
#   RUN_BEGIN 1 | BASELINE_START x2 | ANGLE_PHASE x2 | MOVE x2x16=32
#   HOLD x2x17=34 | INTERMISSION x2 | RATE_PHASE x2 | RATE x2x7=14
#   RETURN x2 | BASELINE_END x2 | RUN_END 1            ->  94
# ------------------------------------------------------------------
EXPECTED_EVENTS = 94
MIN_EVENTS_DEFAULT = 90

# START -> end of final baseline is 756.664 s; allow generous margin.
DEFAULT_TIMEOUT_S = 1100.0


def banner(msg):
    line = "!" * 72
    print("\n" + line)
    for row in msg.split("\n"):
        print("!! " + row)
    print(line + "\n")


def affine_fit(t_ref, t_ard):
    """Least-squares fit  t_ard = a * t_ref + b  without numpy.
    Returns (a, b, residual_sd, residuals)."""
    n = len(t_ref)
    mx = sum(t_ref) / n
    my = sum(t_ard) / n
    sxx = sum((x - mx) ** 2 for x in t_ref)
    sxy = sum((x - mx) * (y - my) for x, y in zip(t_ref, t_ard))
    if sxx == 0.0:
        return float("nan"), float("nan"), float("nan"), []
    a = sxy / sxx
    b = my - a * mx
    res = [y - (a * x + b) for x, y in zip(t_ref, t_ard)]
    mr = sum(res) / n
    sd = (sum((r - mr) ** 2 for r in res) / (n - 1)) ** 0.5 if n > 1 else float("nan")
    return a, b, sd, res


def main():
    ap = argparse.ArgumentParser(
        description="Serial logger for the final yaw campaign.")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--run", type=int, required=True, choices=range(1, 11),
                    metavar="1-10")
    ap.add_argument("--dir", required=True, choices=["CW", "CCW"])
    ap.add_argument("--out", default="runs")
    ap.add_argument("--min-events", type=int, default=MIN_EVENTS_DEFAULT)
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                    help="seconds to wait for RUN_END after START")
    ap.add_argument("--no-confirm", action="store_true",
                    help="skip the ENTER gate (not recommended)")
    a = ap.parse_args()

    run_dir = os.path.join(a.out, f"run{a.run:02d}_{a.dir}")
    os.makedirs(run_dir, exist_ok=True)
    raw_path = os.path.join(run_dir, "motor_raw.csv")
    ev_path = os.path.join(run_dir, "motor_events.csv")
    fit_path = os.path.join(run_dir, "motor_clockfit.json")

    for p in (raw_path, ev_path):
        if os.path.exists(p):
            banner(f"{p} already exists.\n"
                   f"Refusing to overwrite an existing run.\n"
                   f"Move or rename the folder first.")
            sys.exit(1)

    print(f"Run {a.run:02d} {a.dir}  ->  {run_dir}")
    print(f"Opening {a.port} @ {a.baud} ...")

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.2)
    except Exception as e:
        sys.exit(f"Could not open {a.port}: {e}\n"
                 f"Close any other program using the port "
                 f"(Arduino IDE Serial Monitor, screen, minicom).")

    # Opening the serial port resets an Arduino Uno via DTR.
    # IMPORTANT: do NOT sleep and then reset_input_buffer() here; READY may
    # already have been transmitted during that delay and would be discarded.
    # Clear any stale bytes immediately, then wait through the bootloader/reset
    # interval until the sketch prints READY.
    ser.reset_input_buffer()

    # ---------------- wait for READY -------------------------------
    header_lines = []
    print("Waiting for Arduino READY ...")
    t0 = time.time()
    ready = False
    while time.time() - t0 < 15.0:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", "replace").strip()
        if not line:
            continue
        print("  " + line)
        header_lines.append(line)
        if line == "READY":
            ready = True
            break

    if not ready:
        ser.close()
        banner("Arduino never sent READY.\n"
               "Check the sketch is motor_yaw_FINAL.ino and the baud rate.")
        sys.exit(1)

    # ---------------- operator gate --------------------------------
    if not a.no_confirm:
        try:
            input("\nConfirm CAMERA and IMU are recording, then press ENTER. ")
        except (EOFError, KeyboardInterrupt):
            ser.close()
            sys.exit("\nAborted before START.")

    cmd = f"START,{a.run},{a.dir}\n"
    t_send_wall = time.time_ns()
    t_send_perf = time.perf_counter_ns()
    ser.write(cmd.encode("ascii"))
    ser.flush()
    print(f"\nSent: {cmd.strip()}")
    print("Logging ... (normal completion is automatic on RUN_END)\n")

    # ---------------- logging loop ---------------------------------
    n_events = 0
    n_lines = 0
    saw_run_end = False
    events = []          # (event, t_ard_us, t_wall_ns, t_perf_ns, value, extra)
    t_start = time.time()

    try:
        with open(raw_path, "w", newline="") as fr, \
             open(ev_path, "w", newline="") as fe:

            wr = csv.writer(fr)
            wr.writerow(["t_wall_ns", "t_perf_ns", "t_wall_s", "line"])
            wr.writerow([t_send_wall, t_send_perf,
                         f"{t_send_wall / 1e9:.6f}", "#TX," + cmd.strip()])
            fr.flush()

            we = csv.writer(fe)
            we.writerow(["event", "t_arduino_us", "t_wall_ns", "t_perf_ns",
                         "t_wall_s", "value", "extra"])
            fe.flush()

            while True:
                raw = ser.readline()
                if raw:
                    t_wall_ns = time.time_ns()
                    t_perf_ns = time.perf_counter_ns()
                    line = raw.decode("utf-8", "replace").strip()
                    if line:
                        n_lines += 1
                        wr.writerow([t_wall_ns, t_perf_ns,
                                     f"{t_wall_ns / 1e9:.6f}", line])
                        fr.flush()
                        print(f"{(time.time() - t_start):8.2f}  {line}")

                        if line.startswith("#S,"):
                            p = line.split(",")
                            if len(p) >= 4:
                                ev = p[1]
                                try:
                                    us = int(p[2])
                                except ValueError:
                                    us = None
                                val = p[3] if len(p) > 3 else ""
                                ext = p[4] if len(p) > 4 else ""
                                if us is not None:
                                    we.writerow([ev, us, t_wall_ns, t_perf_ns,
                                                 f"{t_wall_ns / 1e9:.6f}",
                                                 val, ext])
                                    fe.flush()
                                    events.append((ev, us, t_wall_ns,
                                                   t_perf_ns, val, ext))
                                    n_events += 1
                                    if ev == "RUN_END":
                                        saw_run_end = True

                        if line.startswith("#ERR,"):
                            banner("Arduino reported an error:\n" + line)

                if saw_run_end:
                    # drain anything still in flight, then stop
                    t_drain = time.time()
                    while time.time() - t_drain < 1.0:
                        extra_raw = ser.readline()
                        if not extra_raw:
                            continue
                        t_wall_ns = time.time_ns()
                        t_perf_ns = time.perf_counter_ns()
                        ln = extra_raw.decode("utf-8", "replace").strip()
                        if ln:
                            wr.writerow([t_wall_ns, t_perf_ns,
                                         f"{t_wall_ns / 1e9:.6f}", ln])
                            print(f"{(time.time() - t_start):8.2f}  {ln}")
                    fr.flush()
                    break

                if time.time() - t_start > a.timeout:
                    banner(f"TIMEOUT after {a.timeout:.0f} s without RUN_END.\n"
                           f"The run did not complete normally.")
                    break

    except KeyboardInterrupt:
        banner("Interrupted by operator before RUN_END.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

    print(f"\nWrote {raw_path}  ({n_lines} lines)")
    print(f"Wrote {ev_path}  ({n_events} sync events)")

    # ---------------- clock fit ------------------------------------
    fit = {"run": a.run, "direction": a.dir, "n_events": n_events,
           "expected_events": EXPECTED_EVENTS, "saw_run_end": saw_run_end}

    if n_events >= 3:
        t_ard = [e[1] / 1e6 for e in events]                    # us -> s
        t_perf = [e[3] / 1e9 for e in events]                   # ns -> s
        t_wall = [e[2] / 1e9 for e in events]                   # ns -> s
        p0, w0, a0 = t_perf[0], t_wall[0], t_ard[0]

        a_perf, b_perf, sd_perf, _ = affine_fit(
            [x - p0 for x in t_perf], [y - a0 for y in t_ard])
        a_wall, b_wall, sd_wall, _ = affine_fit(
            [x - w0 for x in t_wall], [y - a0 for y in t_ard])

        span = t_perf[-1] - t_perf[0]

        print("\n--- Arduino <-> PC clock fit  (t_arduino = a * t_pc + b) ---")
        print(f"  reference = perf_counter (monotonic)")
        print(f"    slope a        = {a_perf:.9f}")
        print(f"    rate offset    = {(a_perf - 1.0) * 1e6:+.1f} ppm")
        print(f"    offset b       = {b_perf:+.6f} s "
              f"(relative to first event)")
        print(f"    residual SD    = {sd_perf * 1000.0:.3f} ms")
        print(f"  reference = wall clock (time.time)")
        print(f"    slope a        = {a_wall:.9f}"
              f"   ({(a_wall - 1.0) * 1e6:+.1f} ppm)")
        print(f"    residual SD    = {sd_wall * 1000.0:.3f} ms")
        print(f"  events used      = {n_events}   span = {span:.1f} s")

        fit.update(
            a_perf=a_perf, b_perf=b_perf, resid_sd_ms_perf=sd_perf * 1000.0,
            ppm_perf=(a_perf - 1.0) * 1e6,
            a_wall=a_wall, b_wall=b_wall, resid_sd_ms_wall=sd_wall * 1000.0,
            ppm_wall=(a_wall - 1.0) * 1e6,
            span_s=span,
            t_perf_first_ns=events[0][3], t_wall_first_ns=events[0][2],
            t_arduino_first_us=events[0][1])
    else:
        print("\nToo few events for a clock fit.")

    with open(fit_path, "w") as f:
        json.dump(fit, f, indent=2)
    print(f"Wrote {fit_path}")

    # ---------------- validity verdict ------------------------------
    if (not saw_run_end) or (n_events < a.min_events):
        banner(f"RUN INVALID -- CHECK SERIAL LOG BEFORE CONTINUING.\n"
               f"sync events = {n_events} (expected {EXPECTED_EVENTS}, "
               f"minimum {a.min_events})\n"
               f"RUN_END seen = {saw_run_end}\n"
               f"Do NOT count this as one of the ten runs.")
        sys.exit(2)

    print(f"\nRun {a.run:02d} {a.dir} complete: "
          f"{n_events}/{EXPECTED_EVENTS} sync events, RUN_END received.")
    sys.exit(0)


if __name__ == "__main__":
    main()
