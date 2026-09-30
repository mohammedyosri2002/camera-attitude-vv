#!/usr/bin/env python3
"""
motor_serial_logger_pitch_roll_FINAL.py
=======================================
Serial logger for the pitch/roll campaign. Ubuntu, /dev/ttyACM0, 115200.

    python3 motor_serial_logger_pitch_roll_FINAL.py \
        --port /dev/ttyACM0 --run 1 --axis AXIS_A --out runs_pr

Same validated architecture as the yaw campaign logger:
 1. opens the Arduino (which resets on port open) and waits for "READY"
 2. creates runs_pr/run01_AXIS_A/ automatically, refusing to overwrite
 3. prompts once: confirm camera and IMU are recording
 4. sends START,1,AXIS_A itself -- no manual typing
 5. logs every line with time.time_ns() and time.perf_counter_ns()
 6. parses "#S,<event>,<micros>,<value>[,<extra>]" into motor_events.csv
 7. exits by itself on "#S,RUN_END" -- Ctrl-C is not needed
 8. fits the affine Arduino <-> PC clock map and writes motor_clockfit.json

WHAT CHANGED vs the yaw logger
------------------------------
The expected event count is NOT the yaw campaign's hard-coded 94. It is
DERIVED from the metadata the sketch prints in its header
(#STATIC_TARGETS, #RATES_DPS, #RATE_CYCLES), so it stays correct if the
profile is edited. For the default pitch/roll profile the derived count is
114. If the header cannot be parsed the logger says so and falls back to a
count-independent validity check (RUN_END seen, >= 40 events).
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


DEFAULT_TIMEOUT_S = 900.0     # default profile is ~484 s from START
MOVE_RATE_DPS = 5          # must match MOVE_RATE_DPS in the sketch
PREBIAS_SETTLE_S = 3.0     # must match PREBIAS_SETTLE_MS in the sketch
READY_TIMEOUT_S = 15.0        # the sketch prints READY ~500-600 ms after reset


def banner(msg):
    line = "!" * 72
    print("\n" + line)
    for row in msg.split("\n"):
        print("!! " + row)
    print(line + "\n")


def affine_fit(t_ref, t_ard):
    """Least squares  t_ard = a * t_ref + b.  Returns (a, b, sd)."""
    n = len(t_ref)
    mx = sum(t_ref) / n
    my = sum(t_ard) / n
    sxx = sum((x - mx) ** 2 for x in t_ref)
    sxy = sum((x - mx) * (y - my) for x, y in zip(t_ref, t_ard))
    if sxx == 0.0:
        return float("nan"), float("nan"), float("nan")
    a = sxy / sxx
    b = my - a * mx
    res = [y - (a * x + b) for x, y in zip(t_ref, t_ard)]
    mr = sum(res) / n
    sd = (sum((r - mr) ** 2 for r in res) / (n - 1)) ** 0.5 if n > 1 else float("nan")
    return a, b, sd


def expected_events(header):
    """Derive the event count from the sketch's own metadata.

        RUN_BEGIN                        1
        BASELINE_START / _END            2
        STATIC_PHASE_START / _END        2
        MOVE_START / _END                2 * (number of non-zero transits)
        HOLD_START / _END                2 * len(static targets)
        INTERMISSION_START / _END        2
        RATE_PHASE_START / _END          2
        RATE_POSITION_START / _END       2
        RATE_LEG_START / _END            2 * 2 * sum(cycles)
        RETURN_ZERO_START / _END         2
        BASELINE_END / _END              2
        RUN_END                          1
    """
    tg = header.get("STATIC_TARGETS")
    cy = header.get("RATE_CYCLES")
    if not tg or not cy:
        return None, None
    moves = sum(1 for a, b in zip(tg[:-1], tg[1:]) if a != b)
    legs = 2 * sum(cy)
    n = (1 + 2 + 2 + 2 * moves + 2 * len(tg) + 2 + 2 + 2
         + 2 * legs + 2 + 2 + 1)
    return n, dict(targets=len(tg), moves=moves, legs=legs)


def main():
    ap = argparse.ArgumentParser(
        description="Serial logger for the pitch/roll campaign.")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--run", type=int, required=True, choices=range(1, 21),
                    metavar="1-20")
    ap.add_argument("--axis", required=True, choices=["AXIS_A", "AXIS_B"])
    ap.add_argument("--out", default="runs_pr")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    ap.add_argument("--min-events", type=int, default=None,
                    help="override the profile-derived minimum")
    ap.add_argument("--no-confirm", action="store_true")
    ap.add_argument(
        "--prebias",
        type=float,
        default=0.0,
        help="commanded pre-position in degrees before camera/IMU recording"
    )
    a = ap.parse_args()

    run_dir = os.path.join(a.out, f"run{a.run:02d}_{a.axis}")
    os.makedirs(run_dir, exist_ok=True)
    raw_path = os.path.join(run_dir, "motor_raw.csv")
    ev_path = os.path.join(run_dir, "motor_events.csv")
    fit_path = os.path.join(run_dir, "motor_clockfit.json")

    for p in (raw_path, ev_path):
        if os.path.exists(p):
            banner(f"{p} already exists.\n"
                   f"Refusing to overwrite an existing run.\n"
                   f"Move or rename {run_dir} first.")
            sys.exit(1)

    print(f"Run {a.run:02d} {a.axis}  ->  {run_dir}")
    print(f"Opening {a.port} @ {a.baud} ...")
    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.2)
    except Exception as e:
        sys.exit(f"Could not open {a.port}: {e}\n"
                 f"Close anything else holding the port "
                 f"(Arduino IDE Serial Monitor, screen, minicom).")

    # ---- READY race ------------------------------------------------------
    # Opening the port asserts DTR and resets the board. setup() then does
    # Serial.begin(), delay(500) and prints the metadata header followed by
    # READY, i.e. roughly 500-600 ms after the reset. Clearing the input
    # buffer AFTER a settling sleep would therefore discard READY and the
    # whole header. The buffer is cleared ONCE, immediately after opening the
    # port and before the board has had time to say anything, and it is never
    # touched again. From here on we only ever read.
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    # [FIX] motor_raw.csv is opened HERE, before READY and before PREBIAS, so
    # the boot header, the READY handshake and the whole PREBIAS transaction
    # are part of the raw serial record. Previously it opened only after the
    # ENTER gate and all of that was lost.
    fr = open(raw_path, "w", newline="")
    wr = csv.writer(fr)
    wr.writerow(["t_wall_ns", "t_perf_ns", "t_wall_s", "line"])
    fr.flush()

    def log_line(line, tag=""):
        t_wall_ns = time.time_ns()
        t_perf_ns = time.perf_counter_ns()
        wr.writerow([t_wall_ns, t_perf_ns, f"{t_wall_ns / 1e9:.6f}",
                     (tag + line) if tag else line])
        fr.flush()
        return t_wall_ns, t_perf_ns

    header_lines, header = [], {}
    print("Waiting for Arduino READY (reading continuously, buffer never "
          "cleared again) ...")
    t0 = time.time()
    ready = False
    while time.time() - t0 < READY_TIMEOUT_S:
        raw = ser.readline()            # 0.2 s read timeout, so this polls
        if not raw:
            continue
        line = raw.decode("utf-8", "replace").strip()
        if not line:
            continue
        log_line(line)
        print(f"  [{time.time() - t0:5.2f} s]  {line}")
        header_lines.append(line)
        if line.startswith("#FW,"):
            header["FW"] = line
        # capture the metadata even though the full parser runs below, so a
        # header that arrives before READY is never lost
        if line.startswith("#STATIC_TARGETS"):
            header["STATIC_TARGETS"] = [int(x) for x in line.split(",")[1:]
                                        if x.strip()]
        elif line.startswith("#RATE_CYCLES"):
            header["RATE_CYCLES"] = [int(x) for x in line.split(",")[1:]
                                     if x.strip()]
        elif line.startswith("#RATES_DPS"):
            header["RATES_DPS"] = [int(x) for x in line.split(",")[1:]
                                   if x.strip()]
        if line == "READY":
            ready = True
            break
    if not ready:
        fr.close(); ser.close()
        banner(f"Arduino never sent READY within {READY_TIMEOUT_S:.0f} s.\n"
               f"Lines seen: {len(header_lines)}\n"
               "Check the sketch is motor_pitch_roll_FINAL.ino, the baud rate "
               "is 115200, and that nothing else holds the port.")
        sys.exit(1)

    fw = header.get("FW", "")
    if "motor_pitch_roll_FINAL" not in fw:
        fr.close(); ser.close()
        banner(f"Unexpected firmware banner: {fw!r}\n"
               "Expected a line starting '#FW,motor_pitch_roll_FINAL,'.")
        sys.exit(1)
    if abs(a.prebias) > 1e-9 and ",v2" not in fw:
        fr.close(); ser.close()
        banner(f"Firmware is {fw!r} but --prebias needs v2 or later.\n"
               "Upload the corrected motor_pitch_roll_FINAL.ino.")
        sys.exit(1)
    print(f"\nFirmware: {fw}")

    # ------------------------------------------------------
    # PREBIAS working point
    # ------------------------------------------------------
    if abs(a.prebias) > 1e-9:

        # [ROOT-CAUSE FIX] This line previously read
        #     cmd_pre = f"PREBIAS,{a.prebias:.6f}\\n"
        # The DOUBLE backslash produced the two literal characters '\\' and 'n'
        # instead of a newline byte. The Arduino's loop() only calls
        # handleCommand() when it receives 0x0A, so the command was never
        # terminated, handleCommand() was never reached, nothing was printed,
        # and the logger timed out. A single backslash is required.
        cmd_pre = f"PREBIAS,{a.prebias:.6f}\n"

        print()
        print("=" * 70)
        print(f"COMMANDING PREBIAS {a.prebias:+.3f} deg")
        print("Waiting for Arduino to reach and hold the new working point...")
        print("=" * 70)

        # [FIX] timeout is derived from the commanded motion, not a magic 30 s:
        #   move time = |deg| / MOVE_RATE_DPS, plus the Arduino's 3 s settle,
        #   plus generous margin.
        move_s = abs(a.prebias) / MOVE_RATE_DPS
        pre_timeout = move_s + PREBIAS_SETTLE_S + 20.0
        print(f"  commanded move {move_s:.2f} s at {MOVE_RATE_DPS} deg/s, "
              f"+{PREBIAS_SETTLE_S:.0f} s settle -> timeout {pre_timeout:.1f} s")

        t_tx_wall, t_tx_perf = log_line("#TX," + cmd_pre.strip())
        ser.write(cmd_pre.encode("ascii"))
        ser.flush()

        got_ack = got_begin = prebias_ok = False
        pre_lines = []
        t_pre = time.time()

        while time.time() - t_pre < pre_timeout:

            raw = ser.readline()

            if not raw:
                continue

            line = raw.decode("utf-8", "replace").strip()

            if not line:
                continue

            log_line(line)
            pre_lines.append(line)
            print(f"  [{time.time() - t_pre:5.2f} s]  {line}")

            if line.startswith("#ERR,"):
                fr.close(); ser.close()
                banner("Arduino rejected the PREBIAS command:\n" + line +
                       "\nDo NOT start the experiment.")
                sys.exit(2)

            if line.startswith("#ACK,PREBIAS,"):
                got_ack = True
                continue

            if line.startswith("#PREBIAS_BEGIN,"):
                if not got_ack:
                    print("  [note] BEGIN arrived without an ACK; older "
                          "firmware, continuing.")
                got_begin = True
                continue

            if line.startswith("#PREBIAS_DONE,"):
                prebias_ok = True
                break

        if not prebias_ok:
            fr.close(); ser.close()
            stage = ("no response at all -- the Arduino never parsed the "
                     "command (check that the command ends in a real newline)"
                     if not got_ack and not got_begin else
                     "ACK received but the move never completed"
                     if got_ack and not got_begin else
                     "the move started but #PREBIAS_DONE never arrived")
            banner(f"PREBIAS TIMEOUT after {pre_timeout:.1f} s: {stage}.\n"
                   f"Lines seen during PREBIAS: {len(pre_lines)}\n"
                   "Do NOT start the experiment.")
            sys.exit(2)

        done = next(l for l in pre_lines if l.startswith("#PREBIAS_DONE,"))
        begin = next((l for l in pre_lines
                      if l.startswith("#PREBIAS_BEGIN,")), "")
        pre_pulses = None
        for tok in done.split(","):
            if tok.startswith("pulses="):
                pre_pulses = int(tok.split("=", 1)[1])
        if pre_pulses is None and begin:
            try:
                pre_pulses = int(begin.split(",")[2])
            except Exception:
                pass

        meta = {
            "prebias_command_deg": a.prebias,
            "prebias_pulses": pre_pulses,
            "prebias_effective_deg": (pre_pulses * 360.0 / 25600.0
                                      if pre_pulses is not None else None),
            "move_rate_dps": MOVE_RATE_DPS,
            "settle_s": PREBIAS_SETTLE_S,
            "firmware": fw,
            "ack_seen": got_ack,
            "begin_line": begin,
            "done_line": done,
            "t_command_wall_ns": t_tx_wall,
            "t_command_perf_ns": t_tx_perf,
            "elapsed_s": time.time() - t_pre,
            "run": a.run,
            "axis": a.axis,
            "note": ("PULSE-DERIVED COMMANDED pre-position held by motor "
                     "torque. NOT an encoder-verified or measured physical "
                     "angle. The held pose is the software and camera "
                     "relative zero for this run."),
        }
        with open(os.path.join(run_dir, "prebias_meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        print(f"\n  wrote {os.path.join(run_dir, 'prebias_meta.json')}")
        if pre_pulses is not None:
            print(f"  commanded {a.prebias:+.6f} deg -> {pre_pulses:+d} pulses "
                  f"-> {pre_pulses * 360.0 / 25600.0:+.7f} deg effective "
                  f"(quantization "
                  f"{pre_pulses * 360.0 / 25600.0 - a.prebias:+.7f} deg)")

        print()
        print("=" * 70)
        print("PREBIAS COMPLETE")
        print("MOTOR HOLDING NEW WORKING POINT")
        print("DO NOT RESET OR CLOSE THIS LOGGER")
        print("=" * 70)
        print("  The motor is energised and holding the pre-position by torque.")
        print("  Closing this logger, pressing RESET, or opening any other")
        print("  serial program on this port will reset the Arduino, drop the")
        print("  holding torque and destroy the working-point reference.")
        print("  Leave this window alone until RUN_END.")
        print("Motor is HOLDING the new working point.")
        print("Now start CAMERA and IMU.")
        print("Do NOT close this logger.")
        print()

    if not a.no_confirm:
        try:
            input("\nConfirm CAMERA and IMU are recording, then press ENTER. ")
        except (EOFError, KeyboardInterrupt):
            fr.close(); ser.close()
            sys.exit("\nAborted before START.")

    cmd = f"START,{a.run},{a.axis}\n"
    t_send_wall = time.time_ns()
    t_send_perf = time.perf_counter_ns()
    ser.write(cmd.encode("ascii"))
    ser.flush()
    print(f"\nSent: {cmd.strip()}")
    print("Logging ... (normal completion is automatic on RUN_END)\n")

    n_events = n_lines = 0
    saw_run_end = False
    events = []
    t_start = time.time()
    exp_n, exp_parts = None, None

    try:
        # [FIX] fr/wr are the ALREADY-OPEN raw log that captured the boot
        # header, READY and the PREBIAS transaction. Re-opening with "w" here
        # would truncate all of that, so the existing handle is reused and only
        # the event file is created now.
        with open(ev_path, "w", newline="") as fe:
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

                        # ---- metadata the expected count is derived from ----
                        if line.startswith("#STATIC_TARGETS"):
                            header["STATIC_TARGETS"] = [
                                int(x) for x in line.split(",")[1:] if x.strip()]
                        elif line.startswith("#RATE_CYCLES"):
                            header["RATE_CYCLES"] = [
                                int(x) for x in line.split(",")[1:] if x.strip()]
                        elif line.startswith("#RATES_DPS"):
                            header["RATES_DPS"] = [
                                int(x) for x in line.split(",")[1:] if x.strip()]
                        if exp_n is None and "STATIC_TARGETS" in header \
                                and "RATE_CYCLES" in header:
                            exp_n, exp_parts = expected_events(header)
                            if exp_n:
                                print(f"    [profile] {exp_parts['targets']} targets, "
                                      f"{exp_parts['moves']} transits, "
                                      f"{exp_parts['legs']} rate legs "
                                      f"-> {exp_n} events expected")

                        if line.startswith("#S,"):
                            p = line.split(",")
                            if len(p) >= 4:
                                ev = p[1]
                                try:
                                    us = int(p[2])
                                except ValueError:
                                    us = None
                                if us is not None:
                                    val = p[3]
                                    ext = p[4] if len(p) > 4 else ""
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
            fr.flush()
            fr.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

    print(f"\nWrote {raw_path}  ({n_lines} lines)")
    print(f"Wrote {ev_path}  ({n_events} sync events)")

    fit = {"run": a.run, "axis": a.axis,
           "prebias_command_deg": (a.prebias if abs(a.prebias) > 1e-9 else 0.0),
           "prebias_applied": bool(abs(a.prebias) > 1e-9),
           "firmware": header.get("FW", ""),
           "n_events": n_events,
           "expected_events": exp_n, "saw_run_end": saw_run_end,
           "profile": exp_parts,
           "static_targets": header.get("STATIC_TARGETS"),
           "rates_dps": header.get("RATES_DPS"),
           "rate_cycles": header.get("RATE_CYCLES")}

    if n_events >= 3:
        t_ard = [e[1] / 1e6 for e in events]
        t_perf = [e[3] / 1e9 for e in events]
        t_wall = [e[2] / 1e9 for e in events]
        p0, w0, a0 = t_perf[0], t_wall[0], t_ard[0]
        a_perf, b_perf, sd_perf = affine_fit([x - p0 for x in t_perf],
                                             [y - a0 for y in t_ard])
        a_wall, b_wall, sd_wall = affine_fit([x - w0 for x in t_wall],
                                             [y - a0 for y in t_ard])
        span = t_perf[-1] - t_perf[0]
        print("\n--- Arduino <-> PC clock fit  (t_arduino = a * t_pc + b) ---")
        print(f"  reference = perf_counter (monotonic)")
        print(f"    slope a      = {a_perf:.9f}")
        print(f"    rate offset  = {(a_perf - 1.0) * 1e6:+.1f} ppm")
        print(f"    offset b     = {b_perf:+.6f} s (relative to first event)")
        print(f"    residual SD  = {sd_perf * 1000.0:.3f} ms")
        print(f"  reference = wall clock")
        print(f"    slope a      = {a_wall:.9f}  "
              f"({(a_wall - 1.0) * 1e6:+.1f} ppm)")
        print(f"    residual SD  = {sd_wall * 1000.0:.3f} ms")
        print(f"  events used    = {n_events}   span = {span:.1f} s")
        fit.update(a_perf=a_perf, b_perf=b_perf,
                   resid_sd_ms_perf=sd_perf * 1000.0,
                   ppm_perf=(a_perf - 1.0) * 1e6,
                   a_wall=a_wall, b_wall=b_wall,
                   resid_sd_ms_wall=sd_wall * 1000.0,
                   ppm_wall=(a_wall - 1.0) * 1e6, span_s=span,
                   t_perf_first_ns=events[0][3],
                   t_wall_first_ns=events[0][2],
                   t_arduino_first_us=events[0][1])
    else:
        print("\nToo few events for a clock fit.")

    with open(fit_path, "w") as f:
        json.dump(fit, f, indent=2)
    print(f"Wrote {fit_path}")

    if a.min_events is not None:
        min_n = a.min_events
    elif exp_n:
        min_n = exp_n            # the profile is deterministic; demand all of it
    else:
        min_n = 40
        print("\n[note] sketch metadata not parsed; using a count-independent "
              "validity check (RUN_END seen and >= 40 events).")

    if (not saw_run_end) or (n_events < min_n):
        banner(f"RUN INVALID -- CHECK SERIAL LOG BEFORE CONTINUING.\n"
               f"sync events = {n_events} (expected {exp_n}, minimum {min_n})\n"
               f"RUN_END seen = {saw_run_end}\n"
               f"Do NOT count this run.")
        sys.exit(2)

    print(f"\nRun {a.run:02d} {a.axis} complete: "
          f"{n_events}/{exp_n} sync events, RUN_END received.")
    sys.exit(0)


if __name__ == "__main__":
    main()
