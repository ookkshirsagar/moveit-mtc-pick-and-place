#!/usr/bin/env python3
"""Success rate and planning time of the pick and place over random cube and place poses (mock hardware).

    source install/setup.bash
    python3 src/moveit-mtc-pick-and-place/tools/benchmark_mock.py --runs 50

Each run launches mtc_demo.launch.py headless with cube_pose_mode:=random place_pose_mode:=random and
random_seed:=N, so a run can be repeated by its seed. Planning is retried up to planning_attempts times (default
3). Prints one line per run and a Markdown summary.
"""
import argparse
import os
import re
import signal
import statistics
import subprocess
import time

PLAN_TIME = re.compile(r"Init: (\d+) ms \| Plan: (\d+) ms")
OUTCOMES = {
    "Pick and place complete!": "success",
    "Planning failed": "planning failed",
    "Execution failed": "execution failed",
    "process has died": "crashed",
}


def run_once(seed, timeout, side):
    cmd = ["ros2", "launch", "ur5e_rh56dfx_manipulation", "mtc_demo.launch.py", "rviz:=false",
           f"side:={side}", "cube_pose_mode:=random", "place_pose_mode:=random", f"random_seed:={seed}"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True, env={**os.environ, "PYTHONUNBUFFERED": "1"})
    start, outcome, plan_ms, replans = time.monotonic(), "timeout", None, 0
    try:
        for line in proc.stdout:
            replans += "Re-planning" in line
            if m := PLAN_TIME.search(line):
                plan_ms = int(m.group(2))
            hit = next((v for k, v in OUTCOMES.items() if k in line), None)
            if hit:
                outcome = hit
                break
            if time.monotonic() - start > timeout:
                break
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    return outcome, plan_ms, replans, time.monotonic() - start


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=50)
    ap.add_argument("--first-seed", type=int, default=1)
    ap.add_argument("--side", default="left")
    ap.add_argument("--timeout", type=float, default=180.0, help="seconds per run")
    a = ap.parse_args()

    results = []
    for seed in range(a.first_seed, a.first_seed + a.runs):
        outcome, plan_ms, replans, total = run_once(seed, a.timeout, a.side)
        results.append((seed, outcome, plan_ms, total, replans))
        print(f"seed {seed:3d}  {outcome:17s}  plan {plan_ms if plan_ms is not None else '-':>5} ms  "
              f"run {total:5.1f} s" + (f"  ({replans} re-plan)" if replans else ""), flush=True)

    ok = [r for r in results if r[1] == "success"]
    plans = [r[2] for r in ok if r[2] is not None]
    totals = [r[3] for r in ok]
    print("\n| Runs | Succeeded | Planning time (median / max) | Run time incl. startup (median) |")
    print("|---|---|---|---|")
    print(f"| {len(results)} | {len(ok)} ({100 * len(ok) / len(results):.0f} %) | "
          f"{statistics.median(plans) / 1000:.1f} s / {max(plans) / 1000:.1f} s | "
          f"{statistics.median(totals):.0f} s |" if ok else "| no successful run |")
    print(f"runs that needed a re-plan: {sum(1 for r in results if r[4])}")
    for seed, outcome, *_ in results:
        if outcome != "success":
            print(f"failed: seed {seed} ({outcome})")


if __name__ == "__main__":
    main()
