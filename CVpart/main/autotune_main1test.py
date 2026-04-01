import json
import os
import random
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
TARGET = ROOT / "main1test.py"
BEST_JSON = ROOT / "autotune_best.json"

FRAMES_PER_RUN = int(os.getenv("AUTOTUNE_FRAMES", "600"))
TRIALS = int(os.getenv("AUTOTUNE_TRIALS", "45"))
SEED = int(os.getenv("AUTOTUNE_SEED", "20260331"))

BOUNDS = {
    "TUNE_KP_STRAIGHT": (0.45, 1.10),
    "TUNE_KD_STRAIGHT": (0.06, 0.22),
    "TUNE_KP_CURVE": (0.80, 1.45),
    "TUNE_KD_CURVE": (0.10, 0.32),
    "TUNE_LOOKAHEAD_GAIN": (0.20, 0.65),
    "TUNE_STEER_SCALE": (16.0, 30.0),
    "TUNE_DEADBAND": (3.0, 10.0),
    "TUNE_SPEED_BASE": (0.95, 1.60),
    "TUNE_SPEED_ERR_GAIN": (0.03, 0.12),
    "TUNE_SPEED_ANG_GAIN": (0.01, 0.05),
    "TUNE_SPEED_TURN_GAIN": (0.15, 0.70),
    "TUNE_SPEED_LOST_PENALTY": (0.10, 0.45),
    "TUNE_CONF_MIN": (0.18, 0.40),
    "TUNE_TH_OFFSET": (4, 14),
    "TUNE_TURN_ERR_CM": (2.2, 5.8),
}


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def rand_range(rng, lo, hi):
    return lo + (hi - lo) * rng.random()


def sample_params(rng):
    return {
        "TUNE_KP_STRAIGHT": rand_range(rng, *BOUNDS["TUNE_KP_STRAIGHT"]),
        "TUNE_KD_STRAIGHT": rand_range(rng, *BOUNDS["TUNE_KD_STRAIGHT"]),
        "TUNE_KP_CURVE": rand_range(rng, *BOUNDS["TUNE_KP_CURVE"]),
        "TUNE_KD_CURVE": rand_range(rng, *BOUNDS["TUNE_KD_CURVE"]),
        "TUNE_LOOKAHEAD_GAIN": rand_range(rng, *BOUNDS["TUNE_LOOKAHEAD_GAIN"]),
        "TUNE_STEER_SCALE": rand_range(rng, *BOUNDS["TUNE_STEER_SCALE"]),
        "TUNE_DEADBAND": rand_range(rng, *BOUNDS["TUNE_DEADBAND"]),
        "TUNE_SPEED_BASE": rand_range(rng, *BOUNDS["TUNE_SPEED_BASE"]),
        "TUNE_SPEED_ERR_GAIN": rand_range(rng, *BOUNDS["TUNE_SPEED_ERR_GAIN"]),
        "TUNE_SPEED_ANG_GAIN": rand_range(rng, *BOUNDS["TUNE_SPEED_ANG_GAIN"]),
        "TUNE_SPEED_TURN_GAIN": rand_range(rng, *BOUNDS["TUNE_SPEED_TURN_GAIN"]),
        "TUNE_SPEED_LOST_PENALTY": rand_range(rng, *BOUNDS["TUNE_SPEED_LOST_PENALTY"]),
        "TUNE_CONF_MIN": rand_range(rng, *BOUNDS["TUNE_CONF_MIN"]),
        "TUNE_TH_OFFSET": int(rand_range(rng, float(BOUNDS["TUNE_TH_OFFSET"][0]), float(BOUNDS["TUNE_TH_OFFSET"][1]))),
        "TUNE_TURN_ERR_CM": rand_range(rng, *BOUNDS["TUNE_TURN_ERR_CM"]),
    }


def blend_params(best_params, rng, scale=0.20):
    p = dict(best_params)
    for k in p:
        lo, hi = BOUNDS[k]
        if k == "TUNE_TH_OFFSET":
            p[k] = int(clamp(round(float(p[k]) + rng.uniform(-3, 3)), int(lo), int(hi)))
            continue
        base = float(p[k])
        jitter = 1.0 + rng.uniform(-scale, scale)
        p[k] = clamp(base * jitter, float(lo), float(hi))
    return p


def scenario_list():
    return [
        {"SIM_INIT_X_CM": -3.0, "SIM_INIT_YAW_DEG": -4.0, "SIM_LANE_AMP_SCALE": 1.00, "SIM_YAW_RATE_SCALE": 1.00},
        {"SIM_INIT_X_CM": 4.5, "SIM_INIT_YAW_DEG": 6.5, "SIM_LANE_AMP_SCALE": 1.10, "SIM_YAW_RATE_SCALE": 1.05},
        {"SIM_INIT_X_CM": -5.0, "SIM_INIT_YAW_DEG": 8.0, "SIM_LANE_AMP_SCALE": 0.95, "SIM_YAW_RATE_SCALE": 0.95},
    ]


def run_once(params, scenario):
    env = os.environ.copy()
    env["TUNE_MAX_FRAMES"] = str(FRAMES_PER_RUN)
    env["TUNE_PRINT_EVERY"] = "0"
    env["TUNE_ENABLE_DEBUG_DRAW"] = "0"

    for k, v in params.items():
        env[k] = str(v)
    for k, v in scenario.items():
        env[k] = str(v)

    cp = subprocess.run(
        [sys.executable, str(TARGET)],
        cwd=str(ROOT),
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    out = cp.stdout + "\n" + cp.stderr
    m = re.search(r"RESULT\s+score=([\-0-9.]+)\s+mae_ex=([\-0-9.]+)\s+mae_ang=([\-0-9.]+)\s+mean_speed=([\-0-9.]+)\s+mean_conf=([\-0-9.]+)\s+lost_ratio=([\-0-9.]+)\s+sat_ratio=([\-0-9.]+)", out)
    if not m:
        return None

    return {
        "score": float(m.group(1)),
        "mae_ex": float(m.group(2)),
        "mae_ang": float(m.group(3)),
        "mean_speed": float(m.group(4)),
        "mean_conf": float(m.group(5)),
        "lost_ratio": float(m.group(6)),
        "sat_ratio": float(m.group(7)),
    }


def eval_params(params):
    rows = []
    for sc in scenario_list():
        r = run_once(params, sc)
        if r is None:
            return None
        rows.append(r)

    n = len(rows)
    agg = {
        "score": sum(r["score"] for r in rows) / n,
        "mae_ex": sum(r["mae_ex"] for r in rows) / n,
        "mae_ang": sum(r["mae_ang"] for r in rows) / n,
        "mean_speed": sum(r["mean_speed"] for r in rows) / n,
        "mean_conf": sum(r["mean_conf"] for r in rows) / n,
        "lost_ratio": sum(r["lost_ratio"] for r in rows) / n,
        "sat_ratio": sum(r["sat_ratio"] for r in rows) / n,
    }
    return agg


def main():
    rng = random.Random(SEED)

    best = None
    best_params = None

    for i in range(1, TRIALS + 1):
        if best_params is None or i <= TRIALS // 3:
            p = sample_params(rng)
        else:
            p = blend_params(best_params, rng, scale=0.18)

        agg = eval_params(p)
        if agg is None:
            print(f"[{i:02d}/{TRIALS}] run failed")
            continue

        print(
            f"[{i:02d}/{TRIALS}] score={agg['score']:.4f} ex={agg['mae_ex']:.3f} ang={agg['mae_ang']:.3f} "
            f"lost={agg['lost_ratio']:.4f} sat={agg['sat_ratio']:.4f} v={agg['mean_speed']:.3f}"
        )

        if best is None or agg["score"] < best["score"]:
            best = agg
            best_params = p

    if best is None:
        print("No valid result.")
        sys.exit(1)

    result = {
        "meta": {
            "trials": TRIALS,
            "frames_per_run": FRAMES_PER_RUN,
            "seed": SEED,
            "target": str(TARGET.name),
        },
        "best_metrics": best,
        "best_params": best_params,
    }

    BEST_JSON.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\n=== BEST ===")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"\nSaved: {BEST_JSON}")


if __name__ == "__main__":
    main()
